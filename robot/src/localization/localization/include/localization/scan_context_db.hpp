#ifndef LOCALIZATION_SCAN_CONTEXT_DB_HPP
#define LOCALIZATION_SCAN_CONTEXT_DB_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

namespace localization {

/**
 * @brief Scan-Context place recognition over the keyframes shipped with a map.
 *
 * The mapper's post-processing (edge-agent/roamerx_edge/map_loop_closure.py) already
 * writes a scan_context/ directory beside every map, but only ever uses it to propose
 * loop closures. Those descriptors answer exactly the question relocalization needs:
 * where in the map a single scan was taken, and at what heading. GlobalLocalization
 * cannot answer it - it is a bare ICP seeded from the last trusted pose, with no yaw
 * search at all - so this class rebuilds the database in C++ and asks directly.
 *
 * Descriptors are recomputed from keyframes/ rather than read back from
 * descriptors.bin. Far more maps ship keyframes than descriptors, and deriving both
 * the database and the live query through one code path makes them consistent by
 * construction, which matters more here than matching the Python byte for byte.
 *
 * Deliberately free of rclcpp so the offline checker can link it without a node.
 */
struct ScanContextParams {
  int rings = 20;
  int sectors = 60;
  double max_radius_m = 80.0;

  int cells() const { return rings * sectors; }
};

struct ScanContextCandidate {
  /// Row index in keyframes.csv, not necessarily the position in the database.
  int keyframe_index = -1;
  /// Mean absolute descriptor difference at the best yaw shift. Lower is better.
  double distance = 0.0;
  /// Yaw the query is rotated by relative to the keyframe, in (-pi, pi].
  double yaw_offset_rad = 0.0;
  /// Map-frame pose to hand a scan matcher as its initial guess.
  Eigen::Matrix4d seed_pose = Eigen::Matrix4d::Identity();
};

enum class ScanContextDistanceMetric {
  kMeanAbsoluteHeight,
  kSectorCosine,
  /// Rank places with sector cosine, but estimate yaw with absolute height distance.
  kSectorCosineAbsoluteYaw,
};

class ScanContextDatabase {
public:
  /**
   * @brief Build the database from a map session directory.
   *
   * Reads keyframes/keyframes.csv plus each keyframes/scan_XXXXX.pcd. Keyframe clouds
   * are stored in the original world frame, so each is moved into its own lidar frame
   * using the immutable raw pose. If map_manifest.json selects the optimized trajectory,
   * trajectory_optimized.csv supplies the map-frame seed pose returned to localization.
   *
   * @param map_dir  Map session directory, i.e. the parent of map.pcd.
   * @param error    Optional human-readable reason on failure.
   * @return true when at least one keyframe was indexed.
   */
  bool load(const std::string& map_dir, std::string* error = nullptr);

  void clear();

  bool empty() const { return descriptors_.empty(); }
  std::size_t size() const { return keyframe_indices_.size(); }
  const ScanContextParams& params() const { return params_; }
  const std::string& source_dir() const { return source_dir_; }
  const std::string& seed_pose_source() const { return seed_pose_source_; }

  /**
   * @brief Rank map keyframes against one scan already in the lidar frame.
   *
   * A ring-key prefilter (rotation invariant) narrows the field before the expensive
   * per-shift comparison, then every surviving candidate is compared at all `sectors`
   * circular shifts. The shift that wins is the yaw estimate.
   *
   * @param scan          Live cloud in the lidar frame, e.g. straight off /front_lidar.
   * @param top_k         Maximum candidates returned, best first.
   * @param max_distance  Reject candidates above this descriptor distance.
   * @param exclude_index keyframes.csv index to skip, or -1. Used for leave-one-out.
   * @param min_index_gap Skip keyframes within this index distance of exclude_index.
   *                      Loop closure needs it to ignore temporal neighbours;
   *                      relocalization must leave it at 0 and match anything.
   */
  std::vector<ScanContextCandidate> query(
    const pcl::PointCloud<pcl::PointXYZI>& scan,
    int top_k,
    double max_distance,
    int exclude_index = -1,
    int min_index_gap = 0) const;

  /// query() past the descriptor step, for callers that already hold one.
  std::vector<ScanContextCandidate> queryDescriptor(
    const std::vector<float>& descriptor,
    int top_k,
    double max_distance,
    int exclude_index = -1,
    int min_index_gap = 0,
    ScanContextDistanceMetric metric = ScanContextDistanceMetric::kMeanAbsoluteHeight,
    int prefilter_candidates = 0) const;

  /// Max-height descriptor of a lidar-frame cloud, row-major [ring][sector].
  std::vector<float> describe(const pcl::PointCloud<pcl::PointXYZI>& scan) const;

  /// Per-ring mean across sectors. Rotation invariant, so it survives the prefilter.
  std::vector<float> ringKey(const std::vector<float>& descriptor) const;

  /// Mean absolute difference between `query` and `target` shifted by `shift` sectors.
  double descriptorDistance(
    const std::vector<float>& query,
    const std::vector<float>& target,
    int shift) const;

  /// Standard Scan Context distance: one minus mean cosine similarity of valid sectors.
  double descriptorCosineDistance(
    const std::vector<float>& query,
    const std::vector<float>& target,
    int shift) const;

  /// Map-frame pose of an indexed keyframe, for scoring retrieval offline.
  const Eigen::Matrix4d& keyframePose(std::size_t slot) const { return poses_[slot]; }
  int keyframeIndex(std::size_t slot) const { return keyframe_indices_[slot]; }
  /// Stored descriptor of an indexed keyframe. Lets the offline checker run
  /// leave-one-out without re-reading and re-transforming every cloud.
  const std::vector<float>& keyframeDescriptor(std::size_t slot) const {
    return descriptors_[slot];
  }

  /// Resolve a keyframes.csv index to the compact database slot used by the accessors.
  bool findKeyframeSlot(int keyframe_index, std::size_t& slot) const;

  /**
   * @brief Load one stored world-frame keyframe and return it in its raw lidar frame.
   *
   * Geometry verification calls this only for Scan Context Top-K candidates. Keeping
   * paths and poses, instead of every cloud, bounds long-running localization memory.
   */
  bool loadKeyframeScan(
    std::size_t slot,
    pcl::PointCloud<pcl::PointXYZI>& scan,
    std::string* error = nullptr) const;

  /// Merge center +/- half_window scans into the center keyframe's lidar frame.
  bool loadKeyframeSubmap(
    std::size_t center_slot,
    int half_window,
    pcl::PointCloud<pcl::PointXYZI>& submap,
    std::string* error = nullptr) const;

private:
  struct KeyframeRow;

  static bool readIndexJson(const std::string& path, ScanContextParams& params);
  static bool readKeyframeCsv(const std::string& path, std::vector<KeyframeRow>& rows);

  ScanContextParams params_;
  std::string source_dir_;
  std::string seed_pose_source_ = "raw";
  std::vector<int> keyframe_indices_;
  std::vector<Eigen::Matrix4d> poses_;
  std::vector<std::string> keyframe_cloud_paths_;
  std::vector<Eigen::Matrix4d> raw_lidar_poses_;
  /// Flattened rings*sectors descriptor per keyframe.
  std::vector<std::vector<float>> descriptors_;
  std::vector<std::vector<float>> ring_keys_;
};

}  // namespace localization

#endif  // LOCALIZATION_SCAN_CONTEXT_DB_HPP
