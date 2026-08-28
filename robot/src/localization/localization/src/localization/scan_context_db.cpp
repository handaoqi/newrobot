#include "localization/scan_context_db.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>

namespace localization {

namespace {

constexpr float kUnfilled = -1e9f;
constexpr float kUnfilledTest = -1e8f;

double wrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

/// Column lookup that mirrors _load_keyframes()'s fallbacks in map_loop_closure.py.
class CsvRow {
public:
  CsvRow(const std::unordered_map<std::string, std::size_t>& header,
         const std::vector<std::string>& fields)
  : header_(header), fields_(fields) {}

  bool has(const std::string& name) const {
    const auto it = header_.find(name);
    return it != header_.end() && it->second < fields_.size() && !fields_[it->second].empty();
  }

  double number(const std::string& name, double fallback = 0.0) const {
    if (!has(name)) {
      return fallback;
    }
    try {
      return std::stod(fields_.at(header_.at(name)));
    } catch (const std::exception&) {
      return fallback;
    }
  }

private:
  const std::unordered_map<std::string, std::size_t>& header_;
  const std::vector<std::string>& fields_;
};

/// Pull one numeric field out of a flat JSON object without taking a JSON dependency.
bool jsonNumber(const std::string& text, const std::string& key, double& value) {
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = text.find(needle);
  if (key_pos == std::string::npos) {
    return false;
  }
  const auto colon = text.find(':', key_pos + needle.size());
  if (colon == std::string::npos) {
    return false;
  }
  try {
    value = std::stod(text.substr(colon + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

}  // namespace

struct ScanContextDatabase::KeyframeRow {
  int index = 0;
  Eigen::Vector3d world_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond world_rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d lidar_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond lidar_rotation = Eigen::Quaterniond::Identity();
};

void ScanContextDatabase::clear() {
  source_dir_.clear();
  keyframe_indices_.clear();
  poses_.clear();
  descriptors_.clear();
  ring_keys_.clear();
}

bool ScanContextDatabase::readIndexJson(const std::string& path, ScanContextParams& params) {
  std::ifstream stream(path);
  if (!stream) {
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(stream)),
                         std::istreambuf_iterator<char>());
  double value = 0.0;
  if (jsonNumber(text, "rings", value) && value >= 1.0) {
    params.rings = static_cast<int>(value);
  }
  if (jsonNumber(text, "sectors", value) && value >= 1.0) {
    params.sectors = static_cast<int>(value);
  }
  if (jsonNumber(text, "max_radius_m", value) && value > 0.0) {
    params.max_radius_m = value;
  }
  return true;
}

bool ScanContextDatabase::readKeyframeCsv(const std::string& path, std::vector<KeyframeRow>& rows) {
  std::ifstream stream(path);
  if (!stream) {
    return false;
  }
  std::string line;
  if (!std::getline(stream, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::unordered_map<std::string, std::size_t> header;
  {
    const auto names = splitCsv(line);
    for (std::size_t i = 0; i < names.size(); ++i) {
      header[names[i]] = i;
    }
  }
  if (header.find("index") == header.end()) {
    return false;
  }

  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsv(line);
    const CsvRow row(header, fields);
    if (!row.has("index")) {
      continue;
    }

    KeyframeRow keyframe;
    keyframe.index = static_cast<int>(row.number("index", -1.0));
    if (keyframe.index < 0) {
      continue;
    }

    // Older map schemas only carry the bare x/y/z and a yaw; the newer ones split the
    // body pose from the lidar pose. Fall back the same way the Python loader does so
    // a map recorded before the split still indexes.
    const double yaw = row.number("yaw", 0.0);
    const double lidar_x = row.has("lidar_x") ? row.number("lidar_x") : row.number("x");
    const double lidar_y = row.has("lidar_y") ? row.number("lidar_y") : row.number("y");
    const double lidar_z = row.has("lidar_z") ? row.number("lidar_z") : row.number("z");
    const double qx = row.number("lidar_qx", 0.0);
    const double qy = row.number("lidar_qy", 0.0);
    const double qz = row.has("lidar_qz") ? row.number("lidar_qz") : std::sin(yaw / 2.0);
    const double qw = row.has("lidar_qw") ? row.number("lidar_qw") : std::cos(yaw / 2.0);

    keyframe.lidar_translation = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);
    keyframe.lidar_rotation = Eigen::Quaterniond(qw, qx, qy, qz);
    keyframe.world_translation = Eigen::Vector3d(
      row.has("world_x") ? row.number("world_x") : lidar_x,
      row.has("world_y") ? row.number("world_y") : lidar_y,
      row.has("world_z") ? row.number("world_z") : lidar_z);
    keyframe.world_rotation = Eigen::Quaterniond(
      row.has("world_qw") ? row.number("world_qw") : qw,
      row.has("world_qx") ? row.number("world_qx") : qx,
      row.has("world_qy") ? row.number("world_qy") : qy,
      row.has("world_qz") ? row.number("world_qz") : qz);

    if (keyframe.lidar_rotation.norm() < 1e-9 || keyframe.world_rotation.norm() < 1e-9) {
      continue;
    }
    keyframe.lidar_rotation.normalize();
    keyframe.world_rotation.normalize();
    rows.push_back(keyframe);
  }
  return true;
}

bool ScanContextDatabase::load(const std::string& map_dir, std::string* error) {
  clear();
  const auto fail = [&error](const std::string& reason) {
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  };

  params_ = ScanContextParams{};
  readIndexJson(map_dir + "/scan_context/index.json", params_);
  if (params_.rings < 1 || params_.sectors < 1 || params_.max_radius_m <= 0.0) {
    return fail("invalid scan context geometry in scan_context/index.json");
  }

  std::vector<KeyframeRow> rows;
  if (!readKeyframeCsv(map_dir + "/keyframes/keyframes.csv", rows)) {
    return fail("keyframes/keyframes.csv is missing or has no index column");
  }
  if (rows.empty()) {
    return fail("keyframes/keyframes.csv contains no usable rows");
  }

  descriptors_.reserve(rows.size());
  ring_keys_.reserve(rows.size());
  poses_.reserve(rows.size());
  keyframe_indices_.reserve(rows.size());

  pcl::PointCloud<pcl::PointXYZI> world_cloud;
  pcl::PointCloud<pcl::PointXYZI> lidar_cloud;
  for (const KeyframeRow& row : rows) {
    char name[64];
    std::snprintf(name, sizeof(name), "/keyframes/scan_%05d.pcd", row.index);
    world_cloud.clear();
    if (pcl::io::loadPCDFile(map_dir + name, world_cloud) < 0 || world_cloud.empty()) {
      continue;
    }

    // Keyframe clouds are stored in the world frame; the descriptor is only
    // comparable to a live scan once the cloud is back in its own lidar frame.
    const Eigen::Matrix3d rotation = row.lidar_rotation.toRotationMatrix().transpose();
    lidar_cloud.clear();
    lidar_cloud.reserve(world_cloud.size());
    for (const auto& point : world_cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Eigen::Vector3d local = rotation *
        (Eigen::Vector3d(point.x, point.y, point.z) - row.lidar_translation);
      pcl::PointXYZI transformed;
      transformed.x = static_cast<float>(local.x());
      transformed.y = static_cast<float>(local.y());
      transformed.z = static_cast<float>(local.z());
      transformed.intensity = point.intensity;
      lidar_cloud.push_back(transformed);
    }
    if (lidar_cloud.empty()) {
      continue;
    }

    std::vector<float> descriptor = describe(lidar_cloud);
    ring_keys_.push_back(ringKey(descriptor));
    descriptors_.push_back(std::move(descriptor));

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = row.world_rotation.toRotationMatrix();
    pose.block<3, 1>(0, 3) = row.world_translation;
    poses_.push_back(pose);
    keyframe_indices_.push_back(row.index);
  }

  if (descriptors_.empty()) {
    return fail("no keyframe cloud under keyframes/ could be read");
  }
  source_dir_ = map_dir;
  return true;
}

std::vector<float> ScanContextDatabase::describe(
    const pcl::PointCloud<pcl::PointXYZI>& scan) const {
  const int rings = params_.rings;
  const int sectors = params_.sectors;
  std::vector<float> descriptor(static_cast<std::size_t>(rings) * sectors, kUnfilled);

  for (const auto& point : scan) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double radius = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (radius <= 1e-6 || radius > params_.max_radius_m) {
      continue;
    }
    const int ring = std::min(rings - 1,
      static_cast<int>(radius / params_.max_radius_m * rings));
    const double yaw = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
    int sector = static_cast<int>((yaw + M_PI) / (2.0 * M_PI) * sectors) % sectors;
    if (sector < 0) {
      sector += sectors;
    }
    float& cell = descriptor[static_cast<std::size_t>(ring) * sectors + sector];
    if (point.z > cell) {
      cell = point.z;
    }
  }

  // Empty cells become 0 rather than staying at the sentinel, so an unobserved
  // bin reads as ground level instead of dominating every distance it appears in.
  for (float& cell : descriptor) {
    if (cell < kUnfilledTest) {
      cell = 0.0f;
    }
  }
  return descriptor;
}

std::vector<float> ScanContextDatabase::ringKey(const std::vector<float>& descriptor) const {
  const int rings = params_.rings;
  const int sectors = params_.sectors;
  std::vector<float> key(static_cast<std::size_t>(rings), 0.0f);
  for (int ring = 0; ring < rings; ++ring) {
    double total = 0.0;
    for (int sector = 0; sector < sectors; ++sector) {
      total += descriptor[static_cast<std::size_t>(ring) * sectors + sector];
    }
    key[ring] = static_cast<float>(total / std::max(1, sectors));
  }
  return key;
}

double ScanContextDatabase::descriptorDistance(
    const std::vector<float>& query,
    const std::vector<float>& target,
    int shift) const {
  const int rings = params_.rings;
  const int sectors = params_.sectors;
  double total = 0.0;
  for (int ring = 0; ring < rings; ++ring) {
    const std::size_t base = static_cast<std::size_t>(ring) * sectors;
    for (int sector = 0; sector < sectors; ++sector) {
      const int shifted = (sector + shift) % sectors;
      total += std::abs(query[base + sector] - target[base + shifted]);
    }
  }
  return total / std::max(1, rings * sectors);
}

std::vector<ScanContextCandidate> ScanContextDatabase::query(
    const pcl::PointCloud<pcl::PointXYZI>& scan,
    int top_k,
    double max_distance,
    int exclude_index,
    int min_index_gap) const {
  if (descriptors_.empty() || top_k <= 0) {
    return {};
  }
  return queryDescriptor(describe(scan), top_k, max_distance, exclude_index, min_index_gap);
}

std::vector<ScanContextCandidate> ScanContextDatabase::queryDescriptor(
    const std::vector<float>& query_descriptor,
    int top_k,
    double max_distance,
    int exclude_index,
    int min_index_gap) const {
  std::vector<ScanContextCandidate> results;
  if (descriptors_.empty() || top_k <= 0 ||
      query_descriptor.size() != static_cast<std::size_t>(params_.cells())) {
    return results;
  }

  const std::vector<float> query_key = ringKey(query_descriptor);

  // Ring keys average across sectors, so they survive rotation. Ranking on them first
  // keeps the per-shift comparison off the whole database.
  std::vector<std::pair<double, std::size_t>> ranked;
  ranked.reserve(descriptors_.size());
  for (std::size_t slot = 0; slot < descriptors_.size(); ++slot) {
    const int index = keyframe_indices_[slot];
    if (exclude_index >= 0 && std::abs(index - exclude_index) < std::max(1, min_index_gap)) {
      continue;
    }
    double total = 0.0;
    for (std::size_t ring = 0; ring < query_key.size(); ++ring) {
      total += std::abs(query_key[ring] - ring_keys_[slot][ring]);
    }
    ranked.emplace_back(total / std::max<std::size_t>(1, query_key.size()), slot);
  }
  if (ranked.empty()) {
    return results;
  }

  const std::size_t shortlist = std::min(ranked.size(), static_cast<std::size_t>(top_k) * 3);
  std::partial_sort(ranked.begin(), ranked.begin() + shortlist, ranked.end());

  const int sectors = params_.sectors;
  std::vector<ScanContextCandidate> scored;
  scored.reserve(shortlist);
  for (std::size_t i = 0; i < shortlist; ++i) {
    const std::size_t slot = ranked[i].second;
    double best_distance = std::numeric_limits<double>::max();
    int best_shift = 0;
    for (int shift = 0; shift < sectors; ++shift) {
      const double distance = descriptorDistance(query_descriptor, descriptors_[slot], shift);
      if (distance < best_distance) {
        best_distance = distance;
        best_shift = shift;
      }
    }

    ScanContextCandidate candidate;
    candidate.keyframe_index = keyframe_indices_[slot];
    candidate.distance = best_distance;
    candidate.yaw_offset_rad = wrapAngle(best_shift * (2.0 * M_PI / sectors));

    // Sign settled empirically over 8577 co-located keyframe pairs across 21 recorded
    // maps: yaw_query = yaw_match + yaw_offset holds to 3.66 deg median, which is the
    // half-sector quantisation floor. The opposite sign lands 37 deg out.
    const Eigen::Matrix4d& keyframe_pose = poses_[slot];
    candidate.seed_pose = Eigen::Matrix4d::Identity();
    candidate.seed_pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(candidate.yaw_offset_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      keyframe_pose.block<3, 3>(0, 0);
    candidate.seed_pose.block<3, 1>(0, 3) = keyframe_pose.block<3, 1>(0, 3);
    scored.push_back(candidate);
  }

  std::sort(scored.begin(), scored.end(),
    [](const ScanContextCandidate& lhs, const ScanContextCandidate& rhs) {
      return lhs.distance < rhs.distance;
    });

  for (const ScanContextCandidate& candidate : scored) {
    if (static_cast<int>(results.size()) >= top_k) {
      break;
    }
    if (max_distance > 0.0 && candidate.distance > max_distance) {
      break;
    }
    results.push_back(candidate);
  }
  return results;
}

}  // namespace localization
