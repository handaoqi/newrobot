/**
 * Offline leave-one-out Scan Context + real FastGICP/ICP geometry validation.
 *
 * This tool never starts ROS and never publishes a pose. It uses recorded keyframes as
 * both queries and candidates, hides temporal neighbours with --min-gap, and answers:
 * does the first candidate accepted by geometry land at the known query pose?
 */

#include "localization/relocalization_geometry.hpp"
#include "localization/scan_context_db.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

double wrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawOf(const Eigen::Matrix4d& pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t slot = static_cast<std::size_t>(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(values.size() - 1));
  return values[slot];
}

struct MetricSamples {
  std::vector<double> rmse;
  std::vector<double> overlap;
  std::vector<double> hessian_condition;
  std::vector<double> icp_translation;
  std::vector<double> icp_rotation_deg;
};

struct Totals {
  int queries = 0;
  int answerable = 0;
  int retrieval_topk_hits = 0;
  int accepted_queries = 0;
  int accepted_correct = 0;
  int accepted_wrong = 0;
  int candidates_checked = 0;
  int accepted_correct_pairs = 0;
  int accepted_wrong_pairs = 0;
  std::map<std::string, int> rejections;
  std::map<std::string, int> correct_rejections;
  std::map<std::string, int> wrong_rejections;
  MetricSamples correct_pairs;
  MetricSamples wrong_pairs;
  std::vector<double> accepted_position_error;
  std::vector<double> accepted_yaw_error_deg;
  std::vector<double> candidate_elapsed_ms;
};

void appendFinite(std::vector<double>& values, double value) {
  if (std::isfinite(value)) {
    values.push_back(value);
  }
}

void recordMetrics(MetricSamples& samples,
                   const localization::RelocalizationGeometryResult& result) {
  appendFinite(samples.rmse, result.rmse_m);
  appendFinite(samples.overlap, result.bidirectional_overlap);
  appendFinite(samples.hessian_condition, result.hessian_condition);
  appendFinite(samples.icp_translation, result.icp_translation_disagreement_m);
  appendFinite(samples.icp_rotation_deg,
    result.icp_rotation_disagreement_rad * 180.0 / M_PI);
}

void mergeSamples(MetricSamples& into, const MetricSamples& from) {
  const auto append = [](std::vector<double>& lhs, const std::vector<double>& rhs) {
    lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  };
  append(into.rmse, from.rmse);
  append(into.overlap, from.overlap);
  append(into.hessian_condition, from.hessian_condition);
  append(into.icp_translation, from.icp_translation);
  append(into.icp_rotation_deg, from.icp_rotation_deg);
}

void mergeTotals(Totals& into, const Totals& from) {
  into.queries += from.queries;
  into.answerable += from.answerable;
  into.retrieval_topk_hits += from.retrieval_topk_hits;
  into.accepted_queries += from.accepted_queries;
  into.accepted_correct += from.accepted_correct;
  into.accepted_wrong += from.accepted_wrong;
  into.candidates_checked += from.candidates_checked;
  into.accepted_correct_pairs += from.accepted_correct_pairs;
  into.accepted_wrong_pairs += from.accepted_wrong_pairs;
  for (const auto& rejection : from.rejections) {
    into.rejections[rejection.first] += rejection.second;
  }
  for (const auto& rejection : from.correct_rejections) {
    into.correct_rejections[rejection.first] += rejection.second;
  }
  for (const auto& rejection : from.wrong_rejections) {
    into.wrong_rejections[rejection.first] += rejection.second;
  }
  mergeSamples(into.correct_pairs, from.correct_pairs);
  mergeSamples(into.wrong_pairs, from.wrong_pairs);
  into.accepted_position_error.insert(into.accepted_position_error.end(),
    from.accepted_position_error.begin(), from.accepted_position_error.end());
  into.accepted_yaw_error_deg.insert(into.accepted_yaw_error_deg.end(),
    from.accepted_yaw_error_deg.begin(), from.accepted_yaw_error_deg.end());
  into.candidate_elapsed_ms.insert(into.candidate_elapsed_ms.end(),
    from.candidate_elapsed_ms.begin(), from.candidate_elapsed_ms.end());
}

void reportSamples(const char* label, const MetricSamples& samples) {
  std::cout << "  " << label
            << " rmse_med/p90=" << percentile(samples.rmse, 0.5) << "/"
            << percentile(samples.rmse, 0.9) << "m"
            << " overlap_p10/med=" << percentile(samples.overlap, 0.1) << "/"
            << percentile(samples.overlap, 0.5)
            << " hessian_p90=" << percentile(samples.hessian_condition, 0.9)
            << " icp_delta_med/p90=" << percentile(samples.icp_translation, 0.5) << "/"
            << percentile(samples.icp_translation, 0.9) << "m,"
            << percentile(samples.icp_rotation_deg, 0.5) << "/"
            << percentile(samples.icp_rotation_deg, 0.9) << "deg\n";
}

void report(const std::string& label, const Totals& totals) {
  const double scale = totals.answerable > 0 ? 100.0 / totals.answerable : 0.0;
  std::cout << std::fixed << std::setprecision(1)
            << label << ": " << totals.answerable << "/" << totals.queries << " answerable"
            << " retrieval_topk=" << totals.retrieval_topk_hits * scale << "%"
            << " geometry_accept=" << totals.accepted_queries * scale << "%"
            << " correct=" << totals.accepted_correct * scale << "%"
            << " FALSE_ACCEPT=" << totals.accepted_wrong
            << " no_accept=" << (totals.answerable - totals.accepted_queries)
            << " pairs=" << totals.candidates_checked
            << " pair_accept(correct/wrong)=" << totals.accepted_correct_pairs << "/"
            << totals.accepted_wrong_pairs
            << std::setprecision(2)
            << " pos_err_med/p90=" << percentile(totals.accepted_position_error, 0.5) << "/"
            << percentile(totals.accepted_position_error, 0.9) << "m"
            << " yaw_err_med/p90=" << percentile(totals.accepted_yaw_error_deg, 0.5) << "/"
            << percentile(totals.accepted_yaw_error_deg, 0.9) << "deg"
            << " time_med/p90=" << percentile(totals.candidate_elapsed_ms, 0.5) << "/"
            << percentile(totals.candidate_elapsed_ms, 0.9) << "ms\n";
  reportSamples("correct candidates", totals.correct_pairs);
  reportSamples("wrong candidates  ", totals.wrong_pairs);
  std::cout << "  rejections:";
  for (const auto& rejection : totals.rejections) {
    std::cout << " " << rejection.first << "=" << rejection.second;
  }
  std::cout << "\n";
  std::cout << "  correct candidate rejections:";
  for (const auto& rejection : totals.correct_rejections) {
    std::cout << " " << rejection.first << "=" << rejection.second;
  }
  std::cout << "\n  wrong candidate rejections:";
  for (const auto& rejection : totals.wrong_rejections) {
    std::cout << " " << rejection.first << "=" << rejection.second;
  }
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  int top_k = 5;
  int min_gap = 20;
  double radius_m = 2.0;
  double max_descriptor_distance = 0.0;
  bool quiet = false;
  localization::ScanContextDistanceMetric scan_context_metric =
    localization::ScanContextDistanceMetric::kMeanAbsoluteHeight;
  int prefilter_candidates = 0;
  int query_half_window = 0;
  int target_half_window = 0;
  int yaw_neighbors = 0;
  localization::RelocalizationGeometryConfig geometry_config;
  std::vector<std::string> map_dirs;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](double fallback) {
      return i + 1 < argc ? std::atof(argv[++i]) : fallback;
    };
    if (arg == "--top-k") {
      top_k = static_cast<int>(next(top_k));
    } else if (arg == "--min-gap") {
      min_gap = static_cast<int>(next(min_gap));
    } else if (arg == "--radius") {
      radius_m = next(radius_m);
    } else if (arg == "--max-distance") {
      max_descriptor_distance = next(max_descriptor_distance);
    } else if (arg == "--metric") {
      if (i + 1 >= argc) {
        std::cerr << "--metric requires abs, cosine or hybrid\n";
        return 2;
      }
      const std::string name = argv[++i];
      if (name == "cosine") {
        scan_context_metric = localization::ScanContextDistanceMetric::kSectorCosine;
      } else if (name == "hybrid") {
        scan_context_metric = localization::ScanContextDistanceMetric::kSectorCosineAbsoluteYaw;
      } else if (name == "abs") {
        scan_context_metric = localization::ScanContextDistanceMetric::kMeanAbsoluteHeight;
      } else {
        std::cerr << "unknown metric " << name << "\n";
        return 2;
      }
    } else if (arg == "--prefilter") {
      prefilter_candidates = static_cast<int>(next(prefilter_candidates));
    } else if (arg == "--query-window") {
      query_half_window = std::max(0, static_cast<int>(next(query_half_window)));
    } else if (arg == "--target-window") {
      target_half_window = std::max(0, static_cast<int>(next(target_half_window)));
    } else if (arg == "--yaw-neighbors") {
      yaw_neighbors = std::max(0, static_cast<int>(next(yaw_neighbors)));
    } else if (arg == "--voxel") {
      geometry_config.voxel_size_m = static_cast<float>(next(geometry_config.voxel_size_m));
    } else if (arg == "--max-correspondence") {
      geometry_config.max_correspondence_distance_m = static_cast<float>(
        next(geometry_config.max_correspondence_distance_m));
    } else if (arg == "--max-iterations") {
      geometry_config.max_iterations = static_cast<int>(next(geometry_config.max_iterations));
    } else if (arg == "--min-inliers") {
      geometry_config.min_inliers = static_cast<int>(next(geometry_config.min_inliers));
    } else if (arg == "--min-overlap") {
      geometry_config.min_bidirectional_overlap = next(
        geometry_config.min_bidirectional_overlap);
    } else if (arg == "--max-rmse") {
      geometry_config.max_rmse_m = next(geometry_config.max_rmse_m);
    } else if (arg == "--max-icp-translation") {
      geometry_config.max_icp_translation_disagreement_m = next(
        geometry_config.max_icp_translation_disagreement_m);
    } else if (arg == "--max-icp-rotation-deg") {
      geometry_config.max_icp_rotation_disagreement_rad =
        next(geometry_config.max_icp_rotation_disagreement_rad * 180.0 / M_PI) * M_PI / 180.0;
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "unknown option " << arg << "\n";
      return 2;
    } else {
      map_dirs.push_back(arg);
    }
  }

  if (map_dirs.empty()) {
    std::cerr << "usage: localization_relocalization_geometry_check [options] <map_dir> ...\n";
    return 2;
  }

  Totals overall;
  int loaded_maps = 0;
  for (const auto& map_dir : map_dirs) {
    localization::ScanContextDatabase database;
    std::string error;
    if (!database.load(map_dir, &error)) {
      std::cerr << map_dir << ": skipped (" << error << ")\n";
      continue;
    }
    ++loaded_maps;
    localization::RelocalizationGeometryVerifier verifier(geometry_config);
    Totals totals;

    for (std::size_t query_slot = 0; query_slot < database.size(); ++query_slot) {
      ++totals.queries;
      const int query_index = database.keyframeIndex(query_slot);
      const Eigen::Matrix4d truth = database.keyframePose(query_slot);

      bool answerable = false;
      for (std::size_t other = 0; other < database.size(); ++other) {
        if (std::abs(database.keyframeIndex(other) - query_index) < std::max(1, min_gap)) {
          continue;
        }
        if ((database.keyframePose(other).block<3, 1>(0, 3) -
             truth.block<3, 1>(0, 3)).norm() <= radius_m) {
          answerable = true;
          break;
        }
      }
      if (!answerable) {
        continue;
      }
      ++totals.answerable;

      pcl::PointCloud<pcl::PointXYZI> query_scan;
      if (!database.loadKeyframeSubmap(
          query_slot, query_half_window, query_scan, &error)) {
        ++totals.rejections["query_load_failed"];
        continue;
      }
      const auto candidates = database.queryDescriptor(
        database.keyframeDescriptor(query_slot), top_k, max_descriptor_distance,
        query_index, min_gap, scan_context_metric, prefilter_candidates);

      bool retrieval_hit = false;
      bool selected = false;
      for (const auto& candidate : candidates) {
        std::size_t candidate_slot = 0;
        if (!database.findKeyframeSlot(candidate.keyframe_index, candidate_slot)) {
          ++totals.rejections["candidate_not_found"];
          continue;
        }
        const bool retrieval_correct =
          (database.keyframePose(candidate_slot).block<3, 1>(0, 3) -
           truth.block<3, 1>(0, 3)).norm() <= radius_m;
        retrieval_hit = retrieval_hit || retrieval_correct;

        pcl::PointCloud<pcl::PointXYZI> candidate_scan;
        if (!database.loadKeyframeSubmap(
            candidate_slot, target_half_window, candidate_scan, &error)) {
          ++totals.rejections["candidate_load_failed"];
          continue;
        }

        const Eigen::Matrix4d map_to_candidate = database.keyframePose(candidate_slot).inverse();
        localization::RelocalizationGeometryResult result;
        double total_geometry_ms = 0.0;
        double best_rejected_rmse = std::numeric_limits<double>::infinity();
        for (int yaw_attempt = 0; yaw_attempt <= yaw_neighbors * 2; ++yaw_attempt) {
          const int signed_step = yaw_attempt == 0 ? 0 :
            ((yaw_attempt + 1) / 2) * (yaw_attempt % 2 == 1 ? 1 : -1);
          Eigen::Matrix4d adjusted_seed = candidate.seed_pose;
          const double yaw_delta = signed_step * (2.0 * M_PI / database.params().sectors);
          adjusted_seed.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            adjusted_seed.block<3, 3>(0, 0);
          const Eigen::Matrix4f initial = (map_to_candidate * adjusted_seed).cast<float>();
          auto attempt_result = verifier.verify(query_scan, candidate_scan, initial);
          total_geometry_ms += attempt_result.elapsed_ms;
          if (attempt_result.accepted) {
            result = std::move(attempt_result);
            break;
          }
          if (attempt_result.rmse_m < best_rejected_rmse) {
            best_rejected_rmse = attempt_result.rmse_m;
            result = std::move(attempt_result);
          }
        }
        result.elapsed_ms = total_geometry_ms;
        ++totals.candidates_checked;
        totals.candidate_elapsed_ms.push_back(result.elapsed_ms);
        recordMetrics(retrieval_correct ? totals.correct_pairs : totals.wrong_pairs, result);
        if (!result.accepted) {
          ++totals.rejections[result.rejection_reason];
          ++(retrieval_correct ? totals.correct_rejections : totals.wrong_rejections)
            [result.rejection_reason];
          continue;
        }
        if (retrieval_correct) {
          ++totals.accepted_correct_pairs;
        } else {
          ++totals.accepted_wrong_pairs;
        }

        // Runtime policy selects the first descriptor-ranked candidate that survives
        // geometry. Keep evaluating later candidates for diagnostics, but do not replace it.
        if (!selected) {
          selected = true;
          ++totals.accepted_queries;
          const Eigen::Matrix4d final_map_pose = database.keyframePose(candidate_slot) *
            result.source_to_target.cast<double>();
          const double position_error =
            (final_map_pose.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm();
          const double yaw_error_deg = std::abs(
            wrapAngle(yawOf(final_map_pose) - yawOf(truth))) * 180.0 / M_PI;
          totals.accepted_position_error.push_back(position_error);
          totals.accepted_yaw_error_deg.push_back(yaw_error_deg);
          if (position_error <= radius_m && yaw_error_deg <= 20.0) {
            ++totals.accepted_correct;
          } else {
            ++totals.accepted_wrong;
            if (!quiet) {
              std::cout << "FALSE_ACCEPT query=" << query_index
                        << " candidate=" << candidate.keyframe_index
                        << " position_error=" << position_error
                        << " yaw_error_deg=" << yaw_error_deg
                        << " rmse=" << result.rmse_m
                        << " overlap=" << result.bidirectional_overlap << "\n";
            }
          }
        }
      }
      if (retrieval_hit) {
        ++totals.retrieval_topk_hits;
      }
    }

    if (!quiet) {
      report(map_dir, totals);
    }
    mergeTotals(overall, totals);
  }

  std::cout << "\nloaded " << loaded_maps << " of " << map_dirs.size()
            << " map directories\n";
  report("TOTAL", overall);
  // A false geometry acceptance is always an unsafe result. Coverage shortfall is
  // reported but left to rollout policy because single-pass maps can be unanswerable.
  return overall.queries > 0 && overall.accepted_wrong == 0 ? 0 : 1;
}
