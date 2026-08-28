/**
 * @brief Offline accuracy check for ScanContextDatabase. No ROS, no robot, no map service.
 *
 * For every map directory given on the command line the tool rebuilds the database from
 * keyframes/, then runs leave-one-out retrieval: each keyframe becomes a query against a
 * database with only that keyframe removed. Because the true pose of every query is known
 * from keyframes.csv, this scores exactly what relocalization depends on:
 *
 *   - hit rate  - how often the returned seed is within --radius of the truth. If this is
 *                 low the whole approach does not work and nothing downstream can save it.
 *   - yaw error - the returned seed heading against the true heading. This is the check
 *                 that the yaw sign convention is right; a systematic sign flip shows up
 *                 as a bimodal error pinned near the true heading's negation, not as noise.
 *   - distance  - the descriptor distance separating correct from incorrect retrievals,
 *                 which is what max_descriptor_distance has to be set from.
 *
 * Usage: localization_scan_context_check [options] <map_dir> [<map_dir> ...]
 *   --top-k N        candidates to consider     (default 5)
 *   --radius M       hit threshold in metres    (default 2.0)
 *   --max-distance D descriptor cutoff, 0 = off (default 0, so the full curve is visible)
 *   --min-gap N      hide keyframes within N indices of the query (default 1, self only)
 *   --quiet          per-map summary only
 *
 * --min-gap is the difficulty dial. At 1 the query's own temporal neighbours are still in
 * the database, so a hit only proves the descriptor and shift search are self-consistent.
 * Raising it forces retrieval to match a genuine revisit from a different pass, which is
 * the situation a relocalizing robot is actually in.
 */

#include "localization/scan_context_db.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

double yawOf(const Eigen::Matrix4d& pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

double wrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto slot = static_cast<std::size_t>(fraction * (values.size() - 1));
  return values[std::min(slot, values.size() - 1)];
}

struct Totals {
  int queries = 0;
  /// Queries for which the database, after exclusions, still holds at least one keyframe
  /// within --radius of the truth. Everything else is unanswerable by construction and
  /// scoring it as a retrieval failure would understate the method on single-pass maps.
  int answerable = 0;
  int top1_hits = 0;
  int topk_hits = 0;
  std::vector<double> top1_position_error;
  std::vector<double> hit_yaw_error_deg;
  std::vector<double> hit_distance;
  std::vector<double> miss_distance;
};

void report(const std::string& label, const Totals& totals) {
  if (totals.answerable == 0) {
    std::cout << label << ": " << totals.queries << " queries, none answerable\n";
    return;
  }
  const double scale = 100.0 / totals.answerable;
  std::cout << std::fixed << std::setprecision(1)
            << label << ": " << totals.answerable << "/" << totals.queries << " answerable"
            << "  top1 " << totals.top1_hits * scale << "%"
            << "  topk " << totals.topk_hits * scale << "%"
            << std::setprecision(2)
            << "  top1_err_med " << median(totals.top1_position_error) << " m"
            << "  yaw_err_med " << median(totals.hit_yaw_error_deg) << " deg"
            << "  yaw_err_p90 " << percentile(totals.hit_yaw_error_deg, 0.9) << " deg"
            << "  dist_hit_p90 " << percentile(totals.hit_distance, 0.9)
            << "  dist_miss_med " << median(totals.miss_distance)
            << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  int top_k = 5;
  double radius = 2.0;
  double max_distance = 0.0;
  int min_gap = 1;
  bool quiet = false;
  std::vector<std::string> map_dirs;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](double fallback) {
      return (i + 1 < argc) ? std::atof(argv[++i]) : fallback;
    };
    if (arg == "--top-k") {
      top_k = static_cast<int>(next(top_k));
    } else if (arg == "--radius") {
      radius = next(radius);
    } else if (arg == "--max-distance") {
      max_distance = next(max_distance);
    } else if (arg == "--min-gap") {
      min_gap = static_cast<int>(next(min_gap));
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
    std::cerr << "usage: localization_scan_context_check [options] <map_dir> [<map_dir> ...]\n";
    return 2;
  }

  Totals overall;
  int loaded_maps = 0;
  for (const std::string& map_dir : map_dirs) {
    localization::ScanContextDatabase database;
    std::string error;
    if (!database.load(map_dir, &error)) {
      if (!quiet) {
        std::cout << map_dir << ": skipped (" << error << ")\n";
      }
      continue;
    }
    ++loaded_maps;
    if (!quiet) {
      std::cout << map_dir << ": seed_pose_source=" << database.seed_pose_source() << "\n";
    }

    Totals totals;
    for (std::size_t slot = 0; slot < database.size(); ++slot) {
      const Eigen::Matrix4d& truth = database.keyframePose(slot);
      const int query_index = database.keyframeIndex(slot);
      const auto candidates = database.queryDescriptor(
        database.keyframeDescriptor(slot), top_k, max_distance, query_index, min_gap);

      ++totals.queries;

      bool answerable = false;
      for (std::size_t other = 0; other < database.size() && !answerable; ++other) {
        if (std::abs(database.keyframeIndex(other) - query_index) < std::max(1, min_gap)) {
          continue;
        }
        answerable = (database.keyframePose(other).block<3, 1>(0, 3) -
                      truth.block<3, 1>(0, 3)).norm() <= radius;
      }
      if (!answerable) {
        continue;
      }
      ++totals.answerable;

      if (candidates.empty()) {
        continue;
      }

      const double top1_error =
        (candidates.front().seed_pose.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm();
      totals.top1_position_error.push_back(top1_error);
      if (top1_error <= radius) {
        ++totals.top1_hits;
        totals.hit_distance.push_back(candidates.front().distance);
        totals.hit_yaw_error_deg.push_back(std::abs(
          wrapAngle(yawOf(candidates.front().seed_pose) - yawOf(truth))) * 180.0 / M_PI);
      } else {
        totals.miss_distance.push_back(candidates.front().distance);
      }

      for (const auto& candidate : candidates) {
        if ((candidate.seed_pose.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm() <= radius) {
          ++totals.topk_hits;
          break;
        }
      }
    }

    if (!quiet) {
      report(map_dir, totals);
    }

    overall.queries += totals.queries;
    overall.answerable += totals.answerable;
    overall.top1_hits += totals.top1_hits;
    overall.topk_hits += totals.topk_hits;
    const auto extend = [](std::vector<double>& into, const std::vector<double>& from) {
      into.insert(into.end(), from.begin(), from.end());
    };
    extend(overall.top1_position_error, totals.top1_position_error);
    extend(overall.hit_yaw_error_deg, totals.hit_yaw_error_deg);
    extend(overall.hit_distance, totals.hit_distance);
    extend(overall.miss_distance, totals.miss_distance);
  }

  std::cout << "\nloaded " << loaded_maps << " of " << map_dirs.size() << " map directories\n";
  report("TOTAL", overall);
  return overall.queries > 0 ? 0 : 1;
}
