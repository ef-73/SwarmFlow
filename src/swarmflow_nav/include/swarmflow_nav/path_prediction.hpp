// Prediction of another robot along its published plan (improvements_v2 §15, J-04). Header-only so it is unit-tested
// without ROS (test/test_path_prediction.cpp).
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace swarmflow_nav
{

struct Pose2
{
  double x, y, yaw;
};

// Poses at arc lengths v*step_s*k (k = 1..n) along the polyline `path`, measured from the point of the path nearest to
// (x, y); yaw = direction of the segment. Stops at the path's end (the robot stops at its goal). Empty when the path
// has < 2 points, the robot is more than `max_offset` m from it, or v <= 0 (reversing or standing: caller falls back).
inline std::vector<Pose2> posesAlongPath(
  const std::vector<std::pair<double, double>> & path, double x, double y, double v, double step_s, int n,
  double max_offset)
{
  std::vector<Pose2> out;
  if (path.size() < 2 || v <= 0.0 || step_s <= 0.0 || n <= 0) {return out;}
  // nearest point: projection onto every segment
  double best = std::numeric_limits<double>::infinity();
  size_t seg = 0;
  double frac = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i].first, ay = path[i].second;
    const double dx = path[i + 1].first - ax, dy = path[i + 1].second - ay;
    const double len2 = dx * dx + dy * dy;
    const double u = len2 > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) : 0.0;
    const double d = std::hypot(ax + u * dx - x, ay + u * dy - y);
    if (d < best) {best = d; seg = i; frac = u;}
  }
  if (best > max_offset) {return out;}
  // walk along the path from (seg, frac)
  double yaw = std::atan2(path[seg + 1].second - path[seg].second, path[seg + 1].first - path[seg].first);
  double left_in_seg = (1.0 - frac) * std::hypot(path[seg + 1].first - path[seg].first,
      path[seg + 1].second - path[seg].second);
  double walked = 0.0;
  for (int k = 1; k <= n; ++k) {
    double want = v * step_s * k - walked;
    while (want > left_in_seg && seg + 2 < path.size()) {
      want -= left_in_seg;
      walked += left_in_seg;
      ++seg;
      left_in_seg = std::hypot(path[seg + 1].first - path[seg].first, path[seg + 1].second - path[seg].second);
      if (left_in_seg > 1e-9) {
        yaw = std::atan2(path[seg + 1].second - path[seg].second, path[seg + 1].first - path[seg].first);
      }
    }
    if (want > left_in_seg + 1e-9) {  // past the last segment: the goal (landing on a corner is not the end)
      out.push_back(Pose2{path.back().first, path.back().second, yaw});
      break;
    }
    walked += want;
    left_in_seg -= want;
    const double seg_len = std::hypot(path[seg + 1].first - path[seg].first, path[seg + 1].second - path[seg].second);
    const double t = seg_len > 0.0 ? 1.0 - left_in_seg / seg_len : 1.0;
    out.push_back(Pose2{path[seg].first + t * (path[seg + 1].first - path[seg].first),
        path[seg].second + t * (path[seg + 1].second - path[seg].second), yaw});
  }
  return out;
}

}  // namespace swarmflow_nav
