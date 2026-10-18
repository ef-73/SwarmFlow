// Keep-right costmap layer (N-09, decided 2026-10-10).
//
// Robots drive on the right-hand side of the central aisle and the storage aisles. Inside every aisle rectangle from
// the layout (`aisles` in zones.yaml, passed in by robot.launch.py as a flat parameter list), the half on the robot's
// LEFT relative to its direction of travel gets a moderate, non-lethal cost: the planner and MPPI prefer the right
// half but can still use the left one to get around an obstacle. The direction of travel along an aisle is taken
// from the robot's heading; while it is crossing an aisle (heading mostly across the axis) the aisle is not costed.
// Costs are only raised, never lowered, so walls, racks and robots stay lethal.

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"

namespace swarmflow_nav
{

class KeepRightLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("KeepRightLayer: node expired");}
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("cost", rclcpp::ParameterValue(70));
    declareParameter("min_heading_cos", rclcpp::ParameterValue(0.5));
    // x_min, y_min, x_max, y_max, axis (0 = x, 1 = y) per aisle, map frame (identity to odom with ground truth)
    declareParameter("aisles", rclcpp::ParameterValue(std::vector<double>{}));
    node->get_parameter(name_ + ".enabled", enabled_);
    int cost = 70;
    node->get_parameter(name_ + ".cost", cost);
    cost_ = static_cast<unsigned char>(std::clamp(cost, 1, 252));
    node->get_parameter(name_ + ".min_heading_cos", min_cos_);
    std::vector<double> flat;
    node->get_parameter(name_ + ".aisles", flat);
    for (size_t i = 0; i + 4 < flat.size(); i += 5) {
      aisles_.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4]});
    }
    RCLCPP_INFO(node->get_logger(), "KeepRightLayer: %zu aisles, cost %d", aisles_.size(), cost);
    current_ = true;
  }

  void updateBounds(
    double, double, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_ || aisles_.empty()) {return;}
    yaw_ = robot_yaw;
    for (const auto & a : aisles_) {  // the costed halves flip with the heading: always recompute every aisle
      *min_x = std::min(*min_x, a[0] - 1.0);   // 1 m margin: bounds are in the map frame, costmap may be odom
      *min_y = std::min(*min_y, a[1] - 1.0);
      *max_x = std::max(*max_x, a[2] + 1.0);
      *max_y = std::max(*max_y, a[3] + 1.0);
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & m, int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_) {return;}
    // aisles are in the map frame; the local costmap works in odom, which drifts from map under AMCL (realism ladder)
    double tx = 0.0, ty = 0.0, tyaw = 0.0;
    const std::string gf = layered_costmap_->getGlobalFrameID();
    if (gf != "map") {
      try {
        auto tf = tf_->lookupTransform(gf, "map", tf2::TimePointZero);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        const auto & q = tf.transform.rotation;
        tyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      } catch (const tf2::TransformException &) {
        return;
      }
    }
    const double tc = std::cos(tyaw), ts = std::sin(tyaw);
    // heading in the map frame decides the lane
    const double c = std::cos(yaw_ - tyaw), s = std::sin(yaw_ - tyaw);
    for (const auto & a : aisles_) {
      const bool along_x = a[4] < 0.5;
      const double d = along_x ? c : s;              // heading component along the aisle axis
      if (std::abs(d) < min_cos_) {continue;}        // crossing the aisle: no lane rule
      double x0 = a[0], y0 = a[1], x1 = a[2], y1 = a[3];
      if (along_x) {           // travelling +x: left is +y (north half); travelling -x: left is the south half
        const double mid = 0.5 * (y0 + y1);
        if (d > 0) {y0 = mid;} else {y1 = mid;}
      } else {                 // travelling +y: left is -x (west half); travelling -y: left is the east half
        const double mid = 0.5 * (x0 + x1);
        if (d > 0) {x1 = mid;} else {x0 = mid;}
      }
      // map -> costmap frame: axis-aligned bounds of the transformed half (the map/odom rotation is small)
      double bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9;
      for (const auto & p : {std::array<double, 2>{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}}) {
        const double wx = tx + tc * p[0] - ts * p[1], wy = ty + ts * p[0] + tc * p[1];
        bx0 = std::min(bx0, wx); by0 = std::min(by0, wy); bx1 = std::max(bx1, wx); by1 = std::max(by1, wy);
      }
      int i0, j0, i1, j1;
      m.worldToMapEnforceBounds(bx0, by0, i0, j0);
      m.worldToMapEnforceBounds(bx1, by1, i1, j1);
      i0 = std::max(i0, min_i); j0 = std::max(j0, min_j);
      i1 = std::min(i1, max_i - 1); j1 = std::min(j1, max_j - 1);
      for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
          const unsigned char cur = m.getCost(i, j);
          if (cur == nav2_costmap_2d::NO_INFORMATION || cur < cost_) {m.setCost(i, j, cost_);}
        }
      }
    }
  }

  void reset() override {}
  bool isClearable() override {return false;}

private:
  bool enabled_{true};
  unsigned char cost_{70};
  double min_cos_{0.5};
  double yaw_{0.0};
  std::vector<std::array<double, 5>> aisles_;
};

}  // namespace swarmflow_nav

PLUGINLIB_EXPORT_CLASS(swarmflow_nav::KeepRightLayer, nav2_costmap_2d::Layer)
