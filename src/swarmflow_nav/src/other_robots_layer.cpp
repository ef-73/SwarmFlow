// Costmap layer that stamps the other robots' footprints into a robot's costmaps (improvements_v2 N-05a).
//
// Why: the v2 robots carry their 360° lidar inside a horizontal sensor slot at 0.20 m (N-06). Every robot has that
// slot at the same height, so a scan plane passes through the other robots and only hits their four thin deck posts:
// robots are nearly invisible to each other's lidar (found in Gazebo, M14 session 3). Robots therefore read each
// other's reported state on /fleet/robot_states (pose, speed, turn rate; decided 2026-10-10, N-05) and this layer
// writes, for every other robot with a fresh state:
//   - its current padded footprint as LETHAL;
//   - its footprint at the predicted positions over `horizon_s` (constant speed and turn rate), with a cost that
//     falls from `predicted_cost` to half of it, so the planner and MPPI keep out of its way (stage a of N-05).
// States are in the map frame and are transformed into the costmap's global frame (odom for the local costmap).
// The orchestrator is not in this loop, so avoidance keeps working when it is down.

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarmflow_interfaces/msg/robot_state.hpp"
#include "tf2/exceptions.h"

namespace swarmflow_nav
{

class OtherRobotsLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("OtherRobotsLayer: node expired");}
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("length", rclcpp::ParameterValue(0.70));
    declareParameter("width", rclcpp::ParameterValue(0.60));
    declareParameter("carry_length", rclcpp::ParameterValue(1.25));
    declareParameter("carry_width", rclcpp::ParameterValue(1.05));
    declareParameter("horizon_s", rclcpp::ParameterValue(2.0));
    declareParameter("step_s", rclcpp::ParameterValue(0.4));
    declareParameter("max_age_s", rclcpp::ParameterValue(1.0));
    declareParameter("predicted_cost", rclcpp::ParameterValue(200));
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".length", length_);
    node->get_parameter(name_ + ".width", width_);
    node->get_parameter(name_ + ".carry_length", carry_length_);
    node->get_parameter(name_ + ".carry_width", carry_width_);
    node->get_parameter(name_ + ".horizon_s", horizon_s_);
    node->get_parameter(name_ + ".step_s", step_s_);
    node->get_parameter(name_ + ".max_age_s", max_age_s_);
    int pc = 200;
    node->get_parameter(name_ + ".predicted_cost", pc);
    predicted_cost_ = static_cast<unsigned char>(std::clamp(pc, 1, 252));
    // own id = first component of the costmap node's namespace ("/robot_1/global_costmap" -> "robot_1"); a
    // mismatch would stamp the robot onto itself ("Start occupied", M14 session 6)
    self_ = node->get_namespace();
    self_.erase(0, self_.find_first_not_of('/'));
    self_ = self_.substr(0, self_.find('/'));
    declareParameter("robot_id", rclcpp::ParameterValue(self_));
    node->get_parameter(name_ + ".robot_id", self_);
    RCLCPP_INFO(node->get_logger(), "OtherRobotsLayer: own robot id '%s'", self_.c_str());
    clock_ = node->get_clock();
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    sub_ = node->create_subscription<swarmflow_interfaces::msg::RobotState>(
      "/fleet/robot_states", rclcpp::QoS(50).best_effort(),
      [this](swarmflow_interfaces::msg::RobotState::ConstSharedPtr m) {onState(*m);}, opts);
    current_ = true;
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {return;}
    shapes_.clear();
    collect();
    // include last cycle's shapes so cells the robots left are recomputed (this layer writes into the master only)
    for (const auto & b : {last_bounds_, boundsOf(shapes_)}) {
      if (b[0] > b[2]) {continue;}
      *min_x = std::min(*min_x, b[0]);
      *min_y = std::min(*min_y, b[1]);
      *max_x = std::max(*max_x, b[2]);
      *max_y = std::max(*max_y, b[3]);
    }
    last_bounds_ = boundsOf(shapes_);
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_) {return;}
    for (const auto & s : shapes_) {stamp(master, s, min_i, min_j, max_i, max_j);}
  }

  void reset() override {last_bounds_ = {1, 1, 0, 0};}
  bool isClearable() override {return false;}

private:
  struct Other
  {
    double x, y, yaw, v, w, t;  // map frame, stamp in seconds (node clock = sim time)
    bool carrying;
  };
  struct Shape
  {
    double x, y, yaw, half_l, half_w;
    unsigned char cost;
  };

  void onState(const swarmflow_interfaces::msg::RobotState & m)
  {
    if (m.robot_id == self_) {return;}
    std::lock_guard<std::mutex> lk(mutex_);
    others_[m.robot_id] = Other{m.x, m.y, m.yaw, m.linear_speed, m.angular_speed,
      rclcpp::Time(m.header.stamp, RCL_ROS_TIME).seconds(),
      m.mode == swarmflow_interfaces::msg::RobotState::MODE_CARRYING ||
      m.mode == swarmflow_interfaces::msg::RobotState::MODE_LIFTING};
  }

  void collect()
  {
    const double now = clock_->now().seconds();
    std::vector<Other> fresh;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      for (const auto & kv : others_) {
        if (now - kv.second.t <= max_age_s_) {fresh.push_back(kv.second);}
      }
    }
    if (fresh.empty()) {return;}
    // map -> costmap global frame (identity with ground-truth localization, map -> odom with AMCL)
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
    const double c = std::cos(tyaw), s = std::sin(tyaw);
    for (const auto & o : fresh) {
      const double hl = 0.5 * (o.carrying ? carry_length_ : length_);
      const double hw = 0.5 * (o.carrying ? carry_width_ : width_);
      double x = o.x, y = o.y, yaw = o.yaw;
      const int n = step_s_ > 0.0 ? static_cast<int>(horizon_s_ / step_s_) : 0;
      for (int k = 0; k <= n; ++k) {
        if (k > 0) {  // constant speed and turn rate
          x += o.v * std::cos(yaw) * step_s_;
          y += o.v * std::sin(yaw) * step_s_;
          yaw += o.w * step_s_;
          if (std::abs(o.v) < 0.02 && std::abs(o.w) < 0.05) {break;}  // standing still: current footprint is enough
        }
        const unsigned char cost = k == 0 ? nav2_costmap_2d::LETHAL_OBSTACLE :
          static_cast<unsigned char>(predicted_cost_ - (predicted_cost_ / 2) * k / std::max(n, 1));
        shapes_.push_back(Shape{tx + c * x - s * y, ty + s * x + c * y, yaw + tyaw, hl, hw, cost});
      }
    }
  }

  static std::array<double, 4> boundsOf(const std::vector<Shape> & shapes)
  {
    std::array<double, 4> b{1, 1, 0, 0};
    bool first = true;
    for (const auto & sh : shapes) {
      const double r = std::hypot(sh.half_l, sh.half_w);
      if (first) {b = {sh.x - r, sh.y - r, sh.x + r, sh.y + r}; first = false; continue;}
      b[0] = std::min(b[0], sh.x - r);
      b[1] = std::min(b[1], sh.y - r);
      b[2] = std::max(b[2], sh.x + r);
      b[3] = std::max(b[3], sh.y + r);
    }
    return b;
  }

  static void stamp(nav2_costmap_2d::Costmap2D & m, const Shape & sh, int min_i, int min_j, int max_i, int max_j)
  {
    const double r = std::hypot(sh.half_l, sh.half_w);
    int i0, j0, i1, j1;
    m.worldToMapEnforceBounds(sh.x - r, sh.y - r, i0, j0);
    m.worldToMapEnforceBounds(sh.x + r, sh.y + r, i1, j1);
    i0 = std::max(i0, min_i); j0 = std::max(j0, min_j);
    i1 = std::min(i1, max_i - 1); j1 = std::min(j1, max_j - 1);
    const double c = std::cos(sh.yaw), s = std::sin(sh.yaw);
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        double wx, wy;
        m.mapToWorld(i, j, wx, wy);
        const double dx = wx - sh.x, dy = wy - sh.y;
        if (std::abs(c * dx + s * dy) <= sh.half_l && std::abs(-s * dx + c * dy) <= sh.half_w) {
          if (m.getCost(i, j) == nav2_costmap_2d::NO_INFORMATION || m.getCost(i, j) < sh.cost) {
            m.setCost(i, j, sh.cost);
          }
        }
      }
    }
  }

  bool enabled_{true};
  double length_{0.70}, width_{0.60}, carry_length_{1.25}, carry_width_{1.05};
  double horizon_s_{2.0}, step_s_{0.4}, max_age_s_{1.0};
  unsigned char predicted_cost_{200};
  std::string self_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<swarmflow_interfaces::msg::RobotState>::SharedPtr sub_;
  std::mutex mutex_;
  std::unordered_map<std::string, Other> others_;
  std::vector<Shape> shapes_;
  std::array<double, 4> last_bounds_{1, 1, 0, 0};
};

}  // namespace swarmflow_nav

PLUGINLIB_EXPORT_CLASS(swarmflow_nav::OtherRobotsLayer, nav2_costmap_2d::Layer)
