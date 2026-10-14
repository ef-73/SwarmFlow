// C++ port of swarmflow_viz/tf_relay.py (T054, X-01): merge per-robot TF trees into one prefixed global tree.
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using tf2_msgs::msg::TFMessage;
using sensor_msgs::msg::LaserScan;

namespace {

std::string prefixed(const std::string & in, const std::string & robot)
{
  size_t i = 0;
  while (i < in.size() && in[i] == '/') {++i;}
  std::string frame = in.substr(i);
  if (frame.empty() || frame == "map" || frame.rfind(robot + "/", 0) == 0) {return frame;}
  return robot + "/" + frame;
}

bool ends_with(const std::string & s, const std::string & suffix)
{
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

class TfRelay : public rclcpp::Node
{
public:
  TfRelay()
  : Node("swarmflow_tf_relay")
  {
    auto robots = declare_parameter<std::vector<std::string>>(
      "robots", std::vector<std::string>{"robot_1", "robot_2", "robot_3"});
    rclcpp::QoS tf_qos(100);
    tf_qos.reliable().durability_volatile();
    rclcpp::QoS static_pub_qos(1);
    static_pub_qos.reliable().transient_local();
    rclcpp::QoS static_sub_qos(100);
    static_sub_qos.reliable().transient_local();

    tf_pub_ = create_publisher<TFMessage>("/tf", tf_qos);
    static_pub_ = create_publisher<TFMessage>("/tf_static", static_pub_qos);
    for (const auto & r : robots) {
      if (r.empty()) {continue;}
      subs_.push_back(create_subscription<TFMessage>(
        "/" + r + "/tf", tf_qos, [this, r](TFMessage::UniquePtr m) {on_tf(r, std::move(m));}));
      subs_.push_back(create_subscription<TFMessage>(
        "/" + r + "/tf_static", static_sub_qos, [this, r](TFMessage::UniquePtr m) {on_static(r, *m);}));
      auto pub = create_publisher<LaserScan>("/viz/" + r + "/scan", rclcpp::SensorDataQoS());
      scan_pubs_[r] = pub;
      subs_.push_back(create_subscription<LaserScan>(
        "/" + r + "/scan", rclcpp::SensorDataQoS(),
        [this, r, pub](LaserScan::UniquePtr m) {on_scan(r, pub, std::move(m));}));
    }
  }

private:
  void on_tf(const std::string & robot, TFMessage::UniquePtr msg)
  {
    auto out = std::make_unique<TFMessage>();
    for (auto & t : msg->transforms) {
      if (ends_with(t.child_frame_id, "wheel_link")) {continue;}
      t.header.frame_id = prefixed(t.header.frame_id, robot);
      t.child_frame_id = prefixed(t.child_frame_id, robot);
      out->transforms.push_back(std::move(t));
    }
    if (!out->transforms.empty()) {tf_pub_->publish(std::move(out));}
  }

  void on_static(const std::string & robot, const TFMessage & msg)
  {
    for (auto t : msg.transforms) {
      t.header.frame_id = prefixed(t.header.frame_id, robot);
      t.child_frame_id = prefixed(t.child_frame_id, robot);
      static_[{t.header.frame_id, t.child_frame_id}] = t;
    }
    TFMessage out;
    for (const auto & kv : static_) {out.transforms.push_back(kv.second);}
    static_pub_->publish(out);
  }

  void on_scan(
    const std::string & robot, const rclcpp::Publisher<LaserScan>::SharedPtr & pub,
    LaserScan::UniquePtr msg)
  {
    msg->header.frame_id = prefixed(msg->header.frame_id, robot);
    for (auto & r : msg->ranges) {
      if (!std::isfinite(r)) {r = 0.0f;}
    }
    pub->publish(std::move(msg));
  }

  rclcpp::Publisher<TFMessage>::SharedPtr tf_pub_, static_pub_;
  std::map<std::string, rclcpp::Publisher<LaserScan>::SharedPtr> scan_pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::map<std::pair<std::string, std::string>, geometry_msgs::msg::TransformStamped> static_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfRelay>());
  rclcpp::shutdown();
  return 0;
}
