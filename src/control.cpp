#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using point = geometry_msgs::msg::Point;
using std::placeholders::_1;

class ControlTurtle : public rclcpp::Node
{
  rclcpp::Subscription<point>::SharedPtr mouse_subs_;
  void mouse_callback(const point & msg) const;
  void window_callback(const point & msg) const;
  public:
  ControlTurtle()
    : Node("turtle_controller")
  {
    mouse_subs_ = this->create_subscription<point>(
        "mouse/location",
        10, 
        std::bind(&ControlTurtle::mouse_callback, this, _1));
  }
};

void ControlTurtle::mouse_callback(const point & msg) const
{
  RCLCPP_INFO(this->get_logger(), "here.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTurtle>());
  rclcpp::shutdown();
  return 0;
}
