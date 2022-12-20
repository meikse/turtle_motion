#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

using point = geometry_msgs::msg::Point;

class ControlTurtle : public rclcpp::Node {
  void mouse_callback(point &msg)
  public:
    ControlTurtle() : Node("turtle_controller") {
      mouse_subs_ = this->create_subscription<point>(
          "location/mouse", 10, std::bind(&ControlTurtle::mouse_callback, this)
          )
    }
}

void ControlTurtle::mouse_callback(point &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.x.c_str());
}


int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTurtle>());
  rclcpp::shutdown();
  return 0;
}
