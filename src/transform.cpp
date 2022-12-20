#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using point = geometry_msgs::msg::Point;
using std::placeholders::_1;

class Transform : public rclcpp::Node
{
  rclcpp::Subscription<point>::SharedPtr mouse_subs_;
  rclcpp::Subscription<point>::SharedPtr wind_loc_subs_;
  rclcpp::Subscription<point>::SharedPtr wind_dim_subs_;
  std::vector<double> mouse, window, dimension;
  void mouse_callback(const point & msg){
    this->mouse = {msg.x, msg.y};
    RCLCPP_INFO(this->get_logger(), "mouse: %.6f", this->mouse[0]);
  }
  void window_loc_callback(const point & msg){
    this->window= {msg.x, msg.y};
    RCLCPP_INFO(this->get_logger(), "window: %.6f", this->window[0]);
  }
  void window_dim_callback(const point & msg){
    this->dimension= {msg.x, msg.y};
    RCLCPP_INFO(this->get_logger(), "dimension: %.6f", this->dimension[0]);
  }
  public:
  Transform() : Node("transformation_node")
  {
    mouse_subs_ = this->create_subscription<point>(
        "mouse/location",
        10, 
        std::bind(&Transform::mouse_callback, this, _1));

    wind_loc_subs_ = this->create_subscription<point>(
        "window/location",
        10, 
        std::bind(&Transform::window_loc_callback, this, _1));

    wind_dim_subs_ = this->create_subscription<point>(
        "window/dimension",
        10, 
        std::bind(&Transform::window_dim_callback, this, _1));
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transform>());
  rclcpp::shutdown();
  return 0;
}
