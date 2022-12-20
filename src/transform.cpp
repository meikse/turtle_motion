#include <functional>
#include <memory>
#include <chrono>
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

  rclcpp::Publisher<point>::SharedPtr center_pub_;
  rclcpp::Publisher<point>::SharedPtr distance_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> mouse = {0., 0.};     // import to pass value
  std::vector<double> dimension = {0., 0.}; // if not, segmentation error
  std::vector<double> window = {0., 0.};    // rclcpp needs time to init

  void mouse_callback(const point & msg){
    this->mouse = {msg.x, msg.y};
    /* RCLCPP_INFO(this->get_logger(), "mouse: %.6f", this->mouse[0]); */
  }

  void window_loc_callback(const point & msg){
    this->window= {msg.x, msg.y};
    /* RCLCPP_INFO(this->get_logger(), "window: %.6f", this->window[0]); */
  }

  void window_dim_callback(const point & msg){
    this->dimension= {msg.x, msg.y};
    /* RCLCPP_INFO(this->get_logger(), "dimension: %.6f", this->dimension[0]); */
  }

  void timer_callback(){

    auto center = point();  // origin of turtle coord system (center of window)
    center.x = this->window[0] + this->dimension[0] * .5;
    center.y = this->window[1] + this->dimension[1] * .5;
    center.z = 0.;
    RCLCPP_INFO(this->get_logger(), "center: %.3f, %.3f", center.x, center.y);
    center_pub_->publish(center);

    auto distance = point(); // rel distance between mouse and turtle origin
    distance.x = this->mouse[0] - center.x;
    distance.y = (this->mouse[1] - center.y) * -1; // *-1 =>rotate origin frame
    distance.z = 0.;                               // image frame -> real frame
    RCLCPP_INFO(this->get_logger(),
        "distan: %.3f, %.3f", distance.x, distance.y);
    distance_pub_->publish(distance);
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

    distance_pub_ = this->create_publisher<point>(
        "mouse/distance", 
        10);

    center_pub_ = this->create_publisher<point>(
        "window/center", 
        10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Transform::timer_callback, this));
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transform>());
  rclcpp::shutdown();
  return 0;
}
