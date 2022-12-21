#include <functional>
#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using point = geometry_msgs::msg::Point;
using twist = geometry_msgs::msg::Twist;
using pose = turtlesim::msg::Pose;

using std::placeholders::_1;

class ControlTurtle : public rclcpp::Node
{
  rclcpp::Subscription<pose>::SharedPtr turtle_subs_;
  rclcpp::Subscription<point>::SharedPtr mouse_subs_, window_subs_;
  rclcpp::Publisher<twist>::SharedPtr turtle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  pose feedback = pose();
  point setpoint = point();
  point dimension = point();
  void timer_callback();
  void get_pose_callback(const pose &msg)
  {
    this->feedback = msg;
  }
  void get_size_callback(const point &msg){
    this->dimension = msg;
  }
  void get_distance_callback(const point &msg)
  {
    float ratio = 11.0888 / this->dimension.y;      // measured (turtle domain)
    this->setpoint.x = msg.x * ratio;
    this->setpoint.y = msg.y * ratio;
    /* RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f ", */
    /*     this->setpoint.x, this->setpoint.y); */
  }
  public:
  ControlTurtle() : Node("turtle_controller")
  {
    turtle_subs_ = this->create_subscription<pose>(
        "turtle1/pose",
        10, 
        std::bind(&ControlTurtle::get_pose_callback, this, _1));

    window_subs_ = this->create_subscription<point>(
        "window/dimension",
        10, 
        std::bind(&ControlTurtle::get_size_callback, this, _1));

    mouse_subs_ = this->create_subscription<point>(
        "mouse/distance",
        10, 
        std::bind(&ControlTurtle::get_distance_callback, this, _1));

    turtle_pub_ = this->create_publisher<twist>(
        "turtle1/cmd_vel",
        10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ControlTurtle::timer_callback, this));
  }
};

void ControlTurtle::timer_callback()
{
  // placeholder
  auto goal = twist();
  // error (turtle domain)
  float x = setpoint.x - feedback.x;
  float y = setpoint.y - feedback.y;
  auto delta = atan2(y,x) - feedback.theta;
  // controller (plant only single integrators)
  float P = .1;
  float vector = sqrt(x*x + y*y);
  RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f ",delta,vector);
  // control values
  goal.linear.x = vector * P;
  goal.angular.z = delta;
  turtle_pub_->publish(goal);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTurtle>());
  rclcpp::shutdown();
  return 0;
}
