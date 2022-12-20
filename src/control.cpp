/* #include <functional> */
/* #include <memory> */

/* #include <rclcpp/rclcpp.hpp> */
/* #include <geometry_msgs/msg/point.hpp> */

/* #include "std_msgs/msg/string.hpp" */

/* using point = geometry_msgs::msg::Point; */
/* using mytype = std_msgs::msg::String; */
/* using std::placeholders::_1; */

/* class ControlTurtle : public rclcpp::Node */ 
/* { */
/*   public: */
/*     ControlTurtle() : */ 
/*     Node("turtle_controller") */
/*     { */
/*       mouse_subs_ = this->create_subscription<mytype>( */
/*       "location/mouse", 10, std::bind(&ControlTurtle::mouse_callback, this, _1)); */
/*     } */
/*   private: */
/*     void mouse_callback(mytype & msg) const { */
/*       RCLCPP_INFO(this->get_logger(), "here!"); */
/*       /1* RCLCPP_INFO(this->get_logger(), "I heard: %lf", msg.x); *1/ */
/*     }; */
/*     rclcpp::Subscription<mytype>::SharedPtr mouse_subs_; */
/* }; */


/* int main(int argc, char** argv){ */
/* { */
/*   rclcpp::init(argc, argv); */
/*   rclcpp::spin(std::make_shared<ControlTurtle>()); */
/*   rclcpp::shutdown(); */
/*   return 0; */
/* } */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using point = geometry_msgs::msg::Point;
using std::placeholders::_1;

class ControlTurtle : public rclcpp::Node
{
public:
  ControlTurtle()
  : Node("turtle_controller")
  {
    mouse_subs_ = this->create_subscription<point>(
      "location/mouse", 10, std::bind(&ControlTurtle::topic_callback, this, _1));
  }

private:
  void topic_callback(const point & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "here.");
  }
  rclcpp::Subscription<point>::SharedPtr mouse_subs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTurtle>());
  rclcpp::shutdown();
  return 0;
}
