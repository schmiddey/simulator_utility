#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"



class SimJoy2Vel_node : public rclcpp::Node {
public:
  SimJoy2Vel_node() : Node("sim_joy2vel_node")
  {
    this->declare_parameter<double>     ("max_vel_lin_x", _params.max_vel_lin_x);
    this->declare_parameter<double>     ("max_vel_lin_y", _params.max_vel_lin_y);
    this->declare_parameter<double>     ("max_vel_ang", _params.max_vel_ang);
    this->declare_parameter<std::string>("mode", _params.mode);


    _params.max_vel_lin_x = this->get_parameter("max_vel_lin_x").as_double();
    _params.max_vel_lin_y = this->get_parameter("max_vel_lin_y").as_double();
    _params.max_vel_ang   = this->get_parameter("max_vel_ang").as_double();
    _params.mode          = this->get_parameter("mode").as_string();

    this->setMode(_params.mode);
    

    _sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SimJoy2Vel_node::sub_joy_callback, this, std::placeholders::_1));

    _pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //callback for param
    _callback_handle = this->add_on_set_parameters_callback(std::bind(&SimJoy2Vel_node::param_callback, this, std::placeholders::_1));
  }

  ~SimJoy2Vel_node()
  {

  }
  
private: //functions
  void sub_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if(_fcn_create_twist)
    {
      _pub_vel->publish(_fcn_create_twist(msg));
    }
  }

  geometry_msgs::msg::Twist createTwistDiff(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto cmd = geometry_msgs::msg::Twist();

    cmd.linear.x   = (msg->axes[5] * -1 + msg->axes[2]) * 0.5 * _params.max_vel_lin_x;
    cmd.angular.z  = msg->axes[0] * _params.max_vel_ang;

    return cmd;
  }

  geometry_msgs::msg::Twist createTwistOmni(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto cmd = geometry_msgs::msg::Twist();

    cmd.linear.y   = msg->axes[3] * _params.max_vel_lin_y;
    cmd.linear.x   = msg->axes[4] * _params.max_vel_lin_x;
    cmd.angular.z  = msg->axes[0] * _params.max_vel_ang;

    return cmd;
  }


  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& param)
  {
    std::cout << "GOT PARAM CHANGE: " << param.size() << std::endl;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for(const auto& e : param)
    {
      if(e.get_name() == "max_vel_lin_x")
      {
        _params.max_vel_lin_x = e.as_double();
      }
      else if(e.get_name() == "max_vel_lin_y")
      {
        _params.max_vel_lin_y = e.as_double();
      }
      else if(e.get_name() == "max_vel_ang")
      {
        _params.max_vel_ang   = e.as_double();
      }
      else if(e.get_name() == "mode")
      {
        this->setMode(e.as_string());
      }
    }
    return result;
  }

  void setMode(const std::string& mode)
  {
    if(mode == "diff")
    {
      _fcn_create_twist = std::bind(&SimJoy2Vel_node::createTwistDiff, this, std::placeholders::_1);
    }
    else if(mode == "omni")
    {
      _fcn_create_twist = std::bind(&SimJoy2Vel_node::createTwistOmni, this, std::placeholders::_1);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invaild mode -> fallback mode diff is used");
      _fcn_create_twist = std::bind(&SimJoy2Vel_node::createTwistDiff, this, std::placeholders::_1);
    }
  }

private:

  //todo add dynamic stuff???
  struct SimJoy2Vel_params
  {
    double max_vel_lin_x = 1.0;
    double max_vel_lin_y = 1.0;
    double max_vel_ang = 1.0;
    std::string mode = "diff";
  } _params;
  
  std::function<geometry_msgs::msg::Twist(const sensor_msgs::msg::Joy::SharedPtr)> _fcn_create_twist;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  _sub_joy;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_vel;

  OnSetParametersCallbackHandle::SharedPtr _callback_handle;
};



int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimJoy2Vel_node>());
  rclcpp::shutdown();

  return 0;
}
