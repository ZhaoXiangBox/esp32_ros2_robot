#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class JoystickPublisher : public rclcpp::Node
{
  public:
    JoystickPublisher()
    : Node("joystick_publisher")
    {
      Twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      Servo1_publisher_ = this->create_publisher<std_msgs::msg::Int8>("servo1",10);
      Servo2_publisher_ = this->create_publisher<std_msgs::msg::Int8>("servo2",10);

      // LED Control State Publisher
      Front_light_state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("Front_light_state",10);
      Back_light_state_publisher_  = this->create_publisher<std_msgs::msg::Int8>("Back_light_state",10);
      Blink_light_state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("Blink_light_state",10);
      Close_light_state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("Close_light_state",10);

      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),std::bind(&JoystickPublisher::joyCallback, this, std::placeholders::_1) );
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Servo1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Servo2_publisher_;

    // Light state control
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Front_light_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Back_light_state_publisher_ ;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Blink_light_state_publisher_ ;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr Close_light_state_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    bool light_state_flag_ = true;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        /*********************     joy_msg      *****************************
        axes[i]: 0   1   2   3   4   5   6   7
            0：角速度 [-1.0,1.0]；topic name : cmd_vel
            1：线速度 [-1.0,1.0]；topic name : cmd_vel 
            2：空闲；
            3：舵机一的参考角度 [-90,90]；topic name : servo1
            4：舵机二的参考角度 [-90,90]；topic name : servo2
            5：空闲；
        **************************************************************/
        auto servo1_msg = std_msgs::msg::Int8();
        servo1_msg.data = (int)(90*joy_msg->axes[3]);
        Servo1_publisher_->publish(servo1_msg);

        auto servo2_msg = std_msgs::msg::Int8();
        servo2_msg.data = (int)(90.0*joy_msg->axes[4] );
        Servo2_publisher_->publish(servo2_msg);

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = joy_msg->axes[1] *0.4;
        twist_msg.angular.z = joy_msg->axes[0];
        
        if( fabs( twist_msg.linear.x ) > 0.005)
            Twist_publisher_->publish(twist_msg);

        /*********************     joy_msg      *****************************
        buttons[i]: 0   1   2   3   4   5   6   7   8   9   10  11  12  13
            0：关闭所有灯光；
            1：打开/关闭 前灯；
            2：打开/关闭 双闪；
            3：打开/关闭 后灯；
        ****************************************************************/
       auto front_light_msg = std_msgs::msg::Int8();
       auto back_light_msg = std_msgs::msg::Int8();
       auto blink_light_msg = std_msgs::msg::Int8();
       auto close_light_msg = std_msgs::msg::Int8();
       
        if(joy_msg->buttons[0]> 0)
        {
            close_light_msg.data = 1;
            Close_light_state_publisher_->publish(close_light_msg);
        }

        if(joy_msg->buttons[1]> 0)
        {
            front_light_msg.data = 1;
            Front_light_state_publisher_->publish(front_light_msg);
        }
        
        if(joy_msg->buttons[2]> 0)
        {
            blink_light_msg.data = 1;
            Blink_light_state_publisher_->publish(blink_light_msg);
        }
        
        if(joy_msg->buttons[3]> 0)
        {
            back_light_msg.data = 1;
            Back_light_state_publisher_->publish(back_light_msg);
        }

    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickPublisher>());
  rclcpp::shutdown();
  return 0;
}