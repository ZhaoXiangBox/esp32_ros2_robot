#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

typedef struct
{
    int pit_servo;
    int yaw_servo;

    bool Front_light_state;
    bool Back_light_state;
    bool Blink_light_state;
    bool Close_light_state;
}ESP32_STATE;
ESP32_STATE esp32_state;

class JoystickPublisher : public rclcpp::Node
{
  public:
    JoystickPublisher()
    : Node("joystick_publisher")
    {
        Twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        Esp32_state_publisher_ = this->create_publisher<std_msgs::msg::String>("esp32_state",10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),std::bind(&JoystickPublisher::joyCallback, this, std::placeholders::_1) ); 

        publish_en_count = 0;
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Esp32_state_publisher_;  
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    int publish_en_count;

    void Reset_esp32_state( ESP32_STATE &t)
    {
        t.pit_servo = 0;
        t.yaw_servo = 0;

        t.Front_light_state = false;
        t.Back_light_state  = false;
        t.Blink_light_state = false;
        t.Close_light_state = false;

        publish_en_count = 0;
    }

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
        if( fabs(joy_msg->axes[3] *0.4) > 0.01)
        {
            esp32_state.pit_servo = (int)(90*joy_msg->axes[3]);
            publish_en_count ++;
        }
        if( fabs(joy_msg->axes[4] *0.4) > 0.01 )
        {
            esp32_state.yaw_servo = (int)(90.0*joy_msg->axes[4] );
            publish_en_count ++;
        }

        // publish cmd_vel topic.
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = joy_msg->axes[1] *0.3;
        twist_msg.angular.z = joy_msg->axes[0]*0.5;
        
        if( fabs(joy_msg->axes[1] *0.4) > 0.001)
        {
            Twist_publisher_->publish(twist_msg);
            // std::cout<<" > min: "<<"linear x: "<< twist_msg.linear.x<<" "<<"angular z: "<<twist_msg.angular.z <<std::endl;
        }


        /*********************     joy_msg      *****************************
        buttons[i]: 0   1   2   3   4   5   6   7   8   9   10  11  12  13
            0：关闭所有灯光；
            1：打开/关闭 前灯；
            2：打开/关闭 双闪；
            3：打开/关闭 后灯；
        ****************************************************************/
        if(joy_msg->buttons[0]> 0)
        {
           esp32_state.Close_light_state = true;
            publish_en_count ++;
        }

        if(joy_msg->buttons[1]> 0)
        {
            esp32_state.Front_light_state = true;
            publish_en_count ++;
        }
        
        if(joy_msg->buttons[2]> 0)
        {
            esp32_state.Blink_light_state = true;
            publish_en_count ++;
        }
        
        if(joy_msg->buttons[3]> 0)
        {
            esp32_state.Back_light_state = true;
            publish_en_count ++;
        }
        auto esp32_state_msg =  std_msgs::msg::String();
        esp32_state_msg.data = std::to_string(esp32_state.pit_servo) + " " + std::to_string(esp32_state.yaw_servo) + " " +
                                             std::to_string(esp32_state.Front_light_state) + " " +  std::to_string(esp32_state.Back_light_state)  + " "+
                                             std::to_string(esp32_state.Blink_light_state) +  " " +  std::to_string(esp32_state.Close_light_state);

        Esp32_state_publisher_->publish(esp32_state_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", esp32_state_msg.data.c_str());

        // if( publish_en_count !=0 )
        // {
        // Esp32_state_publisher_->publish(esp32_state_msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", esp32_state_msg.data.c_str());
        // }
        Reset_esp32_state(esp32_state);
    }

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickPublisher>());
  rclcpp::shutdown();
  return 0;
}