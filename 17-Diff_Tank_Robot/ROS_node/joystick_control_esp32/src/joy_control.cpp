#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

typedef struct
{
    int pit_servo = 0;
    int yaw_servo = 0;

    bool Front_light_state = false;
    bool Back_light_state = false;
    bool Blink_light_state = false;
    bool Close_light_state = false;
}ESP32_STATE;
ESP32_STATE esp32_state;

class JoystickPublisher : public rclcpp::Node
{
  public:
    JoystickPublisher():Node("joystick_publisher")
    {
        Twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        Esp32_state_publisher_ = this->create_publisher<std_msgs::msg::String>("esp32_state",10);
        timer_ = this->create_wall_timer(500ms, std::bind(&JoystickPublisher::timer_callback, this));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),std::bind(&JoystickPublisher::joyCallback, this, std::placeholders::_1) ); 
        
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Esp32_state_publisher_;  
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        auto esp32_state_msg =  std_msgs::msg::String();
        esp32_state_msg.data = std::to_string(esp32_state.pit_servo) + " " + std::to_string(esp32_state.yaw_servo) + " " +
                                         std::to_string(esp32_state.Front_light_state) + " " +  std::to_string(esp32_state.Back_light_state)  + " "+
                                         std::to_string(esp32_state.Blink_light_state) +  " " +  std::to_string(esp32_state.Close_light_state);
        Esp32_state_publisher_->publish(esp32_state_msg);
        RCLCPP_INFO(this->get_logger(), "esp32_state: '%s'", esp32_state_msg.data.c_str());
        Reset_esp32_state(esp32_state);
    }

    void Reset_esp32_state( ESP32_STATE &t)
    {
        t.pit_servo = 0;
        t.yaw_servo = 0;

        t.Close_light_state = false;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        /*********************     joy_msg      *****************************
        axes[i]: 0   1   2   3   4   5   6   7
            0：角速度 [-1.0,1.0]；topic name : cmd_vel
            1：线速度 [-1.0,1.0]；topic name : cmd_vel 
            2：小车手柄速度控制开关，按下手柄左侧 L1 按键，控制生效；
            3：舵机一的参考角度 [-90,90]；topic name : servo1
            4：舵机二的参考角度 [-90,90]；topic name : servo2
            5：小车手柄状态控制开关，按下手柄左侧 R1 按键，控制生效；
        **************************************************************/
        if( fabs(joy_msg->axes[3] *0.4) > 0.01)
        {
            esp32_state.pit_servo = (int)(90*joy_msg->axes[3]);
        }
        if( fabs(joy_msg->axes[4] *0.4) > 0.01 )
        {
            esp32_state.yaw_servo = (int)(90.0*joy_msg->axes[4] );
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        if( joy_msg->axes[2] < 1.0)   // 手柄左侧 L2 按下才能触发 车辆速度控制
        {
            twist_msg.linear.x =  (joy_msg->axes[1] *0.3) ;
            twist_msg.angular.z = joy_msg->axes[0];
            Twist_publisher_->publish(twist_msg);
            RCLCPP_INFO(this->get_logger(), "Twist: '%lf'  , '%lf' " ,twist_msg.linear.x,  twist_msg.angular.z );
        }
        else
        {   
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            Twist_publisher_->publish(twist_msg);
        }
        
        /*********************     joy_msg      *****************************
        buttons[i]: 0   1   2   3   4   5   6   7   8   9   10  11  12  13
            0：关闭所有灯光；
            1：打开/关闭 前灯；
            2：打开/关闭 双闪；
            3：打开/关闭 后灯；
        ****************************************************************/
        if(joy_msg->buttons[0]> 0){
           esp32_state.Close_light_state = true;
            esp32_state.Front_light_state = false;
            esp32_state.Blink_light_state = false;
            esp32_state.Back_light_state = false;
        }

        if(joy_msg->buttons[1]> 0){
            esp32_state.Front_light_state = true;
        }
        
        if(joy_msg->buttons[2]> 0){
            esp32_state.Blink_light_state = true;
        }
        
        if(joy_msg->buttons[3]> 0){
            esp32_state.Back_light_state = true;
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