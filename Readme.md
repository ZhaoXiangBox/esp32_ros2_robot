![Setup_environment_01](pics/Setup_environment_01.png)

整个项目的目标：构建基于ESP32-WROOM-32开发的ROS机器人

​	1、设计一款 ESP32-WROOM-32 的扩展板，具备以下功能：

​		· 带 两路霍尔编码器的直流减速电机接口

​		· 带 3路舵机 PWM 控制接口

​		· 带 1路HC-SR04 超声波接口

​		· 带 MPU6050 模块

​		· 带 GPS 模块

​		· 带 6路 LED 控制接口

​		· 带 5V/3A 的负载输出

​		· 带 A4950T 模组接口

​		· 电量显示模组接口

​	2、ESP32 和 该扩展板将实现如下功能

​	2、基于micro ros 与ROS2 主机进行通信

​		· 订阅 速度话题 cmd_vel 

​		· 发布 IMU 、GPS 、Ultra 话题数据

​		· 适配两轮差速小车、差速履带小车、基于舵机转向的阿克曼小车

​		· 可通过Wifi、Serial的方式控制上

​		· 后续还需要开发 App （展望）

​	3、设计对应的硬件结构

​		· 差速小车

​		· 差速履带小车

​		· 基于舵机转向的阿克曼小车



**PS**. 边学边更新，我之前的项目中有详细的 ROS1 学习教程，及ROS2 foxy 的仿真，链接如下：

**[neor_mini](https://github.com/COONEO/neor_mini)**

```
https://github.com/COONEO/neor_mini
```



------

## Chapter 1 install_Arduino_ROS2_ESP32

![Setup_environment_02](pics/Setup_environment_02.png)

Videos from Bilibili 照祥同学: [第一节：搭建ESP32和Arduino的ROS2开发环境](https://www.bilibili.com/video/BV1Rh41177Af/)



#### First : Install ROS2 Foxy in Ubuntu 20.04

Reference to: 

​	1、鱼香ROS一键安装 Page： https://fishros.com/install/install1s/docs/index.html#/

```yaml
wget http://fishros.com/install -O fishros && . fishros
```

	2. ROS Foxy Page: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html



#### Second :Download Arduino IDE 2.0.4 and Install ESP32-WROOM-32 Board

 1.Download Arduino IDE 2.0 [Download the latest release](https://downloads.arduino.cc/arduino-ide/arduino-ide_latest_Linux_64bit.AppImage?_gl=1*18hna5h*_ga*MTk2NTU5MDE3OC4xNjgxNDczNzg0*_ga_NEXN8H46L5*MTY4MTQ3Mzc4My4xLjAuMTY4MTQ3Mzc4My4wLjAuMA..).

![](pics/Arduino_execute.png)



2. Install ESP32-Board in Arduino IDE

​	**·ESP32 Board URL**

```
https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json
```

​	**·Additional Boards Manager** 

![](pics/install_esp32_1.png)



**·Install ESP32 Board **

![install_esp32_2](pics/install_esp32_2.png)



**· Select ESP32 Dev Module && Port ID**

![select_board](pics/select_board.png)

------



## Chapter 2 install_micro_ros

![Setup_environment_03](pics/Setup_environment_03.png)

Videos from Bilibili 照祥同学: [第二节：安装micro_ros 的 Arduino 开发环境](https://www.bilibili.com/video/BV1LM4y1y7TD/)

#### First : Install micro-ROS Application On Ubuntu 20.04

Ref Url : [**Teensy with Arduino**](https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/)

```yaml
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```



#### Second : Creating a new firmware workspace

```
# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```



**Tips**: May need update your cmake version to 3.16.3

```
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

![build_agen_suss](pics/build_agen_suss.png)



#### Third : Download micro_ros_arduino Library from github

Download URl : **[micro_ros_arduino ](https://github.com/micro-ROS/micro_ros_arduino)**     (**Tips**: branch: foxy)

![add_microros_lib](pics/add_microros_lib.png)



#### Fourth : communication with serial 

example : **micro_ros_publisher**

![test_serial](pics/test_serial.png)

**Tips**: May be need Permission before upload code into esp32 board.

```yaml
sudo chmod 0777 /dev/ttyACM0
```



**·Run micro_ros_agent** 

```yaml
cd microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Tips: Run the ros2 node ,and then press the boot button on the ESP32 Board, you will see.

![communication_suss](pics/communication_suss.png)



**·Open another Terminal and Subscribe the Topic by ESP32 pub**

![subscribe_topic](pics/subscribe_topic.png)



You can try other example !!!                                  

**update by zhaoxiangli 2023.04.14**

------



## Chapter 3 ESP32_Hall_Encoder_Motor

![Fourth : communication with serial ](pics/Setup_environment_04.png)

Videos from Bilibili 照祥同学: [第三节：ESP32捕获霍儿编码电机的脉冲](https://www.bilibili.com/video/BV1La4y1K7mN/)

#### First : Connect Motor's Hall Encoder Sensor with ESP32

![chapter_hall_encoder_motor](pics/chapter_hall_encoder_motor.png)

​																																								（From Fritzing）

```yaml
接线说明：
	霍尔传感器正极 +（蓝色）   <------->  ESP32_Vin (5.0V)
	霍尔传感器负极 - （黑色）   <------->  ESP32_Gnd (0V)
	编码器 Encoder A（绿色）   <------->  ESP32_GPIO4 (Input)
	编码器 Encoder B（黄色）   <------->  ESP32_GPIO5 (Input)
```



------



## Chapter 4 ESP32_HC_SR04

![Setup_environment_05](pics/Setup_environment_05.png)

Videos from Bilibili 照祥同学: [第四节：使用ESP32获取超声波传感器的测量值](https://www.bilibili.com/video/BV1Rv4y1J73M/)

#### First : Connect HC_SR04 Sensor with ESP32

![chapter_sr04](pics/chapter_sr04.png)



```
接线说明：
	超声波传感器正极 +（红色）   <------->  ESP32_Vin (5.0V)
	超声波传感器负极 -（黑色）   <------->  ESP32_Gnd (0V)
	声波发生引脚 TRIG（绿色）   <------->  ESP32_GPIO32 (Output)
	声波接收引脚 ECHO（黄色）   <------->  ESP32_GPIO33 (Input)
```

**update by zhaoxiangli 2023.04.16**

------



## Chapter 5 ESP32_MPU6050_Module

![Setup_environment_06](pics/Setup_environment_06.png)

Videos from Bilibili 照祥同学: 

#### First : Connect MPU6050 Module with ESP32

![chapter_imu](pics/chapter_imu.png)



```
接线说明：
	MPU6050 正极 +（红色）<------->  ESP32_Vin (3.3V)
	MPU6050 负极 -（黑色）<------->  ESP32_Gnd (0V)
	数据引脚 SCL（绿色）   <------->  ESP32_GPIO22 (Output)
	数据引脚 SDA（黄色）   <------->  ESP32_GPIO21 (Input)
```

------



## Chapter 6 ESP32_GPS_BDS_Module

![Setup_environment_07](pics/Setup_environment_07.png)

Videos from Bilibili 照祥同学: 

#### First : Connect MPU6050 Module with ESP32

![chapter_esp32_gps_bds](pics/chapter_esp32_gps_bds.png)



```
接线说明：
	GPS 正极 +（红色）<------->  ESP32_Vin (5.0V)
	GPS 负极 -（黑色）<------->  ESP32_Gnd (0V)
	数据引脚 Rx（绿色）<------->  ESP32_GPIO17 (Tx2 Ouput)
	数据引脚 Tx（黄色）<------->  ESP32_GPIO16 (Rx2 Input)
```



------

