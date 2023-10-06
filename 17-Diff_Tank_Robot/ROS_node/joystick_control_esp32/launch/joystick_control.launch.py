<launch>
  <node pkg="joy" exec="joy_node" name="joy_node" />
  <node pkg="joystick_control_esp32" exec="joystick_node" name="joystick_node" />
  <node pkg="rviz2" exec="rviz2" name="rviz2" />
 <node pkg="tf2_ros" exec="static_transform_publisher" name="base_link_to_mpu6050" args="0.2 0 0 0 0 0 base_link ultrasonar" />

</launch>