#!/bin/bash
while inotifywait -e modify "/home/antonio/prova_ws/ROS/src/rotors_gazebo/resource" 
do    
    rosparam load /home/antonio/prova_ws/ROS/src/rotors_gazebo/resource/crazyflie_mellinger_controller.yaml
    rosparam load /home/antonio/prova_ws/ROS/src/rotors_gazebo/resource/crazyflie_parameters.yaml
    rosservice call /crazyflie2/init_model "initialize: true"
    echo "ok"
done
