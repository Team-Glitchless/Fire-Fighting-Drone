# Fire-Fighting-Drone

   [src folder](https://github.com/Team-Glitchless/Fire-Fighting-Drone/tree/controller/src) contains the source code for controller and yolo node. 
     Download the file [controller.py](https://github.com/Team-Glitchless/Fire-Fighting-Drone/blob/controller/src/controler.py) and [yolo_script_v1.py](https://github.com/Team-Glitchless/Fire-Fighting-Drone/blob/controller/src/yolo_script_v1.py) into the ```~/PX4-Autopilot/Tools/sitl_gazebo/src/``` directory and launch px4.
      Runnung the commands ```rosrun px4 controller.py``` and ```rosrun px4 yolo_script_v1.py``` in seperate terminal windows will launch the nodes. 
      The NBVP planner is in the [master](https://github.com/Team-Glitchless/Fire-Fighting-Drone/tree/master/src) branch. For details on running it read the [Readme](https://github.com/Team-Glitchless/Fire-Fighting-Drone/blob/master/README.md)
