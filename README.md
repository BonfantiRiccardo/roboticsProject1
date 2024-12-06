# First Project for the "Robotics" course of Politecnico di Milano (Prof. Matteucci, Mentasti)
## Notes on the visualization
The only command to execute all nodes and GUI tools is the following:</br>
`roslaunch first_project launch.launch`</br>
The command does not play the bag, so if you only run the command you will see in the noVNC GUI an empty tf_tree and rviz.

The command to execute the bag is the following:</br>
`rosbag play --clock robotics.bag`</br>
After this command is executed you will be able to see the different reference frames moving in rviz and the pointcloud data from the wheel_odom.</br>
You can dinamically reconfigure the frame of the pointcloud data and change it between wheel_odom and gps_odom.</br>
To visualize the tf tree you'll need to "refresh" it since when it was launched no one was publishing on the tf broadcaster (to refresh click the "spinning arrows" in the top left).

## Divergence between gps_odom and wheel_odom
We noticed that the data published on the gps_odom topic and the one published on the wheel_odom topic diverges as the bag goes on.
In particular we observed a strong difference in behaviour between seconds 120 and 230. After that, the movements seem to be coherent again, but there is a big offset in the positions.
Finally, after 350 seconds we observed another big divergence until the end of the bag.

## Team

- [Edoardo Bergamo](https://github.com/Janoww)
- [Riccardo Bonfanti](https://github.com/BonfantiRiccardo)
- [Zining Chen](https://github.com/BiscottiBiscottati)
