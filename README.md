# Reaserch Track 1: second assignment
Author: Monica Fossati s4697871, Robotics Engineering Unige

In this project, a simple robot autonomously travels a track (reproduction of the Monza racing circuit). Thanks to a user interface, the user can decide to increase or decrease the speed of the robot, or reset its position. 

## Installing and running
In order to run multiple nodes at the same time, a launch script is provided.

```second_assignment.launch```:
```
<launch>
   <node name="world" pkg="stage_ros" type="stageros" args="$(find second_assignment)/world/my_world.world" />
   <node name="controller_node" pkg="second_assignment" type="robot_controller" />
   <node name="command_interface" pkg="second_assignment" type="command_interface" required="true" output="screen"/>
</launch>
```
The user can easily run the project with the command:
```bash
$ roslaunch second_assignment second_assignment.launch 
```
## Project structure
The ROS package name for this project is ```second_assignment```.

![graph](https://user-images.githubusercontent.com/62377263/145631182-a5281b6e-e68c-452d-b95d-babde54fbed8.JPG)

There are three nodes:
* ```world```: already provided for this project. It aims to recreate the complete simulation environment of circuit and robot.
* ```controller_node```: it aims to guide the robot without bumping into the walls of the circuit. It communicates with ```world``` through messages: it sends messages to impose the right speed on the robot and receives information regarding the position of the walls provided by the laser scanners with which the robot is equipped. In addition, it also offers a service that can change the speed of the robot according to the command received.
* ```interface_node```: it aims to receive commands from the user and call up the right services.

A ```Service.srv``` service has been created that receives as input the command entered by the user and provides the new speed.

## World
The track in which the robot can move is the following. The robot is represented as a blue point. The light blue area is the area scanned by the laser sensors on the robot.

![monzaCircuit](https://user-images.githubusercontent.com/62377263/145191244-00276a66-588d-4a4e-9afb-adadbb17dfcb.JPG)

The ```world``` node publishes messages on the ```base_scan``` topic. Messages are ```LaserScan``` type, from ```sensor_msg``` package, and they provide the result of a single scan by the robot's laser sensors.

It also subscribes to the topic ```cmd_vel``` to impose a linear and angular velocity to the robot along the three axis. It receives ```Twist``` type messages from ```geometry_msgs``` package.

## Controller
The controller communicates with the ```world``` node via messages.

It subscribes to ```base_scan``` the topic to receive the wall's position and it publishes on the topic ```cmd_vel``` to set the velocity of the robot in the ```wolrd``` node.

It also  implements the ```Service.srv```: taking a command from the user, it decides what will be the velocity of the robot, taking in account min and max velocities.

The operation of this node can be described by the following pseudocode:
```pseudocode
 min_vel = minimum velocity
 max_vel = maximum velocity
 vel = 0
   
if(the service is required)
   com = command from request

   if(vel < max_vel && com == '+')
      encrease vel
   if(vel > min_vel && com == '-')
      decrease vel
   if(com == 'r')
      vel = 0
      
   send vel as new velocity in the response
   
if(there is a message on topic base_scan)
   array = scan result
   
   divide array in 5 parts
   
   left = part1
   front = part3
   right = part5
   
   if(min(front) < threashold)
      if(min(right) < min(left))
         turn left
      else
         turn right
   else
      go straight on with velocity = vel
   
   publish on topic cmd_vel linear and angular velocities decided

```

## Interface
The user interface print on the screen a menu with three choices:
* "+" : encrease velocity
* "-" : decrease velocity
* "r" : reset the system
* "q" : quit

This node uses two services:
*  ```Empty``` type of service in the ```std_srvs``` package. It is very useful to reset the position of the robot if the user pressed 'r'.
*  ```Service```, a custom service defined in ```second_assignment``` package. It is useful to set the velocity of the robot.

The operation of this node can be explained by the following pseudocode:
```pseudocode
velocity = 0

initialize node

create clients for services

initialize servers for the services

print menu and actual velocity

key = r

while(roscore is running)
   key = command read from user
   
   if(key == q)
      kill this node
   else if(key is another valid command)
      send a request to Service with command = key
      if(key == r)
         send a request to Empty service
         tmp = new_velocity in Service response
      if(tmp == 0 && velocity == 0)
         print(Velocity can't be decreased)
      else if(tmp == velocity && tmp == 10)
         print(Velocity can't be encreased)
      else
         velocity = tmp
         
     print velocity
     
  else
     print(invalid command)
```
## Service
It is a custom service described in the file ```Service.srv``` in the ```srv``` folder.
It takes a command (char) and gives the new_velocity (float32).

## Possible improvements
The controller can be improved by making the execution of the curves more precise and trying to modulate the speed better.
