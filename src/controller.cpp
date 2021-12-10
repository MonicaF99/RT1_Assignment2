/* Run with: $ roslaunch second_assignment second_assignment.launch */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Service.h"

// Declare a publisher
ros::Publisher pub;

// Safety distance
double d_th = 1.5;
// Initial velocity
float velocity = 0;
// Set min and max values
float max_vel = 10;
float min_vel = 0;
// Define velocities for the curves
float angular_velocity = 1.5;
float linear_velocity = 0.5;

bool serviceFnc (second_assignment::Service::Request &req, second_assignment::Service::Response &res)
{
	// Read actual velocity
	if(req.command == '+' && velocity < max_vel)
	{
		// Encrease velocity if velocity isn't maximal
		velocity += 0.5;
	}
	else if(req.command == '-' && velocity > min_vel)
	{
		// Encrease velocity if velocity isn't minimal
		velocity -= 0.5;
	}
	else if(req.command == 'r')
	{
		// Reset velocity to 0
		velocity = 0;
	}
        // Set service's output
	res.new_velocity = velocity;

	return true;
}

double check_dist(double scan[], int size)
{
	double min_dist = 100;
	for(int i=0; i < size; i++)
	{
		if(scan[i] < min_dist)
		{
			min_dist = scan[i];
		}
	}
	return min_dist;
}

void robotCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	geometry_msgs::Twist vel;

	// get array by laserscan
	float scan[msg->ranges.size()];
	
	for(int k = 0; k < msg->ranges.size(); k++)
	{
		scan[k] = msg->ranges[k];
	}
	
	//divide the laserscan values in 5 sectors, but consider only 3 of them
	int sector_size = (msg->ranges.size())/5;
	double left[sector_size];
	double front[sector_size];
	double right[sector_size];
	
	//fill sector arrays
	for(int i = 0; i < sector_size; i++)
	{
		left[i] = scan[i + 4*sector_size];
	}

	for(int i = 0; i < sector_size; i++)
	{
		front[i] = scan[i + 2*sector_size];
	}

	for(int i = 0; i < sector_size; i++)
	{
		right[i] = scan[i];
	}

	// check if there is a wall in front of the robot
	if(check_dist(front, sector_size) < d_th)
	{	
		// check if the nearest wall is on the right
		if(check_dist(right,sector_size) < check_dist(left,sector_size))
		{
			vel.angular.z = angular_velocity;
			vel.linear.x = linear_velocity;
		}
		else
		{
			vel.angular.z = -1 * angular_velocity;
			vel.linear.x = linear_velocity;
		}
	}
	else
	{
		//go straight on
		vel.linear.x = velocity;
	}

	pub.publish(vel);

}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/base_scan", 100, robotCallback);
	ros::ServiceServer service = nh.advertiseService("/service", serviceFnc);

	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);

	ros::spin();
	return 0;

}
