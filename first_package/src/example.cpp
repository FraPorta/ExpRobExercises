#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "custom_messages/Two.h"


ros::Publisher pub;
ros::Publisher pub2;

void turtleCallback (const turtlesim::Pose::ConstPtr& msg){
	ROS_INFO("Subscriber, %f, %f, %f", msg->x, msg->y, msg->theta);

	geometry_msgs::Twist vel;
	vel.angular.z = 0.1;
	pub.publish(vel);
	

	custom_messages::Two two;
	two.a = msg->x;
	two.b = msg->y;
	pub2.publish(two);
} 



int main (int argc, char **argv){
	ros::init(argc,argv,"example");
	ros::NodeHandle nh;
	
	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	srv.request.x = 2.0;
	srv.request.y = 2.0;
	srv.request.name = "ruga";
	client.call(srv);

	pub2 = nh.advertise<custom_messages::Two>("/position",1);
	pub = nh.advertise<geometry_msgs::Twist>("/ruga/cmd_vel",1);
	ros::Subscriber sub = nh.subscribe("/ruga/pose",1,turtleCallback);
	ros::spin();
	return 0;
}


