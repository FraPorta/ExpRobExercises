#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "position_server/TwoFloats.h"

#define thr 0.1 //Threshold for position control


ros::Publisher pub; //Publisher for sending velocity commands
ros::Subscriber sub; //Subscriber for odometry
ros::ServiceClient client; // Client of the service
float xt,yt;

float dist(float x, float y, float xt, float yt){
	//Function to retrieve the distance between two points in space
	return (sqrt(pow(x-xt,2)+pow(y-yt,2)));
	}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Callback thread on the received position
	float x, y;
	geometry_msgs::Twist vel;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	
	//If the robot is close enough to the target, ask for a new target
	if (dist(x,y,xt,yt)<thr){
		position_server::TwoFloats srv;
		/*Ask for a new target */
		client.call(srv); 
		xt = srv.response.x;
		yt = srv.response.y;
		
	}

	//Simple proportional position control
	else{
		
		if(fabs(x-xt)>thr/2){
			vel.linear.x=(x-xt);
		}
		if(fabs(y-yt)>thr/2){
			vel.linear.y=(y-yt);
		}
	}

	/*publish the velocity */
	pub.publish(vel);

}


int main (int argc, char **argv){
	//Main function for the robot contro
	ros::init(argc, argv, "exercise1");
	ros::NodeHandle nh;
	// Retrieving the initial position target.
	nh.getParam("/exercise1/xt", xt);
	nh.getParam("/exercise1/yt", yt);

	/* Define a client for asking a new target, a publisher for publishing the velocity, 
	and a subscriber for reading the position*/
	client = nh.serviceClient<position_server::TwoFloats>("/rand_floats");
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	sub = nh.subscribe("/odom",1000,odomCallback);

	ros::spin();
	return 0;
}
