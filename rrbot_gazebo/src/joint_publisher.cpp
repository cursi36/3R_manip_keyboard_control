#include "ros/ros.h"
#include "std_msgs/Float64.h"




int main (int argc, char **argv)
{
ros::init(argc,argv, "joint_publisher_node");
ros::NodeHandle n;

std_msgs::Float64 joint_position;



ros::Publisher joint_pub = n.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 1);

ros::Rate rate(10);


while(ros::ok())
{
joint_position.data += 0.1;
joint_pub.publish(joint_position);
rate.sleep();
}

ros::waitForShutdown();
return 0;


}
