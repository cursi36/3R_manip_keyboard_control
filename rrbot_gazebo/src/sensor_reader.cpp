#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

void Callback( const geometry_msgs::WrenchStamped msg)
{

ROS_INFO("Force");
ROS_INFO_STREAM(msg.wrench.force.x);
ROS_INFO_STREAM(msg.wrench.force.y);
ROS_INFO_STREAM(msg.wrench.force.z);
ROS_INFO("torque");
ROS_INFO_STREAM(msg.wrench.torque.x);
ROS_INFO_STREAM(msg.wrench.torque.y);
ROS_INFO_STREAM(msg.wrench.torque.z);

}

int main(int argc, char **argv)
{
ros::init(argc,argv,"sensor_reader_node");
ros::NodeHandle n;

ros::Subscriber sens_sub = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_topic",1,Callback);

ros::spin();
return 0;
}
