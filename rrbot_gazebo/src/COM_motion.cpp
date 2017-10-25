#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include <geometry_msgs/Pose.h>

 #include "kdl_conversions/kdl_msg.h"
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include "std_msgs/Float64.h"
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <kdl/segment.hpp>
#include "gazebo_msgs/ModelState.h"
#include <kdl/rigidbodyinertia.hpp>

#include <cmath>

std::string urdf_file = "/home/francesco/gazebo_ws/src/gazebo_ros_demos/rrbot_description/urdf/rrbot.xml";
KDL::Tree my_tree;
urdf::Model my_model;

KDL::Chain chain;


class COMMotion 
{
public:
COMMotion()
{

////////////////////////////////////////////
/* Pose Initialization*/

nj = chain.getNrOfJoints();


theta_x = 0;
theta_y = 0;
theta_z = 0;

Rot_mat = Rot_mat.RotX(0);

q_init.resize(nj);

q_out.resize(nj);
qdot_out.resize(nj);

for (int ii = 0; ii<nj; ii++)
{
q_init(ii) = 0;


}


///////////////////////////////////////


//Receive Velocities
motion_sub = node_handle.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,&COMMotion::MotionCallback,this);

joint1_pub = node_handle.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 1000);
 
joint2_pub = node_handle.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 1000);

joint3_pub = node_handle.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 1000);




}



void MotionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
received_twist = *msg;


KDL::ChainFkSolverPos_recursive fksolver(chain);

//Old Pose
 bool kinematics_status;
    kinematics_status = fksolver.JntToCart(q_init,cartpos);

tf::poseKDLToMsg (cartpos,pose);


//Desired Position
//previous position + speed
pose.position.x += msg->linear.x;
pose.position.y +=  msg->linear.y;
pose.position.z += msg->linear.z;

KDL::Vector position_vect(pose.position.x, pose.position.y, pose.position.z);


// cos(t/2)+sin(t/2)(rx x+ry y+rz z);
//Rotation about x of dtheta wrt previou rotation
theta_x += msg->angular.x;
 Rot_mat.DoRotX (msg->angular.x);
//Rotation about y
theta_y += msg->angular.y;
Rot_mat.DoRotY (msg->angular.y);
//Rotation about z
theta_z += msg->angular.z;
Rot_mat.DoRotZ (msg->angular.z);


//tf::poseMsgToKDL (pose,cartpos);

KDL::Frame cartpos(Rot_mat,position_vect);


tf::twistMsgToKDL (*msg, vel_twist); 

///////////////////////
/* Inverse Kin*/
KDL::ChainIkSolverVel_pinv iksolver_v(chain);

int vel_res = iksolver_v.CartToJnt(q_init, vel_twist, qdot_out);

KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolver_v, 100, 1e-6);
int res = iksolver.CartToJnt (q_init, cartpos, q_out);

/*ROS_INFO("Twist");
ROS_INFO_STREAM(*msg);*/
//ROS_INFO_STREAM(cartpos);
/*ROS_INFO_STREAM(position_vect);
ROS_INFO("angles: x=%f, y=%f, z=%f", theta_x, theta_y, theta_z);*/

q_init = q_out;

joint1_position.data = q_out(0);
joint2_position.data = q_out(1);
joint3_position.data = q_out(2);


joint1_pub.publish(joint1_position);
joint2_pub.publish(joint2_position);
joint3_pub.publish(joint3_position);


////////////////////////////////////////////////////////////////////////////////////////
/* COM Position wrt e_e frame*/
/*
Use segments to get mass and COG wrt to their frame;
use matrix transformation to compute total COM

w_P_COM = sum(mi*w_T_i*Pcom_i)/sum(mi)*/

std::vector< KDL::Segment > Segments = chain.segments;
int nseg = Segments.size();


std::vector<KDL::RigidBodyInertia> Links_Inertia;
std::vector<KDL::Vector> COG;

Links_Inertia.resize(nseg);
COG.resize(nseg);

KDL::Frame Links_Pose;
KDL::Vector COG_tot;
KDL::Frame Links_Pose_World(KDL::Vector(0, 0, 0));

double Mass = 0;

for (int ii = 0; ii< nseg; ii++)
{

Links_Inertia[ii] = (Segments[ii].getInertia());	
COG[ii] = (Links_Inertia[ii].getCOG()); //In Link's root_frame
double mass = Links_Inertia[ii].getMass();
Mass += mass;

///////////////////
/* Check joints */
std::string joint_type = (Segments[ii].getJoint().getTypeName());


///F_referce i+1 is F_ tip i

/* F_tip i */
if ( joint_type != "None")
{
Links_Pose = Segments[ii].pose(q_out(ii));

}
else {
Links_Pose = Segments[ii].pose(0);
}

Links_Pose_World = Links_Pose_World*Links_Pose;


// COG of each link WRT world frame
COG[ii] = Links_Pose_World*COG[ii];

COG_tot += mass*COG[ii];
//endfor

}

COG_tot = COG_tot/Mass;

//ROS_INFO_STREAM(COG_tot);


///////////////////////////////////////////////////////////////////////////////////////////////

return;
}

private:
 
KDL::Frame cartpos;    
KDL::Twist vel_twist;

KDL::JntArray q_init, q_out, qdot_out;


std_msgs::Float64 joint1_position;
std_msgs::Float64 joint2_position;
std_msgs::Float64 joint3_position;

geometry_msgs::Pose pose;
geometry_msgs::Twist received_twist;


KDL::Rotation Rot_mat;

ros::NodeHandle node_handle;
ros::Publisher joint1_pub;
ros::Publisher joint2_pub;
ros::Publisher joint3_pub;
ros::Subscriber motion_sub;


double theta_x;
double theta_y;
double theta_z;

unsigned int nj;


};
 
 
int main( int argc, char **argv )
{
ros::init(argc,argv,"COM_motion_node");
ros::NodeHandle node_handle;
ros::Rate r(100);



   if (!my_model.initFile(urdf_file)){
      ROS_ERROR("Failed to parse urdf robot model");
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

my_tree.getChain("world","e_e_link",chain);

 
COMMotion COM_motion;

ros::spin();
return 0;

}
