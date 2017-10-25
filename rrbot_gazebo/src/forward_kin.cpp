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

#include "gazebo_msgs/ModelState.h"

using namespace KDL;
 
/*
rosrun xacro xacro --inorder -o rrbot.xml rrbot.xacro

 */

int main( int argc, char** argv )
{
ros::init(argc,argv,"forward_kin_node");
ros::NodeHandle node_handle;

std::string urdf_file = "/home/francesco/gazebo_ws/src/gazebo_ros_demos/rrbot_description/urdf/rrbot.xml";


ros::Publisher joint_pub1 = node_handle.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 1);

ros::Publisher joint_pub2 = node_handle.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 1);


ros::Rate rate(10);

std_msgs::Float64 joint1_position;
std_msgs::Float64 joint2_position;

gazebo_msgs::ModelState modelstate;
geometry_msgs::Pose pose;

modelstate.model_name = "rrbot";
////////////////////////////////

KDL::Tree my_tree;
   urdf::Model my_model;
   if (!my_model.initFile(urdf_file)){
      ROS_ERROR("Failed to parse urdf robot model");
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

////////////////////////////////

    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
 /*   chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1)))); //rotational joitn about z, with idebtity rotation matrix and offset between joint frame and tip frame

    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,1))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));*/
 

//////////////////////////
/* Get chain */

 my_tree.getChain("link1","link4",chain);


////////////////////////////////////////////////


    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
KDL::JntArray q_init(nj), q_out(nj);

while(ros::ok())
{
  /* // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }*/

jointpositions(0) = 0;
jointpositions(1) = 0;
jointpositions(2) = 1.5707;
 
    // Create the frame that will contain the results
    KDL::Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

///////////////////////
/* Inverse Kin*/
ChainIkSolverVel_pinv iksolver_v(chain);
ChainIkSolverPos_NR iksolver(chain, fksolver, iksolver_v, 100, 1e-6);


 int res = iksolver.CartToJnt (q_init, cartpos, q_out);

//ROS_INFO_STREAM(q_out(0));

//////////////////////////////

ROS_INFO_STREAM(q_out(0));

/*joint1_position.data = q_out(0);
joint2_position.data = q_out(2);*/

joint1_position.data = jointpositions(0);
joint2_position.data = jointpositions(1);

joint_pub1.publish(joint1_position);
joint_pub2.publish(joint2_position);




rate.sleep();
}



ros::waitForShutdown();
return 0;

}
