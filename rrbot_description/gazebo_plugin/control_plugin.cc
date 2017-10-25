#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include "ros/subscribe_options.h"

namespace gazebo
{
	class ControlPlugin : public ModelPlugin
	{
		public:
		ControlPlugin()
		{
			ROS_INFO("HEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");		
		}
		// The load function is called by Gazebo when the plugin is
		// inserted into simulation
		
		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
   		{

		model = _model;
		joints = model->GetJoints(); 
		
		name = joints[1]->GetName();
		ROS_INFO_STREAM(name);


		if (!ros::isInitialized())
		{
  		int argc = 0;
 		char **argv = NULL;
  		ros::init(argc, argv, "gazebo_client",
      		ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
		//rosNode.reset(new ros::NodeHandle("gazebo_client"));
		command_sub = rosNode.subscribe("/rrbot_my_joint_states",1000, &ControlPlugin::PluginCallback, this);
	
    		}

		void PluginCallback(const sensor_msgs::JointStatePtr &msg)
		{

		//ROS_INFO_STREAM(name);

		model->GetJointController()->SetPositionTarget (name, msg->position[0]);

		/*const std::map< std::string, double > _jointPositions;
		
		model.SetJointPositions (_jointPositions);*/

		return;
		

		}

		private:
		ros::Subscriber command_sub;
		ros::Publisher Joint_pub;
		physics::ModelPtr model;
		//private: std::unique_ptr<ros::NodeHandle> rosNode;
		ros::NodeHandle rosNode;
		physics::Joint_V joints;  //vector of JointPtr
		std::string name;

	};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}
#endif
