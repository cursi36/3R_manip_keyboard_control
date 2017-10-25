#ifndef KEYBOARD_CONTROL_H_
#define KEYBOARD_CONTROL_H_

#include <kdl/frames.hpp>
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

#include <kdl/treejnttojacsolver.hpp>

#include<Eigen/Dense>
#include <iostream>

namespace keyboard_control
{
	class KeyboardControl
	{
	public:
		
		KeyboardControl()
		{
	  
		}

		void InitKeyboardControl(KDL::Tree tree,KDL::Chain chain)
		{
		// ============ Initilaization =============== //		
		  Segments = chain.segments;
		nseg = Segments.size();	
		Links_Inertia.resize(nseg); 
		COG.resize(nseg);
		my_tree = tree;
		n_jnts = my_tree.getNrOfJoints();
		
			
		}

	// ======================= Global Members ======================= //		
		std::vector<KDL::Segment> Segments;
		 std::vector<KDL::RigidBodyInertia> Links_Inertia; // Inertia Parameter of each link
		 std::vector<KDL::Vector> COG; //COG of each link wrt link's root frame
		 KDL::Frame Links_Pose;
		 KDL::Frame Links_Pose_World;
		 int nseg, n_jnts;	
		 std::string joint_type;

		 KDL::Tree my_tree;
		
		/*static int Int_provider()
		{
		 return 1; 
		}*/
		
	
	 KDL::Vector COM_computation (KDL::JntArray &q);
	
	 Eigen::MatrixXd COM_Jacobian( KDL::JntArray &q)
	{
	  
	  KDL::Rotation Link_Rot_mat = KDL::Rotation::Identity();
	  double Mass = 0;
	  double m_i;
	 Eigen::MatrixXd COM_Jac= Eigen::MatrixXd::Zero(3,n_jnts);
 
	KDL::TreeJntToJacSolver Tree_Jac_solver(my_tree);
	 
	// Jac of each Link in Position q
	  int ii = 0;
	 for ( int i = 0; i<nseg; i++)
	 {

	    KDL::Jacobian Jac(n_jnts);
	   std::string Link_name =  Segments[i].getName();
	   
	 int res = Tree_Jac_solver.JntToJac (q, Jac, Link_name);
	 
	 Eigen::MatrixXd Jp = Jac.data.block(0,0,3,n_jnts);
	 Eigen::MatrixXd Jo = Jac.data.block(3,0,3,n_jnts);
	 
	 KDL::Vector COM_i = Segments[i].getInertia().getCOG(); // In link's root frame
	 
		  ///F_referce i+1 is F_ tip i

	  /* F_tip i wrt F_root_i = F_tip_i-1*/
	  if ( Segments[i].getJoint().getTypeName() != "None")
	   {
	   Link_Rot_mat = Link_Rot_mat * Segments[i].pose(q(ii)).M; //Rotation wrt  world one
	  ii++;
	  }
	  else {
	   Link_Rot_mat = Link_Rot_mat * Segments[i].pose(0).M; //Rotation wrt  world one
	  }

	 COM_i = Link_Rot_mat*COM_i; //COm of Link i expressed in world frame

	 m_i = Segments[i].getInertia().getMass();
	 Mass += m_i;
	 
	 // J_COM_i = Jp + JoxP in world frame
	 Jp.row(0) += -Jo.row(2)*COM_i[1]+Jo.row(1)*COM_i[2];
	 Jp.row(1) += Jo.row(2)*COM_i[0]-Jo.row(0)*COM_i[2];
	 Jp.row(2) += -Jo.row(1)*COM_i[0]+Jo.row(0)*COM_i[1];
	 
	 
	 
	 COM_Jac += m_i*Jp;
	 }

		 
	 COM_Jac = COM_Jac/Mass;
  
	return COM_Jac; 
	 }
						
	};

}


#endif  /* KEYBOARD_CONTROL_H_ */
