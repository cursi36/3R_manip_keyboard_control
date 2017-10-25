
#include "keyboard_control.h"

#include <sensor_msgs/JointState.h>
#include <kdl/chainiksolverpos_lma.hpp>

#include <boost/shared_ptr.hpp>

#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <cmath>
#include <iostream>

#include <kdl/treejnttojacsolver.hpp>
//#include "non_lin_solver.h"


std::string urdf_file = "/home/francesco/gazebo_ws/src/gazebo_ros_demos/rrbot_description/urdf/rrbot.xml";
KDL::Tree my_tree;
urdf::Model my_model;

KDL::Chain chain;

Eigen::VectorXd Fun_val(keyboard_control::KeyboardControl KeyboardControlObj, Eigen::VectorXd x, KDL::Vector COM_des)
{
  
  Eigen::VectorXd F_vect(x.rows());
  KDL::JntArray x_kdl(x.rows());
  x_kdl.data = x;
  
     KDL::Vector  F_vect_kdl = KeyboardControlObj.COM_computation(x_kdl)-COM_des;
  
  
      for (int i = 0; i <= 2; i++)
      {
	F_vect[i] = F_vect_kdl(i);
      }
      
      return F_vect;
  
}

Eigen::MatrixXd Jac_val(keyboard_control::KeyboardControl KeyboardControlObj, Eigen::VectorXd x)
{
  
        KDL::JntArray x_kdl;
      x_kdl.data = x;
      
      Eigen::MatrixXd Jacob = KeyboardControlObj.COM_Jacobian(x_kdl);
      return Jacob;
  
}


  


/* =================== NON Lin Solver Function=================== */
using namespace roboptim;
// Specify the solver that will be used
typedef Solver<EigenMatrixDense> solver_t;


  class F: public DifferentiableFunction
  {
  public:
    F (int nvar) : DifferentiableFunction( nvar,1,"COM_inverse_kin")
    {

    }
    
    
    void  impl_compute(result_ref result, const_argument_ref x) const 
   {
      
     result[0] = Fun_val(KCObj, x, COM).squaredNorm(); 
     
    }
    
    void impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const
   {
      gradient = 2*(Jac_val(KCObj, x).transpose())*(Fun_val(KCObj, x,COM));
 
    }
    
    keyboard_control::KeyboardControl KCObj;
    KDL::Vector COM;
    
  };
  
  
 Eigen::VectorXd non_lin_solver (boost::shared_ptr<F> fun ,Eigen::VectorXd x_init)
{

 solver_t::problem_t pb(fun);
 
  Function::vector_t start (pb.function ().inputSize ());
 
  start = x_init;
  pb.startingPoint() = start;
  
   SolverFactory<solver_t> factory ("ipopt", pb);
  solver_t& solver = factory();
 
  solver_t::result_t res = solver.minimum ();
   ROS_INFO("HEY");
  
        Result& result = boost::get<Result> (res);

//    switch (res.which ())
//     {
//     case solver_t::SOLVER_VALUE:
//       {
// 
// //         Display the result.
//         std::cout << "A solution has been found: " << std::endl
//                   << result << std::endl;
//    
//  	
//       }
// 
//     case solver_t::SOLVER_ERROR:
//       {
//         std::cout << "A solution should have been found. Failing..."
//                   << std::endl
//                   << boost::get<SolverError> (res).what ()
//                   << std::endl;
// 
//         
//       }
// 
//     case solver_t::SOLVER_NO_SOLUTION:
//       {
//         std::cout << "The problem has not been solved yet."
//                   << std::endl;
// 
//        
//       }
//     }
   
    
    return result.x;

}
  



/* ======================= Gazebo Control Class ================= */

class COMMotion 
{
public:
COMMotion()
{

// ======================== Initialization ===================== //

nj = chain.getNrOfJoints();


theta_x = 0;
theta_y = 0;
theta_z = 0;

Rot_angles.resize(3);

Rot_mat = Rot_mat.RotX(0);

q_init.resize(nj);

q_out.resize(nj);
qdot_out.resize(nj);



joints_state.name = {"joint1", "joint2", "joint3"};


Joint_Pub.resize(nj);
Joint_Pos.resize(nj);

for (int ii = 0; ii<nj; ii++)
{
q_init(ii) = 0;
q_out(ii) = 0;

Joint_Pub[ii] = node_handle.advertise<std_msgs::Float64>("/rrbot/" + joints_state.name[ii] + "_position_controller/command",1000);

}



motion_sub = node_handle.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,&COMMotion::MotionCallback,this);
//joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("/rrbot_my_joint_states",1000);*/


KeyboardControlObj.InitKeyboardControl(my_tree, chain);

  
  func = boost::shared_ptr<F> (new F(nj));
  func->KCObj = KeyboardControlObj;

}



void MotionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  
  

received_twist = *msg;


// ======================== FK to get e_e pose wrt world ===================== //

KDL::ChainFkSolverPos_recursive fksolver(chain);

//Old Pose
bool kinematics_status;
kinematics_status = fksolver.JntToCart(q_init,cartpos);

tf::poseKDLToMsg (cartpos,pose);
Rot_mat = cartpos.M;

Rot_mat.GetRPY(Rot_angles[0],Rot_angles[1],Rot_angles[2]);


//Desired e_e Position
//previous position + speed
pose.position.x += msg->linear.x;
pose.position.y +=  msg->linear.y;
pose.position.z += msg->linear.z;


// Control end effector by solving for its IK
 KDL::Vector position_vect(pose.position.x, pose.position.y, pose.position.z); //e_e desired position


// ======================== Desired e_e Orientation ===================== //
//Rotation about x of dtheta wrt previou rotation

 Rot_mat.DoRotX (msg->angular.x);
//Rotation about y

Rot_mat.DoRotY (msg->angular.y);
//Rotation about z

Rot_mat.DoRotZ (msg->angular.z);

Rot_angles[0] += msg->angular.x;
Rot_angles[1] += msg->angular.y;
Rot_angles[2] += msg->angular.z;

//ROS_INFO_STREAM(Rot_angles[1]);


//KDL::Rotation Rot_mat_rpy = Rot_mat.RPY(Rot_angles[0],Rot_angles[1],Rot_angles[2]);

cartpos.M = Rot_mat;
cartpos.p = position_vect;

//ROS_INFO_STREAM(cartpos);

tf::twistMsgToKDL (*msg, vel_twist); 

// ======================== Inverse Kin ===================== //

/*KDL::ChainIkSolverVel_pinv iksolver_v(chain);

int vel_res = iksolver_v.CartToJnt(q_init, vel_twist, qdot_out);

//KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolver_v, 100, 1e-6); 
KDL::ChainIkSolverPos_LMA iksolver(chain, 1e-5, 500, 1e-10); //only goal position

int res = iksolver.CartToJnt (q_init, cartpos, q_out); */


// ======================== Non Linear Problem Solver for COM Control===================== //


COG_tot_world = KeyboardControlObj.COM_computation(q_init);
COMJac = KeyboardControlObj.COM_Jacobian(q_init);

 COG_tot_world_new = COG_tot_world+KDL::Vector(msg->linear.x, msg->linear.y,  msg->linear.z);

 
Eigen::VectorXd q_newton = Newton_Raphson(COG_tot_world,COG_tot_world_new, COMJac,  q_init.data);

q_out.data = q_newton;

//int res = keyboard_control::KeyboardControl::Int_provider();
//ROS_INFO_STREAM(cartpos);
//ROS_INFO_STREAM(COG_tot_world);


////////////////////////////////////////////
  func->COM   = COG_tot_world_new;  
 Eigen::VectorXd q_nonlin =  non_lin_solver (func,q_init.data);
 ROS_INFO("q_newton");
 ROS_INFO_STREAM(q_newton);
  ROS_INFO("q_nonlin");
 ROS_INFO_STREAM(q_nonlin);
// ======================================================================== //


q_init = q_out;


// ======================== Publish Joint Positions ===================== //
for(int iii = 0; iii<nj; iii++)
{

Joint_Pos[iii].data = q_out(iii);

Joint_Pub[iii].publish(Joint_Pos[iii]);

}

//ros::Duration(5).sleep();

return;

}

/* ======================================================================================= */


/* ================ Newton Raphson Method Member ================================================ */

Eigen::VectorXd Newton_Raphson(KDL::Vector P_kdl, KDL::Vector P_des_kdl, Eigen::MatrixXd Jacob, Eigen::VectorXd q)
{
  // P_new = P_old + J dq = P_des;
  double thresh = 1e-03;
  double Err_q = 1;
  double Err = 1;
  Eigen::VectorXd dq(Jacob.cols());
  Eigen::VectorXd P(3); //old_position
  Eigen::VectorXd P_des(3);
  KDL::JntArray q_kdl(Jacob.cols());
  
  
  // Convert from KDL::Vectro to Eigen
int iter = 0;
 
  while(Err_q >= thresh && iter <= 1000)
  {
    iter++;
      for (int i = 0; i<=2; i++)
  {
  P[i] = P_kdl.data[i];
  P_des[i] = P_des_kdl.data[i];
  }
    
   dq = Jacob.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(P_des-P);
    
   for(int i =0; i<dq.size(); i++)
   {
    dq[i] = atan2(sin(dq[i]),cos(dq[i])); 
   }
   
    Err_q = dq.squaredNorm();
    Err = (P_des-P).squaredNorm();

    q += dq;
    q_kdl.data = q;
    
P_kdl = KeyboardControlObj.COM_computation(q_kdl);
Jacob = KeyboardControlObj.COM_Jacobian(q_kdl);
    
  // ROS_INFO_STREAM(dq); 
  }
  
  
  
  return q;
  
}


// ======================== Other Members ===================== //

private:
 
KDL::Frame cartpos;    
KDL::Twist vel_twist;

KDL::JntArray q_init, q_out, qdot_out;

sensor_msgs::JointState joints_state;

geometry_msgs::Pose pose;
geometry_msgs::Twist received_twist;


KDL::Rotation Rot_mat;

ros::NodeHandle node_handle;
ros::Publisher joint_state_pub;
ros::Subscriber motion_sub;

std::vector<ros::Publisher> Joint_Pub;
std::vector<std_msgs::Float64> Joint_Pos;

double theta_x;
double theta_y;
double theta_z;

unsigned int nj;

keyboard_control::KeyboardControl KeyboardControlObj;



KDL::Vector COG_tot_world;
KDL::Vector COG_tot_world_new;
std::vector<double> Rot_angles;

Eigen::MatrixXd COMJac;

  
  boost::shared_ptr<F> func ;
  

};

// ============================= MAIN ============================ */ 
int main( int argc, char **argv )
{
ros::init(argc,argv,"keyboard_control_node");
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

 /*KDL::TreeJntToJacSolver Tree_Jac_solver(my_tree);
 int n_jnts = my_tree.getNrOfJoints();
KDL::Jacobian Jac(n_jnts);

KDL::JntArray q_jac(n_jnts);
q_jac(0) = 1.5707;

int res = Tree_Jac_solver.JntToJac (q_jac, Jac, "link1");

Eigen::MatrixXd Jp = Jac.data.block(0,0,3,n_jnts);
	 Eigen::MatrixXd Jo = Jac.data.block(3,0,3,n_jnts);

ROS_INFO_STREAM(Jac.data);*/
 
COMMotion COM_motion;

ros::spin();
return 0;

}
