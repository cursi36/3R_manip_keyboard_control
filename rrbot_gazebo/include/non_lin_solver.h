#ifndef NON_LIN_SOLVER_H_
#define NON_LIN_SOLVER_H_

#include "keyboard_control.h"

#include <boost/shared_ptr.hpp>

#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <cmath>
#include <iostream>

     
     using namespace roboptim;

// Specify the solver that will be used
typedef Solver<EigenMatrixDense> solver_t;



namespace Non_Lin_Solver
{
  class NonLinSolver
  {
  public:
    NonLinSolver()
    {
 
    }
    

    
     void Init(KDL::Vector COM, KDL::JntArray& q_init)
    {
     
    this-> n_jnt = q_init.rows();
      x_init = Eigen::VectorXd::Zero(n_jnt);
	this-> COM_des = COM;

    
     this->x_init = q_init.data;


    }
    
   // Eigen::VectorXd non_lin_solver();

   Eigen::VectorXd Fun(Eigen::VectorXd& x)
    {
      Eigen::VectorXd F_vect(3);
      KDL::Vector F_vect_kdl;
  KDL::JntArray x_kdl;
  x_kdl.data = x;
  
      F_vect_kdl = Obj.COM_computation(x_kdl)-COM_des;
  
  
      for (int i = 0; i <= 2; i++)
      {
	F_vect[i] = F_vect_kdl(i);
      }
      
      return F_vect;
     }

   Eigen::MatrixXd Jac(Eigen::VectorXd& x)
    {
      KDL::JntArray x_kdl;
      x_kdl.data = x;
      
      Eigen::MatrixXd Jacob = Obj.COM_Jacobian(x_kdl);
      return Jacob;
    
    }
    
     KDL::Vector COM_des;
     Eigen::VectorXd x_init;
     int n_jnt;
     
     keyboard_control::KeyboardControl Obj;


  };
  
  
 
  
  class F: public DifferentiableFunction
  {
  public:
    F () : DifferentiableFunction(3,1,"COM_inverse_kin")
    {
      
    }
    
    
    void  impl_compute(result_ref result, const_argument_ref x) const 
   {
      
     result[0] = Fun_val(x).squaredNorm(); 
     
    }
    
    void impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const
   {
      gradient = 2*(Jac(x).transpose())* (Fun(x));
     /*gradient << 1,
		  0,
		   0;*/
     
    }
    
  //NonLinSolver NLSObj;

    
  };
  
  
 
  

}

#endif