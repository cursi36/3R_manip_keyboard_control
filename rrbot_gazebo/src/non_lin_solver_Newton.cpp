  #include <Eigen/Dense>
  #include <Eigen/SVD>
  #include <iostream>
  #include <cmath>

  #include <ros/ros.h>
  
   /* int sum (int a, int b)
  {
    int Sum = a+b;
   return Sum; 
  }*/
  
   Eigen::MatrixXf FK(Eigen::MatrixXf q)
  {
    
    Eigen::MatrixXf Pose(2,1);
    Pose(0)= cos(q(0))+cos((q(0)+q(1))); 
  Pose(1) = sin(q(0))+sin((q(0)+q(1)));
  return Pose ;
  }


  Eigen::MatrixXf pseudoinverse( Eigen::MatrixXf &mat, int tolerance ) // choose appropriately
  {
    
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::MatrixXf singularValues = svd.singularValues();
      Eigen::MatrixXf singularValuesInv(mat.cols(), mat.rows());
      
      for (unsigned int i = 0; i < singularValues.size(); ++i) {
	  if (singularValues(i) > tolerance)
	  {
	      singularValuesInv(i, i) = 1 / singularValues(i);
	  }
	  else
	  {
	      singularValuesInv(i, i) = 0;
	  }
      }
      return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
  }
  
  int main(int argc, char **argv) 
{
 ros::init(argc,argv,"non_lin_solver");   
  /* typedef Eigen::Matrix<float, 2, 2> Mat;
    Mat mat(2,2);
    mat(0,0) = 1;
    mat(0,1) = 1;
      mat(1,0) = 1;
      mat(1,1) = 1;

      std::cout << mat ;
  int Sum = sum(1,3);
std::cout<< Sum;*/

  Eigen::MatrixXf q_old(2,1);
  Eigen::MatrixXf q_new(2,1);
  Eigen:: MatrixXf J(2,2);
  Eigen::MatrixXf J_pinv;
  
  Eigen::MatrixXf F = FK(q_old);
    
  Eigen::MatrixXf Des_pose (2,1);
  Des_pose(0) = 0.1; 	Des_pose(1) = 0.1;
  
  Eigen::MatrixXf err ;
  // std::cout << Err ;
   q_old(0) = 0.001;	q_old(1) = 0.001;
   
   float Err_q = 1e+07;   
   float Err = 1e+07; 
   int i = 0;
 while(Err >= 1e-03 && Err_q >= 1e-03)
  {
    i = i+1;
    F = FK(q_old)-Des_pose;
    J(0,0) = -sin(q_old(0));
    J(0,1) = -sin(q_old(0)+q_old(1));
    J(1,0) = cos(q_old(0));
    J(1,1) = cos(q_old(0)+q_old(1));
    
   // J_pinv = pseudoinverse(J, 1e-04);
 Eigen::MatrixXf q_new = q_old-J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(F);
    

    
q_new(0) = atan2(sin(q_new(0)),cos(q_new(0)));
q_new(1) = atan2(sin(q_new(1)),cos(q_new(1)));
 
  //  q_new = q_old -J_pinv*F;
    
    
    Eigen::MatrixXf err_q = (q_new - q_old);
     Err_q = sqrt(pow(err_q(0),2)+pow(err_q(1),2));

    q_old = q_new;
    
    err = (Des_pose - FK(q_new));
  Err = sqrt(pow(err(0),2)+pow(err(1),2));
  ROS_INFO_STREAM(i);
	ROS_INFO_STREAM(Err);
ROS_INFO_STREAM(FK(q_new));
ROS_INFO_STREAM(q_new);
  }
  
    
  
      return 0;
  }



 
  
  
