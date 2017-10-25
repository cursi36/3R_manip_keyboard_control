
#include "non_lin_solver.h"



/*Eigen::VectorXd Non_Lin_Solver::NonLinSolver::non_lin_solver ()
{
  
 boost::shared_ptr<F> fun (new F());
 
 solver_t::problem_t pb(fun);
 
  Function::vector_t start (pb.function ().inputSize ());
  
  //Starting point is older joins position
  
  start = x_init;
  
  pb.startingPoint() = start;
  
   SolverFactory<solver_t> factory ("ipopt", pb);
  solver_t& solver = factory ();
  
  solver_t::result_t res = solver.minimum ();
  
   std::cout << solver << std::endl;
   
   switch (res.which ())
    {
    case solver_t::SOLVER_VALUE:
      {
        // Get the result.
        Result& result = boost::get<Result> (res);

        // Display the result.
        std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;
		  	
	 std::cout << "HETYYYYYYYYYYYYYYYYYYYYY"<< std::endl;    
    std::cout << fun->Y-(fun->X*result.x)<< std::endl;
     std::cout << err<< std::endl;
    
    
    
        return result.x;

	
	
      }

    case solver_t::SOLVER_ERROR:
      {
        std::cout << "A solution should have been found. Failing..."
                  << std::endl
                  << boost::get<SolverError> (res).what ()
                  << std::endl;

        
      }

    case solver_t::SOLVER_NO_SOLUTION:
      {
        std::cout << "The problem has not been solved yet."
                  << std::endl;

       
      }
    }

}*/