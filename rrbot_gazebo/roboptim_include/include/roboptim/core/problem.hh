// Copyright (C) 2009 by Thomas Moulard, AIST, CNRS, INRIA.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ROBOPTIM_CORE_PROBLEM_HH
# define ROBOPTIM_CORE_PROBLEM_HH

# include <iostream>
# include <vector>

# include <boost/mpl/assert.hpp>
# include <boost/mpl/vector.hpp>
# include <boost/optional.hpp>
# include <boost/shared_ptr.hpp>

# include <roboptim/core/fwd.hh>
# include <roboptim/core/portability.hh>
# include <roboptim/core/function.hh>
# include <roboptim/core/detail/utility.hh>

# include <roboptim/core/deprecated.hh>

namespace roboptim
{

  /// \addtogroup roboptim_problem
  /// @{
  /// \brief Optimization problem.
  ///
  /// An optimization problem is defined as:
  /// - a cost function (\f$\mathbb{R}^n \rightarrow \mathbb{R}\f$)
  /// - one or more constraints functions,
  ///   (\f$\mathbb{R}^n \rightarrow \mathbb{R}^m\f$)
  ///  associated with an interval and a scale,
  /// - a set of intervals and scale factors for arguments.
  ///
  /// The goal of the optimization process is finding a point which
  /// minimizes the cost function and which respects the constraints
  /// (i.e. the result of some functions is inside of specific interval).
  ///
  /// To use the class, one has to instantiate a problem with
  /// a reference to a cost function.
  /// Then, constraints can be added through the addConstraint
  /// method: a shared pointer to a function and an interval is needed.
  ///
  /// The cost function is immutable.
  ///
  /// Constraints are stored in a vector of smart pointers to function
  /// objects.
  ///
  /// To add a constraint, use the following syntax:
  /// \code
  /// problem.addConstraint (boost::make_shared<MyFunction> (...), ...);
  /// \endcode
  /// where MyFunction is the function type.
  ///
  ///
  /// Unlike other classes which just copy functions, pointers are used
  /// here in order to allow sub-classes of constraints to be inserted
  /// in the problem.
  /// For instance, a twice-differentiable function can be inserted in
  /// a problem which expects a differentiable function.
  ///
  /// \tparam T matrix type
  template <typename T>
  class ROBOPTIM_GCC_ETI_WORKAROUND Problem
  {
  public:
    /// \brief Function type.
    ///
    /// This has to be either Function or one of its
    /// sub-classes.
    typedef GenericFunction<T> function_t;

    /// \brief Constraints types list.
    typedef boost::mpl::vector<function_t> constraintsList_t;

    /// \brief Constraint's type.
    typedef boost::shared_ptr<function_t> constraint_t;

    /// \brief Import function's value_type type.
    typedef typename function_t::value_type value_type;

    /// \brief Vector type.
    typedef typename function_t::vector_t vector_t;

    /// \brief Argument type.
    typedef typename function_t::argument_t argument_t;

    /// \brief Result type.
    typedef typename function_t::result_t result_t;

    /// \brief Size type.
    typedef typename function_t::size_type size_type;

    /// \brief Constraints are represented as a vector of constraints.
    typedef std::vector<constraint_t> constraints_t;

    /// \brief Optional vector defines a starting point.
    typedef boost::optional<argument_t> startingPoint_t;

    /// \brief Interval type (e.g. for bounds).
    typedef typename function_t::interval_t interval_t;

    /// \brief Intervals type.
    typedef typename function_t::intervals_t intervals_t;

    /// \brief Scaling vector.
    ///
    /// If \f$g_u\f$ is the constraint as seen by the user, \f$g_a\f$
    /// the constraint as seen by the solver's algorithm, and \f$s\f$
    /// the scaling factor, then we have: \f$g_a = s * g_u\f$
    typedef std::vector<value_type> scaling_t;

    /// \brief Scaling vector (deprecated typedef)
    ///
    /// \deprecated Deprecated since version 3.1. Use Problem<T>::scaling_t
    /// instead.
    ROBOPTIM_CORE_DEPRECATED typedef scaling_t scales_t;

    /// \brief Vector of names (e.g. for arguments).
    typedef typename function_t::names_t names_t;

    /// \brief Vector of interval vectors. This type is used to take
    /// into account the fact that constraints can have output values
    /// in \f$\mathbb{C}^{m}\f$.
    ///
    /// If \f$m=1\f$, then the associated interval vector contains
    /// only one element of interval_t type.
    typedef std::vector<intervals_t> intervalsVect_t;

    /// \brief Vector of scaling vectors. This type is used to take into
    /// account the fact that constraints can have output values in
    /// \f$\mathbb{R}^m\f$.
    ///
    /// If \f$m=1\f$, then the associated scaling vector contains only
    /// one element of scaling_t type.
    typedef std::vector<scaling_t> scalingVect_t;

    /// \brief Vector of scaling vectors (deprecated typedef).
    ///
    /// \deprecated Deprecated since version 3.1. Use Problem<T>::scalingVect_t
    /// instead.
    ROBOPTIM_CORE_DEPRECATED typedef scalingVect_t scalesVect_t;

    /// \brief Jacobian matrix type.
    typedef typename GenericFunctionTraits<T>::jacobian_t jacobian_t;

    /// \brief Constant reference to an argument vector.
    typedef typename GenericFunctionTraits<T>::const_argument_ref
    const_argument_ref;

    /// \name Constructors and destructors.
    /// \{

    /// \pre costfunction \f$\mathbb{R}^n \rightarrow \mathbb{R}\f$
    /// \brief Constructor taking a shared_ptr to a cost function.
    /// \param cost cost function.
    explicit Problem (const boost::shared_ptr<const function_t>& cost);

    /// \brief Deprecated constructor taking a reference to a cost function.
    /// This legacy version meant that we simply kept a const reference to the
    /// cost function, which could reference stack variables...
    /// This prepares the transition to something safer (shared_ptr).
    /// \param cost cost function.
    ///
    /// \deprecated Deprecated since version 3.2. Use the shared_ptr
    /// constructor instead.
    // TODO: remove after enough releases (deprecated in 3.2).
    ROBOPTIM_CORE_DEPRECATED explicit Problem (const function_t& cost);

    /// \brief Copy constructor.
    /// \param pb problem to copy.
    explicit Problem (const Problem<T>& pb);

    /// \brief Virtual destructor.
    ///
    /// One may want to inherit from Problem to provide extra information to
    /// a solver plugin.
    virtual ~Problem ();

    /// \}


    /// \name Cost function.
    /// \{

    /// \brief Retrieve cost function.
    /// \return cost function
    const function_t& function () const;

    /// \brief Retrieve arguments bounds.
    /// Arguments bounds define in which interval each argument is valid.
    /// \return arguments bounds
    intervals_t& argumentBounds ();

    /// \brief Retrieve arguments bounds.
    /// Arguments bounds define in which interval each argument is valid.
    /// \return arguments bounds
    const intervals_t& argumentBounds () const;

    /// \brief Retrieve objective scaling.
    /// Objective scaling defines the scaling factors applied to the objective
    /// function.
    /// \return objective scaling
    scaling_t& objectiveScaling ();

    /// \brief Retrieve objective scaling.
    /// Objective scaling defines the scaling factors applied to the objective
    /// function.
    /// \return objective scaling
    const scaling_t& objectiveScaling () const;

    /// \brief Retrieve arguments scaling.
    /// Arguments scaling define which scale factor is applied for each argument.
    /// \return arguments scaling
    scaling_t& argumentScaling ();

    /// \brief Retrieve arguments scaling.
    /// Arguments scaling define which scale factor is applied for
    /// each argument.
    /// \return arguments scaling
    const scaling_t& argumentScaling () const;

    /// \brief Retrieve arguments scaling (deprecated version).
    ///
    /// \deprecated Deprecated since version 3.1. Use argumentScaling() instead.
    ROBOPTIM_CORE_DEPRECATED scaling_t& argumentScales ();

    /// \brief Retrieve arguments scaling (deprecated version).
    ///
    /// \deprecated Deprecated since version 3.1. Use argumentScaling() instead.
    ROBOPTIM_CORE_DEPRECATED const scaling_t& argumentScales () const;

    /// \brief Retrieve arguments names.
    /// Arguments names define a name for each argument. This is particularly
    /// useful when logging data.
    /// \return arguments names
    names_t& argumentNames ();

    /// \brief Retrieve arguments names.
    /// Arguments names define a name for each argument. This is particularly
    /// useful when logging data.
    /// \return arguments names
    const names_t& argumentNames () const;

    /// \}


    /// \name Constraints.
    /// \{

    /// \brief Retrieve constraints.
    /// \return constraints
    const constraints_t& constraints () const;

    /// \brief Add a constraint to the problem.
    ///
    /// Useful only when \f$m=1\f$, use addConstraint
    /// (boost::shared_ptr<C> constraint, intervals_t intervals,
    /// scaling_t scaling) instead.
    ///
    /// \param constraint the constraint that will be added
    /// \param interval interval in which the constraint is satisfied
    /// \param scale constraint scale
    /// \throw std::runtime_error
    void addConstraint (boost::shared_ptr<function_t> constraint,
			interval_t interval,
			value_type scale = 1.);

    /// \brief Add a constraint to the problem.
    ///
    /// The constraint can be multidimensional.
    ///
    /// \param constraint the constraint that will be added
    /// \param intervals interval vector in which the constraint
    /// is satisfied
    /// \param scaling constraint scaling
    /// \throw std::runtime_error
    void addConstraint (boost::shared_ptr<function_t> constraint,
			intervals_t intervals,
			scaling_t scaling);

    /// \brief Retrieve constraints bounds vector.
    /// \return constraints bounds vector
    const intervalsVect_t& boundsVector () const;

    /// \brief Retrieve constraints bounds vector.
    /// \return constraints bounds vector
    intervalsVect_t& boundsVector ();

    /// \brief Retrieve constraints scaling vector.
    /// \return constraints scaling vector
    const scalingVect_t& scalingVector () const;

    /// \brief Retrieve constraints scaling vector (deprecated version).
    ///
    /// \deprecated Deprecated since version 3.1. Use scalingVector() instead.
    ROBOPTIM_CORE_DEPRECATED const scalingVect_t& scalesVector () const;

    /// \brief Return the output size of the problem's constraints.
    size_type constraintsOutputSize () const;

    /// \brief Return the output size of the problem's differentiable
    /// constraints.
    size_type differentiableConstraintsOutputSize () const;

    /// \brief Clear the constraints from the problem.
    void clearConstraints ();

    /// \}


    /// \name Starting point (initial guess).
    /// \{

    /// \brief Set the initial guess.
    /// \return reference on the initial guess
    /// \throw std::runtime_error
    startingPoint_t& startingPoint ();

    /// \brief Get the initial guess.
    /// \return reference on the initial guess
    /// \throw std::runtime_error
    const startingPoint_t& startingPoint () const;

    /// \}


    /// \name Helper methods.
    /// \{

    /// \brief Evaluate the Jacobian matrix of the problem for a given x.
    /// Note: this is a helper method, and is not supposed to be used in any
    /// critical loop.
    ///
    /// \param x evaluation point.
    /// \return Jacobian matrix evaluated at x.
    jacobian_t jacobian (const_argument_ref x) const;

    /// \brief Evaluate the scaled Jacobian matrix of the problem for a given x.
    /// Note: this is a helper method, and is not supposed to be used in any
    /// critical loop. Both constraint and argument scaling parameters are
    /// applied.
    ///
    /// \param x evaluation point.
    /// \return scaled Jacobian matrix evaluated at x.
    jacobian_t scaledJacobian (const_argument_ref x) const;

    /// \brief Evaluate the vector of constraints violation for a given x.
    /// This takes into account both argument bounds and constraint bounds.
    /// If the output value is lower than the lower bound, the violation is
    /// negative.
    /// If the output value is higher than the upper bound, the violation is
    /// positive.
    /// If the output value is within the bounds, the violation is null.
    ///
    /// \param x evaluation point.
    /// \return vector of constraint violation at x.
    result_t constraintsViolationVector (const_argument_ref x) const;

    /// \brief Evaluate the constraint violation for a given x.
    /// This takes into account both argument bounds and constraint bounds.
    ///
    /// \param x evaluation point.
    /// \return constraint violation at x.
    /// \tparam ALG Eigen norm used for the reduction, e.g. 1 or
    /// Eigen::Infinity.
    template <int NORM>
    value_type constraintsViolation (const_argument_ref x) const;

    /// \}


    /// \brief Display the problem on the specified output stream.
    ///
    /// \param o output stream used for display
    /// \return output stream
    std::ostream& print (std::ostream& o) const;

  private:
    /// \brief Initialize attributes and do some checking.
    void initialize ();

  private:
    /// \brief Objective function.
    /// Note: do not give access to this shared_ptr, since for now the legacy
    /// API relying on a simple reference prevents any proper memory
    /// management.
    const boost::shared_ptr<const function_t> function_;

    /// \brief Starting point.
    startingPoint_t startingPoint_;

    /// \brief Vector of constraints.
    constraints_t constraints_;

    /// \brief Constraints intervals vector.
    intervalsVect_t boundsVect_;

    /// \brief Arguments intervals.
    intervals_t argumentBounds_;

    /// \brief Constraints scaling vector.
    scalingVect_t scalingVect_;

    /// \brief Objective scaling.
    scaling_t objectiveScaling_;

    /// \brief Arguments scaling.
    scaling_t argumentScaling_;

    /// \brief Arguments names.
    names_t argumentNames_;
  };

  /// Example shows problem class use.
  /// \example problem-cc.cc

  /// @}

  /// \brief Override operator<< to handle problem display.
  ///
  /// \param o output stream used for display
  /// \param pb problem to be displayed
  /// \return output stream
  template <typename T>
  std::ostream& operator<< (std::ostream& o,
			    const Problem<T>& pb);

  /// \brief Override operator<< to handle problem display.
  ///
  /// \param o output stream used for display
  /// \param pb problem to be displayed
  /// \return output stream
  template <typename T>
  std::ostream& operator<< (std::ostream& o, const Problem<T>& pb);

} // end of namespace roboptim

# include <roboptim/core/problem.hxx>

#endif //! ROBOPTIM_CORE_PROBLEM_HH
