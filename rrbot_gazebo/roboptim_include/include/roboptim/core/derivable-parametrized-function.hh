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

#ifndef ROBOPTIM_TRAJECTORY_DERIVABLE_PARAMETRIZABLE_HH
# define ROBOPTIM_TRAJECTORY_DERIVABLE_PARAMETRIZABLE_HH
# include <utility>

# include <roboptim/core/fwd.hh>
# include <roboptim/core/parametrized-function.hh>
# include <roboptim/core/portability.hh>

namespace roboptim
{
  /// \addtogroup roboptim_meta_function
  /// @{

  /// \brief Parametrized function with parameter derivative available.
  ///
  /// Depending on inner function type, this class allows computation
  /// of parameter derivative or combined parameter/function derivative.
  ///
  /// \tparam F inner function type.
  template <typename F>
  class DerivableParametrizedFunction : public ParametrizedFunction<F>
  {
  public:
    /// \brief Import value type.
    typedef typename F::value_type value_type;
    /// \brief Import size type.
    typedef typename F::size_type size_type;
    /// \brief Import vector type.
    typedef typename F::vector_t vector_t;
    /// \brief Import matrix type.
    typedef typename F::matrix_t matrix_t;
    /// \brief Import  result type.
    typedef F result_t;
    /// \brief Import argument types.
    typedef typename F::argument_t argument_t;
    typedef typename F::const_argument_ref const_argument_ref;
    /// \brief Import gradient type.
    typedef typename F::gradient_t gradient_t;
    typedef typename F::gradient_ref gradient_ref;
    typedef typename F::gradient_ref const_gradient_ref;
    /// \brief Import jacobian type.
    typedef typename F::jacobian_t jacobian_t;
    typedef typename F::jacobian_ref jacobian_ref;
    typedef typename F::const_jacobian_ref const_jacobian_ref;

    /// \brief Import jacobian size type (pair of values).
    typedef typename F::jacobianSize_t jacobianSize_t;


    /// \brief Return the gradient size.
    ///
    /// Gradient size is equals to the input size.
    size_type gradientSize () const
    {
      return this->inputSize ();
    }

    /// \brief Return the jacobian size as a pair.
    ///
    /// Gradient size is equals to (output size, input size).
    jacobianSize_t jacobianSize () const
    {
      return std::make_pair (this->inputSize (),
			     this->functionOutputSize ());
    }

    /// \brief Check if the gradient is valid (check size).
    /// \param gradient checked gradient
    /// \return true if valid, false if not
    bool isValidGradient (const_gradient_ref gradient) const
    {
      return gradient.size () == this->gradientSize ();
    }

    /// \brief Check if the jacobian is valid (check sizes).
    ///
    /// \param jacobian checked jacobian
    /// \return true if valid, false if not
    bool isValidJacobian (const_jacobian_ref jacobian) const throw ()
    {
      return jacobian.rows () == this->jacobianSize ().first
	&& jacobian.cols () == this->jacobianSize ().second;
    }

    /// \brief Computes the jacobian.
    ///
    /// \param argument point at which the jacobian will be computed
    /// \param order derivation order
    /// \return jacobian matrix
    jacobian_t jacobian (const_argument_ref argument, size_type order = 0)
      const
    {
      jacobian_t jacobian (jacobianSize ().first, jacobianSize ().second);
      jacobian.setZero ();
      this->jacobian (jacobian, argument, order);
      return jacobian;
    }

    /// \brief Computes the jacobian.
    ///
    /// Program will abort if the jacobian size is wrong before
    /// or after the jacobian computation.
    /// \param jacobian jacobian will be stored in this argument
    /// \param order derivation order
    /// \param argument inner function point argument value
    void jacobian (jacobian_ref jacobian, const_argument_ref argument,
		   size_type order = 0) const
    {
      assert (argument.size () == this->inputSize ());
      assert (this->isValidJacobian (jacobian));
      this->impl_jacobian (jacobian, argument, order);
      assert (this->isValidJacobian (jacobian));
    }

    /// \brief Computes the gradient.
    ///
    /// \param argument inner function argument value
    /// \param functionId function id in split representation
    /// \param order derivation order
    /// \return gradient vector
    gradient_t gradient (const_argument_ref argument,
			 size_type functionId = 0,
			 size_type order = 0) const
    {
      gradient_t gradient (gradientSize ());
      gradient.setZero ();
      this->gradient (gradient, argument, functionId, order);
      return gradient;
    }

    /// \brief Computes the gradient.
    ///
    /// Program will abort if the gradient size is wrong before
    /// or after the gradient computation.
    /// \param gradient gradient will be stored in this argument
    /// \param argument inner function point argument value
    /// \param functionId function id in split representation
    /// \param order derivation order
    /// \return gradient vector
    void gradient (gradient_ref gradient,
		   const_argument_ref argument,
		   size_type functionId = 0,
		   size_type order = 0) const
    {
      assert (argument.size () == this->inputSize ());
      assert (this->isValidGradient (gradient));
      this->impl_gradient (gradient, argument, functionId, order);
      assert (this->isValidGradient (gradient));
    }

    /// \brief Display the function on the specified output stream.
    ///
    /// \param o output stream used for display
    /// \return output stream
    virtual std::ostream& print (std::ostream& o) const
    {
      return o << "Derivable parametrized function";
    }

  protected:
    /// \brief Concrete class constructor should call this constructor.
    ///
    /// \param inputSize parameter size
    /// \param functionInputSize inner function argument size
    /// \param functionOutputSize inner function result size
    DerivableParametrizedFunction (size_type inputSize,
			  size_type functionInputSize,
			  size_type functionOutputSize)
      : ParametrizedFunction<F> (inputSize,
				 functionInputSize,
				 functionOutputSize)
    {
    }

    /// \brief Jacobian evaluation.
    ///
    /// Computes the jacobian, can be overridden by concrete classes.
    /// The default behavior is to compute the jacobian from the gradient.
    /// \warning Do not call this function directly, call #jacobian instead.
    /// \param jacobian jacobian will be store in this argument
    /// \param arg point where the jacobian will be computed
    /// \param order derivation order
    virtual void impl_jacobian (jacobian_ref jacobian, const_argument_ref arg,
                                 size_type order = 0)
      const
    {
      for (size_type i = 0; i < this->functionOutputSize (); ++i)
	{
	  gradient_t grad = this->gradient (arg, i, order);
	  for (size_type j = 0; j < this->inputSize (); ++j)
	    jacobian (i, j) = grad[j];
      }
    }

    /// \brief Gradient evaluation.
    ///
    /// Compute the gradient, has to be implemented in concrete classes.
    /// The gradient is computed for a specific sub-function which id
    /// is passed through the functionId argument.
    /// \warning Do not call this function directly, call #gradient instead.
    /// \param gradient gradient will be store in this argument
    /// \param argument inner function point argument value
    /// \param functionId evaluated function id in the split representation
    /// \param order derivation order
    virtual void impl_gradient (gradient_ref gradient,
				const_argument_ref argument,
				size_type functionId = 0,
				size_type order = 0)
      const = 0;
  };

  /// @}

} // end of namespace roboptim.

#endif //! ROBOPTIM_TRAJECTORY_N_TIMES_DERIVABLE_HH
