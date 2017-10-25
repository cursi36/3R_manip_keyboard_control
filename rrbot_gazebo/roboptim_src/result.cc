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

#include "debug.hh"

#include <iostream>
#include <vector>

#include <roboptim/core/function.hh>
#include <roboptim/core/indent.hh>
#include <roboptim/core/result.hh>
#include <roboptim/core/util.hh>

namespace roboptim
{
  Result::Result (const size_type inputSize_,
                  const size_type outputSize_)
    : inputSize (inputSize_),
      outputSize (outputSize_),
      x (inputSize),
      value (outputSize),
      constraints (),
      constraint_violation (Function::infinity ()),
      lambda (),
      warnings ()
  {
    x.setZero ();
    value.setZero ();
    constraints.setZero ();
    lambda.setZero ();
  }

  Result::~Result ()
  {
  }

  std::ostream&
  Result::print (std::ostream& o) const
  {
    o << "Result:" << incindent
      << iendl << "Size (input, output): " << inputSize << ", " << outputSize
      << iendl << "X: " << x
      << iendl << "Value: " << value;
    if (constraints.size () > 0)
      o << iendl << "Constraints values: " << constraints;
    if (constraint_violation < Function::infinity ())
      o << iendl << "Constraint violation: " << constraint_violation;
    if (lambda.size () > 0)
      o << iendl << "Lambda: " << lambda;
    if (!warnings.empty ())
      o << iendl << "Warnings: " << warnings;

    return o << decindent;
  }

  std::ostream& operator<< (std::ostream& o, const Result& r)
  {
    return r.print (o);
  }

} // end of namespace roboptim
