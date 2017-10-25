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

#include "roboptim/core/solver-warning.hh"

namespace roboptim
{
  SolverWarning::SolverWarning (const std::string& msg)
    : std::runtime_error (msg)
  {
  }

  std::ostream& SolverWarning::print (std::ostream& o) const
  {
    return o << what ();
  }

  std::ostream& operator<< (std::ostream& o, const SolverWarning& w)
  {
    return w.print (o);
  }
} // end of namespace roboptim
