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

#ifndef ROBOPTIM_CORE_INDENT_HH
# define ROBOPTIM_CORE_INDENT_HH
# include <roboptim/core/sys.hh>
# include <roboptim/core/debug.hh>

# include <iosfwd>

namespace roboptim
{
  /// \brief The current indentation level for \a o.
  ROBOPTIM_CORE_DLLAPI long int& indent (std::ostream& o);

  /// \brief Increment the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& incindent (std::ostream& o);

  /// \brief Decrement the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& decindent (std::ostream& o);

  /// \brief Reset the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& resetindent (std::ostream& o);

  /// \brief Print an end of line, then set the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& iendl (std::ostream& o);

  /// \brief Increment the indentation, print an end of line,
  /// and set the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& incendl (std::ostream& o);

  /// \brief  Decrement the indentation, print an end of line,
  /// and set the indentation.
  ROBOPTIM_CORE_DLLAPI std::ostream& decendl (std::ostream& o);
}

#endif //! ROBOPTIM_CORE_INDENT_HH
