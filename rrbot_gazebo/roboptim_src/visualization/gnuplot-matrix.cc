// Copyright (C) 2013 by Benjamin Chretien, AIST, CNRS.
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

#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-matrix.hh>

# include <boost/format.hpp>
# include <boost/lexical_cast.hpp>

namespace roboptim
{
  namespace visualization
  {
    namespace gnuplot
    {
      namespace detail
      {

        template <typename T>
        void set_matrix_header
        (std::string& str,
         typename GenericFunctionTraits<T>::const_matrix_ref mat)
        {
          // White = 0, Blue = non zero
          str += "set palette defined(0 \"white\",1 \"blue\")\n";
          str += "set grid front\n";

          // Matrix (x,y) range
          str += "set xrange [0:" + boost::lexical_cast<std::string>
            ((float)mat.cols()) + "]\n";
          str += "set yrange [0:" + boost::lexical_cast<std::string>
            ((float)mat.rows()) + "] reverse\n";
          str += "set size ratio -1\n";

          // Remove the colorbox
          str += "unset colorbox\n";

          // Matrix plotting
          // (range offset since pixels are centered on integer coordinates)
          str += "plot '-' using ($1+0.5):($2+0.5):($3 == 0 ? 0 : 1) ";
          str += "matrix with image notitle\n";
        }

        std::string dense_matrix_to_gnuplot
        (GenericFunctionTraits<EigenMatrixDense>::const_matrix_ref mat)
        {
          typedef GenericFunctionTraits<EigenMatrixDense>::matrix_t matrix_t;

          std::string str = "";

          // Set the header of the Gnuplot output (title, range, etc.)
          set_matrix_header<EigenMatrixDense>(str, mat);

          for (matrix_t::Index cstr_id = 0;
               cstr_id < mat.rows (); ++cstr_id)
            for (matrix_t::Index out_id = 0;
                 out_id < mat.cols (); ++out_id)
              {
                str += (boost::format("%2.8f")
                        % normalize (mat (cstr_id, out_id))).str();

                if (out_id < mat.cols() - 1) str += " ";
                else str += "\n";
              }
          str += "e\n";
          str += "e\n";
          return str;
        }


        std::string sparse_matrix_to_gnuplot
        (GenericFunctionTraits<EigenMatrixSparse>::const_matrix_ref mat)
        {
          typedef GenericFunctionTraits<EigenMatrixDense>::matrix_t denseMatrix_t;

          return dense_matrix_to_gnuplot (denseMatrix_t (mat));
        }

      } // end of namespace detail.


      Command plot_mat
      (GenericFunctionTraits<EigenMatrixDense>::const_matrix_ref mat)
      {
        return Command (detail::dense_matrix_to_gnuplot (mat));
      }


      Command plot_mat
      (GenericFunctionTraits<EigenMatrixSparse>::const_matrix_ref mat)
      {
        return Command (detail::sparse_matrix_to_gnuplot (mat));
      }

    } // end of namespace gnuplot.
  } // end of namespace visualization.
} // end of namespace roboptim
