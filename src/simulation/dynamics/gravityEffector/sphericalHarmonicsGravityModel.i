/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

%module(package="Basilisk.simulation") sphericalHarmonicsGravityModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "simulation/dynamics/gravityEffector/sphericalHarmonicsGravityModel.h"
   #include <memory>
%}


%include "swig_eigen.i"

%import "simulation/dynamics/_GeneralModuleFiles/gravityModel.i"

%include "std_vector.i"
%template() std::vector<double>;
%template() std::vector<std::vector<double>>;

%include <std_shared_ptr.i>
%shared_ptr(SphericalHarmonicsGravityModel)

// Declaring cBar and sBar as %naturalvar allows setting them without the MultiArray wrapper:
//    mySphericalHarmonics.cBar = [[1.0], [0.0, 1.0]]
// However, it prevents modifying them in place. The following is illegal:
//    mySphericalHarmonics.cBar[0][0] = 2
// This is because the variables are returned by value, instead of by reference.
%naturalvar SphericalHarmonicsGravityModel::cBar;
%naturalvar SphericalHarmonicsGravityModel::sBar;

%include "simulation/dynamics/gravityEffector/sphericalHarmonicsGravityModel.h"

%extend SphericalHarmonicsGravityModel {
   %pythoncode %{
      def loadFromFile(self, fileName: str, maxDeg: int):
          """Loads the C and S coefficients from a file."""
          from Basilisk.simulation.gravityEffector import loadGravFromFile
          loadGravFromFile(fileName, self, maxDeg)
          return self
   %}
}
