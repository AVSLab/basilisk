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

%module(package="Basilisk.simulation") polyhedralGravityModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "simulation/dynamics/gravityEffector/polyhedralGravityModel.h"
   #include <memory>
%}

%include "swig_eigen.i"

%import "simulation/dynamics/_GeneralModuleFiles/gravityModel.i"

%include <std_shared_ptr.i>
%shared_ptr(PolyhedralGravityModel)

%include "simulation/dynamics/gravityEffector/polyhedralGravityModel.h"

%extend PolyhedralGravityModel {
   %pythoncode %{
      def loadFromFile(self, fileName: str):
          """Loads the vertices and facet data from the given file."""
          from Basilisk.simulation.gravityEffector import loadPolyFromFile
          loadPolyFromFile(fileName, self)
          return self
   %}
}
