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

%module(package="Basilisk.simulation") gravityEffector
%{
   #include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *

from Basilisk.simulation.pointMassGravityModel import PointMassGravityModel
from Basilisk.simulation.polyhedralGravityModel import PolyhedralGravityModel
from Basilisk.simulation.sphericalHarmonicsGravityModel import SphericalHarmonicsGravityModel

Polyhedral = PolyhedralGravityModel
SphericalHarmonics = SphericalHarmonicsGravityModel

from typing import Optional, Union, overload

%}

%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

%include <std_shared_ptr.i>
%shared_ptr(GravBodyData)

%include "std_vector.i"
%template(GravBodyVector) std::vector<std::shared_ptr<GravBodyData>>;

// The central body should be changed by changing the isCentralBody flag
// in GravBodyData
%immutable GravityEffector::centralBody;
%immutable GravityEffector::vehicleGravityPropName;
%immutable GravityEffector::systemTimeCorrPropName;
%immutable GravityEffector::inertialPositionPropName;
%immutable GravityEffector::inertialVelocityPropName;
%immutable GravityEffector::nameOfSpacecraftAttachedTo;

// Methods that users do not need / should not be calling
%ignore GravityEffector::updateInertialPosAndVel;
%ignore GravityEffector::updateEnergyContributions;
%ignore GravityEffector::prependSpacecraftNameToStates;

%pythonappend GravBodyData::GravBodyData() %{
    object.__setattr__(self, "_pyGravityModel", None) # Enable setting _pyGravityModel
    self.gravityModel = PointMassGravityModel() # Re-set gravityModel to populate the _pyGravityModel%}

%import "simulation/dynamics/gravityEffector/gravityModel.i"

%include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
%include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%include "sys_model.h"
%include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsg_C;

%extend GravBodyData {
    %pythoncode %{
    
    """
    If we were to call GravBodyData::gravityModel we would obtain a pointer to the
    parent object GravityModel, as this is what is stored in the GravBodyData C++
    class (the concrete type is "lost"). To overcome this, we store a copy of the
    set object in _pyGravityModel and use the gravityModel property to keep the
    Python and C++ objects synchronized. _pyGravityModel does retain the concrete
    type (PointMassGravityModel, SphericalHarmonicsGravityModel...)
    """
    _gravityModel = gravityModel
    @property
    def gravityModel(self):
        return self._pyGravityModel
    
    @gravityModel.setter
    def gravityModel(self, value):
        self._gravityModel = value
        self._pyGravityModel = value

    @property
    def useSphericalHarmParams(self):
        return isinstance(self.gravityModel, SphericalHarmonicsGravityModel)
   
    @useSphericalHarmParams.setter
    def useSphericalHarmParams(self, value: bool):
        if self.useSphericalHarmParams and not value:
            self.gravityModel = PointMassGravityModel()
        elif not self.useSphericalHarmParams and value:
            self.gravityModel = SphericalHarmonicsGravityModel()

    @property
    def usePolyhedral(self):
        return isinstance(self.gravityModel, PolyhedralGravityModel)
   
    @usePolyhedral.setter
    def usePolyhedral(self, value: bool):
        if self.usePolyhedral and not value:
            self.gravityModel = PointMassGravityModel()
        elif not self.usePolyhedral and value:
            self.gravityModel = PolyhedralGravityModel()

    @property
    def spherHarm(self) -> SphericalHarmonicsGravityModel:
        if self.useSphericalHarmParams:
            return self.gravityModel
        else:
            raise ValueError("GravBodyData is not using spherical harmonics as a gravity model. "
                "Call 'useSphericalHarmonicsGravityModel(...)' or set 'useSphericalHarmParams' to 'True' before retrieving 'spherHarm'.")

    @spherHarm.setter
    def spherHarm(self, value: SphericalHarmonicsGravityModel):
        self.gravityModel = value
   
    @property
    def poly(self) -> PolyhedralGravityModel:
        if self.usePolyhedral:
            return self.gravityModel
        else:
            raise ValueError("GravBodyData is not using the polyhedral gravity model. "
                "Call 'usePolyhedralGravityModel(...)' or set 'usePolyhedral' to 'True' before retrieving 'poly'.")

    @poly.setter
    def poly(self, value: PolyhedralGravityModel):
        self.gravityModel = value

    def usePointMassGravityModel(self):
        self.gravityModel = PointMassGravityModel()

    @overload
    def useSphericalHarmonicsGravityModel(self, maxDeg: int):
        """Makes the GravBodyData use Spherical Harmonics as its gravity model.

        Uses the default spherical harmonics data file for this body. This is
        only available for some select bodies, such as the Earth and Mars. 

        Args:
            maxDeg (int): The maximum degree to use in the spherical harmonics.
        """

    @overload
    def useSphericalHarmonicsGravityModel(self, file: str, maxDeg: int):
        """Makes the GravBodyData use Spherical Harmonics as its gravity model.

        Args:
            file (str): The file that contains the spherical harmonics data in the
                JPL format.
            maxDeg (int): The maximum degree to use in the spherical harmonics.
        """

    def useSphericalHarmonicsGravityModel(self, *args, **kwargs):
        if len(args) == 1 and len(kwargs) == 0:
            maxDeg = args[0]
            file = None
        elif len(args) == 0 and "maxDeg" in kwargs:
            maxDeg = kwargs["maxDeg"]
            file = None
        elif len(args) == 1 and "maxDeg" in kwargs:
            file = args[0]
            maxDeg = kwargs["maxDeg"]
        elif len(args) == 2:
            file, maxDeg = args
        elif "file" in kwargs and "maxDeg" in kwargs:
            file = kwargs["file"]
            maxDeg = kwargs["maxDeg"]
        else:
            raise TypeError("useSphericalHarmonicsGravityModel expects 1 or 2 arguments: (maxDeg: int) or (file: str, maxDeg: int)")

        if file is None:
            file = getattr(self, "_defaultSphericalHarmonicsFile", None)
        
        if file is None:
            raise TypeError("If no 'file' argument is set, the body must have a default spherical harmonics file (which only some bodies have, like Earth and Mars)")
            
        self.gravityModel = SphericalHarmonicsGravityModel().loadFromFile(file, maxDeg)

    def usePolyhedralGravityModel(self, file: str):
        """Makes the GravBodyData use the Polyhedral gravity model.

        Args:
            file (str): The file that contains the vertices and facet
                data for the polyhedral.
        """
        self.gravityModel = PolyhedralGravityModel().loadFromFile(file)
        
    %}
}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])        
%}

%pythoncode "simulation/dynamics/gravityEffector/gravCoeffOps.py"

