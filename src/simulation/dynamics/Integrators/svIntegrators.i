/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module svIntegrators

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();


%{
   #include <vector>
   #include "../_GeneralModuleFiles/stateVecIntegrator.h"
   #include "../_GeneralModuleFiles/svIntegratorRungeKutta.h"
   #include "../_GeneralModuleFiles/svIntegratorRK4.h"
   #include "svIntegratorEuler.h"
   #include "svIntegratorRK2.h"
   #include "svIntegratorRKF45.h"
   #include "svIntegratorRKF78.h"
   #include "architecture/_GeneralModuleFiles/sys_model.h"
   #include "../_GeneralModuleFiles/dynamicObject.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *

# The following maps store the RK base classes w.r.t their stage number
_rk_base_classes = {}
_rk_adaptive_base_classes = {}
%}

%include <std_vector.i>
%template() std::vector<double>;
%template() std::vector<std::vector<double>>;

%typemap(out) std::optional<double> {
    if ($1.has_value()) {
        $result = PyFloat_FromDouble($1.value());
    } else {
        $result = Py_None;
        Py_INCREF($result);
    }
}

%include "sys_model.i"
%include "../_GeneralModuleFiles/stateVecIntegrator.h"

%include "../_GeneralModuleFiles/svIntegratorRungeKutta.h"
%include "../_GeneralModuleFiles/svIntegratorAdaptiveRungeKutta.h"

// We add a constructor for svIntegratorRungeKutta and svIntegratorAdaptiveRungeKutta
// These are useful for us to build these classes on the Python side without having
// to access the RKCoefficients or RKAdaptiveCoefficients structs.
%extend svIntegratorRungeKutta {
   svIntegratorRungeKutta(
      DynamicObject* dynIn,
      std::vector<std::vector<double>> aMatrix,
      std::vector<double> bArray,
      std::vector<double> cArray
   )
   {
      RKCoefficients<numberStages> coefficients;
      for (size_t i = 0; i < numberStages; i++)
      {
          std::copy_n(aMatrix.at(i).begin(), numberStages, coefficients.aMatrix.at(i).begin());
      }
      std::copy_n(bArray.begin(), numberStages, coefficients.bArray.begin());
      std::copy_n(cArray.begin(), numberStages, coefficients.cArray.begin());

      return new svIntegratorRungeKutta<numberStages>(dynIn, coefficients);
  }
}

%extend svIntegratorAdaptiveRungeKutta {
   svIntegratorAdaptiveRungeKutta(
      DynamicObject* dynIn,
      std::vector<std::vector<double>> aMatrix,
      std::vector<double> bArray,
      std::vector<double> bStarArray,
      std::vector<double> cArray,
      double largestOrder
   )
   {
      RKAdaptiveCoefficients<numberStages> coefficients;
      for (size_t i = 0; i < numberStages; i++)
      {
          std::copy_n(aMatrix.at(i).begin(), numberStages, coefficients.aMatrix.at(i).begin());
      }
      std::copy_n(bArray.begin(), numberStages, coefficients.bArray.begin());
      std::copy_n(bStarArray.begin(), numberStages, coefficients.bStarArray.begin());
      std::copy_n(cArray.begin(), numberStages, coefficients.cArray.begin());

      return new svIntegratorAdaptiveRungeKutta<numberStages>(dynIn, coefficients, largestOrder);
  }
}

%define TEMPLATE_HELPER(stageNumber)
%template(svIntegratorRungeKutta ## stageNumber) svIntegratorRungeKutta< ## stageNumber>;
%template(svIntegratorAdaptiveRungeKutta ## stageNumber) svIntegratorAdaptiveRungeKutta< ## stageNumber>;

%pythoncode %{
_rk_base_classes[## stageNumber] = svIntegratorRungeKutta ## stageNumber
_rk_adaptive_base_classes[## stageNumber] = svIntegratorAdaptiveRungeKutta ## stageNumber
%}
%enddef

TEMPLATE_HELPER(1)
TEMPLATE_HELPER(2)
TEMPLATE_HELPER(3)
TEMPLATE_HELPER(4)
TEMPLATE_HELPER(6)
TEMPLATE_HELPER(7)
TEMPLATE_HELPER(9)
TEMPLATE_HELPER(13)

%include "../_GeneralModuleFiles/svIntegratorRK4.h"

%include "svIntegratorEuler.h"
%include "svIntegratorRK2.h"
%include "svIntegratorRKF45.h"
%include "svIntegratorRKF78.h"

// The following methods allow users to create new Runge-Kutta
// methods simply by providing their coefficients on the Python side
%pythoncode %{
from typing import Sequence, Union
import numpy as np

def _validate_coefficients(a_coefficients, **array_coefficients):
    try:
        a_coefficients = np.array(a_coefficients, dtype=float)
        assert a_coefficients.ndim == 2
    except:
        raise ValueError("a_coefficients must be a square matrix or a non-ragged sequence of sequences")

    if a_coefficients.shape[0] != a_coefficients.shape[1]:
        raise ValueError("a_coefficients must be a square matrix")

    stages = a_coefficients.shape[0]
    for input_name, array_coefficient in array_coefficients.items():
        if stages != len(array_coefficient):
            raise ValueError(
                f"The size of a_coefficients is {stages}x{stages}, "
                f"but the size of {input_name} is {len(array_coefficient)}. "
                 "These must be consistent")

def svIntegratorRungeKutta(
    dynamic_object,
    a_coefficients: Union[Sequence[Sequence[float]], np.ndarray],
    b_coefficients: Union[Sequence[float], np.ndarray],
    c_coefficients: Union[Sequence[float], np.ndarray],
    ) -> StateVecIntegrator:
    """Generates an explicit, fixed-step Runge-Kutta integrator
    from the given coefficients.

    The coefficients for an RK method are normally expressed
    in a Butcher table, with the `a_coefficients` being a lower
    triangular square matrix, the `b_coefficients` being the
    bottom of the table, and the `c_coefficients` being on the
    left side.

    Args:
        a_coefficients (Sequence[Sequence[float]] | np.ndarray):
            "a" matrix in the Butcher table.
        b_coefficients (Sequence[float] | np.ndarray):
            "b" array in the Butcher table
        c_coefficients (Sequence[float] | np.ndarray):
            "c" array in the Butcher table

    Returns:
        StateVecIntegrator: A Runge-Kutta integrator object
    """
    _validate_coefficients(a_coefficients, b_coefficients=b_coefficients, c_coefficients=c_coefficients)
    stages = len(b_coefficients)

    return _rk_base_classes[stages](dynamic_object, a_coefficients, b_coefficients, c_coefficients)

def svIntegratorAdaptiveRungeKutta(
    dynamic_object,
    largest_order: float,
    a_coefficients: Union[Sequence[Sequence[float]], np.ndarray],
    b_coefficients: Union[Sequence[float], np.ndarray],
    b_star_coefficients: Union[Sequence[float], np.ndarray],
    c_coefficients: Union[Sequence[float], np.ndarray],
    ) -> StateVecIntegrator:
    """Generates an explicit, adaptive Runge-Kutta integrator
    from the given coefficients.

    The coefficients for an RK method are normally expressed
    in a Butcher table, with the `a_coefficients` being a lower
    triangular square matrix, the `b_coefficients` being the
    bottom of the table, and the `c_coefficients` being on the
    left side. Finally, `b_star_coefficients` are also usually
    represented at the bottom of the table. These coefficients
    represent those of the lower order RK method.

    Args:
        largest_order (float): The order of the higher-order RK method
            used in this adaptive RK method. For example, for RKF45,
            largest_order should be 5.
        a_coefficients (Sequence[Sequence[float]] | np.ndarray):
            "a" matrix in the Butcher table.
        b_coefficients (Sequence[float] | np.ndarray):
            "b" array in the Butcher table
        b_star_coefficients (Sequence[float] | np.ndarray):
            "b" array for the lower order method in the Butcher table
        c_coefficients (Sequence[float] | np.ndarray):
            "c" array in the Butcher table

    Returns:
        StateVecIntegrator: A Runge-Kutta integrator object
    """
    _validate_coefficients(
      a_coefficients, b_coefficients=b_coefficients,
      b_star_coefficients=b_star_coefficients, c_coefficients=c_coefficients)
    stages = len(b_coefficients)

    return _rk_adaptive_base_classes[stages](dynamic_object, a_coefficients, b_coefficients, b_star_coefficients, c_coefficients, largest_order)

import sys
protectAllClasses(sys.modules[__name__])
%}
