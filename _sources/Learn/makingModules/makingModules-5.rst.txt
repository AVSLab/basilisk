.. _makingModules-5:

Common Basilisk Support Files
=============================

The folder ``src/architecture/utilities`` contains a number of C code support libraries that simplify the mathematics of writing Basilisk modules.  This page highlights some common libraries used.


`astroConstants.h`
    :ref:`astroConstants` contains a range of orbital mechanics related variable definition.


`avsEigenMRP.h`
    :ref:`avsEigenMRP` provides MRP capabilities to the `Intel Eigen library <https://eigen.tuxfamily.org/>`__.

`avsEigenSupport.h`
    :ref:`avsEigenSupport` provides a range of helper function to convert between C array variables and `Intel Eigen library <https://eigen.tuxfamily.org/>`__ vectors and matrices.

`discretize.h`
    :ref:`discretize` provides functions to discretize a real number.

`gauss_markov.h`
    :ref:`gauss_markov` provides functions to apply a second-order bounded Gauss-Markov random walk on top of an upper level process.

`geodeticConversion.h`
    :ref:`geodeticConversion` provides a collection of functions to convert to and from planet centered frames.

`keplerianOrbit.h`
    :ref:`keplerianOrbit` class represents an elliptical orbit and provides a coherent set of common outputs such as position and velocity, orbital period, semi-parameter, etc. It uses the utility orbitalMotion to do orbital element to position and velocity conversion.

`linearAlgebra.h`
    :ref:`linearAlgebra` provides a collection of functions perfrom 2D, 3D, 4D and N-dimensional matrix math in C

`macroDefinitions.h`
    :ref:`macroDefinitions` provides a collection of convenient macro definitions

`orbitalMotion.h`
    :ref:`orbitalMotion` provides a collection of orbital mechanics related function.

`rigidBodyKinematics.h`
    :ref:`rigidBodyKinematics` provides a collection of rigid body kinematics transformations.  This includes functions to map between a range of attitude coordinates.

`saturate.h`
    :ref:`saturate` is used to saturate an output variable

`signalCondition.h`
    :ref:`signalCondition` provides provides a low-pass filter to an output variable

`simDefinitions.h`
    :ref:`simDefinitions` provides common simulation related definition such as default epoch states, etc.
