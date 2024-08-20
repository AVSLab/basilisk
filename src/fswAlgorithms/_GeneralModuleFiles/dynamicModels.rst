Executive Summary
-----------------

This class provides an container for all the dynamics models and components to a kalman filter.
This class is a necessary component to any Kalman Filter implementation

Virtual and Private method descriptors
-------------------------------
The following table lists all the class methods and their function

.. list-table:: Interface methods which remain private
    :widths: 25 75 25 50
    :header-rows: 1

    * - Method Name
      - Method Function
      - Protected or Private
      - Virtual or not
    * - rk4
      - RK4 integrator
      - private
      - non-virtual
    * - propagate
      - integrate the equations of motion provided in setter
      - public
      - non-virtual
    * - computeDynamicsMatrix
      - compute the dynamics matrix given the partials provided in setter
      - public
      - non-virtual


Module assumptions and limitations
-------------------------------

Only an RK4 is provided currently as an integrator

User Guide
----------

This section lists all the setters and getters that are defined by the interface

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
      - Necessity
    * - setDynamics
      - set the equations of motion function
      - necessary
    * - setDynamicsMatrix
      - set the dynamics matrix given a state
      - necessary for extended kalman filters
