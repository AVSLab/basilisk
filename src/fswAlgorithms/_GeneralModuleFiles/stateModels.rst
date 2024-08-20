Executive Summary
-----------------

This class provides an container for all the state models to a kalman filter.
This class is a necessary component to any Kalman Filter implementation

Virtual and Private method descriptors
-------------------------------
The following table lists all the state class

.. list-table:: Interface methods which remain private
    :widths: 25 75
    :header-rows: 1

    * - Method Name
      - Method Function
    * - get/setValues
      - return the values of a given state

Position, Velocity, Acceleration, Bias, and ConsiderParameters all inherit the state type.
The following table lists all the stateVector class, which contains all the different state types and a stm matrix

.. list-table:: Interface methods which remain private
    :widths: 25 75
    :header-rows: 1

    * - Method Name
      - Method Function
    * - size
      - return the total size of all the states
    * - add
      - add state vectors
    * - addVector
      - add Eigen::Vector values to the state vector in order Position, Velocity, Accel, Bias, Consider
    * - scale
      - scale a stateVector by a scalar
    * - returnValues
      - return all fo the values in an Eigen::Vector following the previous order


Module assumptions and limitations
-------------------------------

The states are strongly typed, but the covariance is not (yet).
When returning values, the order of the states is Position, Velocity, Accel, Bias, Consider

User Guide
----------

This section lists all the setters and getters that are defined by the interface

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
    * - get/setPosition
      - the position state class
    * - get/setPositionVector
      - the values of the position state class
    * - hasPosition
      - does the state vector contain a position
    * - get/setVelocity
      - the velocity state class
    * - get/setVelocityVector
      - the values of the velocity state class
    * - hasVelocity
      - does the state vector contain a velocity
    * - get/setAcceleration
      - the acceleration state class
    * - get/setAccelerationVector
      - the values of the acceleration state class
    * - hasAcceleration
      - does the state vector contain a acceleration
    * - get/setBias
      - the bias state class
    * - get/setBiasVector
      - the values of the bias state class
    * - hasBias
      - does the state vector contain a bias
    * - get/setConsider
      - the consider parameter state class
    * - get/setConsiderVector
      - the values of the consider parameter state class
    * - hasConsider
      - does the state vector contain a consider parameter
