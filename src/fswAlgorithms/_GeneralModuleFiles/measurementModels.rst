Executive Summary
-----------------

This class provides an container for all the measurement models and components to a kalman filter.
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
    * - model
      - compute measurement model give a state
      - public
      - necessary
    * - computeMeasurementMatrix
      - compute measurement matrix give a state
      - public
      - necessary

Module assumptions and limitations
-------------------------------

None

User Guide
----------

This section lists all the setters and getters that are defined by the interface

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
      - Necessity
    * - get/setMeasurementName
      - set the name of the measurement to distinguish it from others
      - necessary
    * - get/setTimeTag
      - set the measurement time tag
      - necessary
    * - get/setValidity
      - set the measurement validity
      - necessary
    * - get/setObservation
      - set the measurement observation value
      - necessary
    * - get/setMeasurementNoise
      - set the measurement noise matrix
      - necessary
    * - get/setPreFitResiduals
      - set the measurement pre fit residuals
      - necessary
    * - get/setPostFitResiduals
      - set the measurement pre fit residuals
      - necessary

.. list-table:: Static methods for common measurement models
    :widths: 25 75
    :header-rows: 1

    * - Method Name
      - Method Function
    * - positionStates
      - measurement of position state
    * - normalizedPositionStates
      - measurement of heading direction
    * - mrpStates
      - measurement of position state with a shadow set check
    * - velocityStates
      - measurement of velocity states
