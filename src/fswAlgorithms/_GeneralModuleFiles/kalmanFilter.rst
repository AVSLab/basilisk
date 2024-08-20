Executive Summary
-----------------

This virtual class provides an interface for all Kalman Filters. Modules which inherit from it will
benefit from having the core logic of a sequential filter

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
    * - UpdateState
      - Loop through measurements and update filter states according to time
      - public
      - final
    * - timeUpdate
      - filter time update
      - protected
      - virtual
    * - measurementUpdate
      - filter measurement update in the presence of measurements
      - protected
      - virtual
    * - computeResiduals
      - compute filter post fit residuals
      - protected
      - virtual
    * - customReset
      - perform addition reset duties in child class
      - protected
      - virtual, optional
    * - readFilterMeasurements
      - read the specific measurements in a child class
      - protected
      - virtual, needs to be populated
    * - writeOutputMessages
      - write the specific measurements in a child class
      - protected
      - virtual, needs to be populated
    * - orderMeasurementsChronologically
      - Runge-Kutta 4 integrator if needed for propagation
      - private
      - non-virtual


Module assumptions and limitations
-------------------------------

The module inherits all assumptions made while implementing a Kalman filter:
    • Observability considerations
    • Gaussian covariances
    • Linearity limits
    • and more

Otherwise, the limitations are governed by the measurement model and dynamics models relative
to the required performance.

User Guide
----------

This section lists all the setters and getters that are defined by the interface

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
      - Necessity
    * - set/getInitialPosition
      - filter initial position component of the states
      - optional
    * - set/getInitialVelocity
      - filter initial velocity component of the states
      - optional
    * - set/getInitialAcceleration
      - filter initial acceleration component of the states
      - optional
    * - set/getInitialBias
      - filter initial bias component of the states
      - optional
    * - set/getInitialConsider
      - filter initial consider component of the states
      - optional
    * - set/getInitialCovariance
      - filter initial covariance (square matrix size of state)
      - necessary
    * - set/getProcessNoise
      - filter process noise value (square matrix size of state)
      - necessary
    * - set/getUnitConversionFromSItoState
      - conversion from SI to internal filter units
      - optional
    * - set/getMeasurementNoiseScale
      - filter measurement noise scale factor
      - optional
    * - setFilterDynamics
      - provide the dynamics class to the filter
      - necessary

    
