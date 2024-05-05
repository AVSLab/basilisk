Executive Summary
-----------------

This class provides an interface for Extended (and Classical) Kalman Filters. Modules which inherit from it will
benefit from having the core functionality of these basic filters.

The math is derived primarily using the equations in Tapley, Schutz, and Born's Statistical Orbit Determination textbook
Chapter 4. This contains further information on this module's mathematical framework, and the flow diagrams describing
the algorithm implementations.

Virtual and Private method descriptors
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Interface methods which remain private
    :widths: 25 75 25 50
    :header-rows: 1

    * - Method Name
      - Method Function
      - Protected or Private
      - Virtual or not
    * - timeUpdate
      - filter time update
      - private
      - non-virtual
    * - measurementUpdate
      - filter measurement update in the presence of measurements
      - private
      - non-virtual
    * - computeResiduals
      - compute filter post fit residuals
      - private
      - non-virtual
    * - computeKalmanGain
      - compute the Kalman
      - private
      - non-virtual
    * - updateCovariance
      - measurement update for the covariance
      - private
      - non-virtual
    * - ckfUpdate
      - update using the classical kalman filter
      - private
      - non-virtual
    * - ekfUpdate
      - update using the extended kalman filter
      - private
      - non-virtual
    * - orderMeasurementsChronologically
      - order the messages chronologically
      - private
      - non-virtual
    * - customReset
      - perform addition reset duties in child class
      - protected
      - virtual, optional
    * - propagate
      - add a dynamics model for the inputs in child class
      - protected
      - virtual, needs to be populated
    * - readFilterMeasurements
      - read the specific measurements in a child class
      - protected
      - virtual, needs to be populated
    * - writeOutputMessages
      - write the specific measurements in a child class
      - protected
      - virtual, needs to be populated
    * - rk4
      - Runge-Kutta 4 integrator if needed for propagation
      - protected
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

When instantiating this filter, the user must use the FilterType enum class to chose whether this is a
Linear (Classical) Kalman Filter or an Extended Kalman Filter. The Extended Kalman Filter updates the reference
state at each measurement update.

This section also lists all the setters and getters that are defined by the interface.

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
      - Necessity
    * - set/getMinimumCovarianceNormForEkf
      - if using an EKF, set the covariance infinite norm that will switch from CKF updates to EKFs
      - optional
    * - set/getConstantRateStates
      - if using constant rates (not estimated by the filter) that integrate the state linearly, set this rate component
      - necessary
    * - set/getInitialState
      - filter initial state
      - necessary
    * - set/getInitialCovariance
      - filter initial covariance (square matrix size of state)
      - necessary
    * - set/getProcessNoise
      - filter process noise value (square matrix size of state)
      - necessary
    * - set/getConstantMeasurementNoise
      - filter measurement noise (square matrix size of observations)
      - optional
    * - set/getUnitConversionFromSItoState
      - conversion from SI to internal filter units
      - optional
