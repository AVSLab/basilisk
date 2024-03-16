Executive Summary
-----------------

This class provides an interface for square root unscented Kalman Filters. Modules which inherit from it will
benefit from having the core functionality of the SRuKF

The math is derived primarily using the following equations:
:download:`PDF Description </../../src/fswAlgorithms/opticalNavigation/relativeODuKF/_Documentation/inertialUKF_DesignBasis.pdf>`
contains further information on this module's mathematical framework.

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
    * - computePostFitResiudals
      - compute filter post fit residuals
      - private
      - non-virtual
    * - qrDecompositionJustR
      - perform a QR decomposition and return just the R matrix
      - private
      - non-virtual
    * - choleskyUpDownDate
      - perform a cholesky down-date as seen in the paper
      - private
      - non-virtual
    * - backSubstitution
      - perform a back-substitution on a upper triangular matrix
      - private
      - non-virtual
    * - forwardSubstitution
      - perform a forward-substitution on a lower triangular matrix
      - private
      - non-virtual
    * - choleskyDecomposition
      - perform a cholesky decomposition
      - private
      - non-virtual
    * - customReset
      - perform addition reset duties in child class
      - protected
      - virtual, optional
    * - measurementModel
      - add a measurement model for the inputs in child class
      - protected
      - virtual, needs to be populated
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

This section lists all the setters and getters that are defined by the interface

.. list-table:: Interface methods which remain private
    :widths: 25 75 25
    :header-rows: 1

    * - Method Name
      - Method Function
      - Necessity
    * - set/getAlpha
      - filter alpha parameter
      - necessary
    * - set/getBeta
      - filter beta parameter
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

    
