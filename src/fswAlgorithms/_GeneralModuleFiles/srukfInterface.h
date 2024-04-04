/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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
#ifndef SRUKF_INTERFACE_HPP
#define SRUKF_INTERFACE_HPP

#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <array>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"

/*! @brief Container class for measurement data and models */
class Measurement{
public:
    Measurement() = default;

    std::string name = ""; //!< [-] name of measurement  type
    size_t size = 0; //!< [-] size of observation vector
    double timeTag = 0; //!< [-] Observation time tag
    bool validity = false; //!< [-] Observation validity
    Eigen::VectorXd observation; //!< [-] Observation data vector
    Eigen::MatrixXd noise; //!< [-] Constant measurement Noise
    Eigen::MatrixXd choleskyNoise; //!< [-] cholesky of Qnoise
    Eigen::VectorXd postFitResiduals; //!< [-] Observation post fit residuals
    Eigen::VectorXd preFitResiduals; //!< [-] Observation pre fit residuals
     /*! Each measurement must be paired with a measurement model as a function which inputs the
     * sigma point matrix and outputs the modeled measurement for each sigma point */
    std::function<const Eigen::MatrixXd(const Eigen::MatrixXd)> model; //!< [-] observation measurement model
};


/*! @brief Square Root unscented Kalman Filter base class */
class SRukfInterface: public SysModel  {
public:
    SRukfInterface();
    ~SRukfInterface() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    void setAlpha(const double alpha);
    double getAlpha() const;
    void setBeta(const double beta);
    double getBeta() const;
    void setInitialState(const Eigen::VectorXd initialState);
    Eigen::VectorXd getInitialState() const;
    void setInitialCovariance(const Eigen::MatrixXd initialCovariance);
    Eigen::MatrixXd getInitialCovariance() const;
    void setProcessNoise(const Eigen::MatrixXd processNoise);
    Eigen::MatrixXd getProcessNoise() const;
    void setUnitConversionFromSItoState(const double conversion);
    double getUnitConversionFromSItoState() const ;

protected:
    virtual void customReset(){/* virtual */};
    virtual void customInitializeUpdate(){/* virtual */};
    virtual void customFinalizeUpdate(){/* virtual */};
    virtual Eigen::VectorXd propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt)=0;
    /*! Read method neads to read incoming messages containing the measurements for the filter.
     * Their information must be added to the Measurement container class, and added to the measurements vector.
     * Each measurement must be paired with a measurement model provided in the measurementModels.h */
    virtual void readFilterMeasurements()=0;
    virtual void writeOutputMessages(uint64_t CurrentSimNanos)=0;

    Eigen::VectorXd rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                const Eigen::VectorXd& X0,
                double t0,
                double dt) const;

    std::array<std::optional<Measurement>, MAX_MEASUREMENT_DEFAULT> measurements;  //!< [Measurements] All measurement containers in chronological order
    double previousFilterTimeTag = 0; //!< [s]  Time tag for statecovar/etc
    double unitConversion = 1; //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths

    size_t numberSigmaPoints=0; //!< [s]  2n+1 sigma points for convenience
    Eigen::VectorXd state; //!< [-] State estimate for time TimeTag
    Eigen::MatrixXd sBar; //!< [-] Time updated covariance
    Eigen::MatrixXd covar; //!< [-] covariance
    Eigen::VectorXd xBar; //!< [-] Current mean state estimate
    Eigen::MatrixXd sigmaPoints; //!< [-]    sigma point matrix

    Eigen::MatrixXd yMeas; //!< [-] Measurement model data
    Eigen::VectorXd postFits; //!< [-] PostFit residuals
    Eigen::MatrixXd cholProcessNoise; //!< [-] cholesky of Qnoise

private:
    void timeUpdate(const double updateTime);
    void measurementUpdate(const Measurement &measurement);
    Eigen::VectorXd computeResiduals(const Measurement &measurement);
    Eigen::MatrixXd qrDecompositionJustR(const Eigen::MatrixXd input) const;
    Eigen::MatrixXd choleskyUpDownDate(const Eigen::MatrixXd input,
                                       const Eigen::VectorXd inputVector,
                                       const double coefficient) const;
    Eigen::MatrixXd backSubstitution(const Eigen::MatrixXd U, const Eigen::MatrixXd b) const;
    Eigen::MatrixXd forwardSubstitution(const Eigen::MatrixXd L, const Eigen::MatrixXd b) const;
    Eigen::MatrixXd choleskyDecomposition(const Eigen::MatrixXd input) const;
    void orderMeasurementsChronologically();

    double beta=0;
    double alpha=0;
    double lambda=0;
    double eta=0;

    Eigen::VectorXd wM;
    Eigen::VectorXd wC;

    Eigen::MatrixXd processNoise; //!< [-] process noise matrix
    Eigen::VectorXd stateInitial; //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd sBarInitial; //!< [-] Time updated covariance at previous time
    Eigen::MatrixXd covarInitial; //!< [-] covariance at previous time
};

#endif /* SRUKF_INTERFACE_HPP */
