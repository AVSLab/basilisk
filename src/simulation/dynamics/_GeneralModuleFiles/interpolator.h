/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <stdexcept>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"

/** A class that interpolates from a given table of points and
 * outputs the result as a message payload.
 *
 * The table is given in `setDataPoints` as a matrix, where the
 * first column represents the simulation time (the independent
 * variable), and the rest of columns are the data to be interpolated.
 *
 * The result of the interpolation will be a vector of floats.
 * This vector is then taken by the virtual method `setPayloadValues`
 * to fill in the payload as needed.
 *
 * Interpolation through splines of configurable degrees. Zero degrees
 * is piece-wise constant between data points, one degree is linear
 * interpolation, two degrees is quadratic interpolation, three degrees
 * is cubic interpolation... Note that cubic interpolation guarantees
 * continous first and second derivatives, which might be useful for
 * certain applications.
 */
template <typename T, size_t nArgs = 1> class Interpolator : public SysModel
{
public:
    /** Set the points to use for interpolation. The first column corresponds
     * to the independent value, the simulation time in nanoseconds, while the
     * rest of columns are the data to use for interpolation.
     *
     * @param points A matrix containing data points where the first column represents
     *               the independent values and subsequent columns represent the values
     *               to interpolate.
     * @param degrees The degree of the spline to fit (default is 1).
     */
    void setDataPoints(const Eigen::MatrixXd& points, uint16_t degrees = 1)
    {
        if (points.cols() != (nArgs + 1)) {
            auto error =
                "Expected points to have exactly " + std::to_string(nArgs + 1) + " columns.";
            this->bskLogger.bskLog(BSK_ERROR, error.c_str());
            throw std::invalid_argument(error);
        }

        if (points.rows() <= int(degrees)) {
            auto error =
                "Provided 'degrees' is higher or equal than number of data points (illegal for "
                "interpolation).";
            this->bskLogger.bskLog(BSK_ERROR, error);
            throw std::invalid_argument(error);
        }

        /* A zero-degree spline is constant between data points. For example, for the points:
         *     [0, 5]
         *     [1, 20]
         *     [3, 50]
         * The interpolation at 0.5 should return 5, at 1.5 should return 20, etc.
         *
         * Eigen::Spline does not support degrees=0. To support this, we use degree 1 (linear)
         * and modify the points. For example, from the points above, we would generate:
         *     [0, 5]
         *     [1-eps, 5]
         *     [1, 20]
         *     [3-eps, 20]
         *     [3, 50]
         *
         * where eps is a small value (1 nanosecond) so that the numbers are not the same.
         */
        if (degrees == 0) {
            Eigen::MatrixXd modPoints{(points.rows() - 1) * 2 + 1, nArgs + 1};

            for (int i = 0; i < points.rows() - 1; i++) {
                modPoints.row(i * 2) = points.row(i);
                modPoints.row(i * 2 + 1) = points.row(i);
                modPoints(i * 2 + 1, 0) = points(i + 1, 0) - 1; // -1  nanosecond
            }

            modPoints.row(modPoints.rows() - 1) = points.row(points.rows() - 1);

            this->setDataPointsImpl(modPoints, 1);
        } else {
            this->setDataPointsImpl(points, degrees);
        }
    }

    /** Calls `UpdateState` with the given time
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds used to update
     *                        the state of the interpolator and generate the output message.
    */
    void Reset(uint64_t CurrentSimNanos) { UpdateState(CurrentSimNanos); }

    /**
     * Updates the state of the interpolator based on the current simulation time.
     *
     * It then prepares a message payload and sets its values using the current simulation
     * time and the interpolated result from the spline function.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds used to update
     *                        the state of the interpolator and generate the output message.
     * @throws std::invalid_argument if the interpolator has not been initialized properly
     *         (i.e., if `xMin` and `xMax` are both zero).
     */
    void UpdateState(uint64_t CurrentSimNanos)
    {
        if (this->xMin == 0 && this->xMax == 0) {
            auto error = "Tried to call Reset or UpdateState on Interpolator before setDataPoints "
                         "was called.";
            this->bskLogger.bskLog(BSK_ERROR, error);
            throw std::invalid_argument(error);
        }

        auto payload = this->interpolatedOutMsg.zeroMsgPayload;
        this->setPayloadValues(
            payload,
            CurrentSimNanos,
            this->spline(scale(double(CurrentSimNanos)))
        );

        interpolatedOutMsg.write(&payload, this->moduleID, CurrentSimNanos);
    }

public:
    /** Result message of the interpolation */
    Message<T> interpolatedOutMsg;

    /** Logger used to log errors */
    BSKLogger bskLogger;

protected:
    /** Called by `UpdateState` with the simulation time and the interpolated values.
     *
     * This method should modify the given paylod object and fill it with the
     * desired values given the function inputs.
     */
    virtual void setPayloadValues(T& payload,
                                  uint64_t CurrentSimNanos,
                                  const Eigen::Array<double, nArgs, 1>& interp) = 0;

    /**
     * Scales a single value to the range [0, 1] based on the object's xMin and xMax.
     *
     * This method normalizes the input value `val` by subtracting `xMin` and dividing
     * by the range (`xMax - xMin`). The result is clamped to ensure it remains within
     * the range [0, 1].
     *
     * @param val The value to scale.
     * @return The scaled value in the range [0, 1].
     */
    double scale(double val)
    {
        return std::clamp((val - this->xMin) / (this->xMax - this->xMin), 0., 1.);
    }

    /**
     * Scales a vector of values to the range [0, 1] based on the object's xMin and xMax.
     *
     * This method normalizes each element in the input vector `val` by subtracting
     * `xMin` and dividing by the range (`xMax - xMin`), returning a new vector with
     * the scaled values.
     *
     * @param val The vector of values to scale.
     * @return An Eigen::VectorXd containing the scaled values.
     */
    Eigen::VectorXd scale(Eigen::VectorXd val)
    {
        return (val - Eigen::VectorXd::Constant(val.size(), this->xMin)) /
               (this->xMax - this->xMin);
    }

    /**
     * Sets the data points for spline fitting based on the given matrix of points.
     *
     * @param points A matrix containing data points where the first column represents
     *               the independent values and subsequent columns represent the values
     *               to interpolate.
     * @param degrees The degree of the spline to fit (default is 1).
     * @throws std::invalid_argument if the first column of points contains only one
     *         unique value.
     */
    void setDataPointsImpl(const Eigen::MatrixXd& points, uint16_t degrees = 1)
    {
        this->xMax = points.col(0).maxCoeff();
        this->xMin = points.col(0).minCoeff();

        if (this->xMax == this->xMin) {
            auto error = "First column of points (xp) contains only one element or all elements "
                         "are the same.";
            throw std::invalid_argument(error);
        }

        this->spline = Eigen::SplineFitting<decltype(spline)>::Interpolate(
            points.rightCols(nArgs).transpose(), // yp
            degrees,
            scale(points.col(0)));
    }

protected:
    double xMin = 0; /**< Minimum independent value in the data points */
    double xMax = 0; /**< Maximum independent value in the data points */
    Eigen::Spline<double, nArgs> spline; /**< Spline object */
};

#endif
