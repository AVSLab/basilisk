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

#include "flybyODuKF.h"

FlybyODuKF::FlybyODuKF() = default;

FlybyODuKF::~FlybyODuKF() = default;

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
    /*! - Check if the required message has not been connected */
    if (!this->opNavHeadingMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  "Error: flybyODuKF opNavUnitVecInMsg wasn't connected.");
    }

    /*! - Initialize filter parameters and change units to km and s */
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::writeOutputMessages(uint64_t CurrentSimNanos) {
    
    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();
    }
}

/*! Integrate the equations of motion of two body point mass gravity using Runge-Kutta 4 (RK4)
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd FlybyODuKF::propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt) const
{
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    Eigen::VectorXd X = X0;

    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> f = [this](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd stateDerivative(state.size());
        /*! Implement point mass gravity for the propagation */
        stateDerivative.segment(0,3) = state.segment(3, 3);
        stateDerivative.segment(3,3) = - this->muCentral/(pow(state.head(3).norm(),3)) * state.head(3);

        return stateDerivative;
    };

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(f, X, t, step);
        t = t + step;
    }

    return X;
}

}

}

}

}
