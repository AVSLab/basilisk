#include "singleActuatorLTI.h"

Eigen::VectorXd SingleActuatorLTI::readInput(uint64_t CurrentSimNanos)
{
    return Eigen::VectorXd::Constant(1, this->inMsg().input);
}

void SingleActuatorLTI::writeOutput(uint64_t CurrentSimNanos, const Eigen::VectorXd& y)
{
    auto payload = SingleActuatorMsgPayload{y(0)};
    this->outMsg.write(&payload, moduleID, CurrentSimNanos);
}
