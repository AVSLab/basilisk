/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "stateData.h"

std::unique_ptr<StateData> StateData::clone() const
{
    auto result = std::make_unique<StateData>(this->stateName, this->state);
    result->stateDeriv = this->stateDeriv;
    result->stateDiffusion.resize(this->stateDiffusion.size());
    for (size_t i = 0; i < this->stateDiffusion.size(); i++)
    {
        result->stateDiffusion[i] = this->stateDiffusion[i];
    }
    return result;
}

StateData::StateData(std::string inName, const Eigen::MatrixXd & newState)
: stateName(inName)
{
    setState(newState);
    stateDeriv.resizeLike(state);
    setDerivative( Eigen::MatrixXd::Zero(state.innerSize(), state.outerSize()) );
}

void StateData::setNumNoiseSources(size_t numSources)
{
    stateDiffusion.resize(numSources);
    for (size_t i = 0; i < numSources; i++)
    {
        stateDiffusion[i].resizeLike(state);

    }
}

size_t StateData::getNumNoiseSources() const
{
    return stateDiffusion.size();
}

void StateData::setState(const Eigen::MatrixXd & newState)
{
    state = newState;
}

void StateData::propagateState(double dt, std::vector<double> pseudoStep)
{
    state += stateDeriv * dt;

    if (getNumNoiseSources() > 0 && pseudoStep.size() == 0)
    {
        auto errorMsg = "State " + this->getName() + " has stochastic dynamics, but "
            + "the integrator tried to propagate it without pseudoSteps. Are you sure "
            + "you are using a stochastic integrator?";
        bskLogger.bskLog(BSK_ERROR, errorMsg.c_str());
        throw std::invalid_argument(errorMsg);
    }

    for (size_t i = 0; i < getNumNoiseSources(); i++)
    {
        state += stateDiffusion.at(i) * pseudoStep.at(i);
    }
}


void StateData::setDerivative(const Eigen::MatrixXd & newDeriv)
{
    stateDeriv = newDeriv;
}

void StateData::setDiffusion(const Eigen::MatrixXd & newDiffusion, size_t index)
{
    if (index < stateDiffusion.size())
    {
        stateDiffusion[index] = newDiffusion;
    }
    else
    {
        auto errorMsg = "Tried to set diffusion index greater than number of noise sources configured: "
            + std::to_string(index) + " >= " + std::to_string(stateDiffusion.size());
        bskLogger.bskLog(BSK_ERROR, errorMsg.c_str());
        throw std::out_of_range(errorMsg);
    }
}

void StateData::scaleState(double scaleFactor)
{
    state *= scaleFactor;
}

void StateData::addState(const StateData& other)
{
    state += other.state;
}
