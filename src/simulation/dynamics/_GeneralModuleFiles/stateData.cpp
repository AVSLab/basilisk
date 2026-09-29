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

#include "stateRegistry.h"

#include <cstring>
#include <stdexcept>

namespace {
void
requireShape(Eigen::Ref<const Eigen::MatrixXd> value,
             const MatrixShape& expected,
             const std::string& stateName,
             const char* field)
{
    if (value.rows() != static_cast<Eigen::Index>(expected.rows) ||
        value.cols() != static_cast<Eigen::Index>(expected.cols)) {
        throw std::invalid_argument("State '" + stateName + "' " + field + " shape mismatch: expected " +
                                    std::to_string(expected.rows) + "x" + std::to_string(expected.cols) +
                                    ", observed " + std::to_string(value.rows()) + "x" + std::to_string(value.cols()) +
                                    ".");
    }
}

void
copyValues(MutableMatrixView target, const Eigen::Ref<const Eigen::MatrixXd>& source)
{
    // Preserve support for strided blocks while copying contiguous input in one pass.
    if (source.cols() == 1 || source.outerStride() == source.rows()) {
        std::memmove(target.data(), source.data(), static_cast<size_t>(source.size()) * sizeof(double));
    } else {
        target = source;
    }
}
}

StateData::StateData(StateRegistry* registry, size_t registrationSlot, const StateSpec& spec)
  : owner(registry)
  , slot(registrationSlot)
  , viewStateShape(spec.state)
  , viewDerivativeShape(spec.derivative)
{
}

std::string
StateData::getName() const
{
    return this->owner->handleName(this->slot);
}

bool
StateData::usesPerComponentErrorControl() const
{
    return this->owner->handleSpec(this->slot).errorControl == ErrorControlMode::PerComponent;
}

size_t StateData::getNumNoiseSources() const
{
    return this->owner->handleSpec(this->slot).noiseCount;
}

void
StateData::setNumNoiseSources(size_t numSources)
{
    this->owner->setHandleNoiseCount(this->slot, numSources);
}

void
StateData::setState(Eigen::Ref<const Eigen::MatrixXd> newState)
{
    if (newState.rows() != this->viewStateShape.rows || newState.cols() != this->viewStateShape.cols) {
        requireShape(newState, this->viewStateShape, this->owner->handleName(this->slot), "state");
    }
    copyValues(this->stateView(), newState);
}

void
StateData::setDerivative(Eigen::Ref<const Eigen::MatrixXd> newDeriv)
{
    if (newDeriv.rows() != this->viewDerivativeShape.rows || newDeriv.cols() != this->viewDerivativeShape.cols) {
        requireShape(newDeriv, this->viewDerivativeShape, this->owner->handleName(this->slot), "derivative");
    }
    copyValues(this->derivativeView(), newDeriv);
}

void
StateData::setDiffusion(Eigen::Ref<const Eigen::MatrixXd> newDiffusion, size_t index)
{
    const StateSpec& spec = this->owner->handleSpec(this->slot);
    if (index >= spec.noiseCount) {
        const std::string& name = this->owner->handleName(this->slot);
        throw std::out_of_range("State '" + name + "' diffusion index " + std::to_string(index) + " is outside its " +
                                std::to_string(spec.noiseCount) + " noise sources.");
    }
    if (newDiffusion.rows() != spec.diffusionTangent.rows || newDiffusion.cols() != spec.diffusionTangent.cols) {
        requireShape(newDiffusion, spec.diffusionTangent, this->owner->handleName(this->slot), "diffusion");
    }
    MutableMatrixView target(
      this->owner->activeDiffusionData(this->slot, index), spec.diffusionTangent.rows, spec.diffusionTangent.cols);
    copyValues(target, newDiffusion);
}

MutableMatrixView
StateData::diffusionView(size_t localNoiseIndex)
{
    const MatrixShape& shape = this->owner->handleSpec(this->slot).diffusionTangent;
    return MutableMatrixView(this->owner->activeDiffusionData(this->slot, localNoiseIndex), shape.rows, shape.cols);
}

ConstMatrixView
StateData::diffusionView(size_t localNoiseIndex) const
{
    const MatrixShape& shape = this->owner->handleSpec(this->slot).diffusionTangent;
    return ConstMatrixView(this->owner->activeDiffusionData(this->slot, localNoiseIndex), shape.rows, shape.cols);
}

double*
StateData::stateData()
{
    return this->owner->liveStateData(this->slot);
}

double*
StateData::derivativeData()
{
    return this->owner->liveDerivativeData(this->slot);
}

double*
StateData::diffusionData(size_t localNoiseIndex)
{
    return this->owner->liveDiffusionData(this->slot, localNoiseIndex);
}

Eigen::MatrixXd
StateData::getState() const
{
    return Eigen::MatrixXd(this->stateView());
}

Eigen::MatrixXd
StateData::getStateDeriv() const
{
    return Eigen::MatrixXd(this->derivativeView());
}

Eigen::MatrixXd
StateData::getStateDiffusion(size_t index) const
{
    return Eigen::MatrixXd(this->diffusionView(index));
}

MatrixShape
StateData::stateShape() const
{
    return this->viewStateShape;
}

MatrixShape
StateData::derivativeShape() const
{
    return this->viewDerivativeShape;
}

MatrixShape
StateData::diffusionShape() const
{
    return this->owner->handleSpec(this->slot).diffusionTangent;
}
