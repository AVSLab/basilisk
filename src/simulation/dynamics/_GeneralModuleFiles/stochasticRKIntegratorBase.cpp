/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "stochasticRKIntegratorBase.h"
#include <typeinfo>

void
StochasticRKIntegratorBase::bindFlatStochasticCore()
{
    if (this->stochasticObjectDescriptors().size() != this->dynamics().size()) {
        this->resetFlatStochasticStorage();
    }
    this->bindStochasticTopology();
    if (this->flatNoiseOutputBound) {
        return;
    }

    const auto noiseCount = static_cast<Eigen::Index>(this->globalNoiseCount());
    Eigen::VectorXd newDW(noiseCount);
    Eigen::VectorXd newDZ(noiseCount);
    this->dW.swap(newDW);
    this->dZ.swap(newDZ);
    this->flatNoiseOutputBound = true;
}

void
StochasticRKIntegratorBase::resetFlatStochasticStorage() noexcept
{
    this->flatNoiseOutputBound = false;
    Eigen::VectorXd().swap(this->dW);
    Eigen::VectorXd().swap(this->dZ);
    this->resetStochasticTopologyBinding();
}

void
StochasticRKIntegratorBase::generateNoise(double timeStep)
{
    this->generateNoise(timeStep, this->globalNoiseCount());
}

void
StochasticRKIntegratorBase::generateNoise(double timeStep, size_t auxiliaryCount)
{
    this->validateStochasticTimeStep(timeStep);
    if (this->nativeGenerator != nullptr) {
        this->nativeGenerator->RandomGaussianNoiseGenerator::generateWithAuxiliaryInto(
          this->dW, this->dZ, this->globalNoiseCount(), auxiliaryCount, timeStep);
        return;
    }
    const auto generator = this->noiseGenerator();
    generator->generateWithAuxiliaryInto(this->dW, this->dZ, this->globalNoiseCount(), auxiliaryCount, timeStep);
}

void
StochasticRKIntegratorBase::generateWienerNoise(double timeStep)
{
    this->validateStochasticTimeStep(timeStep);
    if (this->nativeGenerator != nullptr) {
        this->nativeGenerator->RandomGaussianNoiseGenerator::generateWienerInto(
          this->dW, this->globalNoiseCount(), timeStep);
        return;
    }
    const auto generator = this->noiseGenerator();
    generator->generateWienerInto(this->dW, this->dZ, this->globalNoiseCount(), timeStep);
}

void
StochasticRKIntegratorBase::setRNGSeed(size_t seed)
{
    const auto generator = this->noiseGenerator();
    generator->setSeed(seed);
}

void
StochasticRKIntegratorBase::setNoiseGenerator(std::shared_ptr<GaussianNoiseGenerator> generator)
{
    if (generator == nullptr) {
        throw std::invalid_argument("Stochastic integrator noise generator cannot be null.");
    }
    this->rvGenerator = std::move(generator);
    auto* activeGenerator = this->rvGenerator.get();
    this->nativeGenerator = typeid(*activeGenerator) == typeid(RandomGaussianNoiseGenerator)
                              ? static_cast<RandomGaussianNoiseGenerator*>(activeGenerator)
                              : nullptr;
}

void
StochasticRKIntegratorBase::prepareIntegrationBinding()
{
    // This runs before first use and after native peer destruction. Method scratch
    // must be sized again along with the shared state and noise buffers.
    this->bindFlatStochasticStorage([this] { this->bindStochasticMethodStorage(); });
}

void
StochasticRKIntegratorBase::bindFlatStochasticStorage()
{
    this->bindFlatStochasticStorage([] {});
}
