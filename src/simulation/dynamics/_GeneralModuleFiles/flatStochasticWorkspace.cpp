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

#include "flatStochasticWorkspace.h"

void
FlatStochasticWorkspace::bind(const std::vector<StochasticObjectDescriptor>& objects,
                              const std::vector<StochasticNoiseBinding>& bindings,
                              const std::vector<StochasticNoiseSlot>& slots,
                              size_t driftStageCount,
                              size_t diffusionStageCount,
                              size_t scratchVectorCount)
{
    const size_t derivativeScalarCount =
      objects.empty() ? 0 : objects.back().derivativeOffset + objects.back().derivativeCount;
    const size_t diffusionScalarCount = slots.empty() ? 0 : slots.back().packedBegin + slots.back().packedCount;

    Eigen::MatrixXd newDriftStages(static_cast<Eigen::Index>(derivativeScalarCount),
                                   static_cast<Eigen::Index>(driftStageCount));
    Eigen::MatrixXd newDiffusionStages(static_cast<Eigen::Index>(diffusionScalarCount),
                                       static_cast<Eigen::Index>(diffusionStageCount));
    Eigen::VectorXd newCombinedDrift(static_cast<Eigen::Index>(derivativeScalarCount));
    Eigen::VectorXd newCombinedDiffusion(static_cast<Eigen::Index>(diffusionScalarCount));
    std::vector<Eigen::VectorXd> newVectors(scratchVectorCount);
    for (Eigen::VectorXd& vector : newVectors) {
        vector.resize(static_cast<Eigen::Index>(slots.size()));
        vector.setZero();
    }

    this->driftStages.swap(newDriftStages);
    this->diffusionStages.swap(newDiffusionStages);
    this->combinedDrift.swap(newCombinedDrift);
    this->combinedDiffusion.swap(newCombinedDiffusion);
    this->vectors.swap(newVectors);
    this->objects = &objects;
    this->bindings = &bindings;
    this->slots = &slots;
    this->bound = true;
}

void
FlatStochasticWorkspace::reset() noexcept
{
    this->bound = false;
    this->objects = nullptr;
    this->bindings = nullptr;
    this->slots = nullptr;
    Eigen::MatrixXd().swap(this->driftStages);
    Eigen::MatrixXd().swap(this->diffusionStages);
    Eigen::VectorXd().swap(this->combinedDrift);
    Eigen::VectorXd().swap(this->combinedDiffusion);
    std::vector<Eigen::VectorXd>().swap(this->vectors);
}
