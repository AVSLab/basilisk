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

/** @file flatStochasticWorkspace.h
 * @brief Reusable drift and packed-diffusion stages for stochastic methods.
 */

#ifndef flatStochasticWorkspace_h
#define flatStochasticWorkspace_h

#include "stateVecStochasticIntegrator.h"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

/**
 * @brief Stage matrices and reusable combinations for stochastic RK recurrences.
 *
 * Drift columns use concatenated derivative-buffer order. Diffusion columns use
 * packed global-slot order and contain only registered tangents. Scratch vectors
 * have one entry per global source. bind() allocates all storage before committing
 * borrowed descriptor references; those descriptors must remain alive and unchanged
 * until reset(). Numerical methods choose the stage counts and populate each stage
 * before reading it. Capture/combination calls do not allocate or resize storage.
 */
class FlatStochasticWorkspace
{
  public:
    /** @brief Report whether stage storage and borrowed topology were committed. */
    bool isBound() const noexcept { return this->bound; }

    /** @brief Allocate method storage against a finalized stochastic binding.
     * @param objects Object buffers in integration order; borrowed until reset().
     * @param bindings Packed diffusion endpoints; borrowed until reset().
     * @param slots Global noise ranges; borrowed until reset().
     * @param driftStageCount Number of derivative-stage columns.
     * @param diffusionStageCount Number of diffusion-stage columns.
     * @param scratchVectorCount Number of per-source scratch vectors.
     * @note Calling bind() on an already bound workspace leaves it unchanged.
     */
    void bind(const std::vector<StochasticObjectDescriptor>& objects,
              const std::vector<StochasticNoiseBinding>& bindings,
              const std::vector<StochasticNoiseSlot>& slots,
              size_t driftStageCount,
              size_t diffusionStageCount,
              size_t scratchVectorCount);

    /** @brief Release stage storage and forget borrowed topology without throwing. */
    void reset() noexcept;

    /** @brief Borrow the latest weighted drift combination. */
    const Eigen::VectorXd& drift() const noexcept { return this->combinedDrift; }

    /** @brief Borrow packed combinations; uncomputed slots are unspecified. */
    const Eigen::VectorXd& diffusions() const noexcept { return this->combinedDiffusion; }

    /** @brief Borrow a method scratch vector indexed by global noise source. */
    Eigen::VectorXd& vector(size_t index) { return this->vectors.at(index); }

    /** @brief Borrow a read-only method scratch vector. */
    const Eigen::VectorXd& vector(size_t index) const { return this->vectors.at(index); }

    /** @brief Copy live derivatives into one stage column. */
    void captureDrift(size_t stage)
    {
        double* destination = this->driftStages.col(static_cast<Eigen::Index>(stage)).data();
        for (const StochasticObjectDescriptor& object : *this->objects) {
            if (object.derivativeCount == 0) {
                continue;
            }
            std::copy_n(object.derivativeData,
                        object.derivativeCount,
                        destination + static_cast<Eigen::Index>(object.derivativeOffset));
        }
    }

    /** @brief Capture every global source into one diffusion-stage column. */
    void captureAllDiffusions(size_t stage)
    {
        for (size_t slot = 0; slot < this->slots->size(); ++slot) {
            this->captureDiffusion(slot, stage);
        }
    }

    /** @brief Capture only the tangents belonging to one global source. */
    void captureDiffusion(size_t slotIndex, size_t stage)
    {
        const StochasticNoiseSlot& slot = this->slots->at(slotIndex);
        for (size_t index = 0; index < slot.bindingCount; ++index) {
            const StochasticNoiseBinding& binding = this->bindings->at(slot.bindingBegin + index);
            std::copy_n(binding.liveData,
                        binding.scalarCount,
                        this->diffusionStages.col(static_cast<Eigen::Index>(stage)).data() +
                          static_cast<Eigen::Index>(binding.packedOffset));
        }
    }

    /** @brief Copy one source's tangent block between stage columns. */
    void copyDiffusion(size_t slotIndex, size_t destination, size_t source)
    {
        const StochasticNoiseSlot& slot = this->slots->at(slotIndex);
        const auto offset = static_cast<Eigen::Index>(slot.packedBegin);
        const auto count = static_cast<Eigen::Index>(slot.packedCount);
        this->diffusionStages.col(static_cast<Eigen::Index>(destination)).segment(offset, count) =
          this->diffusionStages.col(static_cast<Eigen::Index>(source)).segment(offset, count);
    }

    /** @brief Combine length populated drift stages from firstStage; length must be positive. */
    template<size_t count>
    void writeDrift(const std::array<double, count>& factors, size_t length, size_t firstStage = 0)
    {
        this->combinedDrift = this->driftStages.col(static_cast<Eigen::Index>(firstStage)) * factors.at(0);
        for (size_t stage = 1; stage < length; ++stage) {
            if (factors.at(stage) == 0.0) {
                continue;
            }
            this->combinedDrift +=
              this->driftStages.col(static_cast<Eigen::Index>(firstStage + stage)) * factors.at(stage);
        }
    }

    /** @brief Combine one source's populated stages; other source blocks remain unchanged. */
    template<size_t count>
    void writeDiffusion(size_t slotIndex,
                        const std::array<double, count>& factors,
                        size_t length,
                        size_t firstStage = 0)
    {
        const StochasticNoiseSlot& slot = this->slots->at(slotIndex);
        const auto offset = static_cast<Eigen::Index>(slot.packedBegin);
        const auto scalarCount = static_cast<Eigen::Index>(slot.packedCount);
        auto target = this->combinedDiffusion.segment(offset, scalarCount);
        target =
          this->diffusionStages.col(static_cast<Eigen::Index>(firstStage)).segment(offset, scalarCount) * factors.at(0);
        for (size_t stage = 1; stage < length; ++stage) {
            if (factors.at(stage) == 0.0) {
                continue;
            }
            target +=
              this->diffusionStages.col(static_cast<Eigen::Index>(firstStage + stage)).segment(offset, scalarCount) *
              factors.at(stage);
        }
    }

    /** @brief Copy one source's stage block into the combined diffusion buffer. */
    void writeDiffusionStage(size_t slotIndex, size_t stage)
    {
        const StochasticNoiseSlot& slot = this->slots->at(slotIndex);
        const auto offset = static_cast<Eigen::Index>(slot.packedBegin);
        const auto count = static_cast<Eigen::Index>(slot.packedCount);
        this->combinedDiffusion.segment(offset, count) =
          this->diffusionStages.col(static_cast<Eigen::Index>(stage)).segment(offset, count);
    }

  private:
    bool bound = false; ///< True when buffers and descriptor references are ready.
    const std::vector<StochasticObjectDescriptor>* objects = nullptr; ///< Borrowed object buffers.
    const std::vector<StochasticNoiseBinding>* bindings = nullptr; ///< Borrowed packed endpoints.
    const std::vector<StochasticNoiseSlot>* slots = nullptr; ///< Borrowed global-source ranges.
    Eigen::MatrixXd driftStages; ///< Derivative scalars by drift stage; each column is contiguous.
    Eigen::MatrixXd diffusionStages; ///< Packed tangent scalars by diffusion stage.
    Eigen::VectorXd combinedDrift; ///< Reusable weighted sum of drift columns.
    Eigen::VectorXd combinedDiffusion; ///< Weighted sums, written one global slot at a time.
    std::vector<Eigen::VectorXd> vectors; ///< Method scratch, with one entry per global source.
};

#endif /* flatStochasticWorkspace_h */
