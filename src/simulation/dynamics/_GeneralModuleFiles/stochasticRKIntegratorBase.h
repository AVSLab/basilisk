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

/** @file stochasticRKIntegratorBase.h
 * @brief Noise-generator ownership and stochastic method workspace binding.
 */

#ifndef stochasticRKIntegratorBase_h
#define stochasticRKIntegratorBase_h

#include "../_GeneralModuleFiles/stateVecStochasticIntegrator.h"
#include "../_GeneralModuleFiles/stochasticNoiseGenerator.h"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

/**
 * Shared base for the native stochastic Runge-Kutta integrators (Euler-Maruyama, the
 * SRI/SRA strong methods, the weak W2Ito/DRI1/RI/RS/SIESME families, RDI1WM and
 * Euler-Heun/RKMil).
 *
 * It factors out the machinery every one of these integrators needs, so each concrete
 * method implements its own integrateImpl() recurrence and, when needed,
 * bindStochasticMethodStorage() hook:
 *
 *  - a pluggable ``GaussianNoiseGenerator`` (defaulting to a random Mersenne-Twister
 *    source, replaceable by a prescribed-replay generator for tests), plus the
 *    ``setRNGSeed`` / ``setNoiseGenerator`` accessors;
 *  - one canonical flat state/noise binding shared by all concrete methods.
 *
 * Every native stochastic integrator derives from this base. Methods that need only the
 * Wiener increment use the base buffers through the generator's Wiener-only interface;
 * ``dZ`` remains available as compatibility scratch for legacy generators.
 * Generator output and method storage are prepared in the same binding transaction.
 * A method failure restores state through the stochastic base but does not rewind RNG
 * position. The built-in generator uses a cached exact-type pointer; custom generators
 * are held by a local shared owner while their callbacks run.
 *
 * @warning Stochastic integration is in beta.
 */
class StochasticRKIntegratorBase : public StateVecStochasticIntegrator {
public:
    using StateVecStochasticIntegrator::StateVecStochasticIntegrator;

    /** Sets the seed for the (default) Random Number Generator used by this integrator.
     *
     * As a stochastic integrator, random numbers are drawn during each time step. By
     * default a randomly generated seed is used. Setting the seed makes the integrator
     * draw the same sequence each run. Has no effect if a custom noise generator that
     * does not honour the seed was installed via ``setNoiseGenerator``. */
    void setRNGSeed(size_t seed);

    /** Replaces the noise generator used by this integrator. This is primarily useful for
     * testing, where a ``PrescribedGaussianNoiseGenerator`` can be installed so the
     * integrator replays a known sequence of Wiener increments. */
    void setNoiseGenerator(std::shared_ptr<GaussianNoiseGenerator> generator);

protected:
  /** @brief Prepare shared noise output and method scratch before binding commits. */
  void prepareIntegrationBinding() override;

  /** @brief Verify the borrowed topology before reusing numerical storage. */
  void validateIntegrationBinding() const override { this->validateStochasticTopology(); }

  /** Allocates method-specific scratch in the topology-binding transaction. */
  virtual void bindStochasticMethodStorage() {}

  /** Binds topology, generator output, and method scratch as one transaction. */
  template<typename MethodBinder>
  void bindFlatStochasticStorage(MethodBinder&& bindMethodStorage)
  {
      try {
          this->bindFlatStochasticCore();
          std::forward<MethodBinder>(bindMethodStorage)();
      } catch (...) {
          this->resetFlatStochasticStorage();
          throw;
      }
  }

  /** Binds methods that need no additional scratch beyond the flat base. */
  void bindFlatStochasticStorage();

  /** Draws one sample into preallocated ``dW`` and ``dZ`` buffers. */
  void generateNoise(double timeStep);

  /** Draws Wiener increments and only the requested auxiliary prefix. */
  void generateNoise(double timeStep, size_t auxiliaryCount);

  /** Draws only Wiener increments, reusing ``dZ`` as compatibility scratch. */
  void generateWienerNoise(double timeStep);

  /** Preallocated Wiener increments for flat stochastic methods. */
  const Eigen::VectorXd& flatDW() const noexcept { return this->dW; }

  /** Preallocated auxiliary Gaussian increments for flat stochastic methods. */
  const Eigen::VectorXd& flatDZ() const noexcept { return this->dZ; }

private:
  /** @brief Hold the configured generator alive across a potentially replacing callback. */
  std::shared_ptr<GaussianNoiseGenerator> noiseGenerator() const
  {
      if (this->rvGenerator == nullptr) {
          throw std::invalid_argument("Stochastic integrator noise generator cannot be null.");
      }
      return this->rvGenerator;
  }

  /** @brief Bind topology and size Wiener/auxiliary buffers once. */
  void bindFlatStochasticCore();
  /** @brief Discard incomplete topology and noise output after binding failure. */
  void resetFlatStochasticStorage() noexcept;

  bool flatNoiseOutputBound = false; ///< True after topology-sized output allocation commits.
  Eigen::VectorXd dW; ///< Wiener increments indexed by independent global source.
  Eigen::VectorXd dZ; ///< Auxiliary Gaussian output; only the requested prefix is meaningful.
  /** Random Number Generator for the integrator (supplies dW and dZ per noise source). */
  std::shared_ptr<GaussianNoiseGenerator> rvGenerator = std::make_shared<RandomGaussianNoiseGenerator>();
  /** @brief Cached exact built-in type, or null for custom generators.
   * Exact built-in generators cannot replace themselves from a callback.
   */
  RandomGaussianNoiseGenerator* nativeGenerator = static_cast<RandomGaussianNoiseGenerator*>(this->rvGenerator.get());
};

#endif /* stochasticRKIntegratorBase_h */
