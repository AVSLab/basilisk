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

#ifndef stochasticNoiseGenerator_h
#define stochasticNoiseGenerator_h

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <vector>

/** One draw of the Gaussian random variables needed to advance a stochastic
 * integrator by a single time step.
 *
 * Both members have length ``m`` (the number of independent noise sources).
 * ``dW`` is the Wiener increment for each noise source; ``dZ`` is a second,
 * independent Wiener increment required by the higher-order Roessler methods
 * (SRI/SRA) to build the mixed iterated integrals.
 *
 * With time step ``h``, each entry of ``dW`` and ``dZ`` is distributed as
 * \f$N(0, h)\f$.
 */
struct GaussianNoiseSample {
    Eigen::VectorXd dW; //!< Wiener increment per noise source, ~ N(0, h)
    Eigen::VectorXd dZ; //!< Second independent Wiener increment per noise source, ~ N(0, h)
};

/** Interface for the random-variable source used by the native stochastic
 * integrators.
 *
 * Separating the noise generation from the integrator lets a test install a
 * generator that replays a known sequence of increments (see
 * ``PrescribedGaussianNoiseGenerator``), which is how the Basilisk integrators
 * are checked for numerical equivalence against a reference implementation.
 *
 * @warning Stochastic integration is in beta.
 */
class GaussianNoiseGenerator {
public:
    virtual ~GaussianNoiseGenerator() = default;

    /** Sets the seed for the underlying RNG (if any). */
    virtual void setSeed(size_t seed) = 0;

    /** Returns the Gaussian increments for one step with ``m`` noise sources and
     * time step ``h``. Both returned vectors have length ``m``. */
    virtual GaussianNoiseSample generate(size_t m, double h) = 0;

    /** Returns only the Wiener increments for one step.
     *
     * This allocation-bearing convenience method is intended for bindings and
     * non-hot-path callers. Native integrators use ``generateWienerInto``.
     */
    Eigen::VectorXd generateWiener(size_t m, double h)
    {
        Eigen::VectorXd dW(static_cast<Eigen::Index>(m));
        this->generateWienerInto(dW, m, h);
        return dW;
    }

    /** Writes one Gaussian sample into caller-owned, pre-sized storage.
     *
     * The compatibility implementation calls ``generate()`` and may allocate.
     * Generators used on allocation-sensitive paths should override this method
     * to write directly into the supplied buffers.
     *
     * @param dW Pre-sized Wiener-increment output.
     * @param dZ Pre-sized second-increment output.
     * @param m Number of independent noise sources.
     * @param h Integration time step.
     */
    virtual void generateInto(Eigen::VectorXd& dW, Eigen::VectorXd& dZ, size_t m, double h)
    {
        requireOutputSize(dW, dZ, m);
        const GaussianNoiseSample sample = this->generate(m, h);
        requireOutputSize(sample.dW, sample.dZ, m);
        dW = sample.dW;
        dZ = sample.dZ;
    }

    /** Writes Wiener increments and a requested prefix of auxiliary increments.
     *
     * The compatibility implementation forwards to ``generateInto`` and may
     * generate more auxiliary values than requested. Built-in generators
     * override this method so methods that need fewer than ``m`` auxiliary
     * values do not pay for unused random draws.
     */
    virtual void generateWithAuxiliaryInto(Eigen::VectorXd& dW,
                                           Eigen::VectorXd& dZ,
                                           size_t m,
                                           size_t auxiliaryCount,
                                           double h)
    {
        requireAuxiliaryOutputSize(dW, dZ, m, auxiliaryCount);
        if (auxiliaryCount == 0) {
            this->generateWienerInto(dW, dZ, m, h);
            return;
        }
        this->generateInto(dW, dZ, m, h);
    }

    /** Writes only Wiener increments into caller-owned, pre-sized storage.
     *
     * The default preserves source compatibility with custom generators by
     * forwarding through ``generateInto``. Built-in generators override this
     * method to avoid generating an unused auxiliary increment.
     */
    virtual void generateWienerInto(Eigen::VectorXd& dW, size_t m, double h)
    {
        Eigen::VectorXd unusedDZ(static_cast<Eigen::Index>(m));
        this->generateWienerInto(dW, unusedDZ, m, h);
    }

    /** Wiener-only generation with caller-owned compatibility scratch.
     *
     * Existing custom generators inherit this adapter. Built-in generators
     * override it and leave ``unusedDZ`` untouched.
     */
    virtual void generateWienerInto(Eigen::VectorXd& dW, Eigen::VectorXd& unusedDZ, size_t m, double h)
    {
        this->generateInto(dW, unusedDZ, m, h);
    }

  protected:
    /** Validates caller-owned generator output dimensions. */
    static void requireOutputSize(const Eigen::VectorXd& dW, const Eigen::VectorXd& dZ, size_t m)
    {
        const auto expected = static_cast<Eigen::Index>(m);
        if (dW.size() != expected || dZ.size() != expected) {
            throw std::invalid_argument("GaussianNoiseGenerator output buffers must be pre-sized to the "
                                        "requested number of noise sources.");
        }
    }

    /** Validates caller-owned Wiener-only output dimensions. */
    static void requireWienerOutputSize(const Eigen::VectorXd& dW, size_t m)
    {
        if (dW.size() != static_cast<Eigen::Index>(m)) {
            throw std::invalid_argument("GaussianNoiseGenerator output buffer must be pre-sized to the "
                                        "requested number of noise sources.");
        }
    }

    /** Validates a Wiener output and an auxiliary prefix. */
    static void requireAuxiliaryOutputSize(const Eigen::VectorXd& dW,
                                           const Eigen::VectorXd& dZ,
                                           size_t m,
                                           size_t auxiliaryCount)
    {
        if (auxiliaryCount > m || dW.size() != static_cast<Eigen::Index>(m) ||
            dZ.size() < static_cast<Eigen::Index>(auxiliaryCount)) {
            throw std::invalid_argument("GaussianNoiseGenerator output buffers do not match the "
                                        "requested Wiener and auxiliary counts.");
        }
    }
};

/** Draws the Gaussian increments from a Mersenne-Twister RNG.
 *
 * This is the default generator used in production. Each entry of ``dW`` and
 * ``dZ`` is drawn as \f$\sqrt{h}\,N(0,1)\f$. The ``dW`` entries are drawn first
 * (indices 0..m-1), then the ``dZ`` entries.
 */
class RandomGaussianNoiseGenerator : public GaussianNoiseGenerator {
public:
  void setSeed(size_t seed) override
  {
      this->rng.seed(static_cast<std::mt19937::result_type>(seed));
      this->normal_rv.reset();
  }

    GaussianNoiseSample generate(size_t m, double h) override
    {
        GaussianNoiseSample sample;
        sample.dW.resize(static_cast<Eigen::Index>(m));
        sample.dZ.resize(static_cast<Eigen::Index>(m));
        this->generateInto(sample.dW, sample.dZ, m, h);
        return sample;
    }

    void generateInto(Eigen::VectorXd& dW, Eigen::VectorXd& dZ, size_t m, double h) override
    {
        this->generateWithAuxiliaryInto(dW, dZ, m, m, h);
    }

    void generateWithAuxiliaryInto(Eigen::VectorXd& dW,
                                   Eigen::VectorXd& dZ,
                                   size_t m,
                                   size_t auxiliaryCount,
                                   double h) override
    {
        requireAuxiliaryOutputSize(dW, dZ, m, auxiliaryCount);
        const double sqh = std::sqrt(h);
        for (size_t i = 0; i < m; i++) {
            dW(static_cast<Eigen::Index>(i)) = sqh * this->normal_rv(this->rng);
        }
        for (size_t i = 0; i < auxiliaryCount; i++) {
            dZ(static_cast<Eigen::Index>(i)) = sqh * this->normal_rv(this->rng);
        }
    }

    void generateWienerInto(Eigen::VectorXd& dW, size_t m, double h) override
    {
        requireWienerOutputSize(dW, m);

        const double sqh = std::sqrt(h);
        for (size_t i = 0; i < m; i++) {
            dW(static_cast<Eigen::Index>(i)) = sqh * this->normal_rv(this->rng);
        }
    }

    void generateWienerInto(Eigen::VectorXd& dW, Eigen::VectorXd& unusedDZ, size_t m, double h) override
    {
        requireOutputSize(dW, unusedDZ, m);
        this->generateWienerInto(dW, m, h);
    }

protected:
    /** Random Number Generator */
    std::mt19937 rng{std::random_device{}()};

    /** Standard normally distributed random variable */
    std::normal_distribution<double> normal_rv{0., 1.};
};

/** Replays a pre-computed sequence of Gaussian increments.
 *
 * Each call to ``generate`` advances to the next queued sample. This is used by the unit
 * tests to feed a native integrator exactly the same Wiener increments that a
 * reference implementation used, so that the two can be compared to within
 * floating-point tolerance.
 *
 * The queued ``dW``/``dZ`` values are the actual increments (already scaled to
 * the step, i.e. distributed as \f$N(0,h)\f$); they are returned verbatim and
 * the ``h`` argument to ``generate`` is ignored for the values themselves. If a
 * generate call is made after the queue is exhausted, a ``std::runtime_error``
 * is thrown.
 */
class PrescribedGaussianNoiseGenerator : public GaussianNoiseGenerator {
public:
    /** Seeding has no effect on a prescribed generator. */
    void setSeed(size_t) override {}

    /** Appends one step's worth of increments to the replay queue.
     *
     * @param dW Wiener increments for each noise source for this step.
     * @param dZ Second Wiener increments for each noise source for this step. May
     *           be empty for methods that do not use it, in which case a zero
     *           vector of matching length is returned.
     */
    void pushStep(const std::vector<double>& dW, const std::vector<double>& dZ = {})
    {
        GaussianNoiseSample sample;
        sample.dW = Eigen::Map<const Eigen::VectorXd>(dW.data(), static_cast<Eigen::Index>(dW.size()));
        if (dZ.empty()) {
            sample.dZ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dW.size()));
        } else {
            sample.dZ =
                Eigen::Map<const Eigen::VectorXd>(dZ.data(), static_cast<Eigen::Index>(dZ.size()));
        }
        this->samples.push_back(std::move(sample));
    }

    /** Removes all queued samples. */
    void clear()
    {
        this->samples.clear();
        this->cursor = 0;
    }

    /** Discards consumed samples while preserving pending FIFO order. */
    void discardConsumed()
    {
        if (this->cursor == 0) {
            return;
        }
        this->samples.erase(this->samples.begin(), this->samples.begin() + static_cast<std::ptrdiff_t>(this->cursor));
        this->cursor = 0;
    }

    /** Number of steps still queued. */
    size_t remaining() const { return this->samples.size() - this->cursor; }

    GaussianNoiseSample generate(size_t m, double) override { return this->nextAndValidate(m, m); }

    void generateInto(Eigen::VectorXd& dW, Eigen::VectorXd& dZ, size_t m, double) override
    {
        requireOutputSize(dW, dZ, m);
        const GaussianNoiseSample& sample = this->nextAndValidate(m, m);
        dW = sample.dW;
        dZ = sample.dZ.head(static_cast<Eigen::Index>(m));
    }

    void generateWithAuxiliaryInto(Eigen::VectorXd& dW,
                                   Eigen::VectorXd& dZ,
                                   size_t m,
                                   size_t auxiliaryCount,
                                   double) override
    {
        requireAuxiliaryOutputSize(dW, dZ, m, auxiliaryCount);
        const GaussianNoiseSample& sample = this->nextAndValidate(m, auxiliaryCount);
        dW = sample.dW;
        if (auxiliaryCount > 0) {
            dZ.head(static_cast<Eigen::Index>(auxiliaryCount)) =
              sample.dZ.head(static_cast<Eigen::Index>(auxiliaryCount));
        }
    }

    void generateWienerInto(Eigen::VectorXd& dW, size_t m, double) override
    {
        requireWienerOutputSize(dW, m);
        const GaussianNoiseSample& sample = this->nextAndValidate(m, 0);
        dW = sample.dW;
    }

    void generateWienerInto(Eigen::VectorXd& dW, Eigen::VectorXd& unusedDZ, size_t m, double h) override
    {
        requireOutputSize(dW, unusedDZ, m);
        this->generateWienerInto(dW, m, h);
    }

  protected:
    /** Advances to and validates the next queued sample. */
    const GaussianNoiseSample& nextAndValidate(size_t m, size_t auxiliaryCount)
    {
        if (this->cursor == this->samples.size()) {
            throw std::runtime_error(
                "PrescribedGaussianNoiseGenerator ran out of prescribed noise samples. "
                "Push one sample per integration step.");
        }
        const GaussianNoiseSample& sample = this->samples[this->cursor++];

        if (static_cast<size_t>(sample.dW.size()) != m) {
            throw std::runtime_error(
                "PrescribedGaussianNoiseGenerator: the queued sample has a different number of "
                "noise sources than requested by the integrator.");
        }
        if (static_cast<size_t>(sample.dZ.size()) < auxiliaryCount) {
            throw std::runtime_error("PrescribedGaussianNoiseGenerator: the queued sample's dZ is shorter than the "
                                     "auxiliary count requested by the integrator.");
        }
        return sample;
    }

    /** Prescribed increments retained for the lifetime of the replay sequence. */
    std::vector<GaussianNoiseSample> samples;
    size_t cursor = 0; ///< Index of the next prescribed sample to consume.
};

#endif /* stochasticNoiseGenerator_h */
