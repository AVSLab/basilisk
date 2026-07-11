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
#include <deque>
#include <random>
#include <stdexcept>
#include <vector>

/** One draw of the Gaussian random variables needed to advance a stochastic
 * integrator by a single time step.
 *
 * Both members have length ``m`` (the number of independent noise sources).
 * ``dW`` is the Wiener increment for each noise source; ``dZ`` is a second,
 * independent Wiener increment required by the higher-order Roessler methods
 * (SRI/SRA) to build the mixed iterated integrals. Methods that only need the
 * Wiener increment (e.g. Euler-Maruyama, Euler-Heun, RKMil) simply ignore
 * ``dZ``.
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
};

/** Draws the Gaussian increments from a Mersenne-Twister RNG.
 *
 * This is the default generator used in production. Each entry of ``dW`` and
 * ``dZ`` is drawn as \f$\sqrt{h}\,N(0,1)\f$. The ``dW`` entries are drawn first
 * (indices 0..m-1), then the ``dZ`` entries.
 */
class RandomGaussianNoiseGenerator : public GaussianNoiseGenerator {
public:
    void setSeed(size_t seed) override { this->rng.seed(seed); }

    GaussianNoiseSample generate(size_t m, double h) override
    {
        // purge any hidden state so that seeding is always consistent
        this->normal_rv.reset();

        const double sqh = std::sqrt(h);
        GaussianNoiseSample sample;
        sample.dW.resize(m);
        sample.dZ.resize(m);
        for (size_t i = 0; i < m; i++) {
            sample.dW(i) = sqh * this->normal_rv(this->rng);
        }
        for (size_t i = 0; i < m; i++) {
            sample.dZ(i) = sqh * this->normal_rv(this->rng);
        }
        return sample;
    }

protected:
    /** Random Number Generator */
    std::mt19937 rng{std::random_device{}()};

    /** Standard normally distributed random variable */
    std::normal_distribution<double> normal_rv{0., 1.};
};

/** Replays a pre-computed sequence of Gaussian increments.
 *
 * Each call to ``generate`` pops the next queued sample. This is used by the unit
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
        this->queue.push_back(std::move(sample));
    }

    /** Removes all queued samples. */
    void clear() { this->queue.clear(); }

    /** Number of steps still queued. */
    size_t remaining() const { return this->queue.size(); }

    GaussianNoiseSample generate(size_t m, double) override
    {
        if (this->queue.empty()) {
            throw std::runtime_error(
                "PrescribedGaussianNoiseGenerator ran out of prescribed noise samples. "
                "Push one sample per integration step.");
        }
        GaussianNoiseSample sample = this->queue.front();
        this->queue.pop_front();

        if (static_cast<size_t>(sample.dW.size()) != m) {
            throw std::runtime_error(
                "PrescribedGaussianNoiseGenerator: the queued sample has a different number of "
                "noise sources than requested by the integrator.");
        }
        // The higher-order methods also read dZ (per source, and W2Ito reads dZ(0)/dZ(1)),
        // so dZ must hold at least m entries. pushStep zero-fills an omitted dZ to m, so
        // this only rejects an explicitly under-sized dZ (an out-of-bounds read otherwise).
        if (static_cast<size_t>(sample.dZ.size()) < m) {
            throw std::runtime_error(
                "PrescribedGaussianNoiseGenerator: the queued sample's dZ is shorter than the "
                "number of noise sources requested by the integrator.");
        }
        return sample;
    }

protected:
    /** FIFO of prescribed increments, one entry per integration step. */
    std::deque<GaussianNoiseSample> queue;
};

#endif /* stochasticNoiseGenerator_h */
