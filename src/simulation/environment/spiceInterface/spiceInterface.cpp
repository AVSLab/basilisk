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
#include "simulation/environment/spiceInterface/spiceInterface.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>
#include "SpiceUsr.h"
#include <string.h>
#include "architecture/utilities/simDefinitions.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "spiceInterface.h"

namespace {
    /**
     * RAII guard for SPICE error mode.
     *
     * Sets SPICE error action to RETURN while the guard is alive so that
     * calls report failures via failed_c() instead of aborting. Restores
     * the previous error action and print settings on destruction.
     */
    struct SpiceErrorModeGuard
    {
        char oldAction[32];
        char oldPrint[32];

        SpiceErrorModeGuard()
        {
            erract_c("GET", sizeof(oldAction), oldAction);
            errprt_c("GET", sizeof(oldPrint),  oldPrint);

            // Only override the abort behavior
            erract_c("SET", 0, const_cast<char*>("RETURN"));
            // DO NOT suppress printing: errprt is left untouched
        }

        ~SpiceErrorModeGuard()
        {
            erract_c("SET", 0, oldAction);
            errprt_c("SET", 0, oldPrint);
        }
    };

    /**
     * Normalize a file system path to a canonical absolute string.
     *
     * Used to key kernels so that one physical file maps to a single
     * cache entry even if referenced through different relative paths.
     */
    std::string absolutize(const std::filesystem::path& path)
    {
        return std::filesystem::absolute(path).lexically_normal().string();
    }

    //! Uppercase-normalized body name, used as the reconstruction map key (case-insensitive).
    std::string normalizeBodyName(const std::string& name)
    {
        std::string out = name;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
        return out;
    }
}

/*! @brief Opt-in SPICE query reconstruction state for SpiceInterface (PIMPL).
 *
 * Holds, per planet and per channel, the reconstruction configuration and a rolling cache of
 * SPICE "knots", and does the interpolation/extrapolation. It lives entirely in this translation
 * unit so the SpiceInterface header carries only an opaque pointer (no Eigen/STL internals and no
 * SWIG guards). It reads the owning interface's frame/observer settings and logs through it, but
 * owns all reconstruction-specific state itself. */
class SpiceInterface::SpiceReconstruction {
public:
    //! Construct bound to the owning interface (used for frame/observer settings and logging).
    explicit SpiceReconstruction(SpiceInterface& owner) : owner(owner) {}

    /*! How a channel (position or orientation) of a planet is produced. */
    enum class ChannelMode {
        Exact,        //!< query SPICE at the exact time every call (the default)
        Reconstruct,  //!< reconstruct from a coarse cached knot grid
        Disabled      //!< emit the trivial default (zero position / identity attitude), no SPICE query
    };

    /*! Reconstruction configuration for one channel. */
    struct ChannelCfg {
        ChannelMode mode = ChannelMode::Exact;  //!< how this channel is produced
        uint64_t knotStep = SpiceInterface::defaultKnotStep;  //!< [ns] absolute-time knot spacing
        int nKnots = 2;            //!< stencil size / polynomial order; clamped to >= 1
        bool useVelocity = true;   //!< match the SPICE velocity at each knot (Hermite) vs positions only (Lagrange)
        bool interpolate = true;   //!< bracket with past+future knots (interp) vs past-only (extrapolation)
    };

    /*! One cached position knot: translation state at a knot epoch. */
    struct PosKnot {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();  //!< [m]
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();  //!< [m/s]
        bool ok = false;  //!< true once a SPICE query succeeded; a false (fallback) knot is re-queried
    };
    /*! One cached orientation knot: planet-fixed-relative-to-inertial DCM and its rate. */
    struct OrientKnot {
        Eigen::Matrix3d dcm = Eigen::Matrix3d::Identity();  //!< J20002Pfix
        Eigen::Matrix3d dcmDot = Eigen::Matrix3d::Zero();   //!< J20002Pfix_dot
        bool ok = false;  //!< true once a SPICE query succeeded; a false (fallback) knot is re-queried
    };

    /*! Per-planet reconstruction state: independent position and orientation channels, each with
     * its own config and rolling knot cache. SPICE-query tallies live on the owning SpiceInterface
     * (SpiceInterface::spiceQueryCounts), so they cover the exact path as well as knot fills. */
    struct PlanetInterp {
        ChannelCfg posCfg;                          //!< position-channel configuration
        ChannelCfg orientCfg;                       //!< orientation-channel configuration
        std::map<long, PosKnot> posKnots;           //!< cached position knots, keyed by knot index
        std::map<long, OrientKnot> orientKnots;     //!< cached orientation knots, keyed by knot index
    };

    //! Look up (creating if absent) the entry for a planet name.
    PlanetInterp& entry(const std::string& planetName)
    {
        return this->planets[normalizeBodyName(planetName)];
    }

    //! Find an existing entry, or nullptr if the planet is not configured.
    PlanetInterp* find(const std::string& planetName)
    {
        auto it = this->planets.find(normalizeBodyName(planetName));
        return (it == this->planets.end()) ? nullptr : &it->second;
    }

    //! Validate knotStep and set one channel of a planet to Reconstruct mode with the given knobs.
    //! A zero knotStep is rejected (logged, channel left unchanged), shared by the two setters.
    void configure(const std::string& planetName, ChannelCfg PlanetInterp::* channel,
                   uint64_t knotStep, int nKnots, bool useVelocity, bool interpolate)
    {
        if (knotStep == 0) {
            owner.bskLogger.bskError("spiceInterface: knotStep must be positive for planet '%s'.",
                                     planetName.c_str());
            return;
        }
        PlanetInterp& pi = entry(planetName);
        ChannelCfg& cfg = pi.*channel;
        cfg.mode = ChannelMode::Reconstruct;
        cfg.knotStep = knotStep;
        cfg.nKnots = std::max(1, nKnots);
        cfg.useVelocity = useVelocity;
        cfg.interpolate = interpolate;
        // Cached knots are keyed by integer index on the OLD grid, so a changed knotStep would make
        // those indices map to different epochs; drop this channel's cache so it re-queries fresh.
        if (channel == &PlanetInterp::posCfg) {
            pi.posKnots.clear();
        } else {
            pi.orientKnots.clear();
        }
    }

    //! Anchor the knot grid at the given init epoch (ET seconds past J2000) without touching the
    //! caches. Used when reconstruction is first enabled AFTER Reset has already set the epoch, so
    //! a setter called between two ExecuteSimulation() segments does not leave et0 at 0.
    void setEpoch(double et0) { this->et0 = et0; }

    //! Anchor the knot grid at the init epoch and clear every cache (called from Reset). Also
    //! validates configured names against the planets added, and warns if orientation
    //! reconstruction was requested for a body with no orientation frame.
    void reset(double et0, const std::vector<SpicePlanetStateMsgPayload>& planetData)
    {
        this->et0 = et0;
        std::vector<std::string> stale;  // configured names not among the current planets
        for (auto& kv : this->planets) {
            kv.second.posKnots.clear();
            kv.second.orientKnots.clear();

            bool found = false, orientCapable = false;
            for (const auto& pd : planetData) {
                if (normalizeBodyName(pd.PlanetName) == kv.first) {
                    found = true;
                    orientCapable = (pd.computeOrient != 0);
                    break;
                }
            }
            // Misconfiguration is a warning, not a fatal error: leave the affected channel on the
            // exact/identity path and keep running (bskError would throw and abort the sim).
            if (!found) {
                owner.bskLogger.bskLog(BSK_WARNING, "spiceInterface: planet '%s' was configured "
                    "for reconstruction but was not added via addPlanetNames; ignoring it.",
                    kv.first.c_str());
                stale.push_back(kv.first);
            } else if (kv.second.orientCfg.mode == ChannelMode::Reconstruct && !orientCapable) {
                owner.bskLogger.bskLog(BSK_WARNING, "spiceInterface: planet '%s' orientation "
                    "reconstruction was requested, but the body has no orientation frame "
                    "(computeOrient is off); leaving orientation on the exact (identity) path.",
                    kv.first.c_str());
                kv.second.orientCfg.mode = ChannelMode::Exact;
            }
        }
        // Drop configs for planets no longer present, so the map shrinks on reconfiguration
        // rather than re-warning about the same stale name every Reset.
        for (const auto& key : stale) {
            this->planets.erase(key);
        }
    }

    //! Local knot coordinate kf for sim time tNanos on a grid of knotStep ns, computed from
    //! integer nanoseconds so that a time exactly on a knot yields an exact integer kf (no
    //! double-rounding to the wrong interval, which matters when knotStep is a multiple of dt).
    static double knotCoord(uint64_t tNanos, uint64_t knotStep)
    {
        uint64_t k = tNanos / knotStep;                 // exact integer floor for tNanos >= 0
        uint64_t rem = tNanos - k * knotStep;           // exact remainder
        return static_cast<double>(k) + static_cast<double>(rem) / static_cast<double>(knotStep);
    }

    //! Fill position/velocity at sim time tNanos per the channel mode. Returns true if handled
    //! (Reconstruct or Disabled); false means the caller should run the exact SPICE path.
    bool position(PlanetInterp& pi, const std::string& planetName, uint64_t tNanos,
                  double posOut[3], double velOut[3])
    {
        if (pi.posCfg.mode == ChannelMode::Exact) {
            return false;
        }
        if (pi.posCfg.mode == ChannelMode::Disabled) {
            posOut[0] = posOut[1] = posOut[2] = 0.0;
            velOut[0] = velOut[1] = velOut[2] = 0.0;
            return true;
        }

        const ChannelCfg& cfg = pi.posCfg;
        // Local knot coordinate (exact-integer at knot boundaries); kdSec is the knot spacing in
        // seconds, used to convert per-knot-unit derivatives back to per-second.
        const double kf = knotCoord(tNanos, cfg.knotStep);
        const double kdSec = static_cast<double>(cfg.knotStep) * NANO2SEC;
        std::vector<long> idx = buildStencil(kf, cfg.nKnots, cfg.interpolate);
        // If any stencil knot could not be queried (e.g. a look-ahead knot past kernel coverage),
        // do NOT interpolate through a bogus fallback: return false so the caller does an exact
        // SPICE query at the current time, which may still be within coverage.
        bool allOk = true;
        for (long i : idx) {
            allOk &= ensurePosKnot(pi, planetName, i);
        }
        if (!allOk) {
            return false;
        }

        // Confluent Newton nodes: each knot repeated when matching velocity (Hermite), the
        // velocity scaled to knot-spacing units so the polynomial derivative is per-knot-unit.
        const int reps = cfg.useVelocity ? 2 : 1;
        const size_t nNodes = idx.size() * static_cast<size_t>(reps);
        std::vector<double> z;
        std::vector<Eigen::Vector3d> val, der;
        z.reserve(nNodes); val.reserve(nNodes); der.reserve(nNodes);
        for (long i : idx) {
            const PosKnot& kn = pi.posKnots.at(i);  // ensured above; .at() fails loudly if not
            for (int r = 0; r < reps; ++r) {
                z.push_back(static_cast<double>(i));
                val.push_back(kn.pos);
                der.push_back(kn.vel * kdSec);
            }
        }
        Eigen::Vector3d dPos_dk;
        Eigen::Vector3d pos = newtonEval(newtonCoeffs(val, der, z), z, kf, &dPos_dk);
        // Emitted velocity = d/dt of the SAME polynomial that produced the position, so a
        // downstream Euler step stays self-consistent.
        Eigen::Vector3d vel = dPos_dk / kdSec;
        posOut[0] = pos[0]; posOut[1] = pos[1]; posOut[2] = pos[2];
        velOut[0] = vel[0]; velOut[1] = vel[1]; velOut[2] = vel[2];
        return true;
    }

    //! Fill DCM/DCM_dot at sim time tNanos per the channel mode. Returns true if handled
    //! (Reconstruct or Disabled); false means the caller should run the exact SPICE path.
    bool orientation(PlanetInterp& pi, const std::string& planetName,
                     const std::string& planetFrame, uint64_t tNanos,
                     double dcmOut[3][3], double dcmDotOut[3][3])
    {
        if (pi.orientCfg.mode == ChannelMode::Exact) {
            return false;
        }
        if (pi.orientCfg.mode == ChannelMode::Disabled) {
            m33SetIdentity(dcmOut);
            m33SetZero(dcmDotOut);
            return true;
        }

        const ChannelCfg& cfg = pi.orientCfg;
        const double kf = knotCoord(tNanos, cfg.knotStep);
        const double kdSec = static_cast<double>(cfg.knotStep) * NANO2SEC;
        std::vector<long> idx = buildStencil(kf, cfg.nKnots, cfg.interpolate);
        // If any stencil knot could not be queried (e.g. a look-ahead knot past kernel coverage),
        // do NOT interpolate through a bogus fallback: return false so the caller does an exact
        // sxform query at the current time, which may still be within coverage.
        bool allOk = true;
        for (long i : idx) {
            allOk &= ensureOrientKnot(pi, planetName, planetFrame, i);
        }
        if (!allOk) {
            return false;
        }

        // Base knot = the first stencil index, fixed for the WHOLE stencil. Lift each knot's
        // rotation to the tangent space about the base as a principal rotation vector (PRV) and
        // interpolate the PRV. The base must be constant across the stencil interval: PRV
        // polynomials taken about different bases are not equivalent for noncommuting rotations, so
        // a base that flipped mid-interval (e.g. "nearest knot", which switches at the half-knot)
        // would make the reconstructed DCM/rate jump there for a tumbler. Anchoring on idx[0]
        // instead keeps the reconstructed DCM a single smooth function of kf across the interval;
        // the rate is then a finite difference of THAT same function, keeping (dcm, dcmDot)
        // self-consistent without the delicate analytic PRV-rate -> omega inversion.
        const long base = idx[0];
        const Eigen::Matrix3d Cb = pi.orientKnots.at(base).dcm;  // ensured above; .at() fails loudly

        const int reps = cfg.useVelocity ? 2 : 1;
        const size_t nNodes = idx.size() * static_cast<size_t>(reps);
        std::vector<double> z;
        std::vector<Eigen::Vector3d> theta, dtheta;
        z.reserve(nNodes); theta.reserve(nNodes); dtheta.reserve(nNodes);
        for (long i : idx) {
            const OrientKnot& kn = pi.orientKnots.at(i);  // ensured above; .at() fails loudly
            Eigen::Matrix3d Rj = kn.dcm * Cb.transpose();
            double Rj_c[3][3];
            eigenMatrix3d2CArray(Rj, &Rj_c[0][0]);
            double theta_j_c[3];
            C2PRV(Rj_c, theta_j_c);
            Eigen::Vector3d theta_j(theta_j_c[0], theta_j_c[1], theta_j_c[2]);

            Eigen::Vector3d dtheta_j = Eigen::Vector3d::Zero();
            if (cfg.useVelocity) {
                // knot angular velocity from Cdot = -[w x] C -> w = unhat(-Cdot C^T); the relative
                // rotation Rj carries the same w since Cb is constant. Map omega to the PRV rate
                // with the exact B(theta) Jacobian (dPRV), using the B(0)=I limit at the base knot.
                Eigen::Matrix3d W = -kn.dcmDot * kn.dcm.transpose();
                double omega_j_c[3] = {W(2, 1), W(0, 2), W(1, 0)};
                if (theta_j.norm() < 1.0e-7) {
                    dtheta_j = Eigen::Vector3d(omega_j_c[0], omega_j_c[1], omega_j_c[2]) * kdSec;
                } else {
                    double dtheta_j_c[3];
                    dPRV(theta_j_c, omega_j_c, dtheta_j_c);
                    dtheta_j = Eigen::Vector3d(dtheta_j_c[0], dtheta_j_c[1], dtheta_j_c[2]) * kdSec;
                }
            }

            for (int r = 0; r < reps; ++r) {
                z.push_back(static_cast<double>(i));
                theta.push_back(theta_j);
                dtheta.push_back(dtheta_j);
            }
        }

        // Newton coefficients depend only on the nodes, so build them ONCE and reuse across the
        // three DCM samples below (kf, kf +/- h) -- the O(n^2) divided-difference table is not
        // rebuilt per sample. Absolute planet-fixed DCM at local coordinate kfEval on this stencil:
        const std::vector<Eigen::Vector3d> coef = newtonCoeffs(theta, dtheta, z);
        auto dcmAt = [&](double kfEval) -> Eigen::Matrix3d {
            Eigen::Vector3d th = newtonEval(coef, z, kfEval, nullptr);
            double th_c[3] = {th[0], th[1], th[2]};
            double Rrel_c[3][3];
            PRV2C(th_c, Rrel_c);
            return c2DArray2EigenMatrix3d(Rrel_c) * Cb;
        };

        Eigen::Matrix3d dcm = dcmAt(kf);
        eigenMatrix3d2CArray(dcm, &dcmOut[0][0]);

        // J20002Pfix_dot = d(dcm)/dt via a central difference of the same interpolant on this
        // stencil (d/dt = d/dk / kdSec). h is in KNOT units (a fixed fraction of a knot), so it is
        // scale-invariant in knotStep: small enough to resolve the polynomial slope, large enough
        // to avoid float cancellation, independent of the absolute knot spacing. The node set
        // (theta, dtheta, z) is fixed for this call, so dcmAt is well-defined for kf +/- h even
        // when kf is near a knot boundary (the rate is consistent with the reconstruction used at
        // THIS call, which is what a downstream Euler sub-step consumes).
        const double h = 1.0e-5;
        Eigen::Matrix3d dcmDot = (dcmAt(kf + h) - dcmAt(kf - h)) / (2.0 * h * kdSec);
        eigenMatrix3d2CArray(dcmDot, &dcmDotOut[0][0]);
        return true;
    }

private:
    //! Query + cache position knot k (spkezr_c, metres). Re-queries a previously-failed
    //! (fallback) knot so a transient / out-of-coverage failure does not poison the cache.
    //! Returns true if knot k holds a good (successfully queried) sample.
    bool ensurePosKnot(PlanetInterp& pi, const std::string& planetName, long k)
    {
        auto it = pi.posKnots.find(k);
        if (it != pi.posKnots.end() && it->second.ok) {
            return true;  // already have a good knot; a stored fallback (ok==false) falls through
        }
        // knotStep is in nanoseconds; the knot ET (seconds past J2000) is et0 + k*knotStep.
        double et = this->et0 + static_cast<double>(k) * pi.posCfg.knotStep * NANO2SEC;
        double lighttime;
        double localState[6];
        // Reconstruction may request a future knot (interpolate=true); guard against a query past
        // kernel coverage aborting the process, exactly as SpiceKernel does elsewhere.
        SpiceErrorModeGuard guard;
        spkezr_c(planetName.c_str(), et, owner.referenceBase.c_str(), "NONE",
                 owner.zeroBase.c_str(), localState, &lighttime);
        PosKnot kn;
        if (failed_c()) {
            reset_c();
            owner.bskLogger.bskLog(BSK_WARNING, "spiceInterface: position knot query failed for "
                "'%s' at ET %g; using zero state for that knot (will retry on later calls).",
                planetName.c_str(), et);
        } else {
            kn.pos = Eigen::Vector3d(localState[0], localState[1], localState[2]) * 1000.0;
            kn.vel = Eigen::Vector3d(localState[3], localState[4], localState[5]) * 1000.0;
            kn.ok = true;
        }
        pi.posKnots[k] = kn;
        owner.countSpiceQuery(planetName, true);
        evictKnots(pi.posKnots, pi.posCfg.nKnots, k);
        return kn.ok;
    }

    //! Query + cache orientation knot k (sxform_c). Re-queries a previously-failed (fallback)
    //! knot so a transient / out-of-coverage failure does not poison the cache.
    //! Returns true if knot k holds a good (successfully queried) sample.
    bool ensureOrientKnot(PlanetInterp& pi, const std::string& planetName,
                          const std::string& planetFrame, long k)
    {
        auto it = pi.orientKnots.find(k);
        if (it != pi.orientKnots.end() && it->second.ok) {
            return true;  // already have a good knot; a stored fallback (ok==false) falls through
        }
        double et = this->et0 + static_cast<double>(k) * pi.orientCfg.knotStep * NANO2SEC;
        double aux[6][6];
        SpiceErrorModeGuard guard;
        sxform_c(owner.referenceBase.c_str(), planetFrame.c_str(), et, aux);
        OrientKnot kn;
        if (failed_c()) {
            reset_c();
            owner.bskLogger.bskLog(BSK_WARNING, "spiceInterface: orientation knot query failed for "
                "'%s' (frame %s) at ET %g; using identity for that knot (will retry on later calls).",
                planetName.c_str(), planetFrame.c_str(), et);
        } else {
            double C[3][3], Cdot[3][3];
            m66Get33Matrix(0, 0, aux, C);
            m66Get33Matrix(1, 0, aux, Cdot);
            kn.dcm = c2DArray2EigenMatrix3d(C);
            kn.dcmDot = c2DArray2EigenMatrix3d(Cdot);
            kn.ok = true;
        }
        pi.orientKnots[k] = kn;
        owner.countSpiceQuery(planetName, false);
        evictKnots(pi.orientKnots, pi.orientCfg.nKnots, k);
        return kn.ok;
    }

    //! Drop knots far from index k, keeping the live stencil plus a little slack.
    template <typename KnotMap>
    static void evictKnots(KnotMap& knots, int nKnots, long k)
    {
        static const size_t CACHE_SLACK = 4;
        const size_t cap = static_cast<size_t>(std::max(1, nKnots)) + CACHE_SLACK;
        while (knots.size() > cap) {
            long lo = knots.begin()->first;
            long hi = knots.rbegin()->first;
            if (std::abs(hi - k) >= std::abs(lo - k)) {
                knots.erase(hi);
            } else {
                knots.erase(lo);
            }
        }
    }

    //! Trailing (extrapolation) or centred (interpolation) stencil of nKnots indices around kf.
    static std::vector<long> buildStencil(double kf, int nKnots, bool interpolate)
    {
        long k = static_cast<long>(std::floor(kf));  // current interval is [k, k+1)
        int m = std::max(1, nKnots);
        // A single knot is the enclosing knot k for both modes (interp and extrap coincide);
        // the general interpolation formula below would otherwise pick the future knot k+1.
        // Interpolation centres the m knots on the interval [k, k+1] (start = k - (m-1)/2, so
        // m=2 -> [k, k+1], m=3 -> [k-1, k, k+1], m=4 -> [k-1..k+2]); extrapolation takes the m
        // trailing knots ending at k.
        long start;
        if (m == 1) {
            start = k;
        } else {
            start = interpolate ? (k - (m - 1) / 2) : (k - (m - 1));
        }
        std::vector<long> idx;
        idx.reserve(static_cast<size_t>(m));
        for (int j = 0; j < m; ++j) {
            idx.push_back(start + j);
        }
        return idx;
    }

    //! Newton divided-difference coefficients for the (possibly confluent) nodes ``z`` with values
    //! ``vals`` and, at confluent nodes, derivatives ``ders``. The coefficients depend only on the
    //! nodes, not on any evaluation point, so they are computed once and reused across evaluations
    //! (e.g. the 3 samples of the orientation-rate finite difference).
    static std::vector<Eigen::Vector3d> newtonCoeffs(const std::vector<Eigen::Vector3d>& vals,
                                                     const std::vector<Eigen::Vector3d>& ders,
                                                     const std::vector<double>& z)
    {
        const size_t n = z.size();
        std::vector<Eigen::Vector3d> prev(vals);
        std::vector<Eigen::Vector3d> coef{prev[0]};
        for (size_t col = 1; col < n; ++col) {
            std::vector<Eigen::Vector3d> cur(n - col);
            for (size_t i = 0; i < n - col; ++i) {
                cur[i] = (z[i + col] == z[i])
                             ? ders[i]  // confluent node -> analytic derivative
                             : (prev[i + 1] - prev[i]) / (z[i + col] - z[i]);
            }
            coef.push_back(cur[0]);
            prev = std::move(cur);
        }
        return coef;
    }

    //! Horner evaluation of the Newton form (coefficients ``coef`` over nodes ``z``) at ``x``.
    //! When derivOut != nullptr, also returns d/dx via the product-rule Horner recurrence.
    static Eigen::Vector3d newtonEval(const std::vector<Eigen::Vector3d>& coef,
                                      const std::vector<double>& z, double x,
                                      Eigen::Vector3d* derivOut)
    {
        const size_t n = z.size();
        Eigen::Vector3d result = coef[n - 1];
        Eigen::Vector3d deriv = Eigen::Vector3d::Zero();
        for (size_t i = n - 1; i-- > 0;) {
            if (derivOut) {
                deriv = deriv * (x - z[i]) + result;  // product rule, before updating result
            }
            result = result * (x - z[i]) + coef[i];
        }
        if (derivOut) {
            *derivOut = deriv;
        }
        return result;
    }

    SpiceInterface& owner;                            //!< back-reference for frame/observer/logger
    std::map<std::string, PlanetInterp> planets;      //!< per-planet state, keyed by uppercase name
    double et0 = 0.0;                                 //!< ET (s past J2000) at sim time 0
};

SpiceInterface::SpiceReconstruction& SpiceInterface::reconState()
{
    if (!this->recon) {
        this->recon = std::make_unique<SpiceReconstruction>(*this);
        // If Reset has already run (time data initialized), anchor the knot grid at the init epoch
        // now. Otherwise et0 stays 0 until Reset calls reset(); enabling reconstruction between two
        // ExecuteSimulation() segments would otherwise query knots at J2000 ET 0.
        if (this->timeDataInit) {
            this->recon->setEpoch(this->J2000ETInit);
        }
    }
    return *this->recon;
}

/*! This constructor initializes the variables that spice uses.  Most of them are
 not intended to be changed, but a couple are user configurable.
 */

SpiceInterface::SpiceInterface()
{
    SPICEDataPath = "";
    SPICELoaded = false;
    charBufferSize = 512;
    CallCounts = 0;
    J2000ETInit = 0;
    J2000Current = 0.0;
    julianDateCurrent = 0.0;
    GPSSeconds = 0.0;
    GPSWeek = 0;
    GPSRollovers = 0;
    spiceBuffer = new uint8_t[charBufferSize];
    timeDataInit = false;
    JDGPSEpoch = 0.0;
    GPSEpochTime = "1980 January 6, 00:00:00.0";

    referenceBase = "j2000";
    zeroBase = "SSB";
	timeOutPicture = "MON DD,YYYY  HR:MN:SC.#### (UTC) ::UTC";

    //! - set default epoch time information
    char string[255];
    snprintf(string, 255, "%4d/%02d/%02d, %02d:%02d:%04.1f (UTC)", EPOCH_YEAR, EPOCH_MONTH, EPOCH_DAY, EPOCH_HOUR, EPOCH_MIN, EPOCH_SEC);
    this->UTCCalInit = string;

    return;
}

/*! The only needed activity in the destructor is to delete the spice I/O buffer
 that was allocated in the constructor*/
SpiceInterface::~SpiceInterface()
{
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++) {
        delete this->planetStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++) {
        delete this->scStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->attRefStateOutMsgs.size(); c++) {
        delete this->attRefStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->transRefStateOutMsgs.size(); c++) {
        delete this->transRefStateOutMsgs.at(c);
    }
    delete [] this->spiceBuffer;
}

void SpiceInterface::clearKeeper()
{
    kclear_c();
}


/*! Reset the module to origina configuration values.

 */
/*! Reset the module to original configuration values. */
void SpiceInterface::Reset(uint64_t CurrenSimNanos)
{
    // Allow explicit kernels (full paths) to bypass SPICEDataPath requirement
    const bool hasExplicitKernels = !this->kernelPaths.empty();

    //! - Bail if the SPICEDataPath is not present
    if (!hasExplicitKernels && this->SPICEDataPath == "")
    {
        bskLogger.bskError("SPICE data path was not set.  No SPICE.");
    }

    //!- Load the SPICE kernels if they haven't already been loaded
    if(!this->SPICELoaded)
    {
        if (hasExplicitKernels)
        {
            // Load kernels by explicit path list
            for (const auto& kp : this->kernelPaths)
            {
                auto kernel = SpiceKernel::request(std::filesystem::path(kp));
                if (!kernel->wasLoadSuccesful()) {
                    bskLogger.bskError("Unable to load SPICE kernel: %s", kp.c_str());
                    continue;
                }
                this->loadedKernels[kernel->getPath()] = kernel;
            }
        }
        else
        {
            // Load default kernels from SPICEDataPath
            if(loadSpiceKernel((char *)"naif0012.tls", this->SPICEDataPath.c_str())) {
                bskLogger.bskError("Unable to load %s", "naif0012.tls");
            }
            if(loadSpiceKernel((char *)"pck00010.tpc", this->SPICEDataPath.c_str())) {
                bskLogger.bskError("Unable to load %s", "pck00010.tpc");
            }
            if(loadSpiceKernel((char *)"de-403-masses.tpc", this->SPICEDataPath.c_str())) {
                bskLogger.bskError("Unable to load %s", "de-403-masses.tpc");
            }
            if(loadSpiceKernel((char *)"de430.bsp", this->SPICEDataPath.c_str())) {
                bskLogger.bskError("Unable to load %s", "de430.tpc");
            }
        }

        this->SPICELoaded = true;
    }

    //! Set the zero time values that will be used to compute the system time
    this->initTimeData();
    this->J2000Current = this->J2000ETInit;
    this->timeDataInit = true;

    std::vector<SpicePlanetStateMsgPayload>::iterator planit;
    size_t c = 0;  // celestial object counter
    int autoFrame;  // flag to set the frame automatically
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    SpiceBoolean frmFound;
    SpiceInt frmCode;
    for(planit = this->planetData.begin(); planit != planetData.end(); planit++)
    {
        autoFrame = 1;
        planit->computeOrient = 0;  // turn off by default
        if (this->planetFrames.size() > 0) {
            if (c < this->planetFrames.size()) {
                if (this->planetFrames[c].length() > 0) {
                    planit->computeOrient = 1;  // turn on as a custom name is provided
                    autoFrame = 0;
                }
            }
        }
        if (autoFrame > 0) {
            std::string planetFrame = planit->PlanetName;
            cnmfrm_c(planetFrame.c_str(), this->charBufferSize, &frmCode, name, &frmFound);
            planit->computeOrient = frmFound;  // set the flag to the Spice response on finding this frame
        }
        c++;
    }
    delete [] name;

    //! - Zero the SPICE-query tallies so each run counts from its own Reset, seeding an entry for
    //!   every known planet. A planet present but never queried (e.g. a disabled channel) then reads
    //!   {0, 0}; only a name that was never added reads {-1, -1}. The UpdateState below is part of
    //!   the run, so its queries are included.
    this->spiceQueryCounts.clear();
    for (const auto& pd : this->planetData) {
        this->spiceQueryCounts[normalizeBodyName(pd.PlanetName)] = {0, 0};
    }

    //! - Prepare reconstruction (if any planet is configured): anchor the knot grid at the init
    //!   epoch and clear every cache so a re-Reset starts fresh.
    if (this->recon) {
        this->recon->reset(this->J2000ETInit, this->planetData);
    }

    // - Call Update state so that the spice bodies are inputted into the messaging system on reset
    this->UpdateState(CurrenSimNanos);
}

/*! This method is used to initialize the zero-time that will be used to
 calculate all system time values in the Update method.  It also creates the
 output message for time data

 */
void SpiceInterface::initTimeData()
{
    double EpochDelteET;

    /* set epoch information.  If provided, then the epoch message information should be used.  */
    if (this->epochInMsg.isLinked()) {
        // Read in the epoch message and set the internal time structure
        EpochMsgPayload epochMsg;
        epochMsg = this->epochInMsg();
        if (!this->epochInMsg.isWritten()) {
            bskLogger.bskError("The input epoch message name was set, but the message was never written.  Not using the input message.");
        } else {
            // Set the epoch information from the input message
            char string[255];
            snprintf(string, 255, "%4d/%02d/%02d, %02d:%02d:%04.6f (UTC)", epochMsg.year, epochMsg.month, epochMsg.day, epochMsg.hours, epochMsg.minutes, epochMsg.seconds);
            this->UTCCalInit = string;
        }
    }

    //! -Get the time value associated with the GPS epoch
    str2et_c(this->GPSEpochTime.c_str(), &this->JDGPSEpoch);
    //! - Get the time value associate with the requested UTC date
    str2et_c(this->UTCCalInit.c_str(), &this->J2000ETInit);
    //! - Take the JD epoch and get the elapsed time for it
    deltet_c(this->JDGPSEpoch, "ET", &EpochDelteET);

}

/*! This method computes the GPS time data for the current elapsed time.  It uses
 the total elapsed times at both the GPS epoch time and the current time to
 compute the GPS time (week, seconds, rollovers)

 */
void SpiceInterface::computeGPSData()
{
    double JDDifference;

    //! - The difference between the epochs in julian date terms is the total
    JDDifference = this->J2000Current - this->JDGPSEpoch;
    //! - Scale the elapsed by a week's worth of seconds to get week
    this->GPSWeek = (uint16_t) (JDDifference/(7*86400));
    //! - Subtract out the GPS week scaled up to seconds to get time in week
    this->GPSSeconds = JDDifference - this->GPSWeek*7*86400;

    //! - Maximum GPS week is 1024 so get rollovers and subtract out those weeks
    this->GPSRollovers = this->GPSWeek/1024;
    this->GPSWeek = (uint16_t)(this->GPSWeek-this->GPSRollovers*1024);
}

/*! This method takes the values computed in the model and outputs them.
 It packages up the internal variables into the output structure definitions
 and puts them out on the messaging system

 @param CurrentClock The current simulation time (used for time stamping)
 */
void SpiceInterface::writeOutputMessages(uint64_t CurrentClock)
{
    SpiceTimeMsgPayload OutputData;

    //! - Set the members of the time output message structure and write
    OutputData.J2000Current = this->J2000Current;
    OutputData.JulianDateCurrent = this->julianDateCurrent;
    OutputData.GPSSeconds = this->GPSSeconds;
    OutputData.GPSWeek = this->GPSWeek;
    OutputData.GPSRollovers = this->GPSRollovers;
    this->spiceTimeOutMsg.write(&OutputData, this->moduleID, CurrentClock);

    //! - Iterate through all of the planets that are on and write their outputs
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++)
    {
        this->planetStateOutMsgs[c]->write(&this->planetData[c], this->moduleID, CurrentClock);
    }

    //! - Iterate through all of the spacecraft that are on and write their outputs
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++)
    {
        SCStatesMsgPayload scStateMsgData = {};
        v3Copy(this->scData[c].PositionVector, scStateMsgData.r_BN_N);
        v3Copy(this->scData[c].PositionVector, scStateMsgData.r_CN_N);
        v3Copy(this->scData[c].VelocityVector, scStateMsgData.v_BN_N);
        v3Copy(this->scData[c].VelocityVector, scStateMsgData.v_CN_N);
        C2MRP(this->scData[c].J20002Pfix, scStateMsgData.sigma_BN);
        this->scStateOutMsgs[c]->write(&scStateMsgData, this->moduleID, CurrentClock);

        AttRefMsgPayload attRefMsgData = {};
        C2MRP(this->scData[c].J20002Pfix, attRefMsgData.sigma_RN);
        this->attRefStateOutMsgs[c]->write(&attRefMsgData, this->moduleID, CurrentClock);

        TransRefMsgPayload transRefMsgData = {};
        v3Copy(this->scData[c].PositionVector, transRefMsgData.r_RN_N);
        v3Copy(this->scData[c].VelocityVector, transRefMsgData.v_RN_N);
        this->transRefStateOutMsgs[c]->write(&transRefMsgData, this->moduleID, CurrentClock);

    }
}

/*! This method is the interface point between the upper level simulation and
 the SPICE interface at runtime.  It calls all of the necessary lower level
 methods.

 @param CurrentSimNanos The current clock time for the simulation
 */
void SpiceInterface::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Increment the J2000 elapsed time based on init value and Current sim
    this->J2000Current = this->J2000ETInit + CurrentSimNanos*NANO2SEC;

    //! - Compute the current Julian Date string and cast it over to the double
    et2utc_c(this->J2000Current, "J", 14, this->charBufferSize - 1, reinterpret_cast<SpiceChar*>
             (this->spiceBuffer));
    std::string localString = reinterpret_cast<char*> (&this->spiceBuffer[3]);
    this->julianDateCurrent = std::stod(localString);
    //! Get GPS and Planet data and then write the message outputs. Reconstruction is allowed only
    //! on the planet pass (the raw sim time drives the knot grid; the spacecraft pass stays exact).
    this->computeGPSData();
    this->pullSpiceData(&this->planetData, CurrentSimNanos, true);
    this->pullSpiceData(&this->scData, CurrentSimNanos, false);
    this->writeOutputMessages(CurrentSimNanos);
}

/*! take a vector of planet name strings and create the vector of
    planet state output messages and the vector of planet state message payloads */
void SpiceInterface::addPlanetNames(std::vector<std::string> planetNames) {
    std::vector<std::string>::iterator it;

    /* clear the planet state message and payload vectors */
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++) {
        delete this->planetStateOutMsgs.at(c);
    }
    this->planetStateOutMsgs.clear();
    this->planetData.clear();

    for (it = planetNames.begin(); it != planetNames.end(); it++) {
        Message<SpicePlanetStateMsgPayload> *spiceOutMsg;
        spiceOutMsg = new Message<SpicePlanetStateMsgPayload>;
        this->planetStateOutMsgs.push_back(spiceOutMsg);

        SpicePlanetStateMsgPayload newPlanet = {};
        m33SetIdentity(newPlanet.J20002Pfix);
        if(it->size() >= MAX_BODY_NAME_LENGTH)
        {
            bskLogger.bskError("spiceInterface: planet name is %zu characters, but the SPICE payload name "
                             "supports at most %d characters.",
                             it->size(), MAX_BODY_NAME_LENGTH - 1);
        }
        std::snprintf(newPlanet.PlanetName, sizeof(newPlanet.PlanetName), "%s", it->c_str());

        this->planetData.push_back(newPlanet);
    }

    return;
}

/*! take a vector of spacecraft name strings and create the vectors of
    spacecraft state output messages and the vector of spacecraft state message payloads */
void SpiceInterface::addSpacecraftNames(std::vector<std::string> spacecraftNames) {
    std::vector<std::string>::iterator it;
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    SpiceBoolean frmFound;
    SpiceInt frmCode;

    /* clear the spacecraft state message and payload vectors */
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++) {
        delete this->scStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->attRefStateOutMsgs.size(); c++) {
        delete this->attRefStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->transRefStateOutMsgs.size(); c++) {
        delete this->transRefStateOutMsgs.at(c);
    }
    this->scStateOutMsgs.clear();
    this->attRefStateOutMsgs.clear();
    this->transRefStateOutMsgs.clear();
    this->scData.clear();

    for (it = spacecraftNames.begin(); it != spacecraftNames.end(); it++) {
        /* append to spacecraft related output messages */
        Message<SCStatesMsgPayload> *scStateOutMsg;
        scStateOutMsg = new Message<SCStatesMsgPayload>;
        this->scStateOutMsgs.push_back(scStateOutMsg);

        Message<AttRefMsgPayload> *attRefOutMsg;
        attRefOutMsg = new Message<AttRefMsgPayload>;
        this->attRefStateOutMsgs.push_back(attRefOutMsg);

        Message<TransRefMsgPayload> *transRefOutMsg;
        transRefOutMsg = new Message<TransRefMsgPayload>;
        this->transRefStateOutMsgs.push_back(transRefOutMsg);

        SpicePlanetStateMsgPayload newSpacecraft = {};
        m33SetIdentity(newSpacecraft.J20002Pfix);
        if(it->size() >= MAX_BODY_NAME_LENGTH)
        {
            bskLogger.bskError("spiceInterface: spacecraft name is %zu characters, but the SPICE payload name "
                             "supports at most %d characters.",
                             it->size(), MAX_BODY_NAME_LENGTH - 1);
        }
        std::snprintf(newSpacecraft.PlanetName, sizeof(newSpacecraft.PlanetName), "%s", it->c_str());

        std::string planetFrame = *it;
        cnmfrm_c(planetFrame.c_str(), this->charBufferSize, &frmCode, name, &frmFound);
        newSpacecraft.computeOrient = frmFound;
        this->scData.push_back(newSpacecraft);
    }
    delete [] name;

    return;
}

void SpiceInterface::addKernelPath(const std::string& kernelPath)
{
    if (kernelPath.empty()) {
        bskLogger.bskLog(BSK_WARNING, "spiceInterface: ignoring empty kernel path");
        return;
    }
    this->kernelPaths.push_back(kernelPath);
}

void SpiceInterface::addKernelPaths(const std::vector<std::string>& kernelPaths)
{
    for (const auto& k : kernelPaths) {
        addKernelPath(k);
    }
}

void SpiceInterface::clearKernelPaths()
{
    this->kernelPaths.clear();
    this->configuredLoadedKernelKeys.clear();
}


/*! This method gets the state of each spice item that has been added to the module
 and saves the information off into the array.

 */
void SpiceInterface::pullSpiceData(std::vector<SpicePlanetStateMsgPayload> *spiceData,
                                   uint64_t CurrentSimNanos, bool allowReconstruction)
{
    std::vector<SpicePlanetStateMsgPayload>::iterator planit;

    /*! - Loop over the vector of Spice objects and compute values.

     -# Call the Ephemeris file (spkezr)
     -# Copy out the position and velocity values (default in km)
     -# Convert the pos/vel over to meters.
     -# Time stamp the message appropriately
     */
    //! Reconstruction is opt-in and the caller says whether it is allowed for this pass: the
    //! planet pass allows it, the spacecraft pass never does (so a configured planet engine can
    //! never overwrite a spacecraft's state). The raw sim time drives the knot grid.
    const uint64_t tNanos = CurrentSimNanos;

    size_t c = 0; // celestial body counter
    for(planit = spiceData->begin(); planit != spiceData->end(); planit++)
    {
        //! Look up this planet's reconstruction config (when allowed). A null entry, or a
        //! channel left in Exact mode, runs the original SPICE path verbatim -> bit-for-bit
        //! identical to the exact SPICE path when nothing is configured.
        SpiceReconstruction::PlanetInterp* pi = nullptr;
        if (allowReconstruction && this->recon) {
            pi = this->recon->find(planit->PlanetName);
        }

        /* use default IAU planet frame name */
        std::string planetFrame = "IAU_";
        planetFrame += planit->PlanetName;

        /* use specific planet frame if specified */
        if (this->planetFrames.size() > 0) {
            if (c < this->planetFrames.size()) {
                if (this->planetFrames[c].length() > 0){
                    /* use custom planet frame name */
                    planetFrame = this->planetFrames[c];
                }
            }
        }

        //! --- POSITION channel ---
        if (pi && this->recon->position(*pi, planit->PlanetName, tNanos,
                                        planit->PositionVector, planit->VelocityVector)) {
            // handled by reconstruction or disabled (values already written)
        } else {
            double lighttime;
            double localState[6];
            spkezr_c(planit->PlanetName, this->J2000Current, this->referenceBase.c_str(),
                "NONE", this->zeroBase.c_str(), localState, &lighttime);
            v3Copy(&localState[0], planit->PositionVector);
            v3Copy(&localState[3], planit->VelocityVector);
            v3Scale(1000., planit->PositionVector, planit->PositionVector);
            v3Scale(1000., planit->VelocityVector, planit->VelocityVector);
            this->countSpiceQuery(planit->PlanetName, true);
        }

        //! J2000Current (message validity time) is always the current sim time, regardless of
        //! reconstruction -- unchanged from the original behavior.
        planit->J2000Current = this->J2000Current;

        //! --- ORIENTATION channel --- (still gated on computeOrient, so a body with no frame
        //! keeps identity J20002Pfix / zero rate exactly as before)
        if(planit->computeOrient)
        {
            if (pi && this->recon->orientation(*pi, planit->PlanetName, planetFrame, tNanos,
                                               planit->J20002Pfix, planit->J20002Pfix_dot)) {
                // handled by reconstruction or disabled (values already written)
            } else {
                //pxform_c ( referenceBase.c_str(), planetFrame.c_str(), J2000Current,
                //    planit->second.J20002Pfix);

                double aux[6][6];

                sxform_c(this->referenceBase.c_str(), planetFrame.c_str(), this->J2000Current, aux); //returns attitude of planet (i.e. IAU_EARTH) wrt "j2000". note j2000 is actually ICRF in Spice.

                m66Get33Matrix(0, 0, aux, planit->J20002Pfix);

                m66Get33Matrix(1, 0, aux, planit->J20002Pfix_dot);
                this->countSpiceQuery(planit->PlanetName, false);
            }
        }
        c++;
    }
}

// ---------------------------------------------------------------------------
//  SPICE query reconstruction (opt-in, per planet, per channel)
// ---------------------------------------------------------------------------

void SpiceInterface::setPlanetPositionReconstruction(const std::string& planetName,
                                                     uint64_t knotStep, int nKnots,
                                                     bool useVelocity, bool interpolate)
{
    this->reconState().configure(planetName, &SpiceReconstruction::PlanetInterp::posCfg,
                                 knotStep, nKnots, useVelocity, interpolate);
}

void SpiceInterface::setPlanetOrientationReconstruction(const std::string& planetName,
                                                        uint64_t knotStep, int nKnots,
                                                        bool useVelocity, bool interpolate)
{
    this->reconState().configure(planetName, &SpiceReconstruction::PlanetInterp::orientCfg,
                                 knotStep, nKnots, useVelocity, interpolate);
}

void SpiceInterface::setPlanetPositionDisabled(const std::string& planetName)
{
    this->reconState().entry(planetName).posCfg.mode = SpiceReconstruction::ChannelMode::Disabled;
}

void SpiceInterface::setPlanetOrientationDisabled(const std::string& planetName)
{
    this->reconState().entry(planetName).orientCfg.mode = SpiceReconstruction::ChannelMode::Disabled;
}

void SpiceInterface::setPlanetExactSpice(const std::string& planetName)
{
    auto& pi = this->reconState().entry(planetName);
    pi.posCfg.mode = SpiceReconstruction::ChannelMode::Exact;
    pi.orientCfg.mode = SpiceReconstruction::ChannelMode::Exact;
}

void SpiceInterface::countSpiceQuery(const std::string& planetName, bool isPosition)
{
    auto& counts = this->spiceQueryCounts[normalizeBodyName(planetName)];
    if (isPosition) {
        counts.first++;
    } else {
        counts.second++;
    }
}

std::pair<long, long> SpiceInterface::getPlanetSpiceQueryCount(const std::string& planetName)
{
    auto it = this->spiceQueryCounts.find(normalizeBodyName(planetName));
    if (it == this->spiceQueryCounts.end()) {
        return {-1, -1};  // this planet has never been queried
    }
    return it->second;
}

/**
 * Load a SPICE kernel for use by this interface.
 *
 * This function takes a kernel file name and a base directory and
 * ensures that the corresponding SPICE kernel is available to the
 * simulation. Internally the module keeps track of which kernels it
 * has already loaded so that the same file is not loaded multiple
 * times.
 *
 * @param kernelName File name of the kernel inside dataPath.
 * @param dataPath   Directory where the kernel is located.
 * @return 0 on success, 1 if loading the kernel failed.
 */
int SpiceInterface::loadSpiceKernel(char *kernelName, const char *dataPath)
{
    std::filesystem::path base(dataPath);
    std::filesystem::path fullPath = base / kernelName;
    auto kernel = SpiceKernel::request(fullPath.string());
    if (!kernel->wasLoadSuccesful()) return 1;
    this->loadedKernels[kernel->getPath()] = kernel;
    return 0;
}

/**
 * Tell this interface that a SPICE kernel is no longer needed.
 *
 * This function removes the kernel from the set of kernels managed
 * by this interface. Once no users remain, the underlying kernel is
 * also removed from SPICE so it no longer affects future queries.
 *
 * @param kernelName File name of the kernel inside dataPath.
 * @param dataPath   Directory where the kernel is located.
 * @return always 0.
 */
int SpiceInterface::unloadSpiceKernel(char *kernelName, const char *dataPath)
{
    std::filesystem::path base(dataPath);
    std::filesystem::path fullPath = base / kernelName;
    auto key = absolutize(fullPath);
    this->loadedKernels.erase(key);
    return 0;
}

std::string SpiceInterface::getCurrentTimeString()
{
	char *spiceOutputBuffer;
	int64_t allowedOutputLength;

	allowedOutputLength = (int64_t)this->timeOutPicture.size() - 5;

	if (allowedOutputLength < 0)
	{
        bskLogger.bskError("The output format string is not long enough. It should be much larger than 5 characters.  It is currently: %s", this->timeOutPicture.c_str());
	}

	spiceOutputBuffer = new char[allowedOutputLength];
	timout_c(this->J2000Current, this->timeOutPicture.c_str(), (SpiceInt) allowedOutputLength,
		spiceOutputBuffer);
	std::string returnTimeString = spiceOutputBuffer;
	delete[] spiceOutputBuffer;
	return(returnTimeString);
}

std::mutex SpiceKernel::mutex;
std::unordered_map<std::string, std::weak_ptr<SpiceKernel>> SpiceKernel::cache;

std::shared_ptr<SpiceKernel>
SpiceKernel::request(const std::filesystem::path& path)
{
    const std::string key = absolutize(path);

    std::lock_guard<std::mutex> lock(mutex);

    auto it = cache.find(key);
    if (it != cache.end())
    {
        if (auto existing = it->second.lock())
        {
            // Already have a live handle to this kernel
            return existing;
        }
        // Weak pointer expired - fall through and create a new one
    }

    // First live handle for this absolute path in this process
    auto handle = std::shared_ptr<SpiceKernel>(new SpiceKernel(key));

    if (handle->loadSucceeded) cache[key] = handle;

    return handle;
}

SpiceKernel::~SpiceKernel() noexcept
{
    if (!loadSucceeded) return;

    std::lock_guard<std::mutex> lock(mutex);

    SpiceErrorModeGuard guard;
    unload_c(path.c_str());
    if (failed_c())
    {
        reset_c();   // SPICE printed its own messages already
    }
}

SpiceKernel::SpiceKernel(std::string path_)
    : path(std::move(path_))
{
    SpiceErrorModeGuard guard;
    furnsh_c(path.c_str());

    if (failed_c())
    {
        reset_c();             // SPICE already printed diagnostics
        loadSucceeded = false; // destructor will not unload
    }
    else
    {
        loadSucceeded = true;
    }
}
