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

#ifndef SpiceInterface_H
#define SpiceInterface_H

#include <vector>
#include <map>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <set>
#include <utility>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenSupport.h"

#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SpiceTimeMsgPayload.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/TransRefMsgPayload.h"
#include "architecture/messaging/messaging.h"

/**
 * Thin RAII wrapper around a single SPICE kernel.
 *
 * The class furnishes a kernel on construction and unloads it on
 * destruction, and provides a static request function that caches
 * instances by canonical absolute path so that a given kernel file is
 * not furnished multiple times.
 */
class SpiceKernel
{
public:
    /**
     * Request a shared handle for the kernel at the given path.
     *
     * The first call for a canonical path constructs a SpiceKernel, which
     * furnishes the kernel once. Later calls for the same path reuse the
     * existing instance as long as it is still alive.
     */
    static std::shared_ptr<SpiceKernel> request(const std::filesystem::path& path);

    /**
     * Destructor unloads the kernel from SPICE if the load succeeded.
     *
     * This runs once when the last shared_ptr owning this SpiceKernel
     * instance is destroyed.
     */
    ~SpiceKernel() noexcept;

    /// Canonical absolute path used as the cache key and SPICE file name.
    const std::string& getPath() const { return path; }

    /// True if furnsh_c succeeded for this kernel.
    bool wasLoadSuccesful() const {return loadSucceeded; };

    // avoid copy operations
    SpiceKernel(const SpiceKernel&) = delete;
    SpiceKernel& operator=(const SpiceKernel&) = delete;

private:
    /**
     * Construct a SpiceKernel by furnishing the given canonical path.
     *
     * The constructor switches SPICE into RETURN mode, calls furnsh_c,
     * checks failed_c, and records the load status. The destructor will
     * unload only if loadSucceeded is true.
     */
    explicit SpiceKernel(std::string path);

    /// Canonical absolute path used as the cache key and SPICE file name.
    std::string path;

    /// True if furnsh_c succeeded for this kernel.
    bool loadSucceeded;

    /**
     * Static mutex guarding the shared kernel cache.
     *
     * All access to SpiceKernel::cache must take this lock so that repeated
     * calls to request from different threads do not race.
     */
    static std::mutex mutex;

    /**
     * Global cache mapping canonical absolute paths to weak pointers.
     *
     * A non expired weak pointer means there is already a live SpiceKernel
     * instance owning that kernel, so request can reuse it instead of
     * calling furnsh_c again.
     */
    static std::unordered_map<std::string, std::weak_ptr<SpiceKernel>> cache;
};


/*! @brief spice interface class */
class SpiceInterface: public SysModel {
public:
    SpiceInterface();
    ~SpiceInterface();

    void UpdateState(uint64_t CurrentSimNanos);
    int loadSpiceKernel(char *kernelName, const char *dataPath);
    int unloadSpiceKernel(char *kernelName, const char *dataPath);
	std::string getCurrentTimeString();         //!< class method
    void Reset(uint64_t CurrentSimNanos);
    void initTimeData();
    void computeGPSData();
    void pullSpiceData(std::vector<SpicePlanetStateMsgPayload> *spiceData,
                       uint64_t CurrentSimNanos = 0, bool allowReconstruction = false);
    void writeOutputMessages(uint64_t CurrentClock);

    //! Add a SPICE kernel by full path (absolute or relative)
    void addKernelPath(const std::string& kernelPath);

    //! Convenience: add many kernels
    void addKernelPaths(const std::vector<std::string>& kernelPaths);

    //! Clear configured kernel paths (does not unload already loaded kernels)
    void clearKernelPaths();

    /** Resets all data loaded to SPICE.
     *
     * Calls `kclear_c`, which resets all loaded kernels for all simulations
     * in this process. Avoid using this, as it can affect other simulations
     * running in parallel. Kernels loaded with `loadSpiceKernel` will be
     * automatically cleared when all simulations that need it have closed.
     *
     * Deprecated, pending removal 11/20/2026.
     */
    void clearKeeper();
    void addPlanetNames(std::vector<std::string> planetNames);
    void addSpacecraftNames(std::vector<std::string> spacecraftNames);

    /** @name SPICE query reconstruction (opt-in, per planet, per channel)
     *
     * By default a planet's state is queried from SPICE at the exact simulation
     * time on every ``UpdateState`` call. On a dynamics task evaluated once per
     * integrator sub-step (e.g. MJScene) that is one SPICE call per stage per
     * body. These methods instead reconstruct a planet's position and/or
     * orientation from a coarse grid of cached SPICE "knots" (spacing
     * ``knotStep`` nanoseconds), cutting the SPICE cost to about
     * ``horizon / knotStep`` per channel.
     *
     * Knobs: ``nKnots`` (stencil size / polynomial order), ``useVelocity``
     * (also match the SPICE velocity at each knot for a Hermite fit, vs a
     * position-only Lagrange fit), and ``interpolate`` (bracket the time with
     * past and future knots, vs past-only extrapolation). The enabled defaults
     * are cubic Hermite interpolation on a 60 s grid.
     *
     * An unconfigured channel stays on the exact SPICE path, bit-for-bit. The
     * emitted velocity and orientation rate are the derivatives of the same
     * reconstruction, so a consumer that Euler-extrapolates them (the classic
     * ``gravityEffector``) stays self-consistent.
     * @{ */

    //! Default knot spacing when reconstruction is enabled without an explicit ``knotStep`` [ns].
    static constexpr uint64_t defaultKnotStep = 60000000000;  // 60 s

    //! Reconstruct this planet's position from a coarse knot grid. ``knotStep`` in nanoseconds.
    void setPlanetPositionReconstruction(const std::string& planetName,
                                         uint64_t knotStep = defaultKnotStep,
                                         int nKnots = 2, bool useVelocity = true,
                                         bool interpolate = true);

    //! Reconstruct this planet's orientation from a coarse knot grid. ``knotStep`` in nanoseconds.
    void setPlanetOrientationReconstruction(const std::string& planetName,
                                            uint64_t knotStep = defaultKnotStep,
                                            int nKnots = 2, bool useVelocity = true,
                                            bool interpolate = true);

    //! Turn this planet's position channel off: emit zero position/velocity, no SPICE query.
    void setPlanetPositionDisabled(const std::string& planetName);

    //! Turn this planet's orientation channel off: emit identity attitude/zero rate, no SPICE query.
    void setPlanetOrientationDisabled(const std::string& planetName);

    //! Revert both channels of this planet to the exact per-call SPICE query (the default).
    void setPlanetExactSpice(const std::string& planetName);

    //! Total SPICE evaluations spent on this planet since the last Reset, as {position,
    //! orientation}. Counts every spkezr_c/sxform_c call regardless of whether the planet uses
    //! reconstruction: exact-path calls (once per update) and reconstruction-knot fills alike. A
    //! planet added but never queried (e.g. a disabled channel) reads {0, 0}; a name that was never
    //! added via addPlanetNames reads {-1, -1}.
    std::pair<long, long> getPlanetSpiceQueryCount(const std::string& planetName);
    /** @} */

public:
    Message<SpiceTimeMsgPayload> spiceTimeOutMsg;    //!< spice time sampling output message
    ReadFunctor<EpochMsgPayload> epochInMsg;            //!< (optional) input epoch message
    std::vector<Message<SpicePlanetStateMsgPayload>*> planetStateOutMsgs; //!< vector of planet state output messages
    std::vector<Message<SCStatesMsgPayload>*> scStateOutMsgs; //!< vector of spacecraft state output messages
    std::vector<Message<AttRefMsgPayload>*> attRefStateOutMsgs; //!< vector of spacecraft attitude reference state output messages
    std::vector<Message<TransRefMsgPayload>*> transRefStateOutMsgs; //!< vector of spacecraft translational reference state output messages

    std::string SPICEDataPath;           //!< -- Path on file to SPICE data
    std::string referenceBase;           //!< -- Base reference frame to use
    std::string zeroBase;                //!< -- Base zero point to use for states
	std::string timeOutPicture;          //!< -- Optional parameter used to extract time strings
    bool SPICELoaded;                    //!< -- Boolean indicating to reload spice
    int charBufferSize;         //!< -- avert your eyes we're getting SPICE
    uint8_t *spiceBuffer;       //!< -- General buffer to pass down to spice
    std::string UTCCalInit;     //!< -- UTC time string for init time

    std::vector<std::string>planetFrames; //!< -- Optional vector of planet frame names.  Default values are IAU_ + planet name

    bool timeDataInit;          //!< -- Flag indicating whether time has been init
    double J2000ETInit;         //!< s Seconds elapsed since J2000 at init
    double J2000Current;        //!< s Current J2000 elapsed time
    double julianDateCurrent;   //!< s Current JulianDate
    double GPSSeconds;          //!< s Current GPS seconds
    uint16_t GPSWeek;           //!< -- Current GPS week value
    uint64_t GPSRollovers;      //!< -- Count on the number of GPS rollovers

    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    std::string GPSEpochTime;   //!< -- String for the GPS epoch
    double JDGPSEpoch;          //!< s Epoch for GPS time.  Saved for efficiency

    std::vector<SpicePlanetStateMsgPayload> planetData;
    std::vector<SpicePlanetStateMsgPayload> scData;

    //! Optional explicit kernel list. If non-empty, Reset() loads these instead
    // of SPICEDataPath defaults.
    std::vector<std::string> kernelPaths;

    //! Track which configured kernel paths have been loaded.
    std::set<std::string> configuredLoadedKernelKeys;

    /**
     * Map of loaded kernel paths to their RAII handles.
     *
     * As long as an entry is present, the corresponding kernel remains
     * furnished in SPICE. Removing an entry allows the SpiceKernel
     * destructor to unload the kernel when all shared_ptr copies are
     * gone.
     */
    std::unordered_map<std::string, std::shared_ptr<SpiceKernel>> loadedKernels;

    //! Opaque holder for the opt-in reconstruction state (per-planet knot caches, configuration,
    //! and interpolation math). Defined entirely in the .cpp so this header stays free of the
    //! Eigen/STL internals and needs no SWIG guards; null until a setPlanet*Reconstruction/Disabled
    //! call creates it. See @ref SpiceReconstruction in spiceInterface.cpp.
    class SpiceReconstruction;
    std::unique_ptr<SpiceReconstruction> recon;

    //! Return the reconstruction state, lazily creating it on first use.
    SpiceReconstruction& reconState();

    //! {position, orientation} SPICE-query tallies per planet (uppercase name), counting every
    //! spkezr_c/sxform_c call whether it comes from the exact path or a reconstruction knot fill.
    //! Reset to zero for the current planet set at each Reset. getPlanetSpiceQueryCount reads this.
    std::unordered_map<std::string, std::pair<long, long>> spiceQueryCounts;

    //! Increment this planet's position (isPosition=true) or orientation SPICE-query tally.
    void countSpiceQuery(const std::string& planetName, bool isPosition);
};


#endif
