/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef extendedStateVector_h
#define extendedStateVector_h

#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/dynParamManager.h"

#include <Eigen/Dense>
#include <functional>
#include <stdint.h>
#include <unordered_map>

/*
Each DynamicObject has a series of states associated to them. Each state
is associated with a unique std::string (i.e. "hubPosition"), and their
value is stored as Eigen::MatrixXd. Each of these states is integrated
in parallel: each have their own derivative, which are all calculated
in DynamicObject::equationsOfMotion.

Basilisk also supports the parallel integration of multiple DynamicObjects,
which in practice means that all states of all DynamicObjects need to
be integrated in parallel.

In order to facilitate this task, the ExtendedStateVector was created.
This class is a map that can hold an Eigen::MatrixXd for every state
of every DynamicObject that we want to integrate. Thus, it can be
used to store the value of the states, their derivatives, errors...
in a single, flat object. This is similar to the behaviour of StateVector,
except that this supports multiple DynamicObjects.

ExtendedStateVector supports a series of utility functions that
makes performing state-wise operations easier.
*/

/**
 * Because two different DynamicObjects can have states with the same
 * name, ExtendedStateId is used to give a unique identifier to
 * every state. The first item is the index of the DynamicObject
 * in the integrator's dynPtrs vector. The second item in the pair
 * is the name of the state.
 */
using ExtendedStateId = std::pair<size_t, std::string>;

/// @cond DOXYGEN_IGNORE
namespace std // Inject hash for ExtendedStateId into std::
{
    /** Hash implementation for ``std::pair<size_t, std::string>``,
     * allows using it as the key in maps.
     */
    template<> struct hash<ExtendedStateId>
    {
        /** Produce hash from ``std::pair<size_t, std::string>`` */
        std::size_t operator()(const ExtendedStateId& input) const noexcept
        {
            std::size_t h = 0;
            hash_combine(h, input.first, input.second);
            return h;
        }
    };
}
/// @endcond

using StateIdToIndexMap = std::unordered_map<ExtendedStateId, size_t>;

/**
 * Conceptually similar to StateVector, this class allows us to handle
 * the states of multiple DynamicObject with a single object.
 *
 * It also supports several utility functions.
 */
class ExtendedStateVector
    : public std::unordered_map<ExtendedStateId, Eigen::MatrixXd> {
  public:
    /**
     * Builds a ExtendedStateVector from all states in the given
     * dynamic objects
     */
    static ExtendedStateVector fromStates(const std::vector<DynamicObject*>& dynPtrs);

    /**
     * Builds a ExtendedStateVector from the derivatives of all states
     * in the given dynamic objects
     */
    static ExtendedStateVector fromStateDerivs(const std::vector<DynamicObject*>& dynPtrs);

    /**
     * Extracts the diffusion at the specified noise index for the states
     * present in ``stateIdToNoiseIndexMap``.
     */
    static ExtendedStateVector
    fromStateDiffusions(const std::vector<DynamicObject*>& dynPtrs, const StateIdToIndexMap& stateIdToNoiseIndexMap);

    /**
     * Calls and returns the result of ``fromStateDiffusions`` for each map in ``stateIdToNoiseIndexMaps``.
     */
    static std::vector<ExtendedStateVector>
    fromStateDiffusions(const std::vector<DynamicObject*>& dynPtrs, const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps);

    /**
     * This method will call the given std::function for every
     * state in the ExtendedStateVector. The arguments to the functor
     * are the index of the DynamicObject corresponding to the state,
     * the name of the state, and the value of the state stored in ExtendedStateVector.
     * A new ExtendedStateVector is built from the results of each call
     * of the functor.
     */
    ExtendedStateVector
    map(std::function<Eigen::MatrixXd(const size_t&, const std::string&, const Eigen::MatrixXd&)>
            functor) const;

    /**
     * Similar to the map method, except that no
     * ExtendedStateVector is returned because the given functor
     * does not produce any values.
     */
    void apply(std::function<void(const size_t&, const std::string&, const Eigen::MatrixXd&)>
                   functor) const;

    /**
     * Modifies each Eigen::MatrixXd stored in this object according to
     * the given functor
     */
    void modify(std::function<void(const size_t&, const std::string&, Eigen::MatrixXd&)> functor);

    /** Adds the values of `rhs` to this
     *
     * This functions as a state-wise addition operation.
     */
    ExtendedStateVector operator+=(const ExtendedStateVector& rhs);

    /** Subtracts the values of `rhs` to from this
     *
     * This functions as a state-wise subtraction operation.
     */
    ExtendedStateVector operator-(const ExtendedStateVector& rhs) const;

    /** Returns a new ExtendedStateVector that is the result of multiplying each state by a constant
     */
    ExtendedStateVector operator*(const double rhs) const;

    /** Calls StateData::setState for every entry in this */
    void setStates(std::vector<DynamicObject*>& dynPtrs) const;

    /** Calls StateData::setDerivative for every entry in this */
    void setDerivatives(std::vector<DynamicObject*>& dynPtrs) const;

    /** Calls StateData::setDiffusion for every entry in this
     *
     * Note that this extendedStateVector may contain the diffusion
     * corresponding to different noise sources for each state. Thus,
     * the input ``stateIdToNoiseIndexMap`` is necessary to set for
     * which noise source this diffusion is given.
    */
    void setDiffusions(
      std::vector<DynamicObject*>& dynPtrs,
      const StateIdToIndexMap& stateIdToNoiseIndexMap
    ) const;

  private:
    static ExtendedStateVector fromStateData(const std::vector<DynamicObject*>& dynPtrs,
                                             std::function<Eigen::MatrixXd(const StateData&)>);
};

#endif /* extendedStateVector_h */
