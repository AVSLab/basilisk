/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#pragma once
#include "architecture/_GeneralModuleFiles/numbaModel.h"
#include "StatefulSysModel.h"

/** A SysModel that combines NumbaModel (JIT-compiled UpdateStateImpl) with
 * StatefulSysModel (continuous-time states integrated by MJScene's RK4 solver).
 *
 * Users subclass this in Python by inheriting from StatefulNumbaModel and:
 *   1. Overriding registerStates() to register state variables.
 *   2. Providing a static UpdateStateImpl() method (compiled to a Numba cfunc).
 *
 * State and derivative arrays are accessible in UpdateStateImpl via parameters
 * named '<name>State' and '<name>StateDeriv', resolved from self.<name>State.
 *
 * All NumbaModel features (messages, memory, rng, bskLogger) are also available.
 */
class StatefulNumbaModel : public NumbaModel, public StatefulSysModel
{
public:
    /** Default constructor */
    StatefulNumbaModel() = default;

    /** Delegates to NumbaModel::Reset (C++ clear + triggers Python _nbm_compile) */
    void Reset(uint64_t CurrentSimNanos) override;

    /** Delegates to NumbaModel::UpdateState (calls the compiled cfunc) */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /** Must be overridden by user subclasses to register their state variables. */
    virtual void registerStates(DynParamRegisterer registerer) override = 0;
};
