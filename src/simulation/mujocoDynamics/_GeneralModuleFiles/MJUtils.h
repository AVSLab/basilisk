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

#ifndef MJUTILS_H
#define MJUTILS_H

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "architecture/utilities/bskLogging.h"

#include <Eigen/Dense>

namespace MJBasilisk::detail
{

/// @cond
/** Can be used in `std::unique_ptr<T, mjTDelete>` to correctly clear MuJoCo objects. */
template<typename Type, void(Deleter)(Type*)>
struct mjDeleter {
    /** Calls the template argument Deleter on the given pointer */
    void operator()(Type* ptr) const { Deleter(ptr); }
};
/// @endcond

using mjModelDeleter = mjDeleter<mjModel, mj_deleteModel>;

using mjDataDeleter = mjDeleter<mjData, mj_deleteData>;

using mjSpecDeleter = mjDeleter<mjSpec, mj_deleteSpec>;

using mjVFSDeleter = mjDeleter<mjVFS, mj_deleteVFS>;

/** Loggs an error message and then throws an error. */
template <typename T = std::invalid_argument>
[[noreturn]] inline void logAndThrow (const std::string& error, BSKLogger* logger = nullptr)
{
    if (logger) {
        logger->bskLog(BSK_ERROR, error.c_str());
    } else {
        BSKLogger{}.bskLog(BSK_ERROR, error.c_str());
    }
    throw T(error);
}

/** Calls ``logAndThrow<std::runtime_error>`` with the given input
 *
 * Meant to be used as an error callback for MuJoCo's ``mju_user_error``.
*/
void logMujocoError(const char* err);

/** Calls ``BSKLogger::bskLog`` with the given input
 *
 * Meant to be used as an error callback for MuJoCo's ``mju_user_warning``.
*/
void logMujocoWarning(const char* err);

} // namespace MJBasilisk::detail

#endif
