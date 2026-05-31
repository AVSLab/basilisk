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

/** Returns the name of the given MuJoCo spec object. */
template <typename T>
inline std::string getSpecObjectName(T* object)
{
    return mjs_getString(mjs_getName(object->element));
}

/** Sets the name of the given MuJoCo spec object. */
template <typename T>
inline void setSpecObjectName(T* object, const std::string& name)
{
    if (mjs_setName(object->element, name.c_str()) != 0) {
        BSKLogger{}.bskError("Could not set MuJoCo spec object name to: %s", name.c_str());
    }
}

/** Calls ``BSKLogger::bskError`` with the given input
 *
 * Meant to be used as an error callback for MuJoCo's ``mju_user_error``.
*/
void logMujocoError(const char* err);

/** Calls ``BSKLogger::bskLog`` with the given input
 *
 * Meant to be used as an error callback for MuJoCo's ``mju_user_warning``.
*/
void logMujocoWarning(const char* err);

/**
 * Logs an error via BSKLogger then throws an exception of type @p ExceptionType.
 *
 * @tparam ExceptionType Exception to throw (default: @c std::runtime_error).
 * @param message        Human-readable error message.
 */
template <typename ExceptionType = std::runtime_error>
[[noreturn]] inline void logAndThrow(const std::string& message)
{
    BSKLogger{}.bskError("%s", message.c_str());
    throw ExceptionType(message);
}

} // namespace MJBasilisk::detail

#endif
