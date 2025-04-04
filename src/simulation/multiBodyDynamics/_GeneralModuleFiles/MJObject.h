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

#ifndef MJOBJECT_H
#define MJOBJECT_H

#include "MJUtils.h"

#include <mujoco/mujoco.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"

#include <optional>
#include <stdexcept>
#include <string>

#include <iostream>

/// @cond
namespace MJBasilisk::detail
{
/**
 * @brief Returns a human-readable name for a MuJoCo object type.
 *
 * This template function returns the name of a MuJoCo object type,
 * represented by an integer. For example:
 * @code
 * getObjectTypeName<mjsActuator>()
 * @endcode
 * could return "actuator".
 *
 * @tparam T The MuJoCo object type.
 * @return A human-readable name for the object type. Defaults to "object".
 */
template <typename T> constexpr std::string_view getObjectTypeName() { return "object"; }
} // namespace MJBasilisk::detail
/// @endcond

/**
 * @brief A wrapper class for managing a MuJoCo object.
 *
 * This class provides functionality for configuring and accessing a
 * MuJoCo object, including name and ID retrieval. It also assigns
 * a default name to nameless objects.
 *
 * @tparam mjsObjectType The MuJoCo object type.
 */
template <typename mjsObjectType> class MJObject
{
public:
    /**
     * @brief Constructs an MJObject with a given MuJoCo object pointer.
     *
     * Initializes the MuJoCo object's name, setting it to a unique
     * default name if it is unnamed.
     *
     * @param mjsObject Pointer to the MuJoCo object.
     */
    MJObject(mjsObjectType* mjsObject) : mjsObject(mjsObject)
    {
        static size_t namelessIndex = 0; // figure out a better way?

        this->name = mjs_getString(mjsObject->name);
        if (this->name.empty())
        {
            this->name = "_nameless_" + std::to_string(namelessIndex);
            mjs_setString(mjsObject->name, this->name.c_str());
            namelessIndex++;
        }
    };

    /**
     * @brief Configures the MJObject by linking it to the given MuJoCo model.
     *
     * Sets the object's ID by searching for its name within the MuJoCo model.
     * Throws an exception if the object cannot be found.
     *
     * @param mujocoModel Pointer to the MuJoCo model to configure the object with.
     * @throws std::runtime_error If the object name cannot be found in the model.
     */
    void configure(const mjModel* mujocoModel)
    {
        mjtObj mjObjectTypeInt = mjsObject->element->elemtype;
        auto idOrFail = mj_name2id(mujocoModel, mjObjectTypeInt, this->name.c_str());
        if (idOrFail < 0) {
            MJBasilisk::detail::logAndThrow(
                "Could not find " +
                std::string(MJBasilisk::detail::getObjectTypeName<mjsObjectType>()) +
                " in MuJoCo with name: " + this->name);
        }

        this->id = size_t(idOrFail);
    }

    /**
     * @brief Retrieves the name of the MJObject.
     *
     * @return The object's name.
     */
    const std::string& getName() const { return name; }

    /**
     * @brief Retrieves the ID of the MJObject.
     *
     * @return The object's ID.
     * @throws std::runtime_error If the object has not been configured yet.
     */
    size_t getId() const
    {
        if (this->id.has_value()) {
            return this->id.value();
        }
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
            "Tried to get ID of an MJObject before it was configured.");
    }

protected:
    std::string name; ///< The name of the MJObject.
    std::optional<size_t> id; ///< The ID of the MJObject, set during configuration.

    mjsObjectType* mjsObject; ///< Pointer to the underlying MuJoCo object.
};

#endif
