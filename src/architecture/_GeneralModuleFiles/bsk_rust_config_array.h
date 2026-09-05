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

#ifndef BSK_RUST_CONFIG_ARRAY_H
#define BSK_RUST_CONFIG_ARRAY_H

#include <cstddef>
#include <memory>
#include <type_traits>
#include <vector>

/*! @brief Copy a flat Rust configuration array through its guarded accessor.
 *  @tparam Value C++ element type, including aliases of bool.
 *  @tparam Wrapper Generated Rust module wrapper type.
 *  @param wrapper Wrapper providing __bskGetConfigField().
 *  @param fieldIndex Index of the Rust configuration field.
 *  @param length Number of elements across all array dimensions.
 *  @return Copied elements for conversion to a Python sequence.
 *  @note std::vector<bool> packs its elements and cannot supply bool* storage.
 *        C++ type resolution selects this path for every Boolean alias, too.
 */
template<typename Value, typename Wrapper>
std::vector<Value> bskRustGetConfigArray(const Wrapper *wrapper, std::size_t fieldIndex,
                                       std::size_t length)
{
    if constexpr (std::is_same_v<Value, bool>) {
        auto rawValue = std::make_unique<Value[]>(length);
        wrapper->__bskGetConfigField(fieldIndex, rawValue.get(), length * sizeof(Value));
        std::vector<Value> value;
        value.reserve(length);
        for (std::size_t index = 0; index < length; ++index) {
            value.push_back(rawValue[index]);
        }
        return value;
    } else {
        std::vector<Value> value(length);
        wrapper->__bskGetConfigField(fieldIndex, value.data(), value.size() * sizeof(Value));
        return value;
    }
}

/*! @brief Copy a flat Python array into a guarded Rust configuration setter.
 *  @tparam Value C++ element type, including aliases of bool.
 *  @tparam Wrapper Generated Rust module wrapper type.
 *  @param wrapper Wrapper providing __bskSetConfigField().
 *  @param fieldIndex Index of the Rust configuration field.
 *  @param value Converted elements; the generated caller checks the array length.
 *  @note Boolean elements use real bool storage, not vector<bool> packed bits.
 *        Rust still validates the complete proposed value before storing it.
 */
template<typename Value, typename Wrapper>
void bskRustSetConfigArray(Wrapper *wrapper, std::size_t fieldIndex,
                          const std::vector<Value> &value)
{
    if constexpr (std::is_same_v<Value, bool>) {
        auto rawValue = std::make_unique<Value[]>(value.size());
        for (std::size_t index = 0; index < value.size(); ++index) {
            rawValue[index] = value[index];
        }
        wrapper->__bskSetConfigField(fieldIndex, rawValue.get(), value.size() * sizeof(Value));
    } else {
        wrapper->__bskSetConfigField(fieldIndex, value.data(), value.size() * sizeof(Value));
    }
}

#endif /* BSK_RUST_CONFIG_ARRAY_H */
