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

#ifndef EFFECTOR_NAME_H
#define EFFECTOR_NAME_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

/** @brief Select the source of effector names during dynamics preparation. */
enum class EffectorNamingPolicy
{
    Legacy,      //!< Effectors retain their existing constructor-generated names.
    ManagerLocal //!< The dynamics manager resolves declared name groups before registration.
};

/** @brief Identify the existing dynamics-manager namespace to reserve. */
enum class EffectorNameKind
{
    State,   //!< Integrated state namespace.
    Property //!< Non-integrated property namespace.
};

/** @brief Describe one name without inferring whether a user supplied it.
 * @note Automatic names are prefix + decimal group index + suffix. A present
 * customName requests that exact name, even if it matches the automatic pattern.
 * Migrating effectors must set customName from their public name setters and
 * keep constructor defaults separate from explicit assignments.
 */
struct EffectorNameSpec
{
    std::string key;                                 //!< Stable, effector-local lookup key, such as "theta".
    EffectorNameKind kind = EffectorNameKind::State; //!< Namespace in which this name must be unique.
    std::string prefix;                    //!< Automatic text before the group index, including any owner prefix.
    std::string suffix;                    //!< Automatic text after the group index, such as a body index.
    std::optional<std::string> customName; //!< Exact user override; an absent value selects automatic naming.
};

/** @brief Describe all states and properties belonging to one effector.
 * @note All automatic entries share an index. Family selects the index sequence;
 * collision detection also checks entries from other families in each namespace.
 */
struct EffectorNameGroup
{
    std::string family;                  //!< Stable naming family, independent of ModelTag or memory address.
    std::vector<EffectorNameSpec> names; //!< Ordered requests with distinct local keys.
};

/** @brief Immutable request handle issued and retained by one dynamics manager.
 * @note Retain the latest handle returned by collection. Correcting an unresolved
 * request replaces its snapshot and invalidates its old handle. Successful
 * resolution fixes the names and makes this handle their registration authority.
 * Releasing an effector does not release its resolved reservations.
 */
using EffectorNameRequest = std::shared_ptr<const EffectorNameGroup>;

/** @brief Provide a lifetime identity that is renewed on copy and transferred on move.
 * @note Tokens are allocated only when naming preparation uses the identity.
 * Weak references distinguish object lifetimes without depending on memory addresses.
 */
class EffectorNameIdentity
{
  public:
    /** @brief Opaque identity retained by its owning object. */
    struct Token
    {};

    /** @brief Construct an identity without allocating a token. */
    EffectorNameIdentity() = default;
    /** @brief Give a copy its own identity. */
    EffectorNameIdentity(const EffectorNameIdentity&) {}
    /** @brief Renew the destination identity on copy assignment.
     * @param other Source identity; self-assignment preserves the token.
     * @return This identity.
     */
    EffectorNameIdentity& operator=(const EffectorNameIdentity& other)
    {
        if (this != &other) {
            this->token.reset();
        }
        return *this;
    }
    /** @brief Transfer the identity to a moved object. */
    EffectorNameIdentity(EffectorNameIdentity&&) noexcept = default;
    /** @brief Transfer the identity during move assignment.
     * @return This identity.
     */
    EffectorNameIdentity& operator=(EffectorNameIdentity&&) noexcept = default;

    /** @brief Obtain the token, allocating it on first use or after being moved from.
     * @return Token that remains alive until this identity is destroyed or reassigned.
     */
    const std::shared_ptr<const Token>& getToken() const
    {
        if (!this->token) {
            this->token = std::make_shared<const Token>();
        }
        return this->token;
    }

  private:
    mutable std::shared_ptr<const Token> token; //!< Lazily allocated object identity.
};

#endif /* EFFECTOR_NAME_H */
