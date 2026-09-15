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

#include "effectorName.h"
#include "architecture/utilities/bskLogging.h"
#include "dynParamManager.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using NameSets = std::map<EffectorNameKind, std::set<std::string>>;

[[noreturn]] void
namingError(const std::string& message)
{
    throw BasiliskError("Effector naming: " + message);
}

bool
sameSpecification(const EffectorNameGroup& first, const EffectorNameGroup& second)
{
    return first.family == second.family && first.names.size() == second.names.size() &&
           std::equal(first.names.begin(),
                      first.names.end(),
                      second.names.begin(),
                      [](const EffectorNameSpec& left, const EffectorNameSpec& right) {
                          return std::tie(left.key, left.kind, left.prefix, left.suffix, left.customName) ==
                                 std::tie(right.key, right.kind, right.prefix, right.suffix, right.customName);
                      });
}

void
validateGroup(const EffectorNameGroup& group)
{
    if (group.family.empty()) {
        namingError("a naming family must be provided.");
    }
    std::set<std::string> keys;
    std::set<std::tuple<EffectorNameKind, std::string, std::string>> patterns;
    for (const auto& spec : group.names) {
        if (spec.key.empty() || !keys.insert(spec.key).second) {
            namingError("family '" + group.family + "' requires non-empty, distinct local keys.");
        }
        if (spec.kind != EffectorNameKind::State && spec.kind != EffectorNameKind::Property) {
            namingError("invalid namespace for '" + spec.key + "'.");
        }
        if (spec.customName) {
            if (spec.customName->empty()) {
                namingError("custom name for '" + spec.key + "' must not be empty.");
            }
        } else if (!patterns.emplace(spec.kind, spec.prefix, spec.suffix).second) {
            namingError("family '" + group.family + "' requests the same automatic pattern more than once.");
        }
    }
}

std::size_t
requestIndex(const std::vector<EffectorNameRequest>& requests, const EffectorNameRequest& request)
{
    const auto found = std::find(requests.begin(), requests.end(), request);
    if (!request || found == requests.end()) {
        namingError("request does not belong to this dynamics manager.");
    }
    return static_cast<std::size_t>(found - requests.begin());
}

} // namespace

bool
DynParamManager::consumeLegacyAutomaticEffectorNamingWarning()
{
    if (!this->usesLegacyAutomaticEffectorNames) {
        return false;
    }
    const auto& owner = this->effectorNameIdentity.getToken();
    if (this->legacyNamingWarningOwner.lock() == owner) {
        return false;
    }
    this->legacyNamingWarningOwner = owner;
    return true;
}

StateData*
DynParamManager::registerLegacyEffectorState(uint32_t nRow, uint32_t nCol, const std::string& stateName, bool automatic)
{
    if (this->effectorNamingPolicy != EffectorNamingPolicy::Legacy) {
        namingError("legacy effector registration requires the Legacy policy.");
    }
    auto* state = this->registerState(nRow, nCol, stateName);
    this->usesLegacyAutomaticEffectorNames = this->usesLegacyAutomaticEffectorNames || automatic;
    return state;
}

Eigen::MatrixXd*
DynParamManager::createLegacyEffectorProperty(const std::string& propName, const Eigen::MatrixXd& value, bool automatic)
{
    if (this->effectorNamingPolicy != EffectorNamingPolicy::Legacy) {
        namingError("legacy effector registration requires the Legacy policy.");
    }
    auto* property = this->createProperty(propName, value);
    this->usesLegacyAutomaticEffectorNames = this->usesLegacyAutomaticEffectorNames || automatic;
    return property;
}

void
DynParamManager::setEffectorNamingPolicy(EffectorNamingPolicy policy)
{
    if (policy != EffectorNamingPolicy::Legacy && policy != EffectorNamingPolicy::ManagerLocal) {
        namingError("invalid naming policy.");
    }
    if (policy == this->effectorNamingPolicy) {
        return;
    }
    if (!this->effectorNameRequests.empty() || this->effectorNamesResolved || !this->stateContainer.stateMap.empty() ||
        !this->dynProperties.empty()) {
        namingError("select the naming policy before collecting names or registering states and properties.");
    }
    this->effectorNamingPolicy = policy;
}

EffectorNameRequest
DynParamManager::requestEffectorNames(const EffectorNameGroup& group, const EffectorNameRequest& previous)
{
    if (this->effectorNamingPolicy != EffectorNamingPolicy::ManagerLocal) {
        namingError("name collection requires the ManagerLocal policy.");
    }
    if (previous) {
        const auto index = requestIndex(this->effectorNameRequests, previous);
        if (sameSpecification(*previous, group)) {
            return previous;
        }
        if (this->effectorNamesResolved) {
            namingError("names are fixed after successful resolution; rebuild the model to rename effectors.");
        }
        validateGroup(group);
        auto replacement = std::make_shared<const EffectorNameGroup>(group);
        // Preserve the binding when a pending specification is corrected.
        const auto binding = this->effectorNameBindings.find(previous);
        if (binding != this->effectorNameBindings.end()) {
            this->effectorNameBindings.emplace(replacement, binding->second);
            this->effectorNameBindings.erase(binding);
        }
        this->effectorNameRequests.at(index) = replacement;
        return replacement;
    }
    if (this->effectorNamesResolved) {
        namingError("collect all effector names before resolving them; the attachment configuration is now frozen.");
    }
    validateGroup(group);
    auto request = std::make_shared<const EffectorNameGroup>(group);
    this->effectorNameRequests.push_back(request);
    return request;
}

bool
DynParamManager::hasEffectorNameRequest(const EffectorNameRequest& request) const
{
    return request && std::find(this->effectorNameRequests.begin(), this->effectorNameRequests.end(), request) !=
                        this->effectorNameRequests.end();
}

EffectorNameRequest
DynParamManager::requestEffectorNamesForOwner(const EffectorNameGroup& group,
                                              const EffectorNameRequest& previous,
                                              const std::shared_ptr<const EffectorNameIdentity::Token>& owner)
{
    auto available = previous;
    const auto& managerIdentity = this->effectorNameIdentity.getToken();
    const auto binding = this->effectorNameBindings.find(previous);
    if (binding != this->effectorNameBindings.end() && binding->second.manager.lock() == managerIdentity &&
        binding->second.effector.lock() != owner) {
        // Expiration of an effector does not make its existing reservation reusable.
        available.reset();
    }
    const auto request = this->requestEffectorNames(group, available);
    this->effectorNameBindings.insert_or_assign(request, EffectorNameBinding{ managerIdentity, owner });
    return request;
}

void
DynParamManager::cancelEffectorNames(const EffectorNameRequest& request)
{
    const auto index = requestIndex(this->effectorNameRequests, request);
    if (this->effectorNamesResolved) {
        namingError("names are fixed after successful resolution; a resolved request cannot be cancelled.");
    }
    this->effectorNameRequests.erase(this->effectorNameRequests.begin() +
                                     static_cast<std::vector<EffectorNameRequest>::difference_type>(index));
    this->effectorNameBindings.erase(request);
}

void
DynParamManager::resolveEffectorNames()
{
    if (this->effectorNamingPolicy == EffectorNamingPolicy::Legacy || this->effectorNamesResolved) {
        return;
    }

    // Stage all reservations locally so a configuration error cannot partially publish names.
    NameSets occupied;
    for (const auto& entry : this->stateContainer.stateMap) {
        occupied[EffectorNameKind::State].insert(entry.first);
    }
    for (const auto& entry : this->dynProperties) {
        occupied[EffectorNameKind::Property].insert(entry.first);
    }
    for (const auto& request : this->effectorNameRequests) {
        for (const auto& spec : request->names) {
            if (spec.customName && !occupied[spec.kind].insert(*spec.customName).second) {
                namingError("custom " + std::string(spec.kind == EffectorNameKind::State ? "state" : "property") +
                            " name '" + *spec.customName + "' in family '" + request->family + "' is already in use.");
            }
        }
    }

    std::map<std::string, uint64_t> nextIndices;
    std::vector<std::map<std::string, std::string>> resolved;
    std::map<std::pair<EffectorNameKind, std::string>, EffectorNameRequest> owners;
    for (const auto& request : this->effectorNameRequests) {
        const bool hasAutomaticNames = std::any_of(
          request->names.begin(), request->names.end(), [](const EffectorNameSpec& spec) { return !spec.customName; });
        uint64_t index = 1;
        if (hasAutomaticNames) {
            const auto next = nextIndices.emplace(request->family, 1).first;
            index = next->second;
            if (index == 0) {
                namingError("automatic index space exhausted for family '" + request->family + "'.");
            }
            while (true) {
                NameSets candidateNames;
                bool available = true;
                for (const auto& spec : request->names) {
                    if (!spec.customName) {
                        const auto candidate = spec.prefix + std::to_string(index) + spec.suffix;
                        if (occupied[spec.kind].count(candidate) ||
                            !candidateNames[spec.kind].insert(candidate).second) {
                            available = false;
                            break;
                        }
                    }
                }
                if (available) {
                    break;
                }
                if (index == std::numeric_limits<uint64_t>::max()) {
                    namingError("automatic index space exhausted for family '" + request->family + "'.");
                }
                ++index;
            }
            // Zero marks exhaustion without overflowing the counter.
            next->second = index == std::numeric_limits<uint64_t>::max() ? 0 : index + 1;
        }

        std::map<std::string, std::string> names;
        for (const auto& spec : request->names) {
            auto name = spec.customName ? *spec.customName : spec.prefix + std::to_string(index) + spec.suffix;
            occupied[spec.kind].insert(name);
            owners.emplace(std::make_pair(spec.kind, name), request);
            names.emplace(spec.key, std::move(name));
        }
        resolved.push_back(std::move(names));
    }

    this->resolvedEffectorNames.swap(resolved);
    this->effectorNameOwners.swap(owners);
    this->effectorNamesResolved = true;
}

void
DynParamManager::checkUnownedNameRegistration(EffectorNameKind kind, const std::string& name) const
{
    if (this->effectorNamingPolicy == EffectorNamingPolicy::ManagerLocal &&
        this->effectorNameOwners.count({ kind, name })) {
        namingError("name '" + name + "' is reserved; register it using its owning effector request and local key.");
    }
}

const std::string&
DynParamManager::getOwnedEffectorName(const EffectorNameRequest& request,
                                      const std::string& key,
                                      EffectorNameKind kind) const
{
    const auto& name = this->getEffectorName(request, key);
    const auto owner = this->effectorNameOwners.find({ kind, name });
    // A state and property may share a resolved string, even within one request.
    // Validate the local key's kind as well as the reservation owner.
    const auto spec = std::find_if(request->names.begin(),
                                   request->names.end(),
                                   [&key](const EffectorNameSpec& candidate) { return candidate.key == key; });
    if (spec == request->names.end() || spec->kind != kind || owner == this->effectorNameOwners.end() ||
        owner->second != request) {
        namingError("key '" + key + "' does not own a reservation in the requested namespace.");
    }
    return name;
}

Eigen::MatrixXd*
DynParamManager::createEffectorProperty(const EffectorNameRequest& request,
                                        const std::string& key,
                                        const Eigen::MatrixXd& value)
{
    const auto& name = this->getOwnedEffectorName(request, key, EffectorNameKind::Property);
    const auto result = this->dynProperties.try_emplace(name, value);
    if (!result.second &&
        (result.first->second.rows() != value.rows() || result.first->second.cols() != value.cols())) {
        namingError("repeated registration of property '" + name + "' must preserve its dimensions.");
    }
    return &result.first->second;
}

const std::string&
DynParamManager::getEffectorName(const EffectorNameRequest& request, const std::string& key) const
{
    const auto index = requestIndex(this->effectorNameRequests, request);
    if (!this->effectorNamesResolved) {
        namingError("resolve names before retrieving '" + key + "'.");
    }
    const auto& names = this->resolvedEffectorNames.at(index);
    const auto found = names.find(key);
    if (found == names.end()) {
        namingError("unknown local key '" + key + "' in family '" + request->family + "'.");
    }
    return found->second;
}
