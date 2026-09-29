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

#include "MJSpec.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <unordered_set>

#include "MJScene.h"

using MJBasilisk::detail::checkedMjtSizeCast;

namespace
{
/**
 * @brief Assigns scene-local names to unnamed bodies before model compilation.
 * @param spec The parsed MuJoCo specification.
 */
void nameUnnamedBodies(mjSpec* spec)
{
    std::unordered_set<std::string> bodyNames;
    std::vector<mjsBody*> unnamedBodies;
    for (auto element = mjs_firstElement(spec, mjOBJ_BODY); element;
         element = mjs_nextElement(spec, element)) {
        auto body = mjs_asBody(element);
        assert(body != nullptr);
        auto name = MJBasilisk::detail::getSpecObjectName(body);
        if (name.empty()) {
            unnamedBodies.push_back(body);
        } else {
            bodyNames.insert(name);
        }
    }

    // Reserve every explicit name first, including bodies later in the tree.
    std::size_t namelessIndex = 0;
    for (auto body : unnamedBodies) {
        std::string name;
        do {
            name = "_nameless_" + std::to_string(namelessIndex++);
        } while (!bodyNames.insert(name).second);
        MJBasilisk::detail::setSpecObjectName(body, name);
    }
}

std::string
compileErrorMessage(mjSpec* spec, const char* context)
{
    const char* detail = mjs_getError(spec);
    if (detail == nullptr || detail[0] == '\0') {
        return context;
    }
    return std::string(context) + ": " + detail;
}

std::vector<std::string> readCustomSingleSplit(mjSpec* spec, const std::string& key, char delimiter [[maybe_unused]])
{
    std::string value;

    for (auto element = mjs_firstElement(spec, mjOBJ_TEXT); element;
         element = mjs_nextElement(spec, element)) {
                    auto mjstext = mjs_asText(element);
        assert(mjstext != NULL);
        if (MJBasilisk::detail::getSpecObjectName(mjstext) == key) {
            value = mjs_getString(mjstext->data);
            break;
        }
    }
    std::istringstream iss(value);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    return tokens;
}

std::vector<std::pair<std::string, std::string>>
readCustomDoubleSplit(mjSpec* spec, const std::string& key, char delimiter1, char delimiter2)
{
        auto input = readCustomSingleSplit(spec, key, delimiter1);
    std::vector<std::pair<std::string, std::string>> result;

    for (const auto& str : input) {
        size_t pos = str.find(delimiter2);
        if (pos != std::string::npos) {
            result.emplace_back(str.substr(0, pos), str.substr(pos + 1));
        } else {
            result.emplace_back(str, "");
        }
    }
    return result;
}
} // namespace

MJSpec::MJSpec(MJScene& scene, std::string xmlString, const std::vector<std::string>& files)
    : scene(scene)
{
    this->virtualFileSystem.reset(new mjVFS());
    mj_defaultVFS(virtualFileSystem.get());

    std::string loadingError;
        for (auto&& file : files) {
        switch (mj_addFileVFS(virtualFileSystem.get(), "", file.c_str())) {
        case 0:
            break; // success
        case 1:
            loadingError = "Error loading file " + file + ": VFS memory is full.";
            break;
        case 2:
            loadingError = "Error loading file " + file + ": file name used multiple times.";
            break;
        case -1:
            loadingError = "Error loading file " + file + ": internal error.";
            break;
        default:
            assert(false); // should never happen
        };
    }

    if (!loadingError.empty())
    {
        BSKLogger{}.bskError("%s", loadingError.c_str());
    }

    char error[1024];
    auto maybeSpec =
        mj_parseXMLString(xmlString.c_str(), virtualFileSystem.get(), error, sizeof(error));
        // mj_parseXMLString returns null in case of parsing error
    if (maybeSpec) {
        this->spec.reset(maybeSpec);
    } else {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(error);
    }

    // Make sure the gravity is deactivated
    std::fill_n(this->spec->option.gravity, 3, 0);

    // Body wrappers and pre-initialization parent/geometry queries must use
    // the same names as the initial compiled model, without an extra recompile.
    nameUnnamedBodies(this->spec.get());

    // Initial compilation of the model and data
    this->model.reset(mj_compile(this->spec.get(), this->virtualFileSystem.get()));
    if (!this->model) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
          compileErrorMessage(this->spec.get(), "Failed to compile the initial MuJoCo model"));
    }
    this->data.reset(mj_makeData(this->model.get()));
    if (!this->data) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>("Failed to allocate data for the initial MuJoCo model.");
    }

    {
        // This guard ensures that the model/data are not recompiled
        // until it goes out of scope. This allows the rest of the functions
        // to make changes that should trigger a recompile without doing
        // so. This is done for efficiency, but care should be taken.
        MJSpec::NoRecompileGuard guard{*this};
        this->loadBodies();
        this->loadActuators();
        this->loadEqualities();
    }
}

void MJSpec::loadBodies()
{
    // Start at 1 to skip the worldbody
    for (auto i = 1; i < this->model->nbody; i++)
    {
        const auto bodyname = this->model->names + this->model->name_bodyadr[i];

        auto mjsbody = mjs_findBody(this->spec.get(), bodyname);
        assert(mjsbody != NULL);

        this->bodies.emplace_back(mjsbody, *this);
    }
}

void MJSpec::loadActuators()
{

    // Iterate over all the existing actuators in the spec
    // and generate an MJActuatorObject for each of them
    std::unordered_map<std::string, MJActuatorObject> actuatorObjects;

    for (auto element = mjs_firstElement(this->spec.get(), mjOBJ_ACTUATOR); element;
         element = mjs_nextElement(this->spec.get(), element)) {
        auto mjsactuator = mjs_asActuator(element);
        assert(mjsactuator != NULL);

        auto name = MJBasilisk::detail::getSpecObjectName(mjsactuator);
        actuatorObjects.emplace(name, mjsactuator);
    }

    // TODO: using the `basilisk:XXX` to create force/torque actuators
    // doesn't work at the moment.
    // Generate the composite actuators. This will first check the <custom>
    // entries for existing actuators that we may use (for example, for
    // a forceactuator named "foo", we will try to reuse existing "foo_fx").
    // If no actuator was declared with the desired name, we generate our own
    // and add it to the spec. Used existing actuators are removed from actuatorObjects
    // so that the actuators remaining are not used by any composite actuator.
    for (auto&& [actuatorName, siteHint] :
         readCustomDoubleSplit(this->spec.get(), "basilisk:forceactuator", ' ', '@')) {
                    this->actuators.emplace_back(
            this->createActuator<MJForceActuator>(actuatorName, siteHint, actuatorObjects));
    }
    for (auto&& [actuatorName, siteHint] :
         readCustomDoubleSplit(this->spec.get(), "basilisk:torqueactuator", ' ', '@')) {
        this->actuators.emplace_back(
            this->createActuator<MJTorqueActuator>(actuatorName, siteHint, actuatorObjects));
    }
    for (auto&& [actuatorName, siteHint] :
         readCustomDoubleSplit(this->spec.get(), "basilisk:forcetorqueactuator", ' ', '@')) {
        this->actuators.emplace_back(
            this->createActuator<MJForceTorqueActuator>(actuatorName, siteHint, actuatorObjects));
    }
    // For those actuators not used by the composite actuators, we generate
    // a MJSingleActuator
    for (auto&& [actuatorName, actuatorObj] : actuatorObjects) {
        this->actuators.emplace_back(
            std::make_unique<MJSingleActuator>(actuatorName, std::vector{std::move(actuatorObj)}));
    }
}

void MJSpec::loadEqualities()
{
    for (auto element = mjs_firstElement(this->spec.get(), mjOBJ_EQUALITY); element;
         element = mjs_nextElement(this->spec.get(), element)) {
        auto mjsequality = mjs_asEquality(element);
        assert(mjsequality != NULL);

        auto name = MJBasilisk::detail::getSpecObjectName(mjsequality);
        this->equalities.emplace_back(mjsequality, *this);
    }
}

mjModel* MJSpec::getMujocoModel()
{
    recompileIfNeeded();
    return this->model.get();
}

mjData* MJSpec::getMujocoData()
{
    recompileIfNeeded();
    return this->data.get();
}

mjSpec*
MJSpec::getMujocoSpec()
{
    this->scene.requireSceneMutationAllowed("Mutable MuJoCo specification access");
    return this->spec.get();
}

void
MJSpec::markAsNeedingToRecompileModel()
{
    this->scene.requireSceneMutationAllowed("MuJoCo specification changes");
    this->shouldRecompile = true;
}

std::vector<std::string> MJSpec::getBodyNames() const
{
    std::vector<std::string> names;
    names.reserve(this->bodies.size());
    std::transform(std::begin(this->bodies), std::end(this->bodies), std::back_inserter(names),
                   [](const MJBody& b) { return b.getName(); });
    return names;
}

std::string MJSpec::getBodyParentName(const std::string& bodyName) const
{
    int bodyId = mj_name2id(this->model.get(), mjOBJ_BODY, bodyName.c_str());
    if (bodyId < 0) {
        MJBasilisk::detail::logAndThrow<std::invalid_argument>(
            "Tried to get parent of unknown body '" + bodyName + "'.");
    }
    int parentId = this->model->body_parentid[bodyId];
    if (parentId == 0) return "world";
    return std::string(this->model->names + this->model->name_bodyadr[parentId]);
}

std::vector<MJGeomInfo> MJSpec::getGeomInfos() const
{
    std::vector<MJGeomInfo> geoms;
    auto m = this->model.get();
    constexpr std::array<float, 4> defaultGeomRgba = {0.5f, 0.5f, 0.5f, 1.0f}; // [-]

    for (int i = 0; i < m->ngeom; i++) {
        int bodyId = m->geom_bodyid[i];
        if (bodyId == 0) continue;

        auto& info = geoms.emplace_back();
        info.bodyName = std::string(m->names + m->name_bodyadr[bodyId]);
        info.type = m->geom_type[i];
        std::copy_n(m->geom_size + i * 3, 3, std::begin(info.size));
        std::copy_n(m->geom_pos  + i * 3, 3, std::begin(info.pos));
        std::copy_n(m->geom_quat + i * 4, 4, std::begin(info.quat));
        const float* rgba = m->geom_rgba + i * 4;
        const int materialId = m->geom_matid[i];
        // Match MuJoCo's renderer: non-default geom RGBA overrides the entire
        // material color, including alpha. Explicit default gray does not.
        if (materialId >= 0 && std::equal(defaultGeomRgba.begin(), defaultGeomRgba.end(), rgba)) {
            rgba = m->mat_rgba + materialId * 4;
        }
        std::transform(rgba, rgba + 4, std::begin(info.rgba),
                       [](float v) { return static_cast<double>(v); });
    }
    return geoms;
}

bool MJSpec::hasActuator(const std::string& name)
{
    return std::find_if(std::begin(actuators), std::end(actuators), [&](auto&& obj) {
               return obj->getName() == name;
           }) != std::end(actuators);
}

MJSingleActuator& MJSpec::addJointSingleActuator(const std::string& name,
                                            const std::string& joint)
{
    if (this->hasActuator(name)) {
        BSKLogger{}.bskError("Tried to add actuator with name '%s' but one already exists with that name.", name.c_str());
    }

    this->markAsNeedingToRecompileModel();
    auto newMjsActuator = mjs_addActuator(this->spec.get(), 0);
    newMjsActuator->trntype = mjTRN_JOINT;
    MJBasilisk::detail::setSpecObjectName(newMjsActuator, name);
    mjs_setString(newMjsActuator->target, joint.c_str());
    newMjsActuator->dyntype = mjDYN_NONE;
    newMjsActuator->gaintype = mjGAIN_FIXED;
    newMjsActuator->biastype = mjBIAS_NONE;
    newMjsActuator->gainprm[0] = 1;
    auto& actuator = this->actuators.emplace_back(
        std::make_unique<MJSingleActuator>(name, std::vector{MJActuatorObject{newMjsActuator}}));
    return *static_cast<MJSingleActuator*>(actuator.get());
}

MJSingleActuator& MJSpec::addSingleActuator(const std::string& name,
                                            const std::string& site,
                                            const Eigen::Vector6d& gear)
{
    if (this->hasActuator(name)) {
        BSKLogger{}.bskError("Tried to add actuator with name '%s' but one already exists with that name.", name.c_str());
    }

    this->markAsNeedingToRecompileModel();
    auto newMjsActuator = mjs_addActuator(this->spec.get(), 0);
    newMjsActuator->trntype = mjTRN_SITE;
    MJBasilisk::detail::setSpecObjectName(newMjsActuator, name);
    mjs_setString(newMjsActuator->target, site.c_str());
    newMjsActuator->dyntype = mjDYN_NONE;
    newMjsActuator->gaintype = mjGAIN_FIXED;
    newMjsActuator->biastype = mjBIAS_NONE;
    newMjsActuator->gainprm[0] = 1;
    std::copy_n(gear.data(), 6, newMjsActuator->gear);
    auto& actuator = this->actuators.emplace_back(
        std::make_unique<MJSingleActuator>(name, std::vector{MJActuatorObject{newMjsActuator}}));
    return *static_cast<MJSingleActuator*>(actuator.get());
}

bool MJSpec::recompileIfNeeded()
{
    if (!(this->shouldRecompile && this->shouldRecompileWhenAsked)) {
        return false;
    }
    this->scene.requireSceneMutationAllowed("MuJoCo recompilation");

    std::unique_ptr<mjModel, MJBasilisk::detail::mjModelDeleter> candidate(
      mj_compile(this->spec.get(), this->virtualFileSystem.get()));
    if (!candidate) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
          compileErrorMessage(this->spec.get(), "Failed to compile the pending MuJoCo model"));
    }
    if (!this->scene.modelTopologyMatches(*candidate)) {
        MJBasilisk::detail::logAndThrow<std::logic_error>(
          "MuJoCo recompilation would change finalized state or qpos-policy topology.");
    }
    auto objectPrefixMatches = [this, &candidate](mjtObj type, int currentCount, int candidateCount) {
        if (candidateCount < currentCount) {
            return false;
        }
        for (int index = 0; index < currentCount; ++index) {
            const char* currentName = mj_id2name(this->model.get(), type, index);
            const char* candidateName = mj_id2name(candidate.get(), type, index);
            if ((currentName == nullptr) != (candidateName == nullptr) ||
                (currentName != nullptr && std::strcmp(currentName, candidateName) != 0)) {
                return false;
            }
        }
        return true;
    };
    if (this->scene.modelTopology.registered &&
        (!objectPrefixMatches(mjOBJ_BODY, this->model->nbody, candidate->nbody) ||
         !objectPrefixMatches(mjOBJ_JOINT, this->model->njnt, candidate->njnt) ||
         !objectPrefixMatches(mjOBJ_ACTUATOR, this->model->nu, candidate->nu) ||
         !objectPrefixMatches(mjOBJ_EQUALITY, this->model->neq, candidate->neq) ||
         !objectPrefixMatches(mjOBJ_PLUGIN, this->model->nplugin, candidate->nplugin))) {
        MJBasilisk::detail::logAndThrow<std::logic_error>(
          "MuJoCo recompilation would reorder or remove runtime-state objects.");
    }

    auto intPrefixMatches = [](const int* current, const int* pending, int count) {
        if (count == 0) {
            return true;
        }
        return std::equal(current, current + count, pending);
    };
    if (this->scene.modelTopology.registered &&
        (candidate->nhistory != this->model->nhistory || candidate->npluginstate != this->model->npluginstate ||
         candidate->nmocap != this->model->nmocap || candidate->nuserdata != this->model->nuserdata ||
         !intPrefixMatches(this->model->actuator_actadr, candidate->actuator_actadr, this->model->nu) ||
         !intPrefixMatches(this->model->actuator_actnum, candidate->actuator_actnum, this->model->nu) ||
         !intPrefixMatches(this->model->plugin_stateadr, candidate->plugin_stateadr, this->model->nplugin) ||
         !intPrefixMatches(this->model->plugin_statenum, candidate->plugin_statenum, this->model->nplugin) ||
         !intPrefixMatches(this->model->body_mocapid, candidate->body_mocapid, this->model->nbody))) {
        MJBasilisk::detail::logAndThrow<std::logic_error>(
          "MuJoCo recompilation would change positional runtime-state layout.");
    }

    std::unique_ptr<mjData, MJBasilisk::detail::mjDataDeleter> candidateData(mj_makeData(candidate.get()));
    if (!candidateData) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>("Failed to allocate data for the pending MuJoCo model.");
    }

    candidateData->time = this->data->time;
    std::copy_n(this->data->qpos, std::min(this->model->nq, candidate->nq), candidateData->qpos);
    std::copy_n(this->data->qvel, std::min(this->model->nv, candidate->nv), candidateData->qvel);
    std::copy_n(this->data->act, std::min(this->model->na, candidate->na), candidateData->act);
    std::copy_n(this->data->history, std::min(this->model->nhistory, candidate->nhistory), candidateData->history);
    std::copy_n(this->data->qacc_warmstart, std::min(this->model->nv, candidate->nv), candidateData->qacc_warmstart);
    std::copy_n(this->data->plugin_state,
                std::min(this->model->npluginstate, candidate->npluginstate),
                candidateData->plugin_state);
    std::copy_n(this->data->ctrl, std::min(this->model->nu, candidate->nu), candidateData->ctrl);
    std::copy_n(this->data->qfrc_applied, std::min(this->model->nv, candidate->nv), candidateData->qfrc_applied);
    std::copy_n(
      this->data->xfrc_applied, 6 * std::min(this->model->nbody, candidate->nbody), candidateData->xfrc_applied);
    std::copy_n(this->data->eq_active, std::min(this->model->neq, candidate->neq), candidateData->eq_active);
    std::copy_n(this->data->mocap_pos, 3 * std::min(this->model->nmocap, candidate->nmocap), candidateData->mocap_pos);
    std::copy_n(
      this->data->mocap_quat, 4 * std::min(this->model->nmocap, candidate->nmocap), candidateData->mocap_quat);
    std::copy_n(this->data->userdata, std::min(this->model->nuserdata, candidate->nuserdata), candidateData->userdata);

    this->model.swap(candidate);
    this->data.swap(candidateData);
    this->shouldRecompile = false;
    this->configure();
    return true;
}

void
MJSpec::recompileUntilStable()
{
    this->scene.requireSceneMutationAllowed("MuJoCo recompilation");
    while (this->shouldRecompile) {
        if (!this->recompileIfNeeded()) {
            throw std::logic_error("MuJoCo recompilation is disabled while model changes are pending.");
        }
    }
}

void
MJSpec::configureForStateRegistration()
{
    this->scene.requireSceneMutationAllowed("MuJoCo state-registration configuration");
    if (!this->shouldRecompile) {
        this->configure();
    }
    this->recompileUntilStable();
}

void
MJSpec::configure()
{
    this->scene.requireSceneMutationAllowed("MuJoCo configuration");
    for (auto& body : this->bodies) {
        body.configure(this->model.get());
    }
    for (auto& actuator : this->actuators) {
        actuator->configure(this->model.get());
    }
    for (auto& equality : this->equalities) {
        equality.configure(this->model.get());
    }
}
