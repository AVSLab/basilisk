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

#include <cassert>
#include <iterator>

#include "MJScene.h"

namespace
{
std::vector<std::string> readCustomSingleSplit(mjSpec* spec, const std::string& key, char delimiter)
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
        BSKLogger{}.bskError("%s", error);
    }

    // Make sure the gravity is deactivated
    std::fill_n(this->spec->option.gravity, 3, 0);

    // Initial compilation of the model and data
    this->model.reset(mj_compile(this->spec.get(), this->virtualFileSystem.get()));
    this->data.reset(mj_makeData(this->model.get()));

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

    for (int i = 0; i < m->ngeom; i++) {
        int bodyId = m->geom_bodyid[i];
        if (bodyId == 0) continue;

        auto& info = geoms.emplace_back();
        info.bodyName = std::string(m->names + m->name_bodyadr[bodyId]);
        info.type = m->geom_type[i];
        std::copy_n(m->geom_size + i * 3, 3, std::begin(info.size));
        std::copy_n(m->geom_pos  + i * 3, 3, std::begin(info.pos));
        std::copy_n(m->geom_quat + i * 4, 4, std::begin(info.quat));
        std::transform(m->geom_rgba + i * 4, m->geom_rgba + (i + 1) * 4, std::begin(info.rgba),
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

    mj_recompile(this->spec.get(),
                 this->virtualFileSystem.get(),
                 this->model.get(),
                 this->data.get());

    this->shouldRecompile = false;
    configure();

    return true;
}

void MJSpec::configure()
{
    // configure() can run via recompileIfNeeded() while MJScene::initializeDynamics
    // is still registering the bulk states, before they exist. In that case there
    // is nothing to size yet; initializeDynamics calls configure() again once the
    // states are registered.
    if (!this->scene.getQposState()) {
        return;
    }

    // Size the bulk states to the model dimensions. Only sizes are touched, not
    // values: configure() runs on every recompile, and the stored state must
    // survive it. conservativeResize keeps existing entries when the size is
    // unchanged.
    auto resizeState = [](StateData* s, int n) {
        s->state.conservativeResize(n, 1);
        s->stateDeriv.conservativeResize(n, 1);
    };

    // getQposState()->configure() sizes qpos to nq and records the location of
    // every orientation quaternion in it.
    this->scene.getQposState()->configure(this->model.get());
    resizeState(this->scene.getQvelState(), this->model->nv);

    // The bodies seed their own mass entries below; the world body (index 0) has
    // no MJBody, so seed it here to keep the whole vector well-defined.
    resizeState(this->scene.getMassState(), this->model->nbody);
    this->scene.getMassState()->stateDeriv.setZero();
    this->scene.getMassState()->state(0) = this->model->body_mass[0];

    // The act state exists only when na > 0.
    if (this->scene.getActState()) {
        resizeState(this->scene.getActState(), this->model->na);
    }

    // Configure the bodies, which caches the body id corresponding to the name,
    // and also configures all sites and updates the position of the COM
    for (auto&& body : this->bodies) {
        body.configure(this->model.get());
    }

    // Configure the actuators, which caches the actuator id corresponding to the name
    for (auto&& actuator : this->actuators) {
        actuator->configure(this->model.get());
    }

    // Configure the equalities, which caches the actuator id corresponding to the name
    for (auto&& equality : this->equalities) {
        equality.configure(this->model.get());
    }
}
