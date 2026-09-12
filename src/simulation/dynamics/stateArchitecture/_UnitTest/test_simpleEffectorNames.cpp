/* Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * This file is distributed under the ISC License in LICENSE.
 */

#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/FuelTank/fuelTank.h"
#include "simulation/dynamics/LinearSpringMassDamper/linearSpringMassDamper.h"
#include "simulation/dynamics/Thrusters/thrusterStateEffector/thrusterStateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/THRSimConfig.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/igbmNoiseStateEffector/igbmNoiseStateEffector.h"
#include "simulation/dynamics/meanRevertingNoiseStateEffector/meanRevertingNoiseStateEffector.h"
#include "simulation/dynamics/sphericalPendulum/sphericalPendulum.h"
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace {

template<typename Effector>
struct NameAccess
{
    std::string prefix;
    std::function<std::string(const Effector&)> get;
    std::function<void(Effector&, const std::string&)> set;
};

template<typename Effector>
struct NamingTraits;

template<>
struct NamingTraits<LinearSpringMassDamper>
{
    static std::vector<NameAccess<LinearSpringMassDamper>> names()
    {
        return {
            { "linearSpringMassDamperRho",
              [](const LinearSpringMassDamper& effector) { return effector.getNameOfRhoState(); },
              [](LinearSpringMassDamper& effector, const std::string& value) { effector.setNameOfRhoState(value); } },
            { "linearSpringMassDamperRhoDot",
              [](const LinearSpringMassDamper& effector) { return effector.getNameOfRhoDotState(); },
              [](LinearSpringMassDamper& effector, const std::string& value) {
                  effector.setNameOfRhoDotState(value);
              } },
            { "linearSpringMassDamperMass",
              [](const LinearSpringMassDamper& effector) { return effector.getNameOfMassState(); },
              [](LinearSpringMassDamper& effector, const std::string& value) { effector.setNameOfMassState(value); } },
        };
    }
};

template<>
struct NamingTraits<SphericalPendulum>
{
    static std::vector<NameAccess<SphericalPendulum>> names()
    {
        return {
            { "sphericalPendulumPhi",
              [](const SphericalPendulum& effector) { return effector.getNameOfPhiState(); },
              [](SphericalPendulum& effector, const std::string& value) { effector.setNameOfPhiState(value); } },
            { "sphericalPendulumTheta",
              [](const SphericalPendulum& effector) { return effector.getNameOfThetaState(); },
              [](SphericalPendulum& effector, const std::string& value) { effector.setNameOfThetaState(value); } },
            { "sphericalPendulumPhiDot",
              [](const SphericalPendulum& effector) { return effector.getNameOfPhiDotState(); },
              [](SphericalPendulum& effector, const std::string& value) { effector.setNameOfPhiDotState(value); } },
            { "sphericalPendulumThetaDot",
              [](const SphericalPendulum& effector) { return effector.getNameOfThetaDotState(); },
              [](SphericalPendulum& effector, const std::string& value) { effector.setNameOfThetaDotState(value); } },
            { "sphericalPendulumMass",
              [](const SphericalPendulum& effector) { return effector.getNameOfMassState(); },
              [](SphericalPendulum& effector, const std::string& value) { effector.setNameOfMassState(value); } },
        };
    }
};

template<>
struct NamingTraits<FuelTank>
{
    static std::vector<NameAccess<FuelTank>> names()
    {
        return {
            { "fuelTankMass",
              [](const FuelTank& effector) { return effector.getNameOfMassState(); },
              [](FuelTank& effector, const std::string& value) { effector.setNameOfMassState(value); } },
        };
    }
};

template<>
struct NamingTraits<ThrusterStateEffector>
{
    static std::vector<NameAccess<ThrusterStateEffector>> names()
    {
        return {
            { "kappaState",
              [](const ThrusterStateEffector& effector) { return effector.getNameOfKappaState(); },
              [](ThrusterStateEffector& effector, const std::string& value) { effector.setNameOfKappaState(value); } },
        };
    }
};

template<>
struct NamingTraits<MeanRevertingNoiseStateEffector>
{
    static std::vector<NameAccess<MeanRevertingNoiseStateEffector>> names()
    {
        return {
            { "meanRevertingNoiseState",
              [](const MeanRevertingNoiseStateEffector& effector) { return effector.getStateName(); },
              [](MeanRevertingNoiseStateEffector& effector, const std::string& value) {
                  effector.setStateName(value);
              } },
        };
    }
};

template<>
struct NamingTraits<IgbmNoiseStateEffector>
{
    static std::vector<NameAccess<IgbmNoiseStateEffector>> names()
    {
        return {
            { "igbmNoiseState",
              [](const IgbmNoiseStateEffector& effector) { return effector.getStateName(); },
              [](IgbmNoiseStateEffector& effector, const std::string& value) { effector.setStateName(value); } },
        };
    }
};

template<typename Effector>
void
configure(Effector& effector)
{
    if constexpr (std::is_same_v<Effector, FuelTank>) {
        auto model = std::make_shared<FuelTankModelConstantVolume>();
        model->propMassInit = 2.0;   // [kg]
        model->radiusTankInit = 1.0; // [m]
        effector.setTankModel(model);
    } else if constexpr (std::is_same_v<Effector, ThrusterStateEffector>) {
        effector.addThruster(std::make_shared<THRSimConfig>());
    }
}

template<typename Effector>
class SimpleEffectorNames : public testing::Test
{};

using MigratedEffectors = testing::Types<LinearSpringMassDamper,
                                         SphericalPendulum,
                                         FuelTank,
                                         ThrusterStateEffector,
                                         MeanRevertingNoiseStateEffector,
                                         IgbmNoiseStateEffector>;
TYPED_TEST_SUITE(SimpleEffectorNames, MigratedEffectors, );

TYPED_TEST(SimpleEffectorNames, resolvesGroupsInCollectionOrder)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam second;
    TypeParam first;
    configure(first);
    configure(second);
    first.collectEffectorNames(manager);
    second.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    first.registerStates(manager);
    second.registerStates(manager);
    for (const auto& name : NamingTraits<TypeParam>::names()) {
        EXPECT_EQ(name.get(first), name.prefix + "1");
        EXPECT_EQ(name.get(second), name.prefix + "2");
        const auto* firstState = manager.getStateObject(name.get(first));
        const auto* secondState = manager.getStateObject(name.get(second));
        ASSERT_NE(firstState, nullptr);
        ASSERT_NE(secondState, nullptr);
        EXPECT_NE(firstState, secondState);
    }
}

TYPED_TEST(SimpleEffectorNames, keepsLegacyConstructorNames)
{
    DynParamManager manager;
    TypeParam effector;
    configure(effector);
    const auto names = NamingTraits<TypeParam>::names();
    std::vector<std::string> original;
    for (const auto& name : names) {
        original.push_back(name.get(effector));
    }
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    effector.registerStates(manager);
    for (std::size_t index = 0; index < names.size(); ++index) {
        EXPECT_EQ(names[index].get(effector), original[index]);
        EXPECT_NE(manager.getStateObject(original[index]), nullptr);
    }
}

TYPED_TEST(SimpleEffectorNames, recognizesExplicitConstructorNamesForEveryField)
{
    TypeParam unused;
    const auto names = NamingTraits<TypeParam>::names();
    for (const auto& custom : names) {
        DynParamManager manager;
        manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
        TypeParam effector;
        configure(effector);
        const auto original = custom.get(effector);
        ASSERT_NE(original, custom.prefix + "1");
        custom.set(effector, original);
        effector.collectEffectorNames(manager);
        manager.resolveEffectorNames();
        effector.registerStates(manager);
        EXPECT_EQ(custom.get(effector), original);
        for (const auto& name : names) {
            if (name.prefix != custom.prefix) {
                EXPECT_EQ(name.get(effector), name.prefix + "1");
            }
        }
    }
}

TYPED_TEST(SimpleEffectorNames, reservesLaterCustomNamesBeforeAutomaticNames)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam automatic;
    TypeParam custom;
    configure(automatic);
    configure(custom);
    const auto names = NamingTraits<TypeParam>::names();
    names.back().set(custom, names.back().prefix + "1");
    automatic.collectEffectorNames(manager);
    custom.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    automatic.registerStates(manager);
    custom.registerStates(manager);
    for (const auto& name : names) {
        EXPECT_EQ(name.get(automatic), name.prefix + "2");
    }
    EXPECT_EQ(names.back().get(custom), names.back().prefix + "1");
}

TYPED_TEST(SimpleEffectorNames, rejectsRegistrationBeforeResolution)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    EXPECT_THROW(effector.registerStates(manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
}

TYPED_TEST(SimpleEffectorNames, rejectsChangesBetweenResolutionAndRegistration)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    NamingTraits<TypeParam>::names().front().set(effector, "changedAfterResolution");
    EXPECT_THROW(effector.registerStates(manager), BasiliskError);
    EXPECT_TRUE(manager.stateContainer.stateMap.empty());
}

TYPED_TEST(SimpleEffectorNames, repeatsRegistrationWithoutRenamingOrReallocatingStates)
{
    DynParamManager manager;
    manager.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    effector.collectEffectorNames(manager);
    manager.resolveEffectorNames();
    effector.registerStates(manager);
    for (const auto& name : NamingTraits<TypeParam>::names()) {
        const auto original = name.get(effector);
        const auto* state = manager.getStateObject(original);
        EXPECT_NO_THROW(name.set(effector, original));
        effector.registerStates(manager);
        EXPECT_EQ(name.get(effector), original);
        EXPECT_EQ(manager.getStateObject(original), state);
        EXPECT_THROW(name.set(effector, "renamed"), BasiliskError);
        EXPECT_EQ(name.get(effector), original);
    }
}

TYPED_TEST(SimpleEffectorNames, cannotRebindToAnotherLiveManager)
{
    DynParamManager first;
    DynParamManager second;
    first.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    second.setEffectorNamingPolicy(EffectorNamingPolicy::ManagerLocal);
    TypeParam effector;
    configure(effector);
    effector.collectEffectorNames(first);
    first.resolveEffectorNames();
    effector.registerStates(first);
    EXPECT_THROW(effector.registerStates(second), BasiliskError);
    EXPECT_TRUE(second.stateContainer.stateMap.empty());
}

} // namespace
