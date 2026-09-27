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

#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

class TestDynamicObject : public DynamicObject {
public:
    void UpdateState(uint64_t) override {}
    void equationsOfMotion(double, double) override {}
    void preIntegration(uint64_t) override { ++preCalls; }
    void postIntegration(uint64_t) override { ++postCalls; }

    int preCalls = 0;
    int postCalls = 0;
};

class CountingIntegrator : public StateVecIntegrator {
public:
    CountingIntegrator(DynamicObject* object, int& destructions)
        : StateVecIntegrator(object), destructions(destructions) {}
    ~CountingIntegrator() override { ++destructions; }

    int integrationCalls = 0;

protected:
    void prepareIntegrationBinding() override {}
    void validateIntegrationBinding() const override {}
    void integrateImpl(double, double) override { ++integrationCalls; }

private:
    int& destructions;
};

class FailingDynamicObject : public TestDynamicObject {
public:
    explicit FailingDynamicObject(int& destructions)
    {
        this->setIntegrator(new CountingIntegrator(this, destructions));
        throw std::runtime_error("Construction failed after installing an integrator");
    }
};

class FailingSynchronizedObject : public TestDynamicObject {
public:
    FailingSynchronizedObject(DynamicObject& primary, int& destructions)
    {
        this->setIntegrator(new CountingIntegrator(this, destructions));
        primary.syncDynamicsIntegration(this);
        throw std::runtime_error("Construction failed after synchronizing dynamics");
    }
};

} // namespace

/** @brief Replacement and owner destruction each release exactly one integrator. */
TEST(DynamicObjectOwnership, ReplacementAndDestruction)
{
    int oldDestructions = 0;
    int newDestructions = 0;
    {
        TestDynamicObject object;
        object.setIntegrator(new CountingIntegrator(&object, oldDestructions));
        auto* replacement = new CountingIntegrator(&object, newDestructions);
        object.setIntegrator(replacement);
        EXPECT_EQ(oldDestructions, 1);
        EXPECT_EQ(newDestructions, 0);
        EXPECT_EQ(object.getIntegrator(), replacement);
    }
    EXPECT_EQ(oldDestructions, 1);
    EXPECT_EQ(newDestructions, 1);
}

/** @brief Reinstalling the active pointer leaves its storage and dynamics intact. */
TEST(DynamicObjectOwnership, ReinstallActiveIntegrator)
{
    int destructions = 0;
    {
        TestDynamicObject object;
        auto* integrator = new CountingIntegrator(&object, destructions);
        object.setIntegrator(integrator);
        object.setIntegrator(integrator);
        EXPECT_EQ(destructions, 0);
        EXPECT_EQ(object.getIntegrator(), integrator);
        EXPECT_EQ(integrator->getDynamics(), std::vector<DynamicObject*>({&object}));
        object.integrateState(0);
        EXPECT_EQ(integrator->integrationCalls, 1);
    }
    EXPECT_EQ(destructions, 1);
}

/** @brief Rejecting a null replacement preserves the current integrator. */
TEST(DynamicObjectOwnership, RejectNull)
{
    int destructions = 0;
    {
        TestDynamicObject object;
        auto* integrator = new CountingIntegrator(&object, destructions);
        object.setIntegrator(integrator);
        EXPECT_THROW(object.setIntegrator(nullptr), BasiliskError);
        EXPECT_EQ(object.getIntegrator(), integrator);
        EXPECT_EQ(destructions, 0);
    }
    EXPECT_EQ(destructions, 1);
}

/** @brief Rejected unowned integrators are freed, including malformed inputs. */
TEST(DynamicObjectOwnership, RejectWrongOrNullCreator)
{
    int currentDestructions = 0;
    int rejectedDestructions = 0;
    {
        TestDynamicObject object;
        TestDynamicObject other;
        auto* integrator = new CountingIntegrator(&object, currentDestructions);
        object.setIntegrator(integrator);
        EXPECT_THROW(object.setIntegrator(new CountingIntegrator(&other, rejectedDestructions)), BasiliskError);
        EXPECT_EQ(rejectedDestructions, 1);

        auto* malformed = new CountingIntegrator(nullptr, rejectedDestructions);
        EXPECT_THROW(object.setIntegrator(malformed), BasiliskError);
        EXPECT_EQ(rejectedDestructions, 2);
        EXPECT_EQ(currentDestructions, 0);
        EXPECT_EQ(object.getIntegrator(), integrator);
    }
    EXPECT_EQ(currentDestructions, 1);
}

/** @brief Validation compares dynamics pointers without dereferencing an expired creator. */
TEST(DynamicObjectOwnership, RejectIntegratorWithExpiredCreator)
{
    int destructions = 0;
    auto creator = std::make_unique<TestDynamicObject>();
    auto integrator = std::make_unique<CountingIntegrator>(creator.get(), destructions);
    creator.reset();
    TestDynamicObject other;
    EXPECT_THROW(other.setIntegrator(integrator.release()), BasiliskError);
    EXPECT_EQ(other.getIntegrator(), nullptr);
    EXPECT_EQ(destructions, 1);
}

/** @brief Synced objects retain their current integrator and free rejected replacements. */
TEST(DynamicObjectOwnership, RejectReplacementOnSyncedObject)
{
    int primaryDestructions = 0;
    int secondaryDestructions = 0;
    int rejectedDestructions = 0;
    {
        TestDynamicObject primary;
        TestDynamicObject secondary;
        primary.setIntegrator(new CountingIntegrator(&primary, primaryDestructions));
        auto* original = new CountingIntegrator(&secondary, secondaryDestructions);
        secondary.setIntegrator(original);
        primary.syncDynamicsIntegration(&secondary);
        secondary.setIntegrator(new CountingIntegrator(&secondary, rejectedDestructions));
        EXPECT_EQ(rejectedDestructions, 1);
        EXPECT_EQ(secondary.getIntegrator(), original);
        secondary.setIntegrator(original);
        EXPECT_EQ(secondaryDestructions, 0);
    }
    EXPECT_EQ(primaryDestructions, 1);
    EXPECT_EQ(secondaryDestructions, 1);
}

/** @brief Replacing the primary integrator preserves the synchronized dynamics list. */
TEST(DynamicObjectOwnership, ReplacementPreservesSyncedDynamics)
{
    int oldDestructions = 0;
    int newDestructions = 0;
    int secondaryDestructions = 0;
    {
        TestDynamicObject primary;
        TestDynamicObject secondary;
        primary.setIntegrator(new CountingIntegrator(&primary, oldDestructions));
        secondary.setIntegrator(new CountingIntegrator(&secondary, secondaryDestructions));
        primary.syncDynamicsIntegration(&secondary);
        auto* replacement = new CountingIntegrator(&primary, newDestructions);
        primary.setIntegrator(replacement);
        EXPECT_EQ(oldDestructions, 1);
        EXPECT_EQ(replacement->getDynamics(), std::vector<DynamicObject*>({&primary, &secondary}));
        primary.integrateState(0);
        secondary.integrateState(0);
        EXPECT_EQ(replacement->integrationCalls, 1);
        EXPECT_EQ(primary.preCalls, 1);
        EXPECT_EQ(primary.postCalls, 1);
        EXPECT_EQ(secondary.preCalls, 1);
        EXPECT_EQ(secondary.postCalls, 1);
    }
    EXPECT_EQ(newDestructions, 1);
}

/** @brief Constructor failure releases the integrator during stack unwinding. */
TEST(DynamicObjectOwnership, ConstructionFailure)
{
    int destructions = 0;
    EXPECT_THROW({ FailingDynamicObject object(destructions); }, std::runtime_error);
    EXPECT_EQ(destructions, 1);
}

/** @brief A destroyed secondary is removed while other synchronized objects remain usable. */
TEST(SynchronizedDynamicsLifetime, SecondaryDestructionUnlinks)
{
    int destructions = 0;
    TestDynamicObject primary;
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    auto first = std::make_unique<TestDynamicObject>();
    TestDynamicObject second;
    first->setIntegrator(new CountingIntegrator(first.get(), destructions));
    second.setIntegrator(new CountingIntegrator(&second, destructions));
    primary.syncDynamicsIntegration(first.get());
    primary.syncDynamicsIntegration(&second);
    first.reset();
    ASSERT_EQ(primary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&primary, &second}));
    primary.integrateState(0);
    EXPECT_EQ(second.preCalls, 1);
    EXPECT_EQ(second.postCalls, 1);
}

/** @brief A surviving secondary can integrate independently or join another primary. */
TEST(SynchronizedDynamicsLifetime, PrimaryDestructionDetaches)
{
    int destructions = 0;
    TestDynamicObject secondary;
    secondary.setIntegrator(new CountingIntegrator(&secondary, destructions));
    auto primary = std::make_unique<TestDynamicObject>();
    primary->setIntegrator(new CountingIntegrator(primary.get(), destructions));
    primary->syncDynamicsIntegration(&secondary);
    primary.reset();
    ASSERT_FALSE((secondary.getIntegrationOwner() != nullptr));
    secondary.integrateState(0);
    EXPECT_EQ(secondary.preCalls, 1);

    TestDynamicObject replacement;
    replacement.setIntegrator(new CountingIntegrator(&replacement, destructions));
    secondary.setIntegrator(new CountingIntegrator(&secondary, destructions));
    replacement.syncDynamicsIntegration(&secondary);
    replacement.integrateState(0);
    EXPECT_EQ(secondary.preCalls, 2);
}

/** @brief Repeating a connection does not advance the secondary more than once. */
TEST(SynchronizedDynamicsLifetime, RepeatedConnectionIsNoOp)
{
    int destructions = 0;
    TestDynamicObject primary;
    TestDynamicObject secondary;
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    secondary.setIntegrator(new CountingIntegrator(&secondary, destructions));
    primary.syncDynamicsIntegration(&secondary);
    primary.syncDynamicsIntegration(&secondary);
    ASSERT_EQ(primary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&primary, &secondary}));
    primary.integrateState(0);
    EXPECT_EQ(secondary.preCalls, 1);
    EXPECT_EQ(secondary.postCalls, 1);
}

/** @brief Integrator replacement preserves the links used to remove expired secondaries. */
TEST(SynchronizedDynamicsLifetime, ReplacementPreservesUnlinking)
{
    int destructions = 0;
    TestDynamicObject primary;
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    auto secondary = std::make_unique<TestDynamicObject>();
    secondary->setIntegrator(new CountingIntegrator(secondary.get(), destructions));
    primary.syncDynamicsIntegration(secondary.get());
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    secondary.reset();
    EXPECT_EQ(primary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&primary}));
}

/** @brief A rejected null replacement preserves links until either owner is destroyed. */
TEST(SynchronizedDynamicsLifetime, CleanupAfterRejectedReplacement)
{
    for (bool primaryFirst : {false, true}) {
        int destructions = 0;
        auto primary = std::make_unique<TestDynamicObject>();
        auto secondary = std::make_unique<TestDynamicObject>();
        primary->setIntegrator(new CountingIntegrator(primary.get(), destructions));
        secondary->setIntegrator(new CountingIntegrator(secondary.get(), destructions));
        primary->syncDynamicsIntegration(secondary.get());
        EXPECT_THROW(primary->setIntegrator(nullptr), BasiliskError);
        if (primaryFirst) {
            primary.reset();
            EXPECT_FALSE((secondary->getIntegrationOwner() != nullptr));
            secondary.reset();
        } else {
            secondary.reset();
            primary.reset();
        }
        EXPECT_EQ(destructions, 2);
    }
}

/** @brief Invalid group changes are rejected before any connection is modified. */
TEST(SynchronizedDynamicsLifetime, RejectInvalidConnections)
{
    int destructions = 0;
    TestDynamicObject primary;
    TestDynamicObject secondary;
    TestDynamicObject other;
    TestDynamicObject unconfigured;
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    secondary.setIntegrator(new CountingIntegrator(&secondary, destructions));
    other.setIntegrator(new CountingIntegrator(&other, destructions));
    EXPECT_THROW(unconfigured.syncDynamicsIntegration(&secondary), BasiliskError);
    EXPECT_THROW(primary.syncDynamicsIntegration(&unconfigured), BasiliskError);
    EXPECT_THROW(primary.syncDynamicsIntegration(nullptr), BasiliskError);
    EXPECT_THROW(primary.syncDynamicsIntegration(&primary), BasiliskError);
    primary.syncDynamicsIntegration(&secondary);
    EXPECT_THROW(other.syncDynamicsIntegration(&secondary), BasiliskError);
    EXPECT_THROW(secondary.syncDynamicsIntegration(&other), BasiliskError);
    EXPECT_THROW(other.syncDynamicsIntegration(&primary), BasiliskError);
    EXPECT_EQ(primary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&primary, &secondary}));
    EXPECT_EQ(secondary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&secondary}));
    EXPECT_EQ(other.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&other}));
    EXPECT_FALSE((primary.getIntegrationOwner() != nullptr));
    EXPECT_TRUE((secondary.getIntegrationOwner() != nullptr));
    EXPECT_FALSE((other.getIntegrationOwner() != nullptr));
}

/** @brief Constructor failure removes a secondary connected during its construction. */
TEST(SynchronizedDynamicsLifetime, ConstructionFailureUnlinks)
{
    int destructions = 0;
    TestDynamicObject primary;
    primary.setIntegrator(new CountingIntegrator(&primary, destructions));
    EXPECT_THROW({ FailingSynchronizedObject secondary(primary, destructions); }, std::runtime_error);
    EXPECT_EQ(primary.getIntegrator()->getDynamics(), std::vector<DynamicObject*>({&primary}));
}
