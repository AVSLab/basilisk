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

// Include first to verify that the public header supplies its own dependencies.
#include "architecture/system_model/sim_model.h"

#include <array>
#include <atomic>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace {

constexpr uint64_t taskPeriod = 10; // [ns]
constexpr uint64_t stopTime = 20;   // [ns]

class CountingModel : public SysModel
{
  public:
    void SelfInit() override
    {
        ++selfInitCalls;
        if (failure == "selfInit") {
            throw std::runtime_error("Expected self-init failure");
        }
    }
    void Reset(uint64_t) override
    {
        ++resetCalls;
        if (failure == "reset") {
            throw std::runtime_error("Expected reset failure");
        }
        times.clear();
    }
    void UpdateState(uint64_t time) override
    {
        if (failure == "update") {
            throw std::runtime_error("Expected update failure");
        }
        if (onUpdate) {
            onUpdate();
        }
        times.push_back(time);
    }

    int selfInitCalls = 0;
    int resetCalls = 0;
    std::string failure;
    std::vector<uint64_t> times; // [ns]
    std::function<void()> onUpdate;
};

struct ProcessFixture
{
    explicit ProcessFixture(const std::string& name)
      : task(taskPeriod)
      , process(name)
    {
        task.AddNewObject(&model);
        process.addNewTask(&task);
    }
    CountingModel model;
    SysModelTask task;
    SysProcess process;
};

void
initialize(SimModel& simulation)
{
    simulation.assignRemainingProcs();
    simulation.ResetSimulation();
    simulation.selfInitSimulation();
    simulation.resetInitSimulation();
}

static_assert(!std::is_copy_constructible_v<SimModel>);
static_assert(!std::is_move_constructible_v<SimModel>);
static_assert(!std::is_copy_constructible_v<SimThreadExecution>);

} // namespace

/** @brief Unstarted pools can be replaced, deleted repeatedly and destroyed. */
TEST(SimThreadOwnership, UnstartedPoolAndRepeatedDeletion)
{
    SimModel simulation;
    EXPECT_EQ(simulation.getThreadCount(), 1);
    for (uint64_t count : { 1u, 4u, 2u }) {
        simulation.resetThreads(count);
        EXPECT_EQ(simulation.getThreadCount(), count);
        for (auto const& worker : simulation.threadList) {
            EXPECT_FALSE(worker->threadContext.joinable());
        }
    }
    simulation.deleteThreads();
    simulation.deleteThreads();
    EXPECT_EQ(simulation.getThreadCount(), 0);
    EXPECT_THROW(simulation.assignRemainingProcs(), BasiliskError);
    simulation.resetThreads(1);
    EXPECT_THROW(simulation.resetThreads(0), BasiliskError);
    EXPECT_EQ(simulation.getThreadCount(), 1);
}

/** @brief A standalone worker joins during both normal destruction and exception unwinding. */
TEST(SimThreadOwnership, WorkerDestructionJoins)
{
    for (bool unwind : { false, true }) {
        bool stopped = false;
        try {
            SimThreadExecution worker;
            worker.threadContext = std::thread([&]() {
                worker.postInit();
                worker.lockThread();
                stopped = !worker.threadValid();
            });
            worker.waitOnInit();
            if (unwind) {
                throw std::runtime_error("Unwind the worker owner");
            }
        } catch (const std::runtime_error&) {
            EXPECT_TRUE(unwind);
        }
        EXPECT_TRUE(stopped);
    }
}

/** @brief Destruction also joins the started part of a partially constructed pool. */
TEST(SimThreadOwnership, PartialStartupUnwinds)
{
    std::array<bool, 2> stopped{};
    EXPECT_THROW(
      {
          SimModel simulation;
          simulation.resetThreads(4);
          for (std::size_t index = 0; index < stopped.size(); ++index) {
              auto* worker = simulation.threadList[index].get();
              worker->threadContext = std::thread([&, worker, index]() {
                  worker->postInit();
                  worker->lockThread();
                  stopped[index] = !worker->threadValid();
              });
              worker->waitOnInit();
          }
          throw std::runtime_error("Simulate failure after starting some workers");
      },
      std::runtime_error);
    EXPECT_TRUE(stopped[0]);
    EXPECT_TRUE(stopped[1]);
}

/** @brief Empty simulations and workers without assigned processes shut down cleanly. */
TEST(SimThreadOwnership, EmptyPoolStartsAndStops)
{
    SimModel simulation;
    simulation.resetThreads(3);
    initialize(simulation);
    EXPECT_THROW(simulation.assignRemainingProcs(), BasiliskError);
    simulation.deleteThreads();
    EXPECT_EQ(simulation.getThreadCount(), 0);
}

/** @brief Repeated pool resizing preserves borrowed processes and their scheduled outputs. */
TEST(SimThreadOwnership, ResetAndReinitialize)
{
    ProcessFixture first("first");
    ProcessFixture second("second");
    SimModel simulation;
    simulation.addNewProcess(&first.process);
    simulation.addNewProcess(&second.process);
    const std::vector<uint64_t> expectedTimes{ 0, 10, 20 }; // [ns]
    for (uint64_t count : { 1u, 4u, 2u, 1u }) {
        simulation.resetThreads(count);
        EXPECT_FALSE(first.process.getProcessControlStatus());
        simulation.addProcessToThread(&first.process, count - 1);
        initialize(simulation);
        simulation.StepUntilStop(stopTime, -1);
        EXPECT_EQ(first.model.times, expectedTimes);
        EXPECT_EQ(second.model.times, expectedTimes);
    }
    simulation.deleteThreads();
    // The simulation only borrows processes, tasks and models.
    first.model.Reset(0);
    EXPECT_TRUE(first.model.times.empty());
}

/** @brief Shutdown wakes an idle worker without executing queued initialization work. */
TEST(SimThreadOwnership, ShutdownDoesNotRunPendingInitialization)
{
    ProcessFixture fixture("process");
    SimModel simulation;
    simulation.addNewProcess(&fixture.process);
    simulation.assignRemainingProcs();
    simulation.threadList.front()->selfInitNow = true;
    simulation.deleteThreads();
    EXPECT_EQ(fixture.model.selfInitCalls, 0);
}

/** @brief Reset waits for active work before changing process assignments. */
TEST(SimThreadOwnership, ResetJoinsActiveWorker)
{
    ProcessFixture fixture("process");
    SimModel simulation;
    simulation.addNewProcess(&fixture.process);
    initialize(simulation);
    auto* worker = simulation.threadList.front().get();
    std::atomic<bool> entered{ false };
    bool assignmentRetained = false;
    fixture.model.onUpdate = [&]() {
        entered = true;
        while (worker->threadValid()) {
            std::this_thread::yield();
        }
        assignmentRetained = fixture.process.getProcessControlStatus();
    };
    worker->stopThreadNanos = 0; // [ns]
    worker->unlockThread();
    while (!entered) {
        std::this_thread::yield();
    }
    simulation.resetThreads(2);
    EXPECT_TRUE(assignmentRetained);
    EXPECT_FALSE(fixture.process.getProcessControlStatus());
    EXPECT_EQ(fixture.model.times.size(), 1);
}

/** @brief Worker failures at each lifecycle stage can be joined and the simulation restarted. */
TEST(SimThreadOwnership, ShutdownAfterWorkerException)
{
    for (const auto* failure : { "selfInit", "reset", "update" }) {
        ProcessFixture fixture("process");
        SimModel simulation;
        simulation.addNewProcess(&fixture.process);
        simulation.resetThreads(3);
        fixture.model.failure = failure;
        EXPECT_THROW(
          {
              initialize(simulation);
              simulation.StepUntilStop(stopTime, -1);
          },
          std::runtime_error);
        simulation.deleteThreads();
        EXPECT_EQ(simulation.getThreadCount(), 0);
        fixture.model.failure.clear();
        simulation.resetThreads(2);
        initialize(simulation);
        simulation.StepUntilStop(stopTime, -1);
        EXPECT_EQ(fixture.model.times.size(), 3);
    }
}
