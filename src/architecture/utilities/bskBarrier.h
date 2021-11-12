// Keith Rausch

#ifndef COMMON_UTILS_BARRIER_HPP
#define COMMON_UTILS_BARRIER_HPP

#include <mutex>
#include <memory>
#include <condition_variable>
// https://stackoverflow.com/questions/48712881/how-can-i-create-a-barrier-in-c

namespace common::utils
{
    class Barrier
    {
        std::mutex mutex;
        std::condition_variable conditional;
        std::size_t counter;
        std::size_t generation;
        std::size_t reset;

    public:

        Barrier(size_t count) : counter(count), generation(0), reset(count)
        {
        }

        void Wait()
        {
            std::unique_lock<std::mutex> lock(mutex);
            unsigned int gen = generation;

            if (--counter == 0)
            {
                generation++;
                counter = reset;
                lock.unlock();
                conditional.notify_all();
                return;
            }

            while (gen == generation)
                conditional.wait(lock);
        }
    };

    typedef std::shared_ptr<Barrier> sBarrier;
}

#endif