#include "traffic/Parallel.hpp"
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace traffic::detail {
unsigned availableWorkers() { return std::clamp(std::thread::hardware_concurrency(),1u,4u); }
namespace {
class Pool {
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::packaged_task<bool()>> jobs_;
    std::vector<std::thread> threads_;
    bool stopping_=false;
public:
    Pool() {
        try {
            for(unsigned i=1;i<availableWorkers();++i) threads_.emplace_back([this] {
                for(;;) {
                    std::packaged_task<bool()> job;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        ready_.wait(lock,[this] { return stopping_ || !jobs_.empty(); });
                        if(stopping_ && jobs_.empty()) return;
                        job=std::move(jobs_.front()); jobs_.pop_front();
                    }
                    job();
                }
            });
        } catch(...) { stop(); throw; }
    }
    void stop() {
        { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
        ready_.notify_all();
        for(auto& thread:threads_) if(thread.joinable()) thread.join();
    }
    ~Pool() { stop(); }
    std::future<bool> submit(std::function<bool()> work) {
        std::packaged_task<bool()> job(std::move(work)); auto result=job.get_future();
        { std::lock_guard<std::mutex> lock(mutex_); jobs_.push_back(std::move(job)); }
        ready_.notify_one(); return result;
    }
};
}
bool parallelAll(std::size_t count,unsigned workers,const std::function<bool(std::size_t)>& predicate) {
    workers=std::min<unsigned>(std::clamp(workers,1u,availableWorkers()),static_cast<unsigned>(std::min<std::size_t>(count,4)));
    if(workers<2) {
        for(std::size_t i=0;i<count;++i) if(!predicate(i)) return false;
        return true;
    }
    static Pool pool;
    std::vector<std::future<bool>> pending;
    pending.reserve(workers-1);
    auto range=[&](unsigned part) {
        for(std::size_t i=count*part/workers;i<count*(part+1)/workers;++i) if(!predicate(i)) return false;
        return true;
    };
    bool result=true; std::exception_ptr error;
    try {
        for(unsigned part=1;part<workers;++part) pending.push_back(pool.submit([&,part] { return range(part); }));
        result=range(0);
    } catch(...) { error=std::current_exception(); }
    // Drain all jobs even after an exception: captures refer to this call's stack.
    for(auto& future:pending) {
        try { bool value=future.get(); result=value && result; }
        catch(...) { if(!error) error=std::current_exception(); }
    }
    if(error) std::rethrow_exception(error);
    return result;
}
}
