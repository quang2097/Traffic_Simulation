#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace traffic {
// An on-demand duration split into equal physics ticks no larger than 1/120 s.
// The caller may render or process input between ticks without adding wall time.
class StepBatch {
    double requested_=0,tickSeconds_=0;
    std::uint64_t totalTicks_=0,remainingTicks_=0;
public:
    void request(double seconds) {
        if(active()) throw std::logic_error("A simulation step is already pending");
        if(!std::isfinite(seconds) || seconds<1e-8 || seconds>3600)
            throw std::invalid_argument("Step duration must be positive and at most 3600 seconds");
        requested_=seconds;
        totalTicks_=remainingTicks_=static_cast<std::uint64_t>(std::ceil(seconds*120));
        tickSeconds_=seconds/totalTicks_;
    }
    template<class Advance> bool advanceOne(Advance&& advance) {
        if(!active()) return false;
        advance(tickSeconds_);
        // The callback can cancel a batch after an engine error. Do not
        // underflow the counter and turn that cancellation into an endless run.
        if(!remainingTicks_) return false;
        --remainingTicks_; return true;
    }
    void cancel() { requested_=tickSeconds_=0; totalTicks_=remainingTicks_=0; }
    bool active() const { return remainingTicks_>0; }
    double requestedSeconds() const { return requested_; }
    double completedSeconds() const { return (totalTicks_-remainingTicks_)*tickSeconds_; }
    double remainingSeconds() const { return remainingTicks_*tickSeconds_; }
    double progress() const { return totalTicks_?static_cast<double>(totalTicks_-remainingTicks_)/totalTicks_:0; }
};
}
