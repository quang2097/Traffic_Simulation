#pragma once
#include <algorithm>

namespace traffic {
inline bool consumeSpawnClock(double& credit,double seconds,double rate) {
    credit+=seconds*rate;
    if(credit+1e-10<1.) return false;
    // The tolerance can accept 0.999999999999... . fmod would leave that
    // almost-full credit intact and spawn again on the following frame.
    credit=std::max(0.,credit-1.);
    return true;
}
}
