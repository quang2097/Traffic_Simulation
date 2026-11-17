#pragma once
#include <cstddef>
#include <functional>

namespace traffic::detail {
// Read-only predicates run concurrently; every job is joined before returning.
bool parallelAll(std::size_t count,unsigned workers,const std::function<bool(std::size_t)>& predicate);
unsigned availableWorkers();
}
