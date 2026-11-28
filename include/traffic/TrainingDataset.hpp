#pragma once
#include <cstdint>
#include <memory>
#include <string>
namespace traffic {
// Per-run recordings remain immutable; finalized runs are published atomically.
class TrainingDataset {
public:
    TrainingDataset();
    ~TrainingDataset();
    TrainingDataset(const TrainingDataset&)=delete;
    TrainingDataset& operator=(const TrainingDataset&)=delete;
    std::string begin(const std::string& root,bool append,std::uint32_t seed);
    void finish();
    void cancel();
    bool active() const;
    const std::string& root() const {return root_;}
private:
    struct Impl;std::unique_ptr<Impl> impl_;
    std::string root_;
};
}
