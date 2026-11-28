#pragma once
#include "traffic/SignalLearning.hpp"
#include "traffic/TrainingData.hpp"
#include <future>
#include <memory>
#include <string>

namespace traffic {
struct PolicyReply {
    enum Kind {Ready,Action,SelectModel,Error,HoldAction} kind=Error;
    std::string message;
    std::array<double,2> green{};
    bool training=false;
};
class PolicyRuntime {
public:
    PolicyRuntime()=default;
    ~PolicyRuntime();
    void load(const std::string& python,const std::string& script,const std::string& weights,const std::string& algorithm);
    void request(const SignalObservation& observation);
    void requestTraining(const TrainingSnapshot& observation,std::array<double,2> mask);
    bool poll(PolicyReply& reply);
    void shutdown();
    bool busy() const {return pending_.valid();}
    bool loaded() const {return loaded_;}
    bool trainingSchema() const {return training_;}
private:
    struct Worker;
    std::shared_ptr<Worker> worker_;
    std::future<PolicyReply> pending_;
    bool loaded_=false,training_=false;
};
}
