#ifndef RBDLTESTPREP_H
#define RBDLTESTPREP_H

#include <atomic>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>

#include "rbdl_model/BuildRBDLModel.h"
#include "application/Prep.h"
//------------------------------------------------------------------------------
typedef BuildRBDLModel* BuildRBDLModelPtr;
typedef Model* RBDLModelPtr; 
//------------------------------------------------------------------------------

const double TEST_PREC = 1.0e-2;


// constexpr auto tenMill= 10000000;
class RBDLTestPrep
{
public:
    static RBDLTestPrep* getInstance(){
    RBDLTestPrep* sin= instance_.load(std::memory_order_acquire);
    if ( !sin )
    {
        std::lock_guard<std::mutex> myLock(myMutex_);
        sin = instance_.load(std::memory_order_relaxed);
        if( !sin )
        {
            sin = new RBDLTestPrep();
            instance_.store(sin,std::memory_order_release);
        }
    }
        return sin;
    }

    std::chrono::duration<double> getTime();
    BuildRBDLModelPtr getRBDLModelInstance();
private:
    RBDLTestPrep()= default;
    ~RBDLTestPrep()= default;
    RBDLTestPrep(const RBDLTestPrep&)= delete;
    RBDLTestPrep& operator=(const RBDLTestPrep&)= delete;

    static std::atomic<RBDLTestPrep*> instance_;
    static std::mutex myMutex_;

    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/KUKA/kuka.yaml";
    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/ECM2/blender-ecm.yaml";
    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/PSM/blender-psm.yaml";
    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/MTM/blender-mtm.yaml";
//    const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/Neuro/blender-neuro.yaml";

};



#endif // RBDLTESTPREP_H