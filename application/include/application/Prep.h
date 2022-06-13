#ifndef AMBFTESTPREP_H
#define AMBFTESTPREP_H

#include <atomic>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"

#include "ambf_client/ambf_client.h"
#include "application/AMBFParamWrapper.h"
//------------------------------------------------------------------------------
typedef Client* AMBFClientPtr;
typedef AMBFParamWrapper* AMBFParamWrapperPtr; 
//------------------------------------------------------------------------------
constexpr auto tenMill= 10000000;

class ADF {   
public:      
  inline static const std::string filePath = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/ECM2/blender-ecm.yaml";
};

class AMBFTestPrep
{
public:
    static AMBFTestPrep* getInstance(){
    AMBFTestPrep* sin= instance_.load(std::memory_order_acquire);
    if ( !sin )
    {
        std::lock_guard<std::mutex> myLock(myMutex_);
        sin = instance_.load(std::memory_order_relaxed);
        if( !sin )
        {
            sin = new AMBFTestPrep();
            instance_.store(sin,std::memory_order_release);
        }
    }
        return sin;
    }

    std::chrono::duration<double> getTime();
    AMBFClientPtr getAMBFClientInstance();
    AMBFParamWrapperPtr getAMBFParamWrapperInstance();
    inline static const std::string ADFPath() { return ADF::filePath; }

private:
    AMBFTestPrep()= default;
    ~AMBFTestPrep()= default;
    AMBFTestPrep(const AMBFTestPrep&)= delete;
    AMBFTestPrep& operator=(const AMBFTestPrep&)= delete;

    static std::atomic<AMBFTestPrep*> instance_;
    static std::mutex myMutex_;
    // std::string AMBFTestPrep::myconst = "myconst";

    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/KUKA/kuka.yaml";
    // static const char* const adfFilePath_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/ECM2/blender-ecm.yaml";
    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/PSM/blender-psm.yaml";
    // const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/MTM/blender-mtm.yaml";
//    const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/Neuro/blender-neuro.yaml";

};



#endif // AMBFTESTPREP_H
