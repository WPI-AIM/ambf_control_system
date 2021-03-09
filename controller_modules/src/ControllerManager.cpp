#include "controller_modules/ControllerManager.h"


/**
 * @brief Construct a new Controller Manager:: Controller Manager object
 * 
 * @param n ros node handle
 */
ControllerManager::ControllerManager(ros::NodeHandle *n) : nh(*n)
{
    calc_tau_srv = nh.advertiseService("CalcTau", &ControllerManager::calcTauSrv, this);
    controller_list_srv = nh.advertiseService("ControllerList", &ControllerManager::getControllersListSrv, this);
}

ControllerManager::ControllerManager() {}

ControllerManager::~ControllerManager()
{
}

/**
 * @brief Ros serverice to get a list of the controllers
 * 
 * @param req blank message
 * @param res list of controllers
 * @return true service was called
 * @return false service failed to be called
 */
bool ControllerManager::getControllersListSrv(controller_modules::ControllerListRequest &req, controller_modules::ControllerListResponse &res)
{
    res.controllers = getControllerList();
    return true;
}

/**
 * @brief get a list of the attached controllers
 * 
 * @return std::vector<std::string> 
 */
std::vector<std::string> ControllerManager::getControllerList()
{
    std::vector<std::string> names;
    for (std::pair<std::string, boost::shared_ptr<ControllerBase>> node : controller_list)
    {
        names.push_back(node.first);
    }
    return names;
}

/**
 * @brief add a controller to the manager that can be accessed by the server
 * 
 * @param name the name of the controller to be added 
 * @param controller the controller to be added 
 * 
 */
void ControllerManager::addController(std::string name, boost::shared_ptr<ControllerBase> controller)
{
    //boost::shared_ptr<ControllerBase> cntl(*controller) ;
    controller_list[name] = controller;
}

/**
 * @brief service to call the control manager, used the name of the controller to calculat the control input
 * 
 * @param req Joint control request message 
 * @param res Joint control responce message 
 * @return true 
 * @return false 
 */
bool ControllerManager::calcTauSrv(controller_modules::JointControlRequest &req, controller_modules::JointControlResponse &res)
{
    std::vector<double> output; // declare a var to hold the output
    std::string name = req.controller_name; //get the name of the controller to call
    // ROS_INFO( name.c_str()  ) ;
    std::unordered_map<std::string, boost::shared_ptr<ControllerBase>>::const_iterator got = controller_list.find (name); //check is controller exists
    
    //if the controller is not found throw error
    if (got == controller_list.end() )
    {
        ROS_ERROR("Controller not found");
        return false;
    }
    else
    {
        //if the controller is found call the controller
        controller_list[name.c_str()]->update(req.actual, req.desired, output);
        res.control_output.effort = output;
        return true;
    }

}
