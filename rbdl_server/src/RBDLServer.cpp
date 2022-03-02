#include "rbdl_server/RBDLServer.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    
    //  model = new Model();
     have_model = false;
     FD_srv = nh_.advertiseService("ForwardDynamics", &RBDLServer::ForwardDynamics_srv, this);
     ID_srv = nh_.advertiseService("InverseDynamics", &RBDLServer::InverseDynamics_srv, this);
     MD_srv = nh_.advertiseService("CreateModel", &RBDLServer::CreateModel_srv, this);
     Jac_srv = nh_.advertiseService("Jacobian", &RBDLServer::Jacobian_srv, this);
     Kin_srv = nh_.advertiseService("ForwardKinimatics", &RBDLServer::ForwardKinimatics_srv, this);
     InvKin_srv = nh_.advertiseService("InverseKinimatics", &RBDLServer::InverseKinimatics_srv, this);
     JointNames_srv = nh_.advertiseService("JointNames", &RBDLServer::GetJointNames_srv, this);
     JointAlign_srv = nh_.advertiseService("AMBF2RBDL", &RBDLServer::AMBF2RBDL_srv, this);
     Mjoint_srv = nh_.advertiseService("JointSpaceInertia", &RBDLServer::JointSpaceInertia_srv, this);
     PtVel_srv = nh_.advertiseService("PointVelocity", &RBDLServer::PointVelocity_srv, this);
     NonlinEff_srv = nh_.advertiseService("NonlinearEffects", &RBDLServer::NonlinearEffects_srv, this);


     ROS_INFO("RBDL server running");
     ros::spinOnce;
}

RBDLServer::~RBDLServer()
{
    for(std::unordered_map<std::string, RigidBodyDynamics::Model*>::iterator itr = models.begin(); itr != models.end(); itr++)
    {
        delete(itr->second);
    }
}


RigidBodyDynamics::Model* RBDLServer::getModel(std::string name)
{
    return models[name];
}

///
/// \brief RBDLServer::CreateModel_srv
/// \param req
/// \param res
///
bool RBDLServer::CreateModel_srv(rbdl_server::RBDLModelRequest& req, rbdl_server::RBDLModelResponse& res) //parses the AMBF model into  rbdl model
{

    ROS_INFO("Parsing");
    std::string name = req.model_name;


    //create the name of then model 
    if( name.empty()  )
    {
        //if there are not models and the name was not set then make it the default one
        if(!have_model)
        {
            ROS_INFO("Setting the name to default");
            name = default_name;
        }
        //if there is all ready a model create and no name was provide then throw error
        else
        {   
            ROS_ERROR("Need to set model name");
            return false;
        }
        
    }
    
    models[name] = new Model();
    std::string actuator_config_file = req.model;
    BuildRBDLModel  buildRBDLModel(actuator_config_file);
    ROS_INFO("ch0");
    body_ids[name] = buildRBDLModel.getRBDLBodyToIDMap();
    joint_names[name] = buildRBDLModel.getJointNames();
    ROS_INFO("ch1");
    *models[name] = buildRBDLModel.getRBDLModel();
    ROS_INFO("ch2");
    //buildRBDLModel.cleanUp();
    joint_map[name] = buildRBDLModel.getRBDLJointToIDMap();
    std::unordered_map<std::string, unsigned int>::iterator itr;
    ROS_INFO("ch3");
   
    for (itr = joint_map[name].begin(); itr != joint_map[name].end(); itr++) 
    {
        std::cout << "name: " << itr->first << ", id: " << itr->second<<"\n";
    }

    
    have_model = true;
    ROS_INFO("Parsed");

    return true;
}

 bool RBDLServer::GetJointNames_srv(rbdl_server::RBDLBodyNamesRequest& req, rbdl_server::RBDLBodyNamesResponse& res)
 {

    std::string name = req.model_name;
    ROS_INFO("Called Get Joint name Server");
    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (checkModelExists(name))
    {
        for (auto i = joint_names[name].begin(); i != joint_names[name].end(); ++i)
            std::cout << *i << ' ';

        res.names = joint_names[name];
        return true;
    }
    else
    {
        return false;
    }
    
     
 }


 bool RBDLServer::AMBF2RBDL_srv(rbdl_server::RBDLModelAlignmentRequest& req , rbdl_server::RBDLModelAlignmentResponse& res)
 {

    ROS_INFO("Called AMBF to RBDL Server");        
    std::vector<std::string> names;
    std::vector<int> ids;
    std::unordered_map<std::string, unsigned int>::iterator itr;
    std::string name = req.model_name;
   
    if(name.empty())
    {
        name = default_name;
    }

    if (checkModelExists(name))
    {
        for (itr = joint_map[name].begin(); itr != joint_map[name].end(); itr++) 
        {
            names.push_back(itr->first);
            ids.push_back(itr->second);
        }
        res.names = names;
        res.ids = ids;
        return true;
    }
    else
    {
        return false; 
    }
    
 }



///
/// \brief RBDLServer::ForwardDynamics_srv
/// \param req
/// \param res
///
bool  RBDLServer::ForwardDynamics_srv(rbdl_server::RBDLForwardDynamicsRequest& req, rbdl_server::RBDLForwardDynamicsResponse&  res )
{

    ROS_INFO("Called Forward Dynamics Server");
    std::string name = req.model_name;
 
    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (!checkModelExists(name))
    {

        return false;
    }

    // Need to add some checks on the size of the inputs to make sure they are correct
    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (models[name]->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
    if (models[name]->qdot_size != req.qd.size())
    {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }
    if (models[name]->qdot_size != req.tau.size() )
    {
        ROS_INFO("Joint torque (tau) not correct size");
        return false;
    }

    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd Tau = VectToEigen(req.tau);
    VectorNd QDDot = VectorNd::Zero (models[name]->qdot_size);
    ForwardDynamics (*models[name], Q, QDot, Tau, QDDot);
    std::vector<double> qdd(&QDDot[0], QDDot.data()+QDDot.cols()*QDDot.rows());
    res.qdd = qdd;
    return true;
}

///
/// \brief RBDLServer::InverseDynamics_srv
/// \param req
/// \param res
///
bool RBDLServer::InverseDynamics_srv(rbdl_server::RBDLInverseDynamicsRequest& req, rbdl_server::RBDLInverseDynamicsResponse&  res)
{

    ROS_INFO("Called Inverse Dynamics Server");
    std::string name = req.model_name;
 

    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (!checkModelExists(name))
    {

        return false;
    }

    // Need to add some checks on the size of the inputs to make sure they are correct
    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (models[name]->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
    if (models[name]->qdot_size != req.qd.size())
    {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }
    if (models[name]->qdot_size != req.qdd.size() )
    {
        ROS_INFO("Joint torque (tau) not correct size");
        return false;
    }


    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd QDDot = VectToEigen(req.qdd);
    VectorNd Tau = VectorNd::Zero (models[name]->qdot_size);
    InverseDynamics(*models[name], Q, QDot, QDDot, Tau );
    std::vector<double> tau(&Tau[0], Tau.data()+Tau.cols()*Tau.rows());
    res.tau = tau;
    return true;

}


///
/// \brief RBDLServer::InverseKinimatics_srv
/// \param req
/// \param res
///
bool RBDLServer::InverseKinimatics_srv(rbdl_server::RBDLInverseKinimaticsRequest& req, rbdl_server::RBDLInverseKinimaticsResponse& res)
{
    
    ROS_INFO("Called Inverse Kinimatics Server");
    double ste_tol = 1.0e-12;
    double lambda = 0.01;
    unsigned int max_iter = 50;

    std::string name = req.model_name;
    VectorNd Q_res = VectorNd::Zero(models[name]->q_size);
    VectorNd Q = VectorNd::Zero(models[name]->q_size);
    Vector3d local_point (0.0, 0.0, 0.0);
    Vector3d target_point (req.target.x, req.target.y, req.target.z);

    if(req.model_name.empty())
    {
        name = default_name;
    }


    if (checkModelExists(name))
    {
        int id = body_ids[name][req.body_name];
        
        InverseKinematicsConstraintSet cs;
        cs.AddPointConstraint(id,local_point, target_point );

        bool worked = InverseKinematics(*models[name], Q_res, cs, Q  );

        if(worked)
        {
            std::vector<double> q(&Q[0], Q.data()+Q.cols()*Q.rows());
            res.q_res = q;
        }

        res.worked = worked;
        return true;
    }
    else
    {
        return false;
    }
}

///
/// \brief RBDLServer::ForwardKinimatics_srv
/// \param req
/// \param res
///
bool RBDLServer::ForwardKinimatics_srv(rbdl_server::RBDLKinimaticsRequest& req, rbdl_server::RBDLKinimaticsResponse& res)
{

    ROS_INFO("Called Forward Kinimatics Server");

    std::string key;
    int id;
    std::string name = req.model_name;
    VectorNd Q = VectorNd::Zero(models[name]->q_size);
    Vector3d point(0,0,0);
    geometry_msgs::Point current_point;
    std::vector<geometry_msgs::Point> points;
    std::vector<std::string> names;
    Vector3d fk;
    geometry_msgs::Pose pose;
    int size = res.points.size();
   
    
    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (checkModelExists(name))
    {
        if (models[name]->q_size != req.q.size())
        {
            ROS_INFO("Joint length (q) not correct size");
            return false;
        }
        else
        {
            Q = VectToEigen(req.q);
        }
    
        for(std::pair<std::string, int> body : body_ids[name])
        {
            key = body.first;
            id = body.second;
            fk = CalcBodyToBaseCoordinates(*models[name], Q, id, point, true);
            current_point.x = fk(0);
            current_point.y = fk(1);
            current_point.z = fk(2);
            points.push_back(current_point);
            names.push_back(key);
        }

        res.names = names;
        res.points = points;
        return true;

    }
    else
    {
        
        ROS_ERROR("Model not set");
        return false;         
        
    }
    
 
}

///
/// \brief RBDLServer::Jacobian_srv
/// \param req
/// \param res
///
bool RBDLServer::Jacobian_srv(rbdl_server::RBDLJacobianRequest& req, rbdl_server::RBDLJacobianResponse& res)
{
    
    ROS_INFO("Called Jacobian Server");

    std::vector<std::string> names;
    int id;
    std_msgs::Float64MultiArray msg;
    std::string name = req.model_name;
    MatrixNd G (MatrixNd::Zero (6, models[name]->dof_count));
    VectorNd Q = VectToEigen(req.q);
    Vector3d point(req.point.x, req.point.y, req.point.z);
    
 
    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (checkModelExists(name))
    {

        if (models[name]->q_size != req.q.size())
        {
            ROS_INFO("Joint length (q) not correct size");
            return false;
        }
        if (body_ids[name].find(req.body_name) != body_ids[name].end()) 
        {
            id = body_ids[name][req.body_name];
        } 
        else 
        {
            ROS_INFO("That is not a body, the current bodies are");
            GetNames(name, names);
            for(std::string name: names)
            {
                ROS_INFO("%s", name.c_str());
            }

            return false;
        }

        
        CalcPointJacobian6D(*models[name], Q, id, point, G, false);
        tf::matrixEigenToMsg(G, msg);
        res.jacobian = msg;
        return true;
    }
    else
    {
        return false;
    }
    
}

///
/// \brief RBDLServer::GetNames_srv
/// \param req
/// \param res
///
bool RBDLServer::GetNames_srv(rbdl_server::RBDLBodyNamesRequest& req, rbdl_server::RBDLBodyNamesResponse& res)
{
    ROS_INFO("Called Get body name Server");
    
    std::vector<std::string> names;
    std::string name = req.model_name;
 
    if(req.model_name.empty())
    {
        name = default_name;
    }

    if (checkModelExists(name))
    {
        //check to see if the model is active
        if(!have_model)
        {
            ROS_INFO("Model not set");
            return false;
        }

        GetNames(name, names);
        res.names = names;

        return true;
    }
    else
    {
        return false;
    }
}

///
/// \brief RBDLServer::VectToEigen
/// \param msg
///
VectorNd RBDLServer::VectToEigen(const std::vector<double> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    VectorNd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}

///
/// \brief RBDLServer::GetNames
/// \param names
///
void RBDLServer::GetNames(const std::string& name, std::vector<std::string>& names)
{
    //loop through the names and add them to the vect
    for(std::pair<std::string, int> body : body_ids[name])
    {
        names.push_back(body.first);

    }

}


bool RBDLServer::checkModelExists(std::string key) 
{
   
   return (bool)models.count( key );

}


/**
*	ROS service for the current joint-space inertia matrix of the model
*	@param req is the request service data to be sent
*   @param res is the response service data returned
*	@return if the function executed successfully or not (run on an existing model with a valid number of joints)
**/
bool RBDLServer::JointSpaceInertia_srv(rbdl_server::RBDLJointSpaceInertiaRequest& req, rbdl_server::RBDLJointSpaceInertiaResponse& res) {
    
    ROS_INFO("Called Joint Space Inertia Server");

    std_msgs::Float64MultiArray msg;												// initializing the service response value (M matrix at given q) 
    std::string name = req.model_name;												// the name of the model for which calculations will be performed
    

    // Default to another name if the request has an invalid one, program will seg fault if functions are called on an empty model name
    if(req.model_name.empty()) {

        name = default_name;
    }


    int numDoF = models[name]->dof_count;											// number of Degrees of Freedom (number of joints) of the model 
    
    MatrixNd M (MatrixNd::Zero (numDoF, numDoF));									// initializing M(q) matrix output of CompositeRidgidBodyAlgorithm()
    VectorNd Q = VectToEigen(req.q);												// and its input q vector of joint states

    // Only execute the RBDL function on a valid model
    if (checkModelExists(name)) {


    	//ROS_INFO("q size: %d", req.q.size());

    	// Only execute on a valid number of model joints
        if (models[name]->q_size != req.q.size()) {

            ROS_INFO("Joint length (q) not correct size");
            return false;
        }


        // RBDL function to calculate the model M joint space inertia matrix at the current configuration Q
    	CompositeRigidBodyAlgorithm(*models[name], Q, M, false);

    	tf::matrixEigenToMsg(M, msg);												// convert the MatrixNd output to a std_msgs
        res.Mjoint = msg;															// and set it as the response


        //ROS_INFO("model name: %s", name.c_str());       // for testing
        //ROS_INFO("DoF: %d", numDoF);


        return true;
    }
    else {

        return false;
    }


    


    return false;
    
}


/**
*	ROS service for calculating the current velocity of a point on the model
*	@param req is the request service data to be sent
*   @param res is the response service data returned
*	@return if the function executed successfully or not (run on an existing model with a valid number of joints)
**/
bool RBDLServer::PointVelocity_srv(rbdl_server::RBDLPointVelocityRequest& req, rbdl_server::RBDLPointVelocityResponse& res) {

    ROS_INFO("Called Point Velocity Server");

    std::vector<std::string> names;                                                 // vector of body names to print out if invalid name given in request 
    int id;                                                                         // body id of interest to use in the input of the RBDL function
    Vector3d point(req.point.x, req.point.y, req.point.z);                          // point on the body in 3D space to use in the input of the RBDL function

    std::string name = req.model_name;                                              // the name of the model for which calculations will be performed

 
    // Default to another name if the request has an invalid one, program will seg fault if functions are called on an empty model name
    if(req.model_name.empty()) {
        name = default_name;
    }

    // Only execute the RBDL function on a valid model
    if (!checkModelExists(name)) {
        return false;
    }

    // Need to add some checks on the size of the inputs to make sure they are correct
    if(!have_model) {
        ROS_INFO("Model not set");
        return false;
    }

    // Only execute on a valid number of model joints
    if (models[name]->q_size != req.q.size()) {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }

    // Only execute on a valid number of model joint velocities
    if (models[name]->qdot_size != req.qd.size()) {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }

    // Find the body id among the current model's list of body names
    if (body_ids[name].find(req.body_name) != body_ids[name].end()) {
        id = body_ids[name][req.body_name];
    } 
    else {
        ROS_INFO("That is not a body, the current bodies are");
        GetNames(name, names);

        for(std::string name: names) {
            ROS_INFO("%s", name.c_str());
        }

        return false;
    }

    VectorNd Q =  VectToEigen(req.q);                                               // initializing input q vector of joint states
    VectorNd QDot = VectToEigen(req.qd);                                            // and q dot vector or joint velocities

    Math::SpatialVector Vel;                                                        // initializing output 6 DoF velocity vector of the RBDL function

    // RBDL function to calculate the joint veloctiy vector for the specified body of the model at the current configuration Q and QDot
    Vel = CalcPointVelocity6D(*models[name], Q, QDot, id, point, true);
    
    std::vector<double> velocity(&Vel[0], Vel.data()+Vel.cols()*Vel.rows());        // Eigen (RBDL Math::SpatialVector) to std::vector conversion for output
    res.velocity = velocity;                                                        // and then set it as the response

    return true;

}


/**
*	ROS service for calculating the current joint torques caused by nonlinear effects of the model 
*	@param req is the request service data to be sent
*   @param res is the response service data returned
*	@return if the function executed successfully or not (run on an existing model with a valid number of joints)
**/
bool RBDLServer::NonlinearEffects_srv(rbdl_server::RBDLNonlinearEffectsRequest& req, rbdl_server::RBDLNonlinearEffectsResponse& res) {

    ROS_INFO("Called Nonlinear Effects Server");

    std::string name = req.model_name;                                              // the name of the model for which calculations will be performed
 
    // Default to another name if the request has an invalid one, program will seg fault if functions are called on an empty model name
    if(req.model_name.empty()) {
        name = default_name;
    }

    // Only execute the RBDL function on a valid model
    if (!checkModelExists(name)) {
        return false;
    }

    // Need to add some checks on the size of the inputs to make sure they are correct
    if(!have_model) {
        ROS_INFO("Model not set");
        return false;
    }

    // Only execute on a valid number of model joints
    if (models[name]->q_size != req.q.size()) {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }

    // Only execute on a valid number of model joint velocities
    if (models[name]->qdot_size != req.qd.size()) {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }

    VectorNd Q =  VectToEigen(req.q);                                               // initializing input q vector of joint states
    VectorNd QDot = VectToEigen(req.qd);                                            // and q dot vector or joint velocities

    VectorNd Tau = VectorNd::Zero (models[name]->qdot_size);                        // initializing output Tau vector of joint torques

    // RBDL function to calculate the model nonlinear joint torque vector at the current configuration Q and QDot
    NonlinearEffects(*models[name], Q, QDot, Tau);

    std::vector<double> tau(&Tau[0], Tau.data()+Tau.cols()*Tau.rows());             // Eigen (VectorNd) to std::vector conversion for output
    res.tau = tau;                                                                  // and then set it as the response

    return true;

}