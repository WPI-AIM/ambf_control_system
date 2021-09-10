# ambf_control_system

## Authors:
1. [Nathaniel Goldfarb](https://github.com/nag92) (nagoldfarb@wpi.edu)
2. [Shreyas Chandra Sekhar](https://github.com/cshreyastech) (schandrasekhar@wpi.edu)
This package contains the RBDL server and Controller server for 


## Overview
controlling robotic model in AMBF. The purpose is to provide a framework to build model based controller for simulated robotic systems in the AMBF frame work. AMBF can be found [here](https://github.com/WPI-AIM/ambf/workflows/ambf-1.0/badge.svg?branch=ambf-1.0). The frameworks uses ROS servervices message to communicate between you custom application and the servers. This allows 

### RBDL server
The 


## Depencies

The follow need to be installed. Please follow their respected install instructions

1. [Rigid Body Dynamic library](https://github.com/ORB-HD/rbdl-orb). 
2. 



## Installtion

The ambf_control_package needs to be installed in a catkin_ws




## Examples

A server object can be created using the following code

```C++
#include "ros/ros.h"
#include "rbdl_server/RBDLServer.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Dynamic_Server");
    ros::NodeHandle n;
   
    RBDLServer my_server(&n);       

    ros::spin();

    return 0;
}

```


And can be communicated with using the following code. The RBDL model first has to be created using the AMBF model file. The dynamics can then be calculated. IT is important to note that joint order matters a great deal and the joint order will not match. 


```python

    """
    use the RBDL server to create the model 
    """
    import rospy
    from rbdl_server.srv import RBDLModel, RBDLModelAlignment
    from rbdl_server.srv import RBDLInverseDynamics
    
    RBDL = "path to Yaml model file"
    name = "name of model" # This can be anything you want, by assaining spesific names muiple models can be controlled.

    


    try:
        print("making model")
        model_srv = rospy.ServiceProxy('CreateModel', RBDLModel)
        resp1 = model_srv(name, model_path)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


    """
    Then the dynamics can be calculated using the following
    """

   
    tau = np.asarray([0.0] * joint_num)
    rospy.wait_for_service("InverseDynamics")
    try:
        dyn_srv = rospy.ServiceProxy('InverseDynamics', RBDLInverseDynamics)
        resp1 = dyn_srv(model_name, q, qd, qdd)
        tau = resp1.tau   
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

```

Below is an example of how to convert between RBDL model and the AMBF model.


```python


    def ambf_to_rbdl(self, q):
        """
        make the order of the joints for the dynamics
        """

        names = self._selected_joint_names # A list of the joints names you want 
        ambf_joints_names = self.handle.get_joint_names()
        joints_aligned = [0.0]*len(names)
        q_new = [0.0]*len(names)

        for ii, name in enumerate(names):
            if(len(names) > len(ambf_joints_names)):
                index = self._joint_map_selected[name]
            else:
                index = self._joint_map[name] - 1
            joints_aligned[index] = q[ii]

        return joints_aligned

    def rbdl_to_ambf(self, q):
        """
        reverse the order of the AMBF
        """
        
        names = self._selected_joint_names
        ambf_joints_names = self.handle.get_joint_names()
        q_new = [0.0]*len(names)
        for ii, name in enumerate(names):
            if(len(names) > len(ambf_joints_names)):
                index = self._joint_map_selected[name]
            else:
                index = self._joint_map[name] - 1
            
            q_new[ii] = q[index]

        return q_new

```
