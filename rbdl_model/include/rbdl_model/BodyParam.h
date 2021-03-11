#ifndef BODYPARAM_H
#define BODYPARAM_H
#include <iostream>
#include <rbdl_model/Utilities.h>


using namespace RBDLModel;

class BodyParam
{
public:
    BodyParam(YAML::Node bodyNode);
    ~BodyParam(void);
    inline double Mass() { return mass_; }
    inline Vector3d InertialOffsetPosition() { return inertial_offset_position_; }
    inline Vector3d InertialOffsetOrientation() { return inertial_offset_orientation_; }

    inline Math::Matrix3d Inertia() { return inertia_; }

private:

    std::string name_;
    double mass_{0.00000001};
    Math::Matrix3d inertia_;
    Vector3d inertial_offset_position_;
    //substitute this value to com in RBDL
    Vector3d inertial_offset_orientation_;
    std::string trimTrailingSpaces(YAML::Node bodyNode);

};

#endif // AFRIGIDBODY_H
