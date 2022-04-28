#ifndef BODYPARAM_H
#define BODYPARAM_H
#include <iostream>
#include <rbdl_model/Utilities.h>

/*
 * BodyParam Class is used to hold values of Body parameter
 */
using namespace RBDLModel;

class BodyParam
{
public:
    BodyParam(YAML::Node bodyNode);
    ~BodyParam(void);
    inline double Mass() { return mass_; }
    inline std::string MeshName() { return mesh_name_; }
    inline Vector3d Position() { return location_position_; }
    inline Vector3d Orientation() { return location_orientation_; }
    inline Vector3d InertialOffsetPosition() { return inertial_offset_position_; }
    inline Vector3d InertialOffsetOrientation() { return inertial_offset_orientation_; }
    inline Math::Matrix3d Inertia() { return inertia_; }

private:

    std::string name_;
    std::string mesh_name_;

    double mass_{0.00000001};
    Math::Matrix3d inertia_;
    Vector3d inertial_offset_position_;
    //substitute this value to com in RBDL
    Vector3d inertial_offset_orientation_;

    Vector3d location_position_;
    Vector3d location_orientation_;
    std::string trimTrailingSpaces(YAML::Node bodyNode);

};

#endif // AFRIGIDBODY_H
