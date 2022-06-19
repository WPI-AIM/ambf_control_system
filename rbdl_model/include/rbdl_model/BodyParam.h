#ifndef BODYPARAM_H
#define BODYPARAM_H
#include <iostream>
#include "application/Utilities.h"

/*
 * BodyParam Class is used to hold values of Body parameter
 */
using namespace RBDLModel;

class BodyParam
{
public:
    BodyParam(YAML::Node bodyNode);
    ~BodyParam(void);
    inline const std::string Name() const { return name_; }
    inline double Mass() const { return mass_; }
    inline Vector3d InertialOffsetPosition() const { return inertial_offset_position_; }
    inline Vector3d InertialOffsetOrientation() const { return inertial_offset_orientation_; }
    inline Math::Matrix3d Inertia() const { return inertia_; }
    inline bool Passive() const { return passive_; }
private:
    std::string name_;
    // Adding a small mass as RBDL doesnt allow zero mass object.
    double mass_{ 0.00000001 };
    Math::Matrix3d inertia_;
    Vector3d inertial_offset_position_;
    //substitute this value to com in RBDL
    Vector3d inertial_offset_orientation_;
    bool passive_{ false };
    std::string trimTrailingSpaces(YAML::Node bodyNode);
};

#endif // AFRIGIDBODY_H
