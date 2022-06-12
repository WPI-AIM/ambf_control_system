#ifndef JOINTPARAM_H
#define JOINTPARAM_H
#include "application/Utilities.h"

/*
 * JointParam Class is used to hold values of Joint parameter
 */
class JointParam
{
public:
  JointParam(YAML::Node jointNode);
  JointParam(std::string name, std::string parent_name, std::string child, Vector3d parent_axis,
              Vector3d parent_pivot, Vector3d child_axis, Vector3d child_pivot, std::string type);

  ~JointParam(void);

  inline const std::string Name() const { return name_; }
  inline const std::string Parent() const { return parent_; }
  inline const std::string Child() const { return child_; }

  inline const Vector3d ParentAxis() const { return parent_axis_; }
  inline const Vector3d ParentPivot() const { return parent_pivot_; }

  inline const Vector3d ChildAxis() const { return child_axis_; }
  inline const Vector3d ChildPivot() const { return child_pivot_; }

  inline double JointLimitsHigh() const { return joint_limits_high_; }
  inline double JointLimitsLow() const { return joint_limits_low_; }

  inline const std::string Type() const { return type_; }

  inline double Offset() const { return offset_; }
  inline const int Weight() const { return weight_; }

private:
  std::string name_{""};
  std::string parent_{""};
  std::string child_{""};
  Vector3d parent_axis_{ 0.0, 0.0, 0.0 };
  Vector3d parent_pivot_{ 0.0, 0.0, 0.0 };
  Vector3d child_axis_{ 0.0, 0.0, 0.0 };
  Vector3d child_pivot_{ 0.0, 0.0, 0.0 };
  double joint_limits_high_{0.0};
  double joint_limits_low_{0.0};
  std::string type_{""};
  double offset_{ 0.0 };
  int weight_{1};
};

#endif // JOINTS_H
