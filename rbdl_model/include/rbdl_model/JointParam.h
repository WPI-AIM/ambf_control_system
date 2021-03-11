#ifndef JOINTPARAM_H
#define JOINTPARAM_H
#include <rbdl_model/Utilities.h>

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

    inline std::string Name() { return name_; }
    inline std::string Parent() { return parent_; }
    inline std::string Child() { return child_; }

    inline Vector3d ParentAxis() { return parent_axis_; }
    inline Vector3d ParentPivot() { return parent_pivot_; }

    inline Vector3d ChildAxis() { return child_axis_; }
    inline Vector3d ChildPivot() { return child_pivot_; }

    inline std::string Type() { return type_; }

private:
    std::string name_;
    std::string parent_;
    std::string child_;
    Vector3d parent_axis_;
    Vector3d parent_pivot_;
    Vector3d child_axis_;
    Vector3d child_pivot_;
    std::string type_;


};

#endif // JOINTS_H
