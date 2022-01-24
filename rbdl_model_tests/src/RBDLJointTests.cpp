#include "rbdl_model_tests/RBDLTestPrep.h"

TEST ( TestRBBLJointToYAML ) {
   BuildRBDLModelPtr buildRBDLModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();

   std::vector<std::string> bodyNames = buildRBDLModelPtr->getAllBodyNames();
   for(std::string bodyName : bodyNames) {
       std::cout << bodyName << std::endl;
   }

   std::unordered_map<std::string, jointParamPtr> jointParamObjectMap = buildRBDLModelPtr->getJointChildren(bodyNames[4]);

   std::unordered_map<std::string, jointParamPtr> ::iterator jointParamObjectMap_itr;
   for(jointParamObjectMap_itr = jointParamObjectMap.begin(); jointParamObjectMap_itr != jointParamObjectMap.end(); jointParamObjectMap_itr++) {
       std::cout << jointParamObjectMap_itr->first << std::endl;
   }

   jointParamPtr jointParam = jointParamObjectMap["base-link1"];

   std::cout << "jointParam->Name(): " << jointParam->Name()
             << ", jointParam->Parent(): " << jointParam->Parent()
             << ", jointParam->Type(): " << jointParam->Type()
             << ", jointParam->Child(): " << jointParam->Child()
             << std::endl;

   Vector3d joint_parent_axis = jointParam->ParentAxis();
   std::cout << "joint_parent_axis: " << joint_parent_axis(0) << ", " << joint_parent_axis(1) << ", " << joint_parent_axis(2) << std::endl;

   Vector3d joint_parent_pivot = jointParam->ParentPivot();
   std::cout << "joint_parent_pivot: " << joint_parent_pivot(0) << ", " << joint_parent_pivot(1) << ", " << joint_parent_pivot(2) << std::endl;

//    Matrix3_t m = jointParam->BodyRotation();
   
//    std::cout << "BodyRotation: " <<
//                 m(0, 0) << ", " << m(0, 1) << ", " << m(0, 2) << ", " <<
//                 m(1, 0) << ", " << m(1, 1) << ", " << m(1, 2) << ", " <<
//                 m(2, 0) << ", " << m(2, 1) << ", " << m(2, 2) << std::endl;



}

TEST( TestRBDLForwardKinematics ) {
    BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
//    std::vector<std::string> bodyNames = rbdlModelPtr->getAllBodyNames();

//    for(std::string bodyName : bodyNames) {
////        std::cout << bodyName << std::endl;
//        unsigned int body_id = rbdlModelPtr->getBodyId(bodyName.c_str());
//        std::cout << bodyName << ", " << ", body_id: " << body_id << std::endl;
//    }
//    std::cout << "------------------------------------------" << std::endl;

    Model model = rbdlModelPtr->getRBDLModel();
    std::cout << "name of body id 1: " << model.GetBodyName(1) << std::endl;
//    std::vector<std::string> bodyNames_from_model = rbdlModelPtr->getAllBodyNames();
//    for(const std::string bodyName : bodyNames_from_model) {
//        unsigned int body_id = model.GetBodyId(bodyName.c_str());
//        std::cout << bodyName << ", " << ", body_id: " << body_id << std::endl;
//    }


    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "base" << ", " << ", body_id: " << model.GetBodyId("base") << std::endl;
//    std::cout << "link1" << ", " << ", body_id: " << model.GetBodyId("link1") << std::endl;


    std::cout << "model.q_size: " << model.q_size << ", qdot_size: " << model.qdot_size << std::endl;
//    std::cout << "model.dof_count: " << model.dof_count << std::endl;
//    std::cout << model.GetBodyId("base");
////    VectorNd qdot2      = VectorNd::Zero (reference2.qdot_size)
    Math::VectorNd q =VectorNd::Zero (model.q_size);
    const Math::VectorNd qdot =VectorNd::Zero (model.qdot_size);
    const Math::VectorNd qddot =VectorNd::Zero (model.qdot_size);

////    const Math::VectorNd qdot;
////    const Math::VectorNd qddot;
    q[2] = M_PI * 0.5;
    UpdateKinematics (model,
                          q,
                          qdot,
                          qddot);

////    reference_model.at(idx).X_base[reference_body_id.at(idx)].E.data()
////    Matrix3d E = model.X_base[1].E.data();
////    Vector3d r = model.X_base[1].r.data();
//    unsigned int body_id = model.GetBodyId("link1");
//    std:cout << "model.GetBodyName(1): " << body_id << std::endl;

    Eigen::PlainObjectBase<Eigen::Matrix<double, 3, 3> >::Scalar* E = model.X_base[2].E.data();
    std::cout << "FK value E: "
              << E[0, 0] << ", " << E[0, 1] << ", " << E[0, 2] << ", "
              << E[1, 0] << ", " << E[1, 1] << ", " << E[1, 2] << ", "
              << E[2, 0] << ", " << E[2, 1] << ", " << E[2, 2] << std::endl;

    Eigen::PlainObjectBase<Eigen::Matrix<double, 3, 1> >::Scalar* r = model.X_base[2].r.data();
    std::cout << "FK value r: "
              << r[0, 0] << ", " << r[0, 1] << ", " << r[0, 2] << std::endl;
}