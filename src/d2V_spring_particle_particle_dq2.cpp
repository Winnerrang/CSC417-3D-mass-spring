#include <d2V_spring_particle_particle_dq2.h>
#include <iostream>

void d2V_spring_particle_particle_dq2(Eigen::Ref<Eigen::Matrix66d> H, Eigen::Ref<const Eigen::Vector3d> q0,  Eigen::Ref<const Eigen::Vector3d> q1, double l0, double stiffness) {

    
    Eigen::Matrix66d A, B, C;
    Eigen::Matrix3d distance_comb = (q1 - q0) * (q1 - q0).transpose();
    double distance = (q1 - q0).norm();

    A.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
    A.block<3,3>(0, 3) = -Eigen::Matrix3d::Identity();
    A.block<3,3>(3, 0) = -Eigen::Matrix3d::Identity();
    A.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();


    B = -A;

    C.block<3, 3>(0, 0) = distance_comb;
    C.block<3, 3>(0, 3) = -distance_comb;
    C.block<3, 3>(3, 0) = -distance_comb;
    C.block<3, 3>(3, 3) = distance_comb;

    H = stiffness * (A + l0 /(distance) * B + l0 / (distance * distance * distance) * C);
    
    H = -H; 

}