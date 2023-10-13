﻿#include <d2V_spring_particle_particle_dq2.h>

void d2V_spring_particle_particle_dq2_compared(Eigen::Ref<Eigen::Matrix66d> H, Eigen::Ref<const Eigen::Vector3d> q0, Eigen::Ref<const Eigen::Vector3d> q1, double l0, double stiffness) {
    //解析解有误
    double l = (q0 - q1).norm();
    //Eigen::Matrix66d BtB;
    //BtB << 1, 0, 0, -1, 0, 0,
    //    0, 1, 0, 0, -1, 0,
    //    0, 0, 1, 0, 0, -1,
    //    -1, 0, 0, 1, 0, 0,
    //    0, -1, 0, 0, 1, 0,
    //    0, 0, -1, 0, 0, 1;
    //Eigen::Vector6d q;
    //q << q0(0),
    //    q0(1),
    //    q0(2),
    //    q1(0),
    //    q1(1),
    //    q1(2);
    //H = stiffness * BtB - stiffness * l0 * ((BtB * l - BtB * q * (BtB * q).transpose()) / (l * l));

    ////std::cout << "D2V_SPRING_PARTICLE_PARTICLE_DQ2::DEBUG::H" << H << std::endl;
    //H(0, 0) = stiffness * (l0 - l) * 1.0 / l 
    //    - (stiffness * pow(q0[0] - q1[0] , 2.0)) / (l*l)
    //    - (stiffness * pow(q0[0] - q1[0], 2.0) * (l0 - l) / (l*l*l));
    H(0, 0) = -(stiffness - stiffness * l0 * (l - (pow((q0[0] - q1[0]), 2) / l)) / (l * l));
    H(0, 1) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    // H(0, 1) = (stiffness - stiffness * l0 * (l - (pow((q1[0] - q0[0]), 2) / l)) / (l*l));
    H(0, 2) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(0, 3) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(0, 4) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(0, 5) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 0) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 1) = stiffness * (l0 - l) * 1.0 / l - (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) - (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 2) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 3) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 4) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(1, 5) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 0) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 1) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 2) = stiffness * (l0 - l) * 1.0 / l - (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) - (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 3) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 4) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(2, 5) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 0) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 1) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 2) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 3) = stiffness * (l0 - l) * 1.0 / l - (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) - (stiffness * pow(q0[0] * 2.0 - q1[0] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 4) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(3, 5) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 0) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 1) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 2) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 3) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[1] * 2.0 - q1[1] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 4) = stiffness * (l0 - l) * 1.0 / l - (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) - (stiffness * pow(q0[1] * 2.0 - q1[1] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(4, 5) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 0) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 1) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 2) = -stiffness * (l0 - l) * 1.0 / l + (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) + (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 3) = (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * 2 * (q0[0] - q1[0]) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 4) = (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (-1.0 / 4.0)) / (l * l) - (stiffness * (q0[1] * 2.0 - q1[1] * 2.0) * (q0[2] * 2.0 - q1[2] * 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
    H(5, 5) = stiffness * (l0 - l) * 1.0 / l - (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0)) / (pow(q0[0] - q1[0], 2.0) * 4.0 + pow(q0[1] - q1[1], 2.0) * 4.0 + pow(q0[2] - q1[2], 2.0) * 4.0) - (stiffness * pow(q0[2] * 2.0 - q1[2] * 2.0, 2.0) * (l0 - l) * 1.0 / (l * l * l)) / 4.0;
}

void d2V_spring_particle_particle_dq2(Eigen::Ref<Eigen::Matrix66d> H, Eigen::Ref<const Eigen::Vector3d> q0,  Eigen::Ref<const Eigen::Vector3d> q1, double l0, double stiffness) {

    Eigen::Matrix66d A, B, C;
    Eigen::Matrix33d distance_comb = (q1 - q0) * (q1 - q0).transpose();
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

    H = stiffness * A + l0 / distance * B + 1 / (distance * distance * distance) * C;
    
    H = -H; 

    Eigen::Matrix66d true_H;
    d2V_spring_particle_particle_dq2_compared(true_H, q0, q1, l0, stiffness);

    assert(H.isApprox(true_H, 0.001));
}