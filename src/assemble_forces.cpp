#include <assemble_forces.h>
#include <iostream>
#include <dV_spring_particle_particle_dq.h>
//Input:
//  q - generalized coordinates for the mass-spring system
//  qdot - generalized velocity for the mass spring system
//  V - the nx3 matrix of undeformed vertex positions. Each row is a single undeformed vertex position.
//  E - the mx2 spring connectivity matrix. Each row contains to indices into V that indicate a spring between those vertices.
//  l0 - the mx1 vector of undeformed spring lengths
//  m - the mass of each particle in the mass-spring system
//  k - the stiffness of each spring in the mass-spring system
//Output:
//  f - the vector 3xn vector of forces acting on each node of the mass-spring system
void assemble_forces(Eigen::VectorXd &f, Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<const Eigen::VectorXd> qdot, 
                     Eigen::Ref<const Eigen::MatrixXd> V, Eigen::Ref<const Eigen::MatrixXi> E, Eigen::Ref<const Eigen::VectorXd> l0, 
                     double mass, double k) { 
        f.resize(q.size());
        f.setZero();

        for (int springIdx = 0; springIdx < E.rows(); springIdx++) {
            Eigen::Vector6d f_spring;
            dV_spring_particle_particle_dq(f_spring, q.segment<3>(E(springIdx, 0) * 3), q.segment<3>(E(springIdx, 1) * 3), l0(springIdx), k);
            f.segment<3>(E(springIdx, 0) * 3) += f_spring.segment<3>(0);
            f.segment<3>(E(springIdx, 1) * 3) += f_spring.segment<3>(3);
        }


        // f = -dV/dq
        f = -f;



        //f.resize(q.rows());
        //f.setZero();
        //for (int y = 0; y < E.rows(); y++)
        //{
        //    Eigen::Vector6d ft;
        //    //这里必须竖着写，因为要的是列向量
        //    ft << 0,
        //        -9.8,
        //        0,
        //        0,
        //        -9.8,
        //        0;
        //    //ft << 0, -9.6, 0, 0, -9.8, 0;
        //    //dV_gravity_particle_dq(ft, mass, Eigen::Vector3d(0, 0, 0));
        //    int i = E(y, 0);
        //    int j = E(y, 1);
        //    Eigen::Vector3d q0, q1;
        //    q0 << q(i * 3),
        //        q(i * 3 + 1),
        //        q(i * 3 + 2);
        //    q1 << q(j * 3),
        //        q(j * 3 + 1),
        //        q(j * 3 + 2);
        //    dV_spring_particle_particle_dq(ft, q0, q1, l0(y), k);
        //    //f<< -ft.x(),-ft.y(),-ft.z();
        //    f(i * 3) += ft(0);
        //    f(i * 3 + 1) += ft(1);
        //    f(i * 3 + 2) += ft(2);
        //    f(j * 3) += ft(3);
        //    f(j * 3 + 1) += ft(4);
        //    f(j * 3 + 2) += ft(5);
        //}
        //f = -f;
    };