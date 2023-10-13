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
    };