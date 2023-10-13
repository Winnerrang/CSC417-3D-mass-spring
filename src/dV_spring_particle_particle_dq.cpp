#include <dV_spring_particle_particle_dq.h>

//Input:
//  q0 - the generalized coordinates of the first node of the spring
//  q1 - the generalized coordinates of the second node of the spring
//  l0 - the undeformed length of the spring
//  stiffness - the stiffness constant for this spring
//Output:
//  f - the 6x1 per spring generalized energy gradient vector
void dV_spring_particle_particle_dq(Eigen::Ref<Eigen::Vector6d> f, Eigen::Ref<const Eigen::Vector3d> q0,  Eigen::Ref<const Eigen::Vector3d>     q1, double l0, double stiffness) {
    double force_mag = stiffness * ((q1 - q0).norm() - l0);
    
    f.segment<3>(0) = - force_mag * (q1 - q0).normalized();
    f.segment<3>(3) = force_mag * (q1 - q0).normalized();
    
}