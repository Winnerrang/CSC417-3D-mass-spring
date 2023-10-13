#include <T_particle.h>

//Input:
//  qdot - generalized velocity for the specific particle
//  mass - the mass of a particle
//Output:
//  T - kinetic energy of the particles in the mass-spring system
void T_particle(double &T, Eigen::Ref<const Eigen::VectorXd> qdot, double mass) {

    // T = 1.2 m v^2;
    T = 0.5 * mass * qdot.squaredNorm();
}