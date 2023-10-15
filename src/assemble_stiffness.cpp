#include <assemble_stiffness.h>
#include <iostream>
void assemble_stiffness(Eigen::SparseMatrixd &K, Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<const Eigen::VectorXd> qdot, 
                     Eigen::Ref<const Eigen::MatrixXd> V, Eigen::Ref<const Eigen::MatrixXi> E, Eigen::Ref<const Eigen::VectorXd> l0, 
                     double k) { 
    K.resize(q.size(), q.size());
    K.setZero();

    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(q.rows() * q.rows());

    for (int springIdx = 0; springIdx < E.rows(); springIdx++) {
        Eigen::Matrix66d K_spring;
        d2V_spring_particle_particle_dq2(K_spring, q.segment<3>(E(springIdx, 0) * 3), q.segment<3>(E(springIdx, 1) * 3), l0(springIdx), k);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {

                for (int ii = 0; ii < 3; ii++) {
                    for (int jj = 0; jj < 3; jj++) {

                        tripletList.push_back(Eigen::Triplet<double>(E(springIdx, i) * 3 + ii, E(springIdx, j) * 3 + jj, K_spring(i * 3 + ii, j * 3 + jj)));
					}
				}
			}
        }


    }
    K.setFromTriplets(tripletList.begin(), tripletList.end());

    //typedef Eigen::Triplet<double> T;
    //std::vector<T> tripletList;
    //tripletList.reserve(q.rows() * q.rows());
    //K.resize(q.rows(), q.rows());
    //K.setZero();
    ////std::cout << "ASSEMBLE_STIFFNESS::DEBUG::K.rows():" << K.rows() << std::endl;
    //for (int y = 0; y < E.rows(); y++)
    //{
    //    Eigen::Matrix66d H;
    //    int i = E(y, 0);
    //    int j = E(y, 1);
    //    Eigen::Vector3d q0, q1;
    //    //这里不能使用q()来表示q的值
    //    q0 << q(i * 3), q(i * 3 + 1), q(i * 3 + 2);
    //    q1 << q(j * 3), q(j * 3 + 1), q(j * 3 + 2);
    //    d2V_spring_particle_particle_dq2(H, q0, q1, l0(y), k);
    //    for (int m = 0; m < 3; m++) {
    //        for (int n = 0; n < 3; n++) {
    //            tripletList.push_back(T(3 * i + m, 3 * i + n, H(m, n)));
    //            tripletList.push_back(T(3 * i + m, 3 * j + n, H(m, n + 3.0)));
    //            tripletList.push_back(T(3 * j + m, 3 * i + n, H(m + 3.0, n)));
    //            tripletList.push_back(T(3 * j + m, 3 * j + n, H(m + 3.0, n + 3.0)));
    //        }
    //    }
    //}
    //K.setFromTriplets(tripletList.begin(), tripletList.end());
};