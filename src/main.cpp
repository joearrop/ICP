/*
 * ICP example usage code
 * Creator: Joe Ferreira Scholtz
 * Date: 16/01/2022
 *
 */

#include <math.h>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include "icp.hpp"

int main(){
    std::cout << "Starting...\n";
    /*
    Eigen::MatrixXd matA(8,6);
    matA << Eigen::MatrixXd::Identity(8,6);
    Eigen::VectorXd vA(8,1);
    vA << Eigen::MatrixXd::Ones(8,1);
    Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd> > qr(matA);
    std::cout << "qr.solve(vA): " << qr.solve(vA) << "\n";

    Eigen::Matrix<double,3,1> vB;
    vB << Eigen::MatrixXd::Ones(3,1);

    auto a{vB.cross(vB)};
    std::cout << "vB.cross(vB): " << a.transpose();
    Eigen::Matrix<double,Eigen::Dynamic,3> mr(2,3);
    for(int it=0,itMax=a.size();it<itMax;it++){
        mr(1,it) = a[it];
    }
    std::cout << "\nmr: "<< mr;
    */
    Eigen::MatrixXd matA(1000,3);
    matA << Eigen::MatrixXd::Random(1000,3);
    for(int it=0,itMax=(int)matA.rows();it<itMax;it++){
        matA.row(it).normalize();
    }

    Eigen::MatrixXd matB(1000,3);
    matB << matA;
    for(int it=0,itMax=(int)matB.rows();it<itMax;it++){
        matB.row(it) += 0.2*Eigen::Matrix<double,1,3>::Ones();
    }

//    auto a{dot(matA,matB)};
//    auto b{dot(matA,matB)};
//    std::cout<<"a: " << a<<"\n";
//    std::cout<<"b: " << b<<"\n";

    ICP::Estimator estimator(matA,matB,ICP::nullTransform());
    std::cout << "Computing model...\n";
    auto Tf = estimator.computeModel();
    std::cout << "Tf:\n" << Tf << "\n";
    return 0;
}
