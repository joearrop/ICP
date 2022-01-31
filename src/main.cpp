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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(){
    std::cout << "Starting...\n";
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

    ICP::Estimator estimator(matA,matB,ICP::nullTransform());
    std::cout << "Computing model...\n";
    auto Tf = estimator.computeModel();
    std::cout << "Tf:\n" << Tf << "\n";

    /*

    std::string fileName;
    std::cout << "Enter point cloud name:\n";
    std::cin >> fileName;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/"+fileName+".pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    */


    return 0;
}
