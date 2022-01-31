/*
 * ICP header file
 * Implements Iterative closest point algorithm
 * sources:
 * -https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf
 * -https://en.wikipedia.org/wiki/Iterative_closest_point
 *
 * Creator: Joe Ferreira Scholtz
 * Date: 16/01/2022
 *
 */
#pragma once // include guard
#include <iostream> //std::cout etc
#include <math.h> //math stuff
#include <vector> //vector
#include <memory> //smart pointers
#include <eigen3/Eigen/Dense> //Eigen (Linear Algebra Library)
#include <pcl/point_types.h> //PCL: Point Cloud Library
#include <pcl/features/normal_3d.h>
#ifdef WITH_OPENMP //if this computer have OMP installed use multi-thread functions
    #include <pcl/features/normal_3d_omp.h>
#endif

/*
 * Current implementation supports:
 * - PointTPlane ICP
 */

//#define DEBUG
//#define DEBUGOUT
#define ITEROUT
//aliases
using Eigen::Matrix, Eigen::MatrixXd, Eigen::VectorXd, Eigen::Dynamic, Eigen::MatrixBase, Eigen::EigenBase;
template<typename Derived>
using MatrixXT = Matrix<Derived,Dynamic,Dynamic>;

//dot product row-wise between matrixes
template<typename Derived>
VectorXd dot(const EigenBase<Derived> &sA, const EigenBase<Derived> &sB){
    // cast EigenBase (it's like "a base class to every other Eigen class") to Matrix so s1.row(i) will be a vector
    MatrixXd s1 = sA;
    MatrixXd s2 = sB;
    //get min size
    int setSize{(int)std::min(s1.rows(),s2.rows())};
    // initialize return variable
    VectorXd sr(setSize);
    //dot product row-wise
    for(int i=0;i<setSize;i++){
        sr(i) = s1.row(i).dot(s2.row(i));
    }
    return sr;
}

//dot product column-wise between matrixes
template<typename Derived>
Matrix<double,Dynamic,3> cross(const EigenBase<Derived> &sA,const EigenBase<Derived> &sB){
    // cast EigenBase (it's like "a base class to every other Eigen class") to Matrix so s1.row(i) will be a vector
    MatrixXd s1 = sA;
    MatrixXd s2 = sB;
    //get min size
    int setSize{(int)std::min(s1.rows(),s2.rows())};
    // initialize return variable, it has to be a Nx3 or 3xN matrix because crosse product is only defined for 3x1 or 1x3 vectors
    Matrix<double,Dynamic,3> mr(setSize,3);
    //dot product row-wise
    for(int i=0;i<setSize;i++){
        Matrix<double,1,3> s1i = s1.row(i), s2i = s2.row(i);
        auto cr{s1i.cross(s2i)};
        for(int it=0;it<3;it++){
            mr(i,it) = cr(it);
        }
    }
    return mr;
}

namespace ICP {
    enum class ComputeAlgorith {Standard = 0, PointTPlane = 1, Generalized = 2}; //compute algorithms
    //defaults
    constexpr int maxIterDefault = 10;
    constexpr double dMaxDefault = 0.5;
    class Estimator{

    public:
        //Constructor
        template<typename Derived>
        // EigenBase (it's like "a base class to every other Eigen class")
        Estimator(EigenBase<Derived> &firstSet, EigenBase<Derived> &secondSet, Matrix<double,4,4> initTrans, double dMax = dMaxDefault, int maxIterations = maxIterDefault, ComputeAlgorith computeAlgorith = ComputeAlgorith::PointTPlane)
            : _Trans(initTrans), _dMax(dMax), _maxIterations(maxIterations), _computeAlgorith(computeAlgorith){
            _firstSet = firstSet;
            _secondSet = secondSet;
            //reserve memory (so it doenst have to allocate each time _weights.push_back(...) is called
            _weights.reserve(_size);
        }
        // compute ICP function
        MatrixXd computeModel(){
            switch(_computeAlgorith){
            case ComputeAlgorith::Standard : {
#ifdef DEBUG
                std::cout<<"Standard\n";
#endif
               break;
            }

            case ComputeAlgorith::PointTPlane : {
#ifdef DEBUG
                std::cout<<"PointTPlane\n";
#endif
                computeNormals(_secondSet);
               break;
            }

            case ComputeAlgorith::Generalized : {
#ifdef DEBUG
                std::cout<<"Generalized\n";
#endif
               break;
            }

            }
            // TODO: Robustly reject outliers
            // TODO: Picky ICP: throw out duplicated, so _firstSet and _secondSet have a one-to-one corresponence findClosestPoint-wise
            //run algorithm _maxIterations times
            for(int iter = 0;iter<_maxIterations;iter++){
#ifdef ITEROUT
                std::cout << "Iteration: " << iter << std::endl;
#endif
                _weights.clear();
                //iterate through _secondSet
                Matrix<double,3,1> bPoint;
                for(int it = 0, itMax = (int)_secondSet.rows(); it<itMax; it++){
                    bPoint = _secondSet.row(it);
                    //find closest point in the first set to this point in the second
                    auto m = findClosestPoint(_firstSet, bPoint);

                    //transform to homogeneous coordinates
                    Matrix<double,4,1> bPointH;
                    bPointH << bPoint, 1;
                    //compute bPoint transformed by _Trans
                    bPointH = _Trans*bPointH;
                    //back to cartesian coordinates
                    Matrix<double,3,1> bTransformed;
                    bTransformed << bPointH(0), bPointH(1), bPointH(2);
                    //compute diference between bPoint Transformed by _Trans and it's closest point in _firstSet
                    auto n = m - bTransformed;

                    //since we don't necessarily have an exact match between the sets, we use a maximum distance
                    if(n.norm() <= _dMax){
                        _weights.push_back(it);
                    }
                }
                //compute transformation using a mimizing function
                _Trans = minimizeFunction(_weights);

            }
            return _Trans;
        }
        template<typename Derived>
        void computeNormals(const EigenBase<Derived> &setS){
            // cast EigenBase (it's like "a base class to every other Eigen class") to Matrix so s1.row(i) will be a vector
            MatrixXd set = setS;
            int setSize{(int)set.rows()};
            _sNormals.resize(0,0); //clear?
            _sNormals.resize(setSize,3); //resize
            /* Compute normals directly from point cloud using PCL
             * src: https://pointclouds.org/documentation/tutorials/normal_estimation.html
             *
             * compute() consists in:
             * for each point p in cloud P
             *     get the nearest neighbors of p
             *     compute the surface normal n of p
             *     check if n is consistently oriented towards the viewpoint and flip otherwise
             * end
             *
             * The viewpoint is by default (0,0,0) and can be changed with:
             * setViewPoint (float vpx, float vpy, float vpz);
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->resize(setSize);

            // create a point cloud
            Matrix<double,3,1> bPoint;
            for(int it = 0, itMax = setSize; it<itMax; it++){
                bPoint = set.row(it);
                (*cloud)[it] = {(float)bPoint[0],(float)bPoint[1],(float)bPoint[2]};
            }
            // Create the normal estimation class, and pass the input dataset to it
#ifdef WITH_OPENMP //if this computer have OMP installed use multi-thread function
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
#else // use single-thread otherwise
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
#endif
            ne.setInputCloud(cloud);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
            ne.setSearchMethod(tree);

            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

            // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch(_neighborsRadius);

            // Compute the features
            ne.compute(*cloud_normals); // cloud_normals->size() should have the same size as the input cloud->size ()*
            /* get Eigen::Matrix from pointCloud
             * Actually, cloud_normals store a point cloud of plc::Normal
             * that has a size 3 vector containing the the normal vector
             * 0.0
             * curvature
             * 0.0, 0.0, 0.0
             * and we want just the normal vector
             */
            Eigen::MatrixXf M0 = cloud_normals->getMatrixXfMap(), M(M0.cols(),3);
            for(int itx = 0, itxMax= 3; itx<itxMax; itx++){
                for(int ity = 0, ityMax= M.rows(); ity<ityMax; ity++){
                    M(ity,itx) = M0(itx,ity);
                }
            }

            //_sNormals = MatrixXd::Ones(set.rows(),set.cols());
            // cast back to double and store it in _sNormals
            _sNormals = M.cast<double>();
        }
        MatrixXd getCurrentModel(){
            return _Trans;
        }
        //get homogeneous transformation matrix from translation and rotation.
        //it's static so it's not necessary to instanciate an Estimator to use
        static Matrix<double,4,4> getTransform(Matrix<double,6,1> &transVect){
            Matrix<double,4,4> transMat;
            auto roll{transVect(0)}, pitch{transVect(1)}, yaw{transVect(2)};

            //populate rotation matrix
            transMat(0,0) = cos(yaw)*cos(pitch);
            transMat(0,1) = cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
            transMat(0,2) = cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);

            transMat(1,0) = sin(yaw)*cos(pitch);
            transMat(1,1) = sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
            transMat(1,2) = sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);

            transMat(2,0) = -sin(pitch);
            transMat(2,1) = cos(pitch)*sin(roll);
            transMat(2,2) = cos(pitch)*cos(roll);

            //populate translation vector
            transMat(0,3) = transVect(3);
            transMat(1,3) = transVect(4);
            transMat(2,3) = transVect(5);

            //homogeneous coordinates stuff
            transMat(3,0) = 0;
            transMat(3,1) = 0;
            transMat(3,2) = 0;
            transMat(3,3) = 1;

            return transMat;
        }
        //set _neighborsRadius for computeNormals();
        void setNeighborsRadius(double &r){
            _neighborsRadius = r;
        }
    private:
        //mimize function
        MatrixXd minimizeFunction(const std::vector<int> &wi){
            Matrix<double,4,4> Trans;
            filterSets(wi);
#ifdef DEBUGOUT
            static int i{0};
            i++;
            std::cout << "minimizeFunction " << i << "\n";
#endif
            switch(_computeAlgorith) {
            case ComputeAlgorith::Standard : {
#ifdef DEBUG
                std::cout<<"Standard\n";
#endif
                break;
            }

            case ComputeAlgorith::PointTPlane : {
#ifdef DEBUG
                std::cout<<"PointTPlane\n";
#endif
                /*
                 * Minimize the point to plane metric according to
                 * Kok Lim Low : Linear Least Squares Optimization for Point-to-Plane
                 * Source: https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
                 *
                 * Uses a linear aproximation of transform rotation euler angles been approximately zero
                 * so the minimization becomes a LMS: argmin|Ax-b|²
                 *
                 */
                auto b{dot(_secondSetFiltered, _sNormalsFiltered)};
                auto A1{cross((MatrixXd)(_firstSetFilterded-_secondSetFiltered), _sNormalsFiltered)};
                MatrixXd A(A1.rows(), A1.cols()+_sNormalsFiltered.cols());
                A << A1, _sNormalsFiltered;
                //QR decomposition to solve LMS system
                //Eigen::HouseholderQR<Eigen::Ref<Eigen::MatrixXd> > qr(A);
                //Matrix<double,6,1> x{qr.solve(b)};
                Matrix<double,6,1> x;
                x = A.householderQr().solve(b);
                MatrixXd e(b.rows(),b.cols());
                e = A*x;
                //get homogeneous transformation matrix from translation and rotation
                Trans = getTransform(x);
                break;
            }

            case ComputeAlgorith::Generalized : {
#ifdef DEBUG
                std::cout<<"Generalized\n";
#endif
                break;
            }

            }
            return Trans;
        }
        // brute-force closest point other in searchSet, could be done better with a kd-tree or somthing like that
        Matrix<double,3,1> findClosestPoint(MatrixXd &searchSet, const Matrix<double,3,1> &other){
            double minDist = -1;
            Matrix<double,3,1> minDistPoint, aPoint;
            for(int it = 0, itMax = (int)searchSet.rows(); it<itMax; it++){
                aPoint = searchSet.row(it);
                if(minDist<0){
                    minDist = (other-aPoint).norm();
                    minDistPoint = aPoint.transpose();
                }
                else if((other-aPoint).norm()<minDist){
                    minDist = (other-aPoint).norm();
                    minDistPoint = aPoint.transpose();
                }
            }
            return minDistPoint;
        }
        void filterSets(const std::vector<int> &wi){
            int rows{(int)wi.size()};
            _firstSetFilterded.resize(rows,3);
            _secondSetFiltered.resize(rows,3);
            _sNormalsFiltered.resize(rows,3);

            int itx = 0;
            for(const auto index : wi){
                for(int ity = 0;ity<3;ity++){
                    _firstSetFilterded(itx,ity) = _firstSet(index,ity);
                    _secondSetFiltered(itx,ity) = _secondSet(index,ity);
                    _sNormalsFiltered(itx,ity) = _sNormals(index,ity);
                }
                itx++;
            }
        }
    private:
        int _size;
        //Sets to compute transfomation
        MatrixXd _firstSet;
        MatrixXd _firstSetFilterded; //todo? reimplemented it so it has a one-to-one corresponence findClosestPoint-wise with _secondSetFiltered
        MatrixXd _secondSet;
        MatrixXd _secondSetFiltered; //todo? reimplemented it so it has a one-to-one corresponence findClosestPoint-wise with _firstSetFilterded
        MatrixXd _sNormals; // _secondSet normals
        MatrixXd _sNormalsFiltered;
        double _neighborsRadius = 0.5; //neighbors radius to find normals
        Matrix<double,4,4> _Trans; //Transformation between the two sets
        double _dMax;
        std::vector<int> _weights;
        int _maxIterations;
        ComputeAlgorith _computeAlgorith;
    };
    //return null transformation matrix in homogeneous coordinates
    Matrix<double,4,4> nullTransform(){
        Matrix<double,6,1> nullVect = MatrixXd::Zero(6,1);
        return Estimator::getTransform(nullVect);
    }

}
