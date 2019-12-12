#ifndef _UTILS_DYNAMICS_H_
#define _UTILS_DYNAMICS_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include <cnpy.h>
#include <random>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace kamaz {
  namespace hagen{
    class Dynamics {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Dynamics();
        // Dynamics() = default;
        ~Dynamics() = default;
        Eigen::Matrix3d skewSymmetric(Eigen::Vector3d vector);
        Eigen::MatrixXd g(const Eigen::MatrixXd x, const Eigen::MatrixXd u);
        Eigen::MatrixXd f(const Eigen::MatrixXd x, const Eigen::MatrixXd u);
        Eigen::MatrixXd gBar(const Eigen::MatrixXd x, const Eigen::MatrixXd u, double dt);
        Eigen::MatrixXd jacobian1( Eigen::MatrixXd& a,  Eigen::MatrixXd& b);
        Eigen::MatrixXd jacobian2(Eigen::MatrixXd& a, Eigen::MatrixXd& b);
        double extendedLQR(Eigen::MatrixXd startState
        , Eigen::MatrixXd uNominal, std::vector<Eigen::MatrixXd>& L
        , std::vector<Eigen::MatrixXd>& l, std::vector<Eigen::MatrixXd>& xHatSeq, Eigen::Vector3d goal);
        void quadratizeCost(Eigen::MatrixXd& x, Eigen::MatrixXd& u, const size_t& t
        , Eigen::MatrixXd& Pt, Eigen::MatrixXd& Qt, Eigen::MatrixXd& Rt, Eigen::MatrixXd& qt
        , Eigen::MatrixXd& rt, const size_t& iter);
        void init(int max_ittr, double _dt);
        size_t ell;
        Eigen::MatrixXd Q;
        double rotCost;
        Eigen::MatrixXd xStart, uNominal;
        Eigen::MatrixXd R;
        
        double obstacleFactor;
        double scaleFactor;

        // Matrix<DIM> bottomLeft;
        // Matrix<DIM> topRight;
        double robotRadius;
        double dt;

        double gravity;         // gravity,   m/s^2
        // Matrix<3> eX, eY, eZ;   // unit vectors

        // quadrotor constants
        double mass;            // mass, kg
        Eigen::Matrix3d inertia;    // moment of inertia matrix 
        double momentConst;     // ratio between force and moment of rotor
        double dragConst;       // ratio between speed and opposite drag force
        double length;          // distance between center and rotor, m
        double minForce;        // minimum force of rotor, N
        double maxForce;        // maximum force of rotor, N
        Eigen::Vector3d eX;
        Eigen::Vector3d eY;
        Eigen::Vector3d eZ;
        double timeFactor;

        // derivative constants
        Eigen::MatrixXd invInertia; // inverse of intertia matrix
        
        double jStep;
        int xDim;
        int uDim;
        int sDim;
      };
  }  
}
#endif  // _UTILS_DYNAMICS_H_
