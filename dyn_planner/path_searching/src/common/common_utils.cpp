#include "common_utils.h"

namespace kamaz {
namespace hagen{


  void CommonUtils::get_roration_matrix(Eigen::Vector3d a
      , Eigen::Vector3d b, Eigen::Matrix3d& r){
        a = a/a.norm();
        double b_norm = b.norm();
        b = b/b_norm;
        Eigen::Vector3d v = a.cross(b);
        double s = v.norm();
        double c = a.dot(b);
        Eigen::Matrix3d vx;
        vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        r = Eigen::Matrix3d::Identity(3,3);
        if(s != 0 ){
            r = r + vx + vx*vx*((1-c)/std::pow(s, 2));
        }
  }

  void CommonUtils::generate_samples_from_ellipsoid(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat, 
            Eigen::Vector3d cent, Eigen::MatrixXd& container){

        int ndims = container.cols();
        int npts = container.rows();
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
        eigensolver.compute(covmat);
        Eigen::Vector3d eigen_values = eigensolver.eigenvalues().real();
        Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
        std::vector<std::tuple<double, Eigen::Vector3d>> eigen_vectors_and_values; 

        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<double, Eigen::Vector3d> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(), 
            [&](const std::tuple<double, Eigen::Vector3d>& a, const std::tuple<double, Eigen::Vector3d>& b) -> bool{ 
                return std::get<0>(a) <= std::get<0>(b); 
        });
        int index = 0;
        for(auto const vect : eigen_vectors_and_values){
            eigen_values(index) = std::get<0>(vect);
            eigen_vectors.row(index) = std::get<1>(vect);
            index++;
        }

        Eigen::MatrixXd eigen_values_as_matrix = eigen_values.asDiagonal();

        std::random_device rd{};
        std::mt19937 gen{rd()};  
        std::uniform_real_distribution<double> dis(0, 1);
        std::normal_distribution<double> normal_dis{0.0f, 1.0f};
 
        Eigen::MatrixXd pt = Eigen::MatrixXd::Zero(npts, ndims).unaryExpr([&](double dummy){return (double)normal_dis(gen);});
        Eigen::Vector3d rs = Eigen::Vector3d::Zero(npts).unaryExpr([&](double dummy){return dis(gen);});
        Eigen::Vector3d fac = pt.array().pow(2).rowwise().sum();
        Eigen::Vector3d fac_sqrt = fac.array().sqrt();
        Eigen::Vector3d rs_pow = rs.array().pow(1.0/ndims);
        fac = rs_pow.array()/fac_sqrt.array();
        Eigen::Vector3d d = eigen_values_as_matrix.diagonal().array().sqrt();
        for(auto i(0); i<npts; i++){
            container.row(i) = fac(i)*pt.row(i).array();
            Eigen::MatrixXd  fff = (container.row(i).array()*d.transpose().array());
            Eigen::Vector3d bn = rotation_mat*fff.transpose();
            container.row(i) = bn.array() + cent.head(3).array();
        }
        // std::cout << "points: " << container << std::endl;
    }

//     /**
//  * @brief smooth_nonuniform
//  * Implements the method described in  https://dsp.stackexchange.com/questions/1676/savitzky-golay-smoothing-filter-for-not-equally-spaced-data
//  * free to use at the user's risk
//  * @param n the half size of the smoothing sample, e.g. n=2 for smoothing over 5 points
//  * @param the degree of the local polynomial fit, e.g. deg=2 for a parabolic fit
//  */
// bool CommonUtils::smooth_nonuniform(int deg, int n, std::vector<double>const &x, std::vector<double> const &y, std::vector<double>&ysm)
// {
//     if(x.size()!=y.size()) return false; // don't even try
//     if(x.size()<=2*n)      return false; // not enough data to start the smoothing process
// //    if(2*n+1<=deg+1)       return false; // need at least deg+1 points to make the polynomial

//     int m = 2*n+1; // the size of the filter window
//     int o = deg+1; // the smoothing order

//     std::vector<double> A(m*o);        
//      memset(A.data(),   0, m*o*sizeof(double));
//     std::vector<double> tA(m*o);        
//     memset(tA.data(),  0, m*o*sizeof(double));
//     std::vector<double> tAA(o*o);       
//     memset(tAA.data(), 0, o*o*sizeof(double));

//     std::vector<double> t(m);          
//      memset(t.data(),   0, m*  sizeof(double));
//     std::vector<double> c(o);           
//     memset(c.data(),   0, o*  sizeof(double));

//     // do not smooth start and end data
//     int sz = y.size();
//     ysm.resize(sz);           memset(ysm.data(), 0,sz*sizeof(double));
//     for(uint i=0; i<n; i++)
//     {
//         ysm[i]=y[i];
//         ysm[sz-i-1] = y[sz-i-1];
//     }

//     // start smoothing
//     for(uint i=n; i<x.size()-n; i++)
//     {
//         // make A and tA
//         for(int j=0; j<m; j++)
//         {
//             t[j] = x[i+j-n] - x[i];
//         }
//         for(int j=0; j<m; j++)
//         {
//             double r = 1.0;
//             for(int k=0; k<o; k++)
//             {
//                 A[j*o+k] = r;
//                 tA[k*m+j] = r;
//                 r *= t[j];
//             }
//         }

//         // make tA.A
//         matMult(tA.data(), A.data(), tAA.data(), o, m, o);

//         // make (tA.A)-¹ in place
//         if (o==3)
//         {
//             if(!invert33(tAA.data())) return false;
//         }
//         else if(o==4)
//         {
//             if(!invert44(tAA.data())) return false;
//         }
//         else
//         {
//             if(!inverseMatrixLapack(o, tAA.data())) return false;
//         }

//         // make (tA.A)-¹.tA
//         matMult(tAA.data(), tA.data(), A.data(), o, o, m); // re-uses memory allocated for matrix A

//         // compute the polynomial's value at the center of the sample
//         ysm[i] = 0.0;
//         for(int j=0; j<m; j++)
//         {
//             ysm[i] += A[j]*y[i+j-n];
//         }
//     }

//     std::cout << "      x       y       y_smoothed" << std::endl;
//     for(uint i=0; i<x.size(); i++) std::cout << "   " << x[i] << "   " << y[i]  << "   "<< ysm[i] << std::endl;

//     return true'
//    }
}
}