#ifndef _UTILS_COMMON_UTILS_H_
#define _UTILS_COMMON_UTILS_H_

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

namespace kamaz {
  namespace hagen{
class CommonUtils {
 public:
  
  CommonUtils() = default;
  ~CommonUtils() = default;
  void generate_samples_from_ellipsoid(Eigen::MatrixXd covmat
            , Eigen::Matrix3d rotation_mat, Eigen::Vector3d cent
            , Eigen::MatrixXd& container);
  
  void get_roration_matrix(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Matrix3d& r);
  
};

}  
}
#endif  // _UTILS_COMMON_UTILS_H_
