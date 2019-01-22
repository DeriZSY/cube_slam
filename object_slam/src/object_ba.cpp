#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include <object_slam/Object_landmark.h>
#include <object_slam/Frame.h>
#include <object_slam/g2o_Object.h>

#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"

#include "line_lbd/line_lbd_allclass.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namesapce g2o
{
    // Bounding Box 3D
    class BBox3D
    {
      public:
        SE3Quat pose_;      // 6DoF pose [x, y, z, r, p, y]
        Vector3d half_dim_; // [length/2, width/2, height/2]

        BBox3d()
        {
            this.pose_ = SE3Quat();
            this.half_dim_.setZero();
        }

        BBox3d()
        {
        }

        inline const Vector3d &get_translation() const { return pose_.translation(); }
        inline void set_translation(const Vector3D &t) { pose_.setTranslation(t); }
        inline void set_rotation(const Quaterniond &r) { pose_.setRotation(r); }
        inline void set_rotation(const Matrix3d &r) { pose_.setRotation(Quaterniond(r)); }
    }

    // g2o Vertex - BBox3D
    class VertexCuboid : public BaseVertex<9, BBox3D>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexCuboid(){};

        virtual void setToOriginImpl() { _estimate = BBox3D(); }

        virtual void oplusImpl(const double *update_)
        {
            Eigen::Map<const Vector9d> update(update_);
            setEstimate(_estimate.exp_update(update));
        }
    }

    // g2o BinaryEdge - BBox3D positional error
    class EdgeSE3Cuboid : public BaseBinaryEdge<9, BBox3D, VertexSE3Expmap, VertexCuboid>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSE3Cuboid(){};

        void computeError()
        {
            // TODO
        }
    }
}