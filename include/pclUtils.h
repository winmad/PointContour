#include "smallUtils.h"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void path2PointCloud(Path& points , PointCloudT::Ptr& cloud);
void pointCloud2Path(PointCloudT::Ptr& cloud , Path& points);

vec3d transformPoint(Eigen::Matrix4f& transMat , vec3d& op);