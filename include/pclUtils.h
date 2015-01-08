#include "smallUtils.h"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void path2PointCloud(Path& points , PointCloudT::Ptr& cloud);
void pointCloud2Path(PointCloudT::Ptr& cloud , Path& points);

vec3d transformPoint(Eigen::Matrix4f& transMat , vec3d& op);