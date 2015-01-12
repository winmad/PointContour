#include "pclUtils.h"

void path2PointCloud(Path& points , PointCloudT::Ptr& cloud)
{
    cloud->width = points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);
    for (int i = 0; i < points.size(); i++)
    {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
    }
}

void pointCloud2Path(PointCloudT::Ptr& cloud , Path& points)
{
    points.clear();
    for (int i = 0; i < cloud->points.size(); i++)
    {
        vec3d pos;
        pos.x = cloud->points[i].x;
        pos.y = cloud->points[i].y;
        pos.z = cloud->points[i].z;
        points.push_back(pos);
    }
}

vec3d transformPoint(Eigen::Matrix4f& transMat , vec3d& op)
{
    Eigen::Vector4f p;
    for (int i = 0; i < 3; i++) p(i) = op[i];
    p(3) = 1.f;
    p = transMat * p;
    return vec3d(p(0) , p(1) , p(2));
}