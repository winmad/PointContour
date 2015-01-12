#ifndef PARTIAL_SYMMETRY_H
#define PARTIAL_SYMMETRY_H

#include "smallUtils.h"
#include "PointKDTree.h"
#include "TimeManager.h"
#include "plane.h"

class PointCloudUtils;

class PartialSymmetry
{
public:
    PartialSymmetry();
    void init(PointCloudUtils *_pcUtils);
	void findSymmPlanes();

    PointCloudUtils *pcUtils;
	vector<Plane> symmPlanes;
};

#endif