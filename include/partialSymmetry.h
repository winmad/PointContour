#ifndef PARTIAL_SYMMETRY_H
#define PARTIAL_SYMMETRY_H

#include "smallUtils.h"
#include "PointKDTree.h"
#include "TimeManager.h"

class PointCloudUtils;

class PartialSymmetry
{
public:
    PartialSymmetry();
    void init(PointCloudUtils *_pcUtils);

    PointCloudUtils *pcUtils;
};

#endif