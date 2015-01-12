#ifndef PARTIAL_SYMMETRY_H
#define PARTIAL_SYMMETRY_H

#include "smallUtils.h"
#include "plane.h"
#include "PointKDTree.h"
#include "TimeManager.h"
#include "plane.h"

struct Data;
class PointCloudUtils;

class PartialSymmetry
{
public:
    PartialSymmetry();
    void init(PointCloudUtils *_pcUtils);
    void samplePointsUniform(int numSamples);
	void findSymmPlanes();
    void calcVotes();

    PointCloudUtils *pcUtils;
	vector<Plane> symmPlanes;
    std::vector<bool> isSampled;

    std::vector<Data> signData;
    // use Data, CloesPoint, and KnnQuery in pointCloudUtils.h
    // pos: position in signature space, n: position in original space
    PointKDTree<Data> *signSpaceTree;

    std::vector<Plane> votes;
    std::vector<Plane> symPlanes;
};

#endif