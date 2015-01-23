#ifndef PARTIAL_SYMMETRY_H
#define PARTIAL_SYMMETRY_H

#include "smallUtils.h"
#include "plane.h"
#include "PointKDTree.h"
#include "TimeManager.h"
#include "plane.h"

struct Data;
class PointCloudUtils;

const int binNumTheta = 50;
const int binNumPhi = 50;
const int binNumR = 50;


class PartialSymmetry
{
public:
    PartialSymmetry();
    void init(PointCloudUtils *_pcUtils);
    void samplePointsUniform(int numSamples);
	void findSymmPlanes();
	Plane adjustSymmPlane(int i, int j, int k);
    void calcVotes();

	typedef struct {
		double a, b;
	} my_constraint_data;

	static double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
	static double myconstraint(unsigned n, const double *x, double *grad, void *data);


    PointCloudUtils *pcUtils;
	vector<Plane> symmPlanes;
    std::vector<bool> isSampled;

    std::vector<Data> signData;
    // use Data, CloesPoint, and KnnQuery in pointCloudUtils.h
    // pos: position in signature space, n: position in original space
    PointKDTree<Data> *signSpaceTree;

    std::vector<Plane> votes;
    std::vector<Plane> symPlanes;

	double dcostheta, dphi, dr, maxR;

    double weights[binNumTheta][binNumPhi][binNumR];
	std::vector<std::pair<int, int> > symmPoints[binNumTheta][binNumPhi][binNumR];
	std::vector<Plane> candidatePlanes;
};

#endif