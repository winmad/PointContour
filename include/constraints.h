#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "smallUtils.h"
#include "splineUtils.h"
#include "disjointSet.h"
#include "adjMatrix.h"
#include "Plane.h"
#include "curveMappingTool.h"

class CurveNet;

struct SelfSymmIdx
{
	SelfSymmIdx(int _n, int _n1, int _n2): n(_n), n1(_n1), n2(_n2) {}
	int n, n1, n2;
};

class ConstraintDetector
{
public:
    static bool collinearTest(Path& path , BSpline& bsp);
	static bool coplanarTest(BSpline& bsp , Plane& plane);
    static bool checkCollinear(const vec3d& x1 , const vec3d& y1 ,
        const vec3d& x2 , const vec3d& y2 , const double& threshold);
    static bool checkParallel(const vec3d& x1 , const vec3d& y1 ,
        const vec3d& x2 , const vec3d& y2 , const double& threshold);
    static bool checkCoplanar(BSpline& bsp1 , Plane& plane1 ,
        BSpline& bsp2 , Plane& plane2 , const double& threshold);
    //static bool checkCoplanar(const BSpline& bsp1 , const BSpline& bsp2 ,
    //    const double& threshold);
    //static bool checkCoplanar(const vec3d& x1 , const vec3d& y1 ,
    //    const vec3d& x2 , const vec3d& y2 , const double& threshold);
    static bool checkOrtho(const vec3d& x0 , const vec3d& x1 ,
        const vec3d& x2 , const double& threshold);
    static bool checkTangent(const vec3d& x0 , const vec3d& x1 ,
        const vec3d& x2 , const double& threshold);
	static bool checkSymmetry(const vec3d& x, const vec3d& nx,
		const vec3d& y, const vec3d& ny, const double& threshold);
    /*
    static const double collinearThr = 0.05;
    static const double coplanarThr = 0.1;
    static const double parallelThr = 0.05;
    static const double orthoThr = 0.1;
    static const double tangentThr = 0.05;
	static const double symmetryThr = 0.1;
	static const double ratioThr = 0.05;
	static const double planeDiffThr = 0.1;
    */
    static const double collinearThr;
    static const double coplanarThr;
    static const double parallelThr;
    static const double orthoThr;
    static const double tangentThr;
    static const double symmetryThr;
    static const double ratioThr;
    static const double planeDiffThr;
};

class ConstraintSet
{
public:
    ConstraintSet(CurveNet* curveNet);
    void clear();
    void copyFrom(ConstraintSet* conSet);
    
	bool checkCycleSpline(int i);

    void addCollinearConstraint(int bspIndex);
    void addParallelConstraint(int bspIndex);
    void addCoplanarConstraint(int bspIndex);
    void addJunctionConstraint(int bspIndex);
    void addSymmetryConstraint(int bspIndex, bool add = true);
	int addSymmetryPlane(Plane &p, bool add, int a = -1, int b = -1);
	int addSelfSymmPlane(Plane &p, bool add, int l, int a, int b);
	void addSelfSymmetryConstraint(int bspIndex);
	void addTransformConstraint(int bspIndex);

public:
    CurveNet *net;

	std::vector<Plane> symmetricPlanes;
	std::vector<std::vector<std::pair<int, int> > > symmLines;
	std::vector<std::vector<SelfSymmIdx> > symmPoints;
	
    DisjointSet collinearSet;
    DisjointSet parallelSet;
    AdjMatrix coplanarSet;
    AdjMatrix orthoSet;
};

#endif