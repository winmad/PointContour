#ifndef CURVE_NET_H
#define CURVE_NET_H

#include "smallUtils.h"
#include "splineUtils.h"
#include "disjointSet.h"
#include "adjMatrix.h"
#include "plane.h"
#include "curveMappingTool.h"
#include "constraints.h"

struct PolyLineIndex
{
    PolyLineIndex() {}
    PolyLineIndex(int _ni0 , int _ei0 , int _ni1 , int _ei1)
    {
        ni[0] = _ni0; ei[0] = _ei0;
        ni[1] = _ni1; ei[1] = _ei1;
    }
    int ni[2] , ei[2];
};

class CurveEdge
{
public:
    CurveEdge() {}
    CurveEdge(int _link , int _pli) :
        link(_link) , pli(_pli) {}
    int link;
    int pli;
};

class CurveNet
{
public:
    CurveNet();
    ~CurveNet();
    void clear();
    void copyFrom(const CurveNet& net);
    void startPath(const vec3d& st);
    void extendPath(const vec3d& st , const vec3d& ed , const Path& path ,
        bool newNode , const BSpline& bsp , const Path& originPath ,
        bool addConstraints);
    void breakPath(const int& breakLine , const int& breakPoint ,
        bool addConstraints);
    void updatePath(const int& bspIndex , const int& nodeIndex ,
        const vec3d& newPos , bool addConstraints);
    void storeAutoPath(Path& path , BSpline& bsp , Path& originPath ,
        double offset , bool addConstraints);

    bool reflSymPath(const int& bspIndex , const Plane& plane ,
        Path& originPath , Path& path , BSpline& bsp);
    bool transformPath(const int& bspIndex , Eigen::Matrix4f& transMat ,
        Path& originPath , Path& path , BSpline& bsp);

    void calcDispCyclePoints(const Cycle& cycle ,
		std::vector<Path>& cyclePts , vec3d& cycleCenter);
	void addCycle(const Cycle& cycle , const std::vector<Path>& cyclePts , 
		const vec3d& cycleCenter);
	void addCycleGroup(const std::vector<Cycle>& _cycles , 
		const std::vector<std::vector<Path> >& _cyclePts ,
		const std::vector<vec3d>& _cycleCenters);

    void deleteNode(const int& deleteNodeIndex);
    void deletePath(const int& deleteLineIndex);
    void deleteCycle(const int& deleteCycleIndex);
	void deleteCycleGroup(const int& deleteGroupIndex);

    void cycle2boundary(Cycle& cycle , std::vector<std::vector<vec3d> >& inCurves);
    void cycle2boundary(CycleGroup& cycleGroup , std::vector<std::vector<vec3d> >& inCurves);
    
    int getNodeIndex(const vec3d& pos);
    int getNodeIndex(vec3d& pos , double offset);
    bool linkNoEdges(const int& ni);

    void addCurveType(int bspIndex);
    void updateConstraints(int bspIndex);
    void refreshAllConstraints();

    void mapOrigin2polyLines(int bspIndex);

    void test();
    void debugPrint();
    void debugLog();
    void outputPolyLines();
    void saveCurveNet(const char* fileName);
    void loadCurveNet(const char* fileName);
    
    int numNodes , numPolyLines;
    std::vector<vec3d> nodes;
    std::vector<bool> nodesStat;
    std::vector<std::vector<CurveEdge> > edges;
    std::vector<Path> originPolyLines;
    std::vector<Path> polyLines;
	std::vector<std::vector<int> > mapOrigin;
    std::vector<BSpline> bsplines;
    std::vector<PolyLineIndex> polyLinesIndex;

	// 1: line, 2: nothing, 3: coplanar
	std::vector<int> curveType;
    // meaningful if curveType = 3
    std::vector<Plane> planes;

	ConstraintSet *conSet;

	std::vector<Plane> coplanes;

    std::vector<CycleGroup> cycles;
    std::vector<std::vector<vec3d> > cycleCenters;
    std::vector<std::vector<std::vector<Path> > > cyclePoints;

	std::vector<std::vector<std::vector<vec3d> > > meshes;
	std::vector<std::vector<std::vector<vec3d> > > meshNormals;

    std::vector<CurveMapping> curveMaps;
	std::vector<std::vector<std::pair<int, int> > > mapLines;
};

#endif