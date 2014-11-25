#ifndef CURVE_NET_H
#define CURVE_NET_H

#include "smallUtils.h"
#include "splineUtils.h"
#include "disjointSet.h"
#include "adjMatrix.h"
#include "Plane.h"

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
    void clear();
    void copyFrom(const CurveNet& net);
    void startPath(const vec3d& st);
    void extendPath(const vec3d& st , const vec3d& ed , const Path& path ,
        bool newNode , const BSpline& bsp , const Path& originPath);
    void breakPath(const int& breakLine , const int& breakPoint);
    void deleteNode(const int& deleteNodeIndex);
    void deletePath(const int& deleteLineIndex);
    void deleteCycle(const int& deleteCycleIndex);
    
    int getNodeIndex(const vec3d& pos);
    bool linkNoEdges(const int& ni);

    bool collinearTest(Path& path , BSpline& bsp);
    
    bool checkCollinear(const vec3d& x1 , const vec3d& y1 ,
        const vec3d& x2 , const vec3d& y2 , const double& threshold);
    bool checkParallel(const vec3d& x1 , const vec3d& y1 ,
        const vec3d& x2 , const vec3d& y2 , const double& threshold);
    bool checkCoplanar(const BSpline& bsp , const double& threshold);
    bool checkCoplanar(const BSpline& bsp1 , const BSpline& bsp2 ,
        const double& threshold);
    bool checkCoplanar(const vec3d& x1 , const vec3d& y1 ,
        const vec3d& x2 , const vec3d& y2 , const double& threshold);
    bool checkOrtho(const vec3d& x0 , const vec3d& x1 ,
        const vec3d& x2 , const double& threshold);
    bool checkTangent(const vec3d& x0 , const vec3d& x1 ,
        const vec3d& x2 , const double& threshold);
	bool checkSymmetry(const vec3d& x, const vec3d& nx,
		const vec3d& y, const vec3d& ny, const double& threshold);

    void addCurveType(int bspIndex);
    void addCollinearConstraint(int bspIndex);
    void addParallelConstraint(int bspIndex);
    void addCoplanarConstraint(int bspIndex);
    void addJunctionConstraint(int bspIndex);
    void addSymmetryConstraint(int bspIndex, bool add = true);
	int addSymmetryPlane(Plane &p, bool add, int a = -1, int b = -1);

    void calcDispCyclePoints(const Cycle& cycle ,
        std::vector<Path>& cyclePts , vec3d& cycleCenter);
    void addCycle(const Cycle& cycle);

    void test();
    void debugPrint();
    void debugLog();
    void outputPolyLines();
    
    int numNodes , numPolyLines;
    std::vector<vec3d> nodes;
    std::vector<bool> nodesStat;
    std::vector<std::vector<CurveEdge> > edges;
    std::vector<Path> originPolyLines;
    std::vector<Path> polyLines;
    std::vector<BSpline> bsplines;
    std::vector<PolyLineIndex> polyLinesIndex;

    std::vector<Cycle> cycles;
    std::vector<vec3d> cycleCenters;
    std::vector<std::vector<Path> > cyclePoints;

    // 1: line, 2: nothing, 3: coplanar
    std::vector<int> curveType;

    double collinearThr;
    double coplanarThr;
    double parallelThr;
    double orthoThr;
    double tangentThr;
	double symmetryThr;

	std::vector<Plane> reflectPlanes;
	std::vector<std::vector<std::pair<int, int> > > symmLines;
    
    DisjointSet collinearSet;
    DisjointSet parallelSet;
    AdjMatrix coplanarSet;
    AdjMatrix orthoSet;
};

#endif