#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#include "nvVector.h"
#include "smallUtils.h"
#include "splineUtils.h"
#include "curveNet.h"
#include "optimization.h"
#include "cycleDiscovery.h"
#include "colormap.h"
#include "plane.h"
#include <vector>
#include <string>

class PointCloudUtils;
class CurveNet;
struct DijkstraInfo;

class PointCloudRenderer
{
public:
    PointCloudRenderer();
    ~PointCloudRenderer();
    
	bool isShowPointCloud;
	bool isShowPoints;
	bool isHideDrawnPoints;
	bool isShowUniformGrid;
	bool isShowAdaptiveGrid;
	bool isShowHessian;
	bool isShowMetric;
	bool isShowPath;
    bool isShowCtrlNodes;
    bool isShowCollinear;
    int constraintsVisual;
    int patchesVisual;
	int patchesDraw;
    int bspIndex , curveIndex;

    void incBspCurveIndex();
    void decBspCurveIndex();
    
	bool isCtrlPress;
    bool isAltPress;
    bool isShiftPress;
    bool isAutoOpt;

	double discRenderBaseRadius , discRenderRadiusScale;
	double hessianRenderBaseLength , hessianRenderLengthScale;
	double metricRenderBaseLength , metricRenderLengthScale;

	double selectionOffset;

	void init();

	void drawPoint(const vec3d& pos);
	void drawPoints();
	void drawCircle(const vec3d& origin , const vec3d& a , const vec3d& b , 
					const double& r);
	void drawCircle(const vec3d& origin , const vec3d& a , const vec3d& b , const double& r , 
					const vec3d& n);
    void drawPlane(const Plane& plane , const double& r);
    void drawLine(const vec3d& st , const vec3d& ed);
	void drawLines(const Path& v);
	void drawCube(const vec3d& lb , const vec3d& rt);
	void drawEllipse(const vec3d& origin , const vec3d& majorAxis , const double& majorLen ,
					 const vec3d& minorAxis , const double& minorLen);
	void drawEllipsoid(const vec3d& origin , const vec3d& a , const double& la ,
					   const vec3d& b , const double& lb ,
					   const vec3d& c , const double& lc);
    void drawPatch(const cycle::TriangleCycle& triangleCycle ,
        const cycle::TriangleCycle& triangleCycleNormal);
	void drawString(const std::string& str);

	void callListPoints();
	void callListSurfelDisc();
	void callListSelectionBuffer();

	void renderString();
	void renderPoints();
	void renderSurfelDisc();
	void renderPointCloud();
	void renderUniformGrid();
	void renderHessian();
	void renderMetric();
	void renderSelectedPoints();
	void renderCurrentPath();
	void renderStoredPaths();
    void renderPickedCurve();
    void renderPathForComp();
    void renderCtrlNodes();
    void renderDragPlane();
    void renderCollinearLines();
    void renderParallelLines();
    void renderCoplanarLines();
    void renderOrthogonalLines();
    void renderTangentLines();
    void renderUnsavedCycles();
    void renderPickedCycle();
    void renderPickedSavedCycle();
    void renderSavedCycles();
	void renderUnsavedMeshes();
    void renderPickedMesh();
    void renderPickedSavedMesh();
    void renderSavedMeshes();

    void render();

	void cycleColorGenByRandom(std::vector<Cycle>& cycles , std::vector<Colormap::color>& colors);
	void cycleColorGenByRanking(std::vector<double>& cycleScores , std::vector<Colormap::color>& colors);

	void clearPaths();
    void clearTemp();

	void initSelectionBuffer();
	void updateSelectionBuffer();
	int selectionByColorMap(int mouseX , int mouseY);
    int curveSelectionByRay(int mouseX , int mouseY , int& nodeIndex);
    int cycleSelectionByRay(int mouseX , int mouseY ,
        std::vector<vec3d>& cycleCenters);
    int cycleGroupSelectionByRay(int mouseX , int mouseY ,
        std::vector<std::vector<vec3d> >& cycleCenters);
    int ctrlNodeSelectionByRay(int mouseX , int mouseY , int& nodeIndex);
    // op: 0 choose, 1 store, 2 delete
	void pickPoint(int mouseX , int mouseY , int op);
    bool pickCurve(int mouseX , int mouseY , int op);
	// op: 3 add to group, 4 remove from group
    void pickCycle(int mouseX , int mouseY , int op);
    void pickSavedCycle(int mouseX , int mouseY , int op);
    // op: 0 choose, 1 update
    bool pickCtrlNode(int mouseX , int mouseY , int lastX , int lastY , int op);
    
	void surfaceBuilding(// input
		std::vector<int> &numPoints, std::vector<double*> &inCurves, std::vector<double*> &inNorms,
		bool useDelaunay, bool useMinSet, bool useNormal, 
		float areaWeight, float edgeWeight,	float dihedralWeight, float boundaryNormalWeight, 
		// output
		std::vector<std::vector<vec3d> > &mesh , std::vector<std::vector<vec3d> > &meshNormals
		);

    void optUpdate(bool isRefreshConst);
    void cycleDisc();
	void cycleStatusUpdate();

    void surfacingUnsavedCycles();
    void surfacingUnsavedCycleGroup();
    void evalUnsavedCycles();
	void cycleGroupUpdate();
	
    void backup();
    void undo();
public:
	static const int LIST_SURFEL_DISC = 2;
	static const int LIST_POINTS = LIST_SURFEL_DISC + 1;
	static const int LIST_SELECTION_BUFFER = LIST_POINTS + 1;

	bool isNormalInfo;
	PointCloudUtils *pcUtils;
    /* CurveNet *curveNet; */
    CurveNet *dispCurveNet;
    CurveNet backupCurveNet;
	std::vector<vec3d> axisU , axisV;
    
	Path pathVertex;
    BSpline bsp;
    std::vector<Path> pathForComp;
    int pathChoice;
    int smoothIter;
    double smoothScale;
    bool useBSpline;
    
	/* vec3d *pickedPoint; */
    vec3d pickedDispPoint;
	/* vec3d *lastPoint; */
    vec3d lastDispPoint;
    int pickedCurve;
    int pickedCycle;
    int pickedSavedCycle;
    int pickedBsp , pickedCtrlNode;
    Plane dragPlane;
    vec3d dragStartPoint , dragCurPoint;
    bool snapToCurve;
    bool snapToNode;

    int dragPlaneNormalIndex;

    // 0: shortest path, 1: straight line
    int drawMode;

    std::vector<Cycle> unsavedCycles;
    std::vector<bool> toBeSurfacing;
    std::vector<std::vector<Path> > unsavedCyclePoints;
    std::vector<vec3d> unsavedCycleCenters;
	std::vector<double> unsavedCycleScores;
	std::vector<int> unsavedCycleScoreRanks;

	std::vector<int> group;
	std::vector<bool> inGroup;

    std::vector<int> unsavedInCurveNums;
    std::vector<double*> unsavedInCurvePoints;
    std::vector<double*> unsavedInCurveNormals;
    
	std::vector<std::vector<std::vector<cycle::Point> > > unsavedMeshes;
	std::vector<std::vector<std::vector<cycle::Point> > > unsavedNormals;
    
    // true: if not be picked
    std::vector<bool> unsavedStatus;
	/* std::vector<std::vector<std::vector<cycle::Point> > > meshes; */
	/* std::vector<std::vector<std::vector<cycle::Point> > > meshNormals; */
    
    std::vector<vec3uc> glObjColors;
	unsigned char *rgbBuffer;

	int windowSizeX , windowSizeY;

	std::vector<double> sin36 , cos36;

	double calcHessianRenderLength(const double& l)
	{
		return powf(std::abs(l) , 0.5) * hessianRenderLengthScale;
	}

	double calcMetricRenderLength(const double& l)
	{
		return powf(std::abs(l) , 0.25) * metricRenderLengthScale;
	}
};

#endif