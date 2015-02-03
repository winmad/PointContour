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
	bool isShowCoplanes;
    bool isShowFeaturePoints;
    bool isShowDebugPoints;
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

    void drawPointCloud();
    void drawPoint(const vec3d& pos);
	void drawPoints(const std::vector<vec3d>& points);
    void drawPoints(const std::vector<bool>& mask);
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
	void renderCoplanes();
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
    void renderAutoGenPaths();
    void renderFreeSketches();
    void renderCrossPlane();

    void renderFeaturePoints();
    void renderDebug();

    void render();

	void cycleColorGenByRandom(std::vector<Cycle>& cycles , std::vector<Colormap::color>& colors);
	void cycleColorGenByRanking(std::vector<double>& cycleScores , std::vector<Colormap::color>& colors);

	void clearPaths();
    void clearTemp();

	void initSelectionBuffer();
	void updateSelectionBuffer();
	int selectionByColorMap(int mouseX , int mouseY);
    int curveSelectionByRay(int mouseX , int mouseY , int& nodeIndex ,
        std::vector<Path>& polyLines);
    int cycleSelectionByRay(int mouseX , int mouseY ,
        std::vector<vec3d>& cycleCenters);
    int cycleGroupSelectionByRay(int mouseX , int mouseY ,
        std::vector<std::vector<vec3d> >& cycleCenters);
    int ctrlNodeSelectionByRay(int mouseX , int mouseY , int& nodeIndex);
    // op: 0 choose, 1 store, 2 delete
	void pickPoint(int mouseX , int mouseY , int op);
    bool pickCurve(int mouseX , int mouseY , int op); // extra: op = 3, choose/not
    bool pickAutoCurve(int mouseX , int mouseY , int op);
    void pickAllAutoCurves();
	// op: 3 add to group, 4 remove from group
    void pickCycle(int mouseX , int mouseY , int op);
    void pickSavedCycle(int mouseX , int mouseY , int op);
    // op: 0 choose, 1 update
    bool pickCtrlNode(int mouseX , int mouseY , int lastX , int lastY , int op);

    void initFreeSketchMode();
    void freeSketchOnPointCloud(std::vector<std::pair<unsigned , unsigned> >& seq);

    void surfaceBuilding(// input
		std::vector<int> &numPoints, std::vector<double*> &inCurves, std::vector<double*> &inNorms,
		bool useDelaunay, bool useMinSet, bool useNormal, 
		float areaWeight, float edgeWeight,	float dihedralWeight, float boundaryNormalWeight, 
		// output
		std::vector<std::vector<vec3d> > &mesh , std::vector<std::vector<vec3d> > &meshNormals
		);

    void afterCurveNetUpdate();
    void optUpdate(bool isRefreshConst);
    void cycleDisc();
	void cycleStatusUpdate();

    void surfacingUnsavedCycles();
    void surfacingUnsavedCycleGroup();
    void evalUnsavedCycles();
	void cycleGroupUpdate();

    void autoGenBySymmetry();
    void initTranslationMode();
    void autoGenByTranslation(double offset);
    void autoGenByICP();
    void autoGenByPclNurbsFitting();
    void clearAutoGen();

    void calcPointsNearCrossPlane();

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
    int toExpandCurve;
    int pickedAutoCurve;
    bool snapToCurve;
    bool snapToNode;
    Plane sketchPlane;

    int dragPlaneNormalIndex;

    // 0: shortest path, 1: straight line,
    // 2: circle or arc, 3: free 2d sketch
    int drawMode;

    // 0: none, 1: translation
    int copyMode;

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

    std::vector<Path> autoGenOriginPaths;
    std::vector<Path> autoGenPaths;
    std::vector<BSpline> autoGenBsp;
    std::vector<bool> autoPathStatus;

    std::vector<vec3d> chosenPoints;
    Path sketchLine;
    std::vector<Path> freeSketchLines;

    Plane crossPlane;
    std::vector<vec3d> crossPoints3d;
    std::vector<vec2d> crossPoints2d;

    Plane axisPlane;

    std::vector<bool> isChosen;
    std::vector<bool> isCrossing;

    std::vector<bool> isCurvesChosen;

    // for debug
    std::vector<vec3d> debugPoints;

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

    std::vector<vec3d> getRay(int mouseX , int mouseY);
};

#endif