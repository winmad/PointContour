#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#include "nvVector.h"
#include "smallUtils.h"
#include "splineUtils.h"
#include "curveNet.h"
#include "optimization.h"
#include <vector>

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
	bool isShowUniformGrid;
	bool isShowAdaptiveGrid;
	bool isShowHessian;
	bool isShowMetric;
	bool isShowPath;
    bool isShowCtrlNodes;
    bool isShowCollinear;
    int constraintsVisual;
    int bspIndex , curveIndex;

    void incBspCurveIndex();
    void decBspCurveIndex();
    
	bool isCtrlPress;
    bool isAltPress;

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
	void drawQuad(const vec3d& origin , const vec3d& a , const vec3d& b , const double& r ,
				  const vec3d& n);
    void drawLine(const vec3d& st , const vec3d& ed);
	void drawLines(const Path& v);
	void drawCube(const vec3d& lb , const vec3d& rt);
	void drawEllipse(const vec3d& origin , const vec3d& majorAxis , const double& majorLen ,
					 const vec3d& minorAxis , const double& minorLen);
	void drawEllipsoid(const vec3d& origin , const vec3d& a , const double& la ,
					   const vec3d& b , const double& lb ,
					   const vec3d& c , const double& lc);

	void callListPoints();
	void callListSurfelDisc();
	void callListSelectionBuffer();

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
    void renderCollinearLines();
    void renderParallelLines();
    void renderCoplanarLines();
    void renderOrthogonalLines();
    void renderTangentLines();

	void render();

	void clearPaths();

	void initSelectionBuffer();
	void updateSelectionBuffer();
	int selectionByColorMap(int mouseX , int mouseY);
	void pickPoint(int mouseX , int mouseY , bool isStore);
    void pickCurve(int mouseX , int mouseY , bool isDelete);
public:
	static const int LIST_SURFEL_DISC = 2;
	static const int LIST_POINTS = LIST_SURFEL_DISC + 1;
	static const int LIST_SELECTION_BUFFER = LIST_POINTS + 1;

	bool isNormalInfo;
	PointCloudUtils *pcUtils;
    /* CurveNet *curveNet; */
    CurveNet *dispCurveNet;
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