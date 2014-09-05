#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#include "nvVector.h"
#include "smallUtils.h"
#include <vector>

class PointCloudUtils;
class CurveNet;
struct DijkstraInfo;

class PointCloudRenderer
{
public:
	PointCloudRenderer() 
	{
		pcUtils = NULL;
		cos36.resize(36);
		sin36.resize(36);
		for(unsigned i=0; i<36; i++)
		{
			cos36[i] = cos(double(i) * PI / 18.);
			sin36[i] = sin(double(i) * PI / 18.);
		}
		init();
	}

	bool isShowPointCloud;
	bool isShowPoints;
	bool isShowUniformGrid;
	bool isShowAdaptiveGrid;
	bool isShowHessian;
	bool isShowMetric;
	bool isShowPath;

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
    void renderPathForComp();

	void render();

	void clearPaths();

	void initSelectionBuffer();
	void updateSelectionBuffer();
	int selectionByColorMap(int mouseX , int mouseY);
	void pickPoint(int mouseX , int mouseY , bool isStore);

public:
	static const int LIST_SURFEL_DISC = 2;
	static const int LIST_POINTS = LIST_SURFEL_DISC + 1;
	static const int LIST_SELECTION_BUFFER = LIST_POINTS + 1;

	bool isNormalInfo;
	PointCloudUtils *pcUtils;
    CurveNet *curveNet;
	std::vector<vec3d> axisU , axisV;

	std::vector<vec3d> selectedPoints;
	Path pathVertex;
    std::vector<Path> pathForComp;
    int pathChoice;
    int smoothIter;
    double smoothScale;
    
	std::vector<Path> storedPaths;
	vec3d *pickedPoint;
	vec3d *lastPoint;

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