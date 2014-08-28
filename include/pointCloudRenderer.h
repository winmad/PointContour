#ifndef POINT_CLOUD_RENDERER_H
#define POINT_CLOUD_RENDERER_H

#include "nvVector.h"
#include "smallUtils.h"
#include <vector>

class PointCloudUtils;
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

	float discRenderBaseRadius , discRenderRadiusScale;
	float hessianRenderBaseLength , hessianRenderLengthScale;
	float metricRenderBaseLength , metricRenderLengthScale;

	float selectionOffset;

	void init();

	void drawPoint(const vec3f& pos);
	void drawPoints();
	void drawCircle(const vec3f& origin , const vec3f& a , const vec3f& b , 
					const float& r);
	void drawCircle(const vec3f& origin , const vec3f& a , const vec3f& b , const float& r , 
					const vec3f& n);
	void drawQuad(const vec3f& origin , const vec3f& a , const vec3f& b , const float& r ,
				  const vec3f& n);
	void drawLines(const Path& v);
	void drawCube(const vec3f& lb , const vec3f& rt);
	void drawEllipse(const vec3f& origin , const vec3f& majorAxis , const float& majorLen ,
					 const vec3f& minorAxis , const float& minorLen);
	void drawEllipsoid(const vec3f& origin , const vec3f& a , const float& la ,
					   const vec3f& b , const float& lb ,
					   const vec3f& c , const float& lc);

	void callListPoints();
	void callListSurfelDisc();
	void callListSelectionBuffer();

	void renderPoints();
	void renderSurfelDisc();
	void renderPointCloud();
	void renderUniformGrid();
	void renderAdaptiveGrid();
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
	std::vector<vec3f> axisU , axisV;

	std::vector<vec3f> selectedPoints;
	Path pathVertex;
    std::vector<Path> pathForComp;
    int pathChoice;
    
	std::vector<Path> storedPaths;
	vec3f *pickedPoint;
	vec3f *lastPoint;

	std::vector<vec3uc> glObjColors;
	unsigned char *rgbBuffer;

	int windowSizeX , windowSizeY;

	std::vector<double> sin36 , cos36;

	float calcHessianRenderLength(const float& l)
	{
		return powf(std::abs(l) , 0.5) * hessianRenderLengthScale;
	}

	float calcMetricRenderLength(const float& l)
	{
		return powf(std::abs(l) , 0.25) * metricRenderLengthScale;
	}
};

#endif