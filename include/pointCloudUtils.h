#ifndef POINT_CLOUD_UTILS_H
#define POINT_CLOUD_UTILS_H
#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_

#include "nvVector.h"
#include "smallUtils.h"
#include "pointCloudRenderer.h"
#include "PointKDTree.h"
#include "Heap.h"
#include "TimeManager.h"
#include <vector>
#include <map>
#include <algorithm>
#include <wx/string.h>
#include <wx/statusbr.h>
#include <ctime>
#include <Eigen/Dense>

using namespace Eigen;

struct BBox
{
	vec3f lb , rt;
};

struct Data
{
	vec3f pos;
	vec3f n;
};

struct DistQuery
{
    float maxSqrDis;
    vec3f nearest;

    void process(Data* d , float& dist2)
    {
        nearest = d->pos;
        maxSqrDis = dist2;
    }
};

struct Tensor
{
	Matrix3d hessian;
	vec3f axis[3];
	double eigenVal[3];
	double axisLen[3];

	Tensor() {}

	Tensor(const Tensor& ts)
	{
		hessian = ts.hessian;
		for (int i = 0; i < 3; i++)
		{
			axis[i] = ts.axis[i];
			eigenVal[i] = ts.eigenVal[i];
			axisLen[i] = ts.axisLen[i];
		}
	}

	Tensor& operator =(const Tensor& ts)
	{
		hessian = ts.hessian;
		for (int i = 0; i < 3; i++)
		{
			axis[i] = ts.axis[i];
			eigenVal[i] = ts.eigenVal[i];
			axisLen[i] = ts.axisLen[i];
		}
		return *this;
	}
};

struct Edge
{
	int y;
	float w;
	Edge() {}
	Edge(const int& y , const float& w) : y(y) , w(w) {}
};

struct ClosePoint
{
	const Data *point;
	float sqrDis;

	ClosePoint(const Data *p = NULL , float _sqrDis = 1e20f)
	{
		point = p;
		sqrDis = _sqrDis;
	}

	bool operator <(const ClosePoint& rhs) const
	{
		return sqrDis == rhs.sqrDis ?
			(point < rhs.point) : (sqrDis < rhs.sqrDis);
	}
};

struct KnnQuery
{
	int KNN;
	vector<ClosePoint> knnPoints;
	float maxSqrDis;

	KnnQuery(int _KNN)
	{
		KNN = _KNN;
		maxSqrDis = 1e20f;
		knnPoints.clear();
	}

	void process(Data* point , Real sqrDis)
	{
		if (knnPoints.size() < KNN)
		{
			knnPoints.push_back(ClosePoint(point , sqrDis));
			if (knnPoints.size() == KNN)
			{
				std::make_heap(knnPoints.begin() , knnPoints.end());
				maxSqrDis = knnPoints[0].sqrDis;
			}
		}
		else
		{
			std::pop_heap(knnPoints.begin() , knnPoints.end());
			knnPoints[KNN - 1] = ClosePoint(point , sqrDis);
			std::push_heap(knnPoints.begin() , knnPoints.end());
			maxSqrDis = knnPoints[0].sqrDis;
		}
	}
};

struct DijkstraInfo
{
	std::vector<float> dist;
	std::vector<int> prev;

	DijkstraInfo() {}

	void init(const int& nodeNum)
	{
		dist.resize(nodeNum);
		prev.resize(nodeNum);

		for (int i = 0; i < nodeNum; i++)
		{
			dist[i] = 1e30f;
			prev[i] = -1;
		}
	}
};

class PointCloudUtils
{
public:
	enum GraphType
	{
		UNIFORM_GRAPH = 0,
		ADAPTIVE_GRAPH,
		POINT_GRAPH
	};

	std::vector<Data> pcData;
	PointKDTree<Data> *tree;
	
	BBox box;

	vec3i gridRes;
	vec3i gridResx;
	vec3f gridSize;

	int*** isPointInside;

	std::vector<float> xval , yval , zval;

	int extNum;
	vec3i sizeOriginF;
	double*** originF;
	std::vector<float> extXval , extYval , extZval;

	double*** f;
	//double*** df[3];
	double*** ddf[3][3];
	Tensor*** tensor;
	double alpha1 , alpha2 , alphaN;

	GraphType graphType;
	int maxGridLength;
	int bandwidth;

	int nodes , edges;
	std::map<double , int> point2Index;
	std::vector<vec3f> index2Point;
	std::vector<Tensor*> index2Tensor;
	int gridNodes , subdivNodes;
	std::vector<Tensor> subdivTensor;
	std::vector<Tensor> pointTensor;

	Graph uniGraph;
	Graph adapGraph;
	Graph pointGraph;
	DijkstraInfo uniGraphInfo;
	DijkstraInfo adapGraphInfo;
	DijkstraInfo pointGraphInfo;
	
	TimeManager timer;

	wxString m_fileName;
	wxStatusBar *statusBar;

public:
	PointCloudUtils();
	~PointCloudUtils();

	void init();
	void preprocess(const int& _gridResX , const int& _gridResY , const int& _gridResZ ,
					const int& _extNum , const int& _filterRadius , 
					const double& _alpha1 , const double& _alpha2 ,
                    const double& _alphaN);

	void getBBox();

	// must be power of 2
	void buildUniformGrid(vec3i size);
	void allocateMemory(vec3i resol , int extra);
	void deallocateMemory(vec3i resol , int extra);

	void buildAdaptiveGrid();	
	void genNarrowBand();

	void calcDistField();

	// filter diameter \approx stddev * 6 
	void gaussianSmooth(double***& origin , double*** &f , double stddev);

	void calcFirstOrderDerivative(double*** &f , const int& dir ,
						double*** &df);
	void calcSecondOrderDerivative(double*** &f , int d1 , int d2 ,
							       double*** &ddf);

	void calcHessian(double*** &f);
	void calcMetric(double*** &f);
	void calcTensorDecomposition(Tensor& ts);
	void calcTensorMetric(Tensor& ts);

	float calcEdgeWeight(const vec3f& v , const Tensor& st , const Tensor& ed);
	void buildGraphFromUniformGrid();
	void buildGraphFromAdaptiveGrid();

	void subdivision();
	void addSubdivisionEdges(Graph& g);

    Matrix3d lerpHessian(const vec3f& pos);
	void calcPointTensor();
	void buildGraphFromPoints();

	bool dijkstra(const Graph& g , const int& source ,
				  DijkstraInfo& info , int *sink = NULL);
	void traceBack(const DijkstraInfo& info , const int& sink ,
				   Path& pathVertex);

	int grid2Index(const vec3i& gridPos);
	vec3i index2Grid(const int& index);
	vec3i nearestGridPoint(const vec3f& pos);

    void laplacianSmooth(Path& path);
    
	// visualization
	PointCloudRenderer *pcRenderer;
};

#endif