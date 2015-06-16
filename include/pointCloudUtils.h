#ifndef POINT_CLOUD_UTILS_H
#define POINT_CLOUD_UTILS_H
#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_

#include "nvVector.h"
#include "smallUtils.h"
#include "pointCloudRenderer.h"
#include "PointKDTree.h"
#include "Heap.h"
#include "TimeManager.h"
#include "curveNet.h"
#include "colormap.h"
#include "partialSymmetry.h"
#include "sketchGLCanvas.h"
#include <vector>
#include <map>
#include <algorithm>
#include <wx/string.h>
#include <wx/statusbr.h>
#include <ctime>
#include <Eigen/Dense>
#ifdef __APPLE__
    #include <sys/uio.h>
#else
    #include <io.h>
#endif

using namespace Eigen;

struct BBox
{
	vec3d lb , rt;
};

struct Data
{
	vec3d pos;
	vec3d n;
    int index;
};

struct PointData
{
    vec3d pos;
    int index;
};

struct PatchPointData
{
	vec3d pos;
	int patchId;

	PatchPointData() {}
	PatchPointData(const vec3d& _pos , const int& _patchId)
		: pos(_pos) , patchId(_patchId) {}
};

struct DistQuery
{
    double maxSqrDis;
	int patchId;
    int pointIndex;
    vec3d nearest;

    void process(Data* d , double& dist2)
    {
        nearest = d->pos;
        maxSqrDis = dist2;
        pointIndex = d->index;
    }

    void process(PointData* d , double& dist2)
    {
        nearest = d->pos;
        maxSqrDis = dist2;
        pointIndex = d->index;
    }

	void process(PatchPointData* d , double& dist2)
	{
		nearest = d->pos;
		maxSqrDis = dist2;
		patchId = d->patchId;
	}
};

struct Tensor
{
	Matrix3d hessian;
    Matrix3d tensor;
	vec3d axis[3];
	double eigenVal[3];
	double axisLen[3];

	Tensor() {}

	Tensor(const Tensor& ts)
	{
		hessian = ts.hessian;
        tensor = ts.tensor;
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
        tensor = ts.tensor;
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
	double w;
	Edge() {}
	Edge(const int& y , const double& w) : y(y) , w(w) {}
};

struct ClosePoint
{
	const Data *point;
	double sqrDis;

	ClosePoint(const Data *p = NULL , double _sqrDis = 1e20f)
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
	double maxSqrDis;

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
	std::vector<double> dist;
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

    enum MetricType
    {
        MIN_CURVATURE = 0,
        MAX_CURVATURE
    };
    
	std::vector<Data> pcData;
	PointKDTree<Data> *tree;

    bool initialized;
    bool needTransform;
	BBox box;

	vec3i gridRes;
	vec3i gridResx;
	vec3d gridSize;

	int*** isPointInside;

	std::vector<double> xval , yval , zval;

	int extNum;
	vec3i sizeOriginF;
	double*** originF;
	std::vector<double> extXval , extYval , extZval;

	double*** f;
	double*** ddf[3][3];
	Tensor*** tensor;
	double alpha1 , alpha2 , alphaN;

	GraphType graphType;
    MetricType metricType;

	int nodes , edges;
    int KNN;
	std::map<double , int> point2Index;
	std::vector<vec3d> index2Point;
	std::vector<Tensor*> index2Tensor;
	std::vector<Tensor> pointTensor;

	Graph pointGraph;
	DijkstraInfo pointGraphInfo;
    
	TimeManager timer;

	wxString m_fileName;
    wxString m_cacheName;
    std::string name;
    std::string matlabFilesPath;
    std::string dataCurvePath;
    std::string dataMeshPath;
    std::string dataVotesPath;

    wxStatusBar *statusBar;

    // visualization
	PointCloudRenderer *pcRenderer;
    SketchGLCanvas *openGLView;
    // curve network
    CurveNet *curveNet;
    // optimization
    Optimization opt;
    // symmetry detector
    PartialSymmetry partSym;

	std::vector<int> pcColor;
	std::vector<Colormap::color> colors;
    std::vector<bool> isFeaturePoint;

public:
	PointCloudUtils();
	~PointCloudUtils();

    void globalInit();
	void init();
	void preprocess(const int& _gridResX , const int& _gridResY , const int& _gridResZ ,
					const int& _extNum , const int& _filterRadius , 
					const double& _alpha1 , const double& _alpha2 ,
                    const double& _alphaN);

	void getBBox();
    
	void buildUniformGrid(vec3i size);
	void allocateMemory(vec3i resol , int extra);
	void deallocateMemory(vec3i resol , int extra);

	void calcDistField();
    void loadDistField();

	// filter diameter \approx stddev * 6 
	void gaussianSmooth(double***& origin , double*** &f , double stddev);

	void calcSecondOrderDerivative(double*** &f , int d1 , int d2 ,
							       double*** &ddf);

	void calcHessian(double*** &f);
	void calcMetric(double*** &f);
	void calcTensorDecomposition(Tensor& ts);
	void calcTensorMetric(Tensor& ts);

	double calcEdgeWeight(const vec3d& v , const Tensor& st , const Tensor& ed);
	
    Matrix3d lerpHessian(const vec3d& pos);
    Matrix3d lerpTensor(const vec3d& pos);
	void calcPointTensor();
	void buildGraphFromPoints();
    bool addPointToGraph(const vec3d& pos);

    void calcSymmetricPlanes();
    void loadSymmetricPlanes();
    
	bool dijkstra(const Graph& g , const int& source ,
				  DijkstraInfo& info , int *sink = NULL);
	void traceBack(const DijkstraInfo& info , const int& sink ,
				   Path& pathVertex);

    void laplacianSmooth(Path& path);
    void gradientDescentSmooth(Path& path , bool moveEnds = false);
    void optimizeJunction(CurveNet* cn , const vec3d& pos);

	// weak one
	double calcPatchScore(std::vector<std::vector<vec3d> >& mesh);
	// strong one
	void calcPatchScores(std::vector<std::vector<std::vector<vec3d> > >& meshes ,
		std::vector<double>& scores);

    // choose center of each triangle
    std::vector<vec3d> samplePointsFromPatchSimple(std::vector<std::vector<vec3d> >& mesh);

    // first: sample triangle according to area, then: unifrom sample points from triangle
	std::vector<vec3d> samplePointsFromPatch(std::vector<std::vector<vec3d> >& mesh);

    void pcSegmentByPatches(std::vector<std::vector<std::vector<vec3d> > >& meshes);
};

#endif