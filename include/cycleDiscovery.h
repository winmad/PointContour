#ifndef CYCLE_DISCOVERY_H
#define CYCLE_DISCOVERY_H

#include <vector>
#include "nvVector.h"

namespace cycle
{
    
typedef vec3d Point;
typedef std::vector<Point> Curve;
typedef std::vector<Curve> LinearCurveNet;

class GraphNode{
public:
	Point pos;
	std::vector<int> arcID;
	std::vector<int> arcDirection;
public:
	GraphNode()
	{

	}
	GraphNode(const Point &pos, const std::vector<int> &arcID,const std::vector<int> &arcDirection)
		:pos(pos),arcID(arcID),arcDirection(arcDirection)
	{
	}
	GraphNode(const GraphNode& that)
	{
		this->pos = that.pos;
		this->arcID = that.arcID;
		this->arcDirection = that.arcDirection;
	}
	~GraphNode()
	{
		std::vector<int>().swap(arcID);
		std::vector<int>().swap(arcDirection);
	}
	GraphNode& operator=(const GraphNode& that)
	{
		this->pos = that.pos;
		this->arcID = that.arcID;
		this->arcDirection = that.arcDirection;
		return *this;
	}
};
class GraphEdge{
public:
	Curve vertexList;
	int capacity;
	std::pair<int,int> endNodesID;
	std::pair<int,int> posInNode;
public:
	GraphEdge()
	{

	}
	GraphEdge(const Curve &vertexList, const int &capacity,
		const std::pair<int,int>  &endNodesID,const std::pair<int,int> &posInNode)
		:vertexList(vertexList),capacity(capacity),endNodesID(endNodesID),posInNode(posInNode)
	{
	}
	GraphEdge(const GraphEdge& that)
	{
		this->vertexList = that.vertexList;
		this->capacity = that.capacity;
		this->endNodesID = that.endNodesID;
		this->posInNode = that.posInNode;
	}
	~GraphEdge()
	{
		Curve().swap(vertexList);
		std::pair<int,int>().swap(endNodesID);
		std::pair<int,int>().swap(posInNode);
	}
	GraphEdge& operator=(const GraphEdge& that)
	{
		this->vertexList = that.vertexList;
		this->capacity = that.capacity;
		this->endNodesID = that.endNodesID;
		this->posInNode = that.posInNode;
		return *this;
	}
};
class Graph{
public:
	std::vector<GraphNode> nodes;
	std::vector<GraphEdge> arcs;
public:
	Graph()
	{

	}
	Graph(const std::vector<GraphNode> &nodes, const std::vector<GraphEdge> &arcs)
		:nodes(nodes),arcs(arcs)
	{
	}
	Graph(const Graph& that)
	{
		this->nodes = that.nodes;
		this->arcs = that.arcs;
	}
	~Graph()
	{
		std::vector<GraphNode>().swap(nodes);
		std::vector<GraphEdge>().swap(arcs);
	}
	Graph& operator=(const Graph& that)
	{
		this->nodes = that.nodes;
		this->arcs = that.arcs;
		return *this;
	}
};

//rotation graph;
typedef std::vector<std::vector<std::vector<std::pair<int,int> > > > RotationGraph;

//pole graph; efficient graph search, implement for dynamic programming;
typedef std::vector<RotationGraph> PoleGraphNodes;
typedef std::vector<std::vector<double> > PoleGraphNodeWeight;
typedef std::vector<std::vector<std::vector<double> > > PoleGraphArcs;
typedef std::vector<std::vector<std::vector<std::vector<std::pair<int,int> > > > > PoleGraphArcsPair;
class ExpandPole{
public:
	int nodeID;
	std::vector<long long> front;
	long long stateNum;
public:
	ExpandPole()
	{

	}
	ExpandPole(const int &nodeID, const std::vector<long long> &front, const long long &stateNum)
		:nodeID(nodeID),front(front),stateNum(stateNum)
	{
	}
	ExpandPole(const ExpandPole& that)
	{
		this->nodeID = that.nodeID;
		this->front = that.front;
		this->stateNum = that.stateNum;
	}
	~ExpandPole()
	{
		std::vector<long long>().swap(front);
	}
	ExpandPole& operator=(const ExpandPole& that)
	{
		this->nodeID = that.nodeID;
		this->front = that.front;
		this->stateNum = that.stateNum;
		return *this;
	}
};
typedef std::vector<ExpandPole> ExpandPoleSequence;

//cycle;
class CycleSegment{
public:
	int arcID;
	int strEndID;
	int instanceID;
public:
	CycleSegment()
	{

	}
	CycleSegment(const int &arcID, const int &strEndID,const int &instanceID)
		:arcID(arcID),strEndID(strEndID),instanceID(instanceID)
	{
	}
	CycleSegment(const CycleSegment& that)
	{
		this->arcID = that.arcID;
		this->strEndID = that.strEndID;
		this->instanceID = that.instanceID;
	}
	~CycleSegment()
	{
	}
	CycleSegment& operator=(const CycleSegment& that)
	{
		this->arcID = that.arcID;
		this->strEndID = that.strEndID;
		this->instanceID = that.instanceID;
		return *this;
	}
};
typedef std::vector<CycleSegment> Cycle;
typedef std::vector<Cycle> CycleSet;

//triangle mesh;
typedef std::vector<std::vector<Point> > TriangleCycle;
typedef std::vector<TriangleCycle > TriangleSurface;

class GraphSearch{
public:
	GraphSearch();
	~GraphSearch();

private:
	typedef std::vector<int> Capacity;
	typedef std::vector<std::pair<double,std::pair<int,int> > > ArcWithWeight;
	typedef std::vector<std::vector<double> > PairArcsWithWeight;
	typedef std::vector<std::pair<int,int> > MultipleArcs;
	typedef std::vector<MultipleArcs> AllMultipleArcs;
	typedef struct{
		std::vector<std::vector<int> > usage;
		Capacity availableCapacity;
		double cost;
		double hcost;
	}StateEx2;
	typedef struct{
		MultipleArcs usedArcs;
		Capacity availableCapacity;
		int nextArcID;
		double weight;
	}State;
	typedef struct{
		MultipleArcs usedArcs;
		Capacity availableCapacity;
		int nextArcID;
		double totalWeight;
		std::vector<double> dihedralWeights;
		std::vector<std::vector<int> > adjacentArcs;
	}StateExtend;

public:

	double computeGeneralHamiltonGraph(const Capacity &org1,
		const ArcWithWeight &org2, const bool &org3, const bool &org4,
		AllMultipleArcs &tar, std::vector<double> &tar2);
	double computeGeneralHamiltonGraph(const Capacity &org1,
		const ArcWithWeight &org2, const PairArcsWithWeight & org3,
		const double org4, const bool &org5, const bool &org6,
		AllMultipleArcs &tar, std::vector<double> &tar2);

	void computeGeneralHamiltonGraph(const Capacity &capacity,
		const std::vector<std::pair<int,int> > &allArcs,
		const std::vector<double> &weights,
		const int K,const bool isConnect,
		const std::vector<std::pair<int,int> > &predefinedArcs,
		AllMultipleArcs &tar,std::vector<double> &tar2);
	void stableMatching(const std::vector<std::vector<int> > &firstGroup,
		const std::vector<std::vector<int> > &secondGroup,
		const std::vector<std::pair<int,int> > &predefinedPairs,std::vector<int> &res);
};

//main class for cycle discovery;
struct cycleDisConfig{

};
class cycleUtils{
public:
	cycleUtils();
	~cycleUtils();

private:

	bool m_isSmothing;

	LinearCurveNet m_curves;
	std::vector<int> m_curveCapacitys;
	Point m_boundingBoxMin;
	Point m_boundingBoxMax;

	Graph m_curveNet;

	int m_twistNormNum;
	double m_angleWeight;
	double m_twistWeight;
	double m_curveWeight;
	double m_nodeWeight;
	int m_bestNeightboreNum;
	int m_rotationGraphNum;
	double m_dihedralWeight;
	int m_stateNum;
	bool m_stateUnLimited;
	bool m_isRoGraphConnect;
	bool m_isDoCycleBreak;
	double m_weightTri,m_weightEdge,m_weightBiTri,m_weightTriBd,m_weightWorsDih;
	bool m_SurfaceSmooth;
	int m_subdivisonSmooth,m_laplacianSmooth;

	std::vector<std::vector<std::vector<double> > > m_twistTables;
	std::vector<std::vector<std::vector<double> > > m_twistTablesConfidence;
	std::vector<std::vector<std::vector<int> > > m_twistTablesIndex;

	std::vector<std::vector<std::pair<int,int> > > m_userDefinedPairsInNode;
	std::vector<std::vector<std::pair<int,int> > > m_userDefinedPairsInArc;
	PoleGraphNodes m_poleGraphNodes;
	PoleGraphNodeWeight m_poleGraphNodeWeight;
	PoleGraphArcs m_poleGraphArcsWeight;
	PoleGraphArcsPair m_poleGraphArcsMatch;
	ExpandPoleSequence m_expandPoleSequence;
	std::vector<int> m_selectedNodeInPole;

	bool m_isDihedral;

	RotationGraph m_rotationGraph;
	RotationGraph m_rotationGraphUpdate;

	CycleSet m_cycleSet;
	bool m_isCycleBreak;
	bool m_isCycleCorrect;
	int m_breakNum;
	std::vector<double> m_arcsCost;
	std::vector<double> m_cyclesCost;

	std::vector<std::vector<std::vector<Point> > > m_cycleNormal;
	std::vector<std::vector<std::vector<std::vector<Point> > > > m_normalsTable;

	//void deleteNodeWithTwoDegree();

	//bool updateConstraintList();

	void computeCurveNormal(const std::vector<Point> &org, Point &tar);
	void computeTransportMatrix(const std::vector<Point> &org,std::vector<double> &tar);
	void computeTransportMatrixAll(const std::vector<Point> &org,std::vector<std::vector<double> > &tar);
	double computeAngle(const Point &org1,const Point &org2);

	double computeAnglebetweenTwoCurves(const int &i,const int &j,const std::vector<Curve> &pointLists);
	Point computeNormalPlanebetweenTwoCurves(const int &i,const int &j,const std::vector<Curve> &pointLists);
	
	void constructJointRotationGraph();
	void constructSegmentRotationGraph();
	
	Point rotateNormal(const Point &normal, const Point &tangent, const double &angle);
	double twistBisectCostSym(const Point &e1,const Point &e2,const Point &n1);
	void computeTwistTables();
	void constructJointRotationGraphbyPoleGraph();
	void constructSegmentRotationGraphbyPoleGraph();
	void constructExpandSequence();
	void searchMinPoleGraph();
	void searchAlmostMinPoleGraph();
	void updateRotationGraphbyPoleGraph();

	void computeArcCost();
	void computeCycleCost();

	GraphSearch *m_graphSearch;

public:

	void dataCleanUp();
	void constructNetwork(std::vector<std::vector<Point> > &curveNet,
        std::vector<std::vector<unsigned> > &cycleMap);
	void constructRandomRotationGraph();
	void constructRotationGraphbyAngleMetric();
	void constructRotationGraphbyAngleDihedralMetric();

	void addCycleConstraint(std::vector<std::vector<unsigned> >&cycles);

	void constructRotationGraphbyPoleGraph();
	void constructCycles();
	void cycleBreaking();
    void surfaceBuilding(std::vector<int> &inCurveNums,
        std::vector<double*> &inCurvePoints,
        std::vector<double*> &inCurveNormals);

	void drawPatch(int patchID);

	CycleSet m_cycleSetBreaked;
	TriangleSurface m_triangleSurface;
	TriangleSurface m_triangleSurfaceNormal;

	std::vector<double*> m_newPoints;
	std::vector<float*> m_newNormals;
	std::vector<int> m_newPointNum;
};


void cycleDiscovery(std::vector<std::vector<Point> > &inCurves,
	std::vector<std::vector<unsigned> > &inCycleConstraint,
    std::vector<bool> &inCycleToBeRemoved,
	std::vector<std::vector<unsigned> > &outCycles,
    std::vector<bool> &outCycleToBeSurfacing,
    std::vector<int> &inCurveNums,
    std::vector<double*> &inCurvePoints,
    std::vector<double*> &inCurveNormals,
	std::vector<std::vector<std::vector<Point> > >&outMeshes,
	std::vector<std::vector<std::vector<Point> > >&outNormals);

void cycleTest();

};


#endif



