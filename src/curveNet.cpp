#include "curveNet.h"
#include "cycleDiscovery.h"

CurveNet::CurveNet()
{
    conSet = new ConstraintSet(this);
    clear();
}

CurveNet::~CurveNet()
{
    delete conSet;
}

void CurveNet::clear()
{
    numNodes = numPolyLines = 0;
    nodes.clear();
    nodesStat.clear();
    edges.clear();
	originPolyLines.clear();
    polyLines.clear();
    bsplines.clear();
    polyLinesIndex.clear();

    curveType.clear();
    conSet->clear();

    cycles.clear();
    cyclePoints.clear();
    cycleCenters.clear();
    meshes.clear();
    meshNormals.clear();
}

void CurveNet::copyFrom(const CurveNet& net)
{
    numNodes = net.numNodes;
    numPolyLines = net.numPolyLines;
    nodes = net.nodes;
    nodesStat = net.nodesStat;
    edges = net.edges;
    originPolyLines = net.originPolyLines;
    polyLines = net.polyLines;
    bsplines = net.bsplines;
    polyLinesIndex = net.polyLinesIndex;
    cycles = net.cycles;
    cycleCenters = net.cycleCenters;
    cyclePoints = net.cyclePoints;
    meshes = net.meshes;
    meshNormals = net.meshNormals;
    curveType = net.curveType;

    conSet->copyFrom(net.conSet);
}

void CurveNet::startPath(const vec3d& st)
{
    if (getNodeIndex(st) != -1) return;
    nodes.push_back(st);
    nodesStat.push_back(true);
    numNodes++;
    edges.resize(numNodes);
    edges[numNodes - 1].clear();
}

void CurveNet::extendPath(const vec3d& st , const vec3d& ed ,
    const Path& path , bool newNode , const BSpline& bsp , const Path& originPath ,
    bool addConstrants)
{
    if (path.size() < 2)
    {
        printf("degenerated path\n");
        return;
    }

    int st_ni , ed_ni;
    // nodes
    if (newNode)
    {
        st_ni = getNodeIndex(st);
        ed_ni = numNodes;
        nodes.push_back(ed);
        nodesStat.push_back(true);
        numNodes++;
        edges.resize(numNodes);
        edges[numNodes - 1].clear();
    }
    else
    {
        st_ni = getNodeIndex(st);
        ed_ni = getNodeIndex(ed);
    }
    // if (st_ni == -1) printf("st index error\n");
    // if (ed_ni == -1) printf("ed index error\n");
    // polyLines
    polyLines.push_back(path);
    originPolyLines.push_back(originPath);
    polyLinesIndex.push_back(PolyLineIndex(st_ni , edges[st_ni].size() ,
            ed_ni , edges[ed_ni].size()));
    // bsplines
    bsplines.push_back(bsp);
    bsplines[numPolyLines].calcCoefs();
    // edges
    edges[st_ni].push_back(CurveEdge(ed_ni , numPolyLines));
    edges[ed_ni].push_back(CurveEdge(st_ni , numPolyLines));
    numPolyLines++;
    // type
    curveType.push_back(2);

    if (bsp.ctrlNodes.size() == 0) return;

    printf("===== path extend =====\n");
    //addCurveType(numPolyLines - 1);
    if (addConstrants)
    {
        updateConstraints(numPolyLines - 1);
    }
}

void CurveNet::breakPath(const int& breakLine , const int& breakPoint ,
    bool addConstrants)
{
    PolyLineIndex index = polyLinesIndex[breakLine];
    int st_ni = index.ni[0] , st_ei = index.ei[0];
    int ed_ni = index.ni[1] , ed_ei = index.ei[1];
    
    vec3d breakPos = polyLines[breakLine][breakPoint];

    nodes.push_back(breakPos);
    nodesStat.push_back(true);
    numNodes++;
    edges.resize(numNodes);
    edges[numNodes - 1].clear();

    int mid_ni = numNodes - 1;

    if (!isEqual(nodes[st_ni] , polyLines[breakLine][0]))
    {
        std::swap(st_ni , ed_ni);
        std::swap(st_ei , ed_ei);
    }
    
    Path path = polyLines[breakLine];
    Path originPath = originPolyLines[breakLine];
    // "left" part
    polyLines[breakLine].erase(polyLines[breakLine].begin() + breakPoint + 1 ,
        polyLines[breakLine].end());
    originPolyLines[breakLine].erase(originPolyLines[breakLine].begin() + breakPoint + 1 ,
        originPolyLines[breakLine].end());
    edges[st_ni][st_ei] = CurveEdge(mid_ni , breakLine);
    edges[mid_ni].push_back(CurveEdge(st_ni , breakLine));
    polyLinesIndex[breakLine].ni[0] = st_ni;
    polyLinesIndex[breakLine].ei[0] = st_ei;
    polyLinesIndex[breakLine].ni[1] = mid_ni;
    polyLinesIndex[breakLine].ei[1] = edges[mid_ni].size();
    
    // "right" part
    path.erase(path.begin() , path.begin() + breakPoint);
    originPath.erase(originPath.begin() , originPath.begin() + breakPoint);
    polyLines.push_back(path);
    originPolyLines.push_back(originPath);
    edges[mid_ni].push_back(CurveEdge(ed_ni , numPolyLines));
    edges[ed_ni][ed_ei] = CurveEdge(mid_ni , numPolyLines);
    polyLinesIndex.push_back(PolyLineIndex(mid_ni , edges[mid_ni].size() - 1 ,
                                           ed_ni , ed_ei));
    numPolyLines++;

    if (bsplines[breakLine].ctrlNodes.size() == 0) return;

    // recalculate both B-Splines
    if (curveType[breakLine] != 1 ||
        !ConstraintDetector::collinearTest(polyLines[breakLine] , bsplines[breakLine]))
    {
        convert2Spline(polyLines[breakLine] , bsplines[breakLine]);
        bsplines[breakLine].calcCoefs();
    }
    BSpline bsp;
    if (curveType[breakLine] != 1 ||
        !ConstraintDetector::collinearTest(polyLines[numPolyLines - 1] , bsp))
    {
        convert2Spline(polyLines[numPolyLines - 1] , bsp);
    }
    bsplines.push_back(bsp);
    bsplines[numPolyLines - 1].calcCoefs();
    curveType.push_back(2);
    /*
    printf("mid: (%.6f,%.6f,%.6f)\n" , breakPos.x , breakPos.y , breakPos.z);
    printf("left: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" , bsplines[breakLine].ctrlNodes[0].x ,
        bsplines[breakLine].ctrlNodes[0].y , bsplines[breakLine].ctrlNodes[0].z ,
        bsplines[breakLine].ctrlNodes[1].x , bsplines[breakLine].ctrlNodes[1].y ,
        bsplines[breakLine].ctrlNodes[1].z);
    printf("right: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" ,
        bsplines[numPolyLines - 1].ctrlNodes[0].x , bsplines[numPolyLines - 1].ctrlNodes[0].y , bsplines[numPolyLines - 1].ctrlNodes[0].z ,
        bsplines[numPolyLines - 1].ctrlNodes[1].x , bsplines[numPolyLines - 1].ctrlNodes[1].y , bsplines[numPolyLines - 1].ctrlNodes[1].z);
    */
    printf("===== path split ======\n");
    int bspIndex[2] = {numPolyLines - 1 , breakLine};
    for (int t = 0; t < 2; t++)
    {
        //addCurveType(bspIndex[t]);
        if (addConstrants)
        {
            updateConstraints(bspIndex[t]);
        }
    }
}

void CurveNet::updatePath(const int& bspIndex , const int& nodeIndex ,
    const vec3d& newPos , bool addConstraints)
{
    std::vector<int> modifiedBsp;
    vec3d pos = bsplines[bspIndex].ctrlNodes[nodeIndex];
    bsplines[bspIndex].updateBSpline(nodeIndex , newPos);
    resampleBsp(bsplines[bspIndex] , polyLines[bspIndex]);
    modifiedBsp.push_back(bspIndex);
    if (nodeIndex == 0 || nodeIndex == (int)bsplines[bspIndex].ctrlNodes.size() - 1)
    {
        int ni = getNodeIndex(pos);
        // printf("pos = (%.6f,%.6f,%.6f), ni = %d\n" , pos.x , pos.y , pos.z , ni);
        nodes[ni] = newPos;
        for (int i = 0; i < edges[ni].size(); i++)
        {
            int pli = edges[ni][i].pli;
			if (curveType[pli] == -1) continue;
            if (isEqual(bsplines[pli].ctrlNodes.front() , pos))
            {
                bsplines[pli].updateBSpline(0 , newPos);
            }
            else if (isEqual(bsplines[pli].ctrlNodes.back() , pos))
            {
                bsplines[pli].updateBSpline((int)bsplines[pli].ctrlNodes.size() - 1 , newPos);
            }
            resampleBsp(bsplines[pli] , polyLines[pli]);
            modifiedBsp.push_back(pli);
        }
    }
    
    for (int i = 0; i < modifiedBsp.size(); i++)
    {
        //addCurveType(modifiedBsp[i]);
        if (addConstraints)
        {
            updateConstraints(modifiedBsp[i]);
        }
    }
}

void CurveNet::deleteNode(const int& deleteNodeIndex)
{
    for (int i = 0; i < edges[deleteNodeIndex].size(); i++)
    {
        int lineIndex = edges[deleteNodeIndex][i].pli;
        if (lineIndex == -1 || curveType[lineIndex] == -1) continue;
        deletePath(lineIndex);
    }
    if (nodesStat[deleteNodeIndex])
    {
        printf("delete node error\n");
    }
}

void CurveNet::deletePath(const int& deleteLineIndex)
{
    polyLines[deleteLineIndex].clear();
    originPolyLines[deleteLineIndex].clear();
    bsplines[deleteLineIndex].clear();
    
    PolyLineIndex& index = polyLinesIndex[deleteLineIndex];
    int st_ni = index.ni[0] , st_ei = index.ei[0];
    int ed_ni = index.ni[1] , ed_ei = index.ei[1];
    edges[st_ni][st_ei] = CurveEdge(-1 , -1);
    edges[ed_ni][ed_ei] = CurveEdge(-1 , -1);

    index.ni[0] = index.ni[1] = index.ei[0] = index.ei[1] = -1;

    if (linkNoEdges(st_ni)) nodesStat[st_ni] = false;
    if (linkNoEdges(ed_ni)) nodesStat[ed_ni] = false;

    bool flag = true;
    int deleteCycleIndex;
    while (flag)
    {
        flag = false;
        deleteCycleIndex = -1;
        for (int i = 0; i < cycles.size(); i++)
        {
            for (int j = 0; j < cycles[i].size(); j++)
            {
                for (int k = 0; k < cycles[i][j].size(); k++)
                {
                    if (cycles[i][j][k] == deleteLineIndex)
                    {
                        deleteCycleIndex = i;
                        break;
                    }
                }
            }
            if (deleteCycleIndex != -1) break;
        }
        if (deleteCycleIndex != -1)
        {
            deleteCycle(deleteCycleIndex);
            flag = true;
        }
    }

    curveType[deleteLineIndex] = -1;
}

void CurveNet::deleteCycle(const int& deleteCycleIndex)
{
    cycles.erase(cycles.begin() + deleteCycleIndex);
    cyclePoints.erase(cyclePoints.begin() + deleteCycleIndex);
    cycleCenters.erase(cycleCenters.begin() + deleteCycleIndex);

    meshes.erase(meshes.begin() + deleteCycleIndex);
    meshNormals.erase(meshNormals.begin() + deleteCycleIndex);
}

void CurveNet::deleteCycleGroup(const int& deleteGroupIndex)
{

}

void CurveNet::cycle2boundary(Cycle& cycle ,
    std::vector<std::vector<vec3d> >& inCurves)
{
    CycleGroup cycleGroup;
    cycleGroup.push_back(cycle);
    cycle2boundary(cycleGroup , inCurves);
}

void CurveNet::cycle2boundary(CycleGroup& cycleGroup ,
    std::vector<std::vector<vec3d> >& inCurves)
{
    inCurves.clear();
    for (int i = 0; i < cycleGroup.size(); i++)
    {
        std::vector<vec3d> curve;
        for (int j = 0; j < cycleGroup[i].size(); j++)
        {
            bool needReverse = false;
            if (j > 0)
            {
                int u = cycleGroup[i][j];
                if (isEqual(curve.back() , polyLines[u].back()))
                {
                    needReverse = true;
                }
                if (!needReverse)
                {
                    for (int i = 1; i < polyLines[u].size(); i++)
                    {
                        curve.push_back(polyLines[u][i]);
                    }
                }
                else
                {
                    for (int i = (int)polyLines[u].size() - 2; i >= 0; i--)
                    {
                        curve.push_back(polyLines[u][i]);
                    }
                }
            }
            else
            {
                int u = cycleGroup[i][0] , v = cycleGroup[i][1];
                if (isEqual(polyLines[u][0] , polyLines[v].front()) ||
                    isEqual(polyLines[u][0] , polyLines[v].back()))
                {
                    needReverse = true;
                }
                if (!needReverse)
                {
                    for (int i = 1; i < polyLines[u].size(); i++)
                    {
                        curve.push_back(polyLines[u][i]);
                    }
                }
                else
                {
                    for (int i = (int)polyLines[u].size() - 2; i >= 0; i--)
                    {
                        curve.push_back(polyLines[u][i]);
                    }
                }
            }
        }
        inCurves.push_back(curve);
    }
}

void CurveNet::updateConstraints(int bspIndex)
{
	addCurveType(bspIndex);
    conSet->addCollinearConstraint(bspIndex);
    conSet->addParallelConstraint(bspIndex);
    conSet->addCoplanarConstraint(bspIndex);
    conSet->addJunctionConstraint(bspIndex);
    //conSet->addSymmetryConstraint(bspIndex);
}

void CurveNet::refreshAllConstraints()
{
    conSet->clear();
    int tmp = numPolyLines;
    numPolyLines = 0;
    for (int i = 0; i < bsplines.size(); i++)
    {
        numPolyLines++;
        if (curveType[i] == -1) continue;
        updateConstraints(i);
    }
}

int CurveNet::getNodeIndex(const vec3d& pos)
{
    for (int i = 0; i < numNodes; i++)
        if (nodesStat[i] && isEqual(pos , nodes[i]))
            return i;
    return -1;
}

bool CurveNet::linkNoEdges(const int& ni)
{
    for (int i = 0; i < edges[ni].size(); i++)
    {
        if (edges[ni][i].link != -1) return false;
    }
    return true;
}

void CurveNet::addCurveType(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    if (bsp.ctrlNodes.size() == 2)
    {
        curveType[numPolyLines - 1] = 1;
    }
    else if (ConstraintDetector::coplanarTest(bsp , ConstraintDetector::coplanarThr))
    {
        curveType[numPolyLines - 1] = 3;
    }
    // printf("curve type %d: %d\n" , bspIndex , curveType[bspIndex]);
}

void CurveNet::mapOrigin2polyLines(int bspIndex)
{
	std::vector<int> tmp;
	int sampleNum = polyLines[bspIndex].size();
	int originNum = originPolyLines[bspIndex].size();
	int range = sampleNum / originNum * 2 + 1;
	if (conSet->checkCycleSpline(bspIndex))
	{
		int s;
		double mindist = 10;
		for (int i = 0; i < sampleNum; ++ i)
		{
			double dist = (originPolyLines[bspIndex][0] - polyLines[bspIndex][i]).length();
			if (dist < mindist)
			{
				mindist = dist;
				s = i;
			}
		}
		tmp.push_back(s);
		for (int i = 1; i < originNum; ++ i)
		{
			mindist = 10;
			int nexts;
			for (int j = s - range; j < s + range; ++ j)
			{
				double dist = (originPolyLines[bspIndex][i] - polyLines[bspIndex][(j+sampleNum) % sampleNum]).length();
				if (dist < mindist)
				{
					mindist = dist;
					nexts = j;
				}
			}
			s = nexts;
			tmp.push_back(s);
		}
	}
	else
	{
		int s = 0;
		double mindist = 10;
		for (int i = 0; i < originNum; ++ i)
		{
			mindist = 10;
			int nexts;
			for (int j = s; j < s + range && j < sampleNum; ++ j)
			{
				double dist = (originPolyLines[bspIndex][i] - polyLines[bspIndex][j]).length();
				if (dist < mindist)
				{
					mindist = dist;
					nexts = j;
				}
			}
			s = nexts;
			tmp.push_back(s);
		}
	}
	if (mapOrigin.size() > bspIndex) mapOrigin[bspIndex] = tmp;
	else mapOrigin.push_back(tmp);
}

void CurveNet::calcDispCyclePoints(const Cycle& cycle ,
    std::vector<Path>& cyclePts , vec3d& cycleCenter)
{
    vec3d center(0.0);
    double N = 0.0;
    for (int i = 0; i < cycle.size(); i++)
    {
        int ci = cycle[i];
        N += (double)polyLines[ci].size();
        for (int j = 0; j < polyLines[ci].size(); j++)
        {
            center += polyLines[ci][j];
        }
    }
    center /= N;
    cycleCenter = center;
    cyclePts.clear();
    for (int i = 0; i < cycle.size(); i++)
    {
        int ci = cycle[i];
        Path curve;
        for (int j = 0; j < polyLines[ci].size(); j++)
        {
            vec3d dir = (center - polyLines[ci][j]) * 0.1;
            curve.push_back(polyLines[ci][j] + dir);
        }
        cyclePts.push_back(curve);
    }
}

void CurveNet::addCycle(const Cycle& cycle , const std::vector<Path>& cyclePts , 
	const vec3d& cycleCenter)
{
    CycleGroup _cycles;
    std::vector<std::vector<Path> >  _cyclePts;
    std::vector<vec3d> _cycleCenters;
    _cycles.push_back(cycle);
    _cycleCenters.push_back(cycleCenter);
    _cyclePts.push_back(cyclePts);
    addCycleGroup(_cycles , _cyclePts , _cycleCenters);
}

void CurveNet::addCycleGroup(const std::vector<Cycle>& _cycles , 
	const std::vector<std::vector<Path> >& _cyclePts , 
	const std::vector<vec3d>& _cycleCenters)
{
    cycles.push_back(_cycles);
    cyclePoints.push_back(_cyclePts);
    cycleCenters.push_back(_cycleCenters);
}

void CurveNet::test()
{
    clear();
    startPath(vec3d(0 , 0 , 0));
    Path path , originPath;
    BSpline bsp;
    for (int i = 0; i <= 10; i++)
    {
        path.push_back(vec3d(i , 0 , 0));
    }
    extendPath(vec3d(0 , 0 , 0) , vec3d(10 , 0 , 0) , path , true , bsp , originPath , false);
    
    startPath(vec3d(5 , 2 , 0));
    path.clear();
    for (int i = 2; i >= 0; i--)
    {
        path.push_back(vec3d(5 , i , 0));
    }
    extendPath(vec3d(5 , 2 , 0) , vec3d(5 , 0 , 0) , path , true , bsp , originPath , false);
    breakPath(0 , 5 , false);
    
    debugLog();
}

void CurveNet::debugPrint()
{
    printf("============ nodes list =============\n");
    for (int i = 0; i < numNodes; i++)
    {
        if (!nodesStat[i]) continue;
        printf("node %d: (%.6f,%.6f,%.6f)\n" , i , nodes[i].x , nodes[i].y ,
                 nodes[i].z);
    }
    printf("========== poly lines list ==========\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        if (polyLines[i].size() == 0) continue;
        printf("------------------------------\n");
        printf("poly line %d:\n" , i);
        for (int j = 0; j < polyLines[i].size(); j++)
        {
            printf("(%.6f,%.6f,%.6f)\n" , polyLines[i][j].x , polyLines[i][j].y ,
                     polyLines[i][j].z);
        }
    }
    printf("=========== edges list ==============\n");
    for (int i = 0; i < numNodes; i++)
    {
        if (!nodesStat[i]) continue;
        printf("------------------------------\n");
        printf("node %d:\n" , i);
        for (int j = 0; j < edges[i].size(); j++)
        {
            printf("(%d -> %d), poly line index = %d\n" , i ,
                     edges[i][j].link , edges[i][j].pli);
        }
    }
    printf("*************** End *****************\n");
}

void CurveNet::debugLog()
{
    writeLog("============ nodes list =============\n");
    for (int i = 0; i < numNodes; i++)
    {
        if (!nodesStat[i]) continue;
        writeLog("node %d: (%.6f,%.6f,%.6f)\n" , i , nodes[i].x , nodes[i].y ,
                 nodes[i].z);
    }
    writeLog("========== poly lines list ==========\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        if (polyLines[i].size() == 0) continue;
        writeLog("------------------------------\n");
        writeLog("poly line %d:\n" , i);
        for (int j = 0; j < polyLines[i].size(); j++)
        {
            writeLog("(%.6f,%.6f,%.6f)\n" , polyLines[i][j].x , polyLines[i][j].y ,
                     polyLines[i][j].z);
        }
    }
    writeLog("=========== edges list ==============\n");
    for (int i = 0; i < numNodes; i++)
    {
        if (!nodesStat[i]) continue;
        writeLog("------------------------------\n");
        writeLog("node %d:\n" , i);
        for (int j = 0; j < edges[i].size(); j++)
        {
            writeLog("(%d -> %d), poly line index = %d\n" , i ,
                     edges[i][j].link , edges[i][j].pli);
        }
    }
    writeLog("======== ctrl nodes list ============\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        if (curveType[i] == -1) continue;
        writeLog("------------------------------\n");
        writeLog("bspline %d:\n" , i);
        for (int j = 0; j < bsplines[i].ctrlNodes.size(); j++)
        {
            writeLog("(%.6f,%.6f,%.6f)\n" , bsplines[i].ctrlNodes[j].x , bsplines[i].ctrlNodes[j].y , bsplines[i].ctrlNodes[j].z);
        }
    }
    writeLog("*************** End *****************\n");
}

void CurveNet::outputPolyLines()
{
    writeLog("%d\n" , numPolyLines);
    for (int i = 0; i < numPolyLines; i++)
    {
        writeLog("%d\n" , polyLines[i].size());
        for (int j = 0; j < polyLines[i].size(); j++)
        {
            writeLog("%.6lf %.6lf %.6lf\n" , polyLines[i][j].x ,
                polyLines[i][j].y , polyLines[i][j].z);
        }
    }
}

void CurveNet::saveCurveNet(const char* fileName)
{
    FILE *fp = fopen(fileName , "w");
    fprintf(fp , "%d %d\n" , numNodes , numPolyLines);
    for (int i = 0; i < numNodes; i++)
    {
        fprintf(fp , "%.6lf %.6lf %.6lf\n" , nodes[i].x , nodes[i].y , nodes[i].z);
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numNodes; i++)
    {
        fprintf(fp , "%d " , (int)nodesStat[i]);
    }
    fprintf(fp , "\n\n");
    for (int i = 0; i < numNodes; i++)
    {
        fprintf(fp , "%lu\n" , edges[i].size());
        for (int j = 0; j < edges[i].size(); j++)
        {
            fprintf(fp , "%d %d\n" , edges[i][j].link , edges[i][j].pli);
        }
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        fprintf(fp , "%lu\n" , originPolyLines[i].size());
        for (int j = 0; j < originPolyLines[i].size(); j++)
        {
            fprintf(fp , "%.6lf %.6lf %.6lf\n" , originPolyLines[i][j].x ,
                originPolyLines[i][j].y , originPolyLines[i][j].z);
        }
        fprintf(fp , "\n");
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        fprintf(fp , "%lu\n" , polyLines[i].size());
        for (int j = 0; j < polyLines[i].size(); j++)
        {
            fprintf(fp , "%.6lf %.6lf %.6lf\n" , polyLines[i][j].x ,
                polyLines[i][j].y , polyLines[i][j].z);
        }
        fprintf(fp , "\n");
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        fprintf(fp , "%lu\n", bsplines[i].ctrlNodes.size());
        for (int j = 0; j < bsplines[i].ctrlNodes.size(); j++)
        {
            fprintf(fp , "%.6lf %.6lf %.6lf\n" , bsplines[i].ctrlNodes[j].x ,
                bsplines[i].ctrlNodes[j].y , bsplines[i].ctrlNodes[j].z);
        }
        fprintf(fp , "%lu\n" , bsplines[i].t.size());
        for (int j = 0; j < bsplines[i].t.size(); j++)
        {
            fprintf(fp , "%.6lf " , bsplines[i].t[j]);
        }
        fprintf(fp , "\n");
        if (bsplines[i].ctrlNodes.size() > 2)
        {
            fprintf(fp , "%lu\n" , bsplines[i].knots.size());
            for (int j = 0; j < bsplines[i].knots.size(); j++)
            {
                fprintf(fp , "%.6lf " , bsplines[i].knots[j]);
            }
            fprintf(fp , "\n");
            fprintf(fp , "%d %d\n" , bsplines[i].N , bsplines[i].K);
            for (int j = 0; j < bsplines[i].N; j++)
            {
                for (int k = 0; k < bsplines[i].K; k++)
                {
                    fprintf(fp , "%.6lf " , bsplines[i].coefs[j][k]);
                }
                fprintf(fp , "\n");
            }
        }
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        fprintf(fp , "%d %d %d %d\n", polyLinesIndex[i].ni[0] , polyLinesIndex[i].ei[0] ,
            polyLinesIndex[i].ni[1] , polyLinesIndex[i].ei[1]);
    }
    fprintf(fp , "\n");
    for (int i = 0; i < numPolyLines; i++)
    {
        fprintf(fp , "%d " , curveType[i]);
    }
    fprintf(fp , "\n");
    fclose(fp);
    printf("Curve saved!\n");
}

void CurveNet::loadCurveNet(const char* fileName)
{
    FILE *fp = fopen(fileName , "r");
    vec3d p;
    int tp;
    clear();
    fscanf(fp , "%d %d" , &numNodes , &numPolyLines);
    for (int i = 0; i < numNodes; i++)
    {
        fscanf(fp , "%lf %lf %lf" , &p.x , &p.y , &p.z);
        nodes.push_back(p);
    }
    for (int i = 0; i < numNodes; i++)
    {
        fscanf(fp , "%d " , &tp);
        nodesStat.push_back((tp == 1));
    }
    edges.resize(numNodes);
    for (int i = 0; i < numNodes; i++)
    {
        fscanf(fp , "%d" , &tp);
        for (int j = 0; j < tp; j++)
        {
            int link , pli;
            fscanf(fp , "%d %d" , &link , &pli);
            edges[i].push_back(CurveEdge(link , pli));
        }
    }
    originPolyLines.resize(numPolyLines);
    for (int i = 0; i < numPolyLines; i++)
    {
        fscanf(fp , "%d" , &tp);
        for (int j = 0; j < tp; j++)
        {
            fscanf(fp , "%lf %lf %lf" , &p.x , &p.y , &p.z);
            originPolyLines[i].push_back(p);
        }
    }
    polyLines.resize(numPolyLines);
    for (int i = 0; i < numPolyLines; i++)
    {
        fscanf(fp , "%d" , &tp);
        for (int j = 0; j < tp; j++)
        {
            fscanf(fp , "%lf %lf %lf" , &p.x , &p.y , &p.z);
            polyLines[i].push_back(p);
        }
    }
    bsplines.resize(numPolyLines);
    for (int i = 0; i < numPolyLines; i++)
    {
        fscanf(fp , "%d" , &tp);
        for (int j = 0; j < tp; j++)
        {
            fscanf(fp , "%lf %lf %lf" , &p.x , &p.y , &p.z);
            bsplines[i].ctrlNodes.push_back(p);
        }
        fscanf(fp , "%d" , &tp);
        for (int j = 0; j < tp; j++)
        {
            fscanf(fp , "%lf " , &p.x);
            bsplines[i].t.push_back(p.x);
        }
        if (bsplines[i].ctrlNodes.size() > 2)
        {
            fscanf(fp , "%d" , &tp);
            for (int j = 0; j < tp; j++)
            {
                fscanf(fp , "%lf " , &p.x);
                bsplines[i].knots.push_back(p.x);
            }
            fscanf(fp , "%d %d" , &bsplines[i].N , &bsplines[i].K);
            bsplines[i].newCoefs();
            for (int j = 0; j < bsplines[i].N; j++)
            {
                for (int k = 0; k < bsplines[i].K; k++)
                {
                    fscanf(fp , "%lf " , &bsplines[i].coefs[j][k]);
                }
            }
        }
    }
    for (int i = 0; i < numPolyLines; i++)
    {
        int ni0 , ei0 , ni1 , ei1;
        fscanf(fp , "%d %d %d %d" , &ni0 , &ei0 , &ni1 , &ei1);
        polyLinesIndex.push_back(PolyLineIndex(ni0 , ei0 , ni1 , ei1));
    }
    for (int i = 0; i < numPolyLines; i++)
    {
        fscanf(fp , "%d" , &tp);
        curveType.push_back(tp);
    }
    fclose(fp);
    printf("Curve loaded!\n");
}