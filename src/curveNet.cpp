#include "curveNet.h"
#include "cycleDiscovery.h"

CurveNet::CurveNet()
{
    clear();
    collinearThr = 0.05;
    parallelThr = 0.1;
    coplanarThr = 0.05;
    orthoThr = 0.1;
    tangentThr = 0.05;
	symmetryThr = 0.1;
	ratioThr = 0.05;
	planeDiffThr = 0.1;
}

void CurveNet::clear()
{
    numNodes = numPolyLines = 0;
    nodes.clear();
    nodesStat.clear();
    edges.clear();
    polyLines.clear();
    bsplines.clear();
    polyLinesIndex.clear();
    
    collinearSet.clear();
    curveType.clear();
    parallelSet.clear();
    coplanarSet.clear();
    orthoSet.clear();
	symmetricPlanes.clear();
	symmLines.clear();

    cycles.clear();
    cyclePoints.clear();
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
    curveType = net.curveType;
    collinearThr = net.collinearThr;
    coplanarThr = net.coplanarThr;
    parallelThr = net.parallelThr;
    orthoThr = net.orthoThr;
    tangentThr = net.tangentThr;
    symmetryThr = net.symmetryThr;
    reflectPlanes = net.reflectPlanes;
    symmLines = net.symmLines;
    collinearSet = net.collinearSet;
    parallelSet = net.parallelSet;
    coplanarSet = net.coplanarSet;
    orthoSet = net.orthoSet;
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
    const Path& path , bool newNode , const BSpline& bsp , const Path& originPath)
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
    addCurveType(numPolyLines - 1);
    addCollinearConstraint(numPolyLines - 1);
    addParallelConstraint(numPolyLines - 1);
    addCoplanarConstraint(numPolyLines - 1);
    addJunctionConstraint(numPolyLines - 1);
	addSymmetryConstraint(numPolyLines - 1);
}

void CurveNet::breakPath(const int& breakLine , const int& breakPoint)
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
    if (curveType[breakLine] != 1 || !collinearTest(polyLines[breakLine] , bsplines[breakLine]))
    {
        convert2Spline(polyLines[breakLine] , bsplines[breakLine]);
        bsplines[breakLine].calcCoefs();
    }
    BSpline bsp;
    if (curveType[breakLine] != 1 || !collinearTest(polyLines[numPolyLines - 1] , bsp))
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
        addCurveType(bspIndex[t]);
        addCollinearConstraint(bspIndex[t]);
        addParallelConstraint(bspIndex[t]);
        addCoplanarConstraint(bspIndex[t]);
        addJunctionConstraint(bspIndex[t]);
		addSymmetryConstraint(bspIndex[t]);
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
                if (cycles[i][j] == deleteLineIndex)
                {
                    deleteCycleIndex = i;
                    break;
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

bool CurveNet::collinearTest(Path& path , BSpline& bsp)
{
    if (path.size() <= 2)
    {
        bsp.clear();
        for (int i = 0; i < path.size(); i++)
        {
            bsp.ctrlNodes.push_back(path[i]);
        }
        return true;
    }
    vec3d x1 = path[0] , x2 = path[path.size() - 1];
    
    double totDist = 0.0;
    vec3d v = x2 - x1;
    double denom = std::abs(v.length());
    for (int i = 1; i < (int)path.size() - 1; i++)
    {
        v = (path[i] - x1).cross(path[i] - x2);
        double numer = std::abs(v.length());
        double d = numer / denom;
        if (d > 0.05) return false;
        totDist += d;
    }

    //printf("--- %.6f ---\n" , totDist / (double)path.size());
    if (totDist / (double)path.size() < 0.02)
    {
        bsp.clear();
        bsp.ctrlNodes.push_back(x1);
        bsp.ctrlNodes.push_back(x2);
        v = x2 - x1;
        v.normalize();
        for (int i = 1; i < (int)path.size() - 1; i++)
        {
            vec3d u = path[i] - x1;
            double proj = u.dot(v);
            path[i] = x1 + v * proj;
        }
        return true;
    }
    return false;
}

bool CurveNet::checkParallel(const vec3d& x1 , const vec3d& y1 ,
    const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    vec3d v1 , v2;
    v1 = y1 - x1;
    v1.normalize();
    v2 = y2 - x2;
    v2.normalize();
    if (1.0 - std::abs(v1.dot(v2)) > threshold) return false;
    return true;
}

bool CurveNet::checkCollinear(const vec3d& x1 , const vec3d& y1 ,
    const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    if (!checkParallel(x1 , y1 , x2 , y2 , threshold)) return false;
    
    vec3d v1 , v2;
    v1 = y1 - x1; v2 = x2 - x1;
    double cosine = v1.dot(v2) / (v1.length() * v2.length());
    if (1.0 - std::abs(cosine) > threshold) return false;

    v1 = y1 - x1; v2 = y2 - x1;
    cosine = v1.dot(v2) / (v1.length() * v2.length());
    if (1.0 - std::abs(cosine) > threshold) return false;

    v1 = x2 - y1; v2 = y2 - y1;
    cosine = v1.dot(v2) / (v1.length() * v2.length());
    if (1.0 - std::abs(cosine) > threshold) return false;

    v1 = x2 - x1; v2 = y2 - x1;
    cosine = v1.dot(v2) / (v1.length() * v2.length());
    if (1.0 - std::abs(cosine) > threshold) return false;
    
    vec3d denom = (y1 - x1).cross(y2 - x2);
    vec3d numer = (x2 - x1).dot(denom);
    double d = numer.length() / denom.length();
    // printf("dist = %.6f\n" , d);
    if (d > 0.01) return false;
    
    return true;
}

bool CurveNet::checkCoplanar(const BSpline& bsp , const double& threshold)
{
    double sum = 0.0;
    double denom = 0.0;
    for (int i = 0; i < (int)bsp.ctrlNodes.size() - 1; i++)
    {
        for (int j = i + 1; j < (int)bsp.ctrlNodes.size() - 1; j++)
        {
            denom += 1.0;
            vec3d x1 = bsp.ctrlNodes[i] , y1 = bsp.ctrlNodes[i + 1];
            vec3d x2 = bsp.ctrlNodes[j] , y2 = bsp.ctrlNodes[j + 1];
            if (checkParallel(x1 , y1 , x2 , y2 , parallelThr))
            {
                continue;
            }
            vec3d n = (y1 - x1).cross(y2 - x2);
            n.normalize();
            vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
            d.normalize();
            sum += std::abs(d.dot(n));
        }
    }
    //printf("%.8f\n" , sum / denom);
    if (sum / denom < threshold) return true;
    return false;
}

bool CurveNet::checkCoplanar(const BSpline& bsp1 , const BSpline& bsp2 ,
    const double& threshold)
{
    double sum = 0.0;
    double denom = 0.0;
    for (int i = 0; i < (int)bsp1.ctrlNodes.size() - 1; i++)
    {
        for (int j = 0; j < (int)bsp2.ctrlNodes.size() - 1; j++)
        {
            denom += 1.0;
            vec3d x1 = bsp1.ctrlNodes[i] , y1 = bsp1.ctrlNodes[i + 1];
            vec3d x2 = bsp2.ctrlNodes[j] , y2 = bsp2.ctrlNodes[j + 1];
            if (checkParallel(x1 , y1 , x2 , y2 , parallelThr))
            {
                continue;
            }
            vec3d n = (y1 - x1).cross(y2 - x2);
            n.normalize();
            vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
            d.normalize();
            sum += std::abs(d.dot(n));
        }
    }
    //printf("%.8f\n" , sum / denom);
    if (sum / denom < threshold) return true;
    return false;
}

bool CurveNet::checkCoplanar(const vec3d& x1 , const vec3d& y1 ,
    const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    if (checkParallel(x1 , y1 , x2 , y2 , parallelThr)) return true;
    vec3d n = (y1 - x1).cross(y2 - x2);
    n.normalize();
    vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
    d.normalize();
    //printf("%.8f\n" , std::abs(d.dot(n)));
    if (std::abs(d.dot(n)) < threshold) return true;
    return false;
}

bool CurveNet::checkOrtho(const vec3d& x0 , const vec3d& x1 ,
    const vec3d& x2 , const double& threshold)
{
    vec3d v1 = x1 - x0;
    v1.normalize();
    vec3d v2 = x2 - x0;
    v2.normalize();
    // printf("v1 = (%.6f,%.6f,%.6f) , v2 = (%.6f,%.6f,%.6f)\n" , v1.x , v1.y , v1.z ,
        // v2.x , v2.y , v2.z);
    if (std::abs(v1.dot(v2)) < threshold) return true;
    return false;
}

bool CurveNet::checkTangent(const vec3d& x0 , const vec3d& x1 ,
    const vec3d& x2 , const double& threshold)
{
    vec3d v1 = x1 - x0;
    v1.normalize();
    vec3d v2 = x2 - x0;
    v2.normalize();
    if (1.0 - std::abs(v1.dot(v2)) < threshold) return true;
    return false;
}

bool CurveNet::checkSymmetry(const vec3d& x , const vec3d& nx ,
    const vec3d& y , const vec3d& ny , const double& threshold)
{
	vec3d v0 = x - y;
	v0.normalize();
	vec3d v1 = nx + ny;
	if (abs(nx.length() - ny.length()) > ny.length() * ratioThr) return false;
	v1.normalize();

	vec3d v2 = nx - ny;
	v2.normalize();
	if (abs(v0.dot(v1)) < threshold) return true;
	//if (abs(v0.dot(v2)) < threshold) return true;
	return false;
}

bool CurveNet::checkCycleSpline(int i)
{
	return (polyLines[i][0] - polyLines[i][polyLines[i].size()-1]).length() < 1e-6;
}

void CurveNet::addCurveType(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    if (bsp.ctrlNodes.size() == 2)
    {
        curveType[numPolyLines - 1] = 1;
    }
    else if (checkCoplanar(bsp , coplanarThr))
    {
        curveType[numPolyLines - 1] = 3;
    }
    printf("curve type %d: %d\n" , bspIndex , curveType[bspIndex]);
}

void CurveNet::addCollinearConstraint(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    if (curveType[bspIndex] == 1)
    {
        collinearSet.makeSet(bspIndex , 0);
        for (int i = 0; i < numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (curveType[i] != 1) continue;
            if (bsplines[i].ctrlNodes.size() == 0) continue;
            if (collinearSet.sameRoot(bspIndex , 0 , i , 0)) continue;
            if (checkCollinear(bsp.ctrlNodes[0] , bsp.ctrlNodes[bsp.ctrlNodes.size() - 1] ,
                    bsplines[i].ctrlNodes[0] ,
                    bsplines[i].ctrlNodes[bsplines[i].ctrlNodes.size() - 1] ,
                    collinearThr))
            {
                printf("collinear: (%d) , (%d)\n" , bspIndex , i);
                collinearSet.merge(bspIndex , 0 , i , 0);
            }
        }
    }
}

void CurveNet::addParallelConstraint(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    if (curveType[bspIndex] == 1)
    {
        parallelSet.makeSet(bspIndex , 0);
        for (int i = 0; i < numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (curveType[i] != 1) continue;
            if (bsplines[i].ctrlNodes.size() == 0) continue;
            if (parallelSet.sameRoot(bspIndex , 0 , i , 0)) continue;
            if (checkParallel(bsp.ctrlNodes[0] , bsp.ctrlNodes[bsp.ctrlNodes.size() - 1] ,
                    bsplines[i].ctrlNodes[0] ,
                    bsplines[i].ctrlNodes[bsplines[i].ctrlNodes.size() - 1] ,
                    parallelThr))
            {
                printf("parallel: (%d) , (%d)\n" , bspIndex , i);
                parallelSet.merge(bspIndex , 0 , i , 0);
            }
        }
    }
}

void CurveNet::addCoplanarConstraint(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    if (curveType[bspIndex] != 2)
    {
        coplanarSet.newCurve(bspIndex , 0);
        coplanarSet.connect(bspIndex , 0 , bspIndex , 0 , 1);
        for (int i = 0; i < numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (curveType[i] != 1 && curveType[i] != 3) continue;
            if (bsplines[i].ctrlNodes.size() == 0) continue;
            if (checkCoplanar(bsp , bsplines[i] , coplanarThr))
            {
                printf("coplanar: (%d) , (%d)\n" , bspIndex , i);
                coplanarSet.connect(bspIndex , 0 , i , 0 , 1);
            }
        }
    }
}

void CurveNet::addJunctionConstraint(int bspIndex)
{
    BSpline& bsp = bsplines[bspIndex];
    int numCtrlCurves = (int)bsp.ctrlNodes.size() - 1;
    for (int i = 0; i < numCtrlCurves; i++)
    {
        orthoSet.newCurve(bspIndex , i);
    }
    for (int i = 0; i < numCtrlCurves; i++)
    {
        if (i - 1 >= 0)
        {
            if (checkOrtho(bsp.ctrlNodes[i] , bsp.ctrlNodes[i + 1] ,
                    bsp.ctrlNodes[i - 1] , orthoThr))
            {
                printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                    bspIndex , i - 1);
                orthoSet.connect(bspIndex , i , bspIndex , i - 1 , 1);
            }
            else if (checkTangent(bsp.ctrlNodes[i] , bsp.ctrlNodes[i + 1] ,
                    bsp.ctrlNodes[i - 1] , tangentThr))
            {
                printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                    bspIndex , i - 1);
                orthoSet.connect(bspIndex , i , bspIndex , i - 1 , 2);
            }
        }
        else
        {
            // printf("bsp.ctrlNodes[0] = (%.6f,%.6f,%.6f)\n" , bsp.ctrlNodes[0].x ,
                // bsp.ctrlNodes[0].y , bsp.ctrlNodes[0].z);
            int ni = getNodeIndex(bsp.ctrlNodes[0]);
            for (int j = 0; j < edges[ni].size(); j++)
            {
                int ei = edges[ni][j].pli;
                if (ei == -1 || ei == bspIndex) continue;
                printf("(%d , %d)'s next: node = %d, line = %d\n" , bspIndex ,
                    i , edges[ni][j].link , edges[ni][j].pli);
                
                int nodeIndex = (int)bsplines[ei].ctrlNodes.size() - 2;
                if (isEqual(bsp.ctrlNodes[0] , bsplines[ei].ctrlNodes[0])) nodeIndex = 1;
                
                int curveIndex = (nodeIndex == 1 ? 0 : nodeIndex);
                if (checkOrtho(bsp.ctrlNodes[0] , bsp.ctrlNodes[1] ,
                        bsplines[ei].ctrlNodes[nodeIndex] , orthoThr))
                {
                    printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                        ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 1);
                }
                else if (checkTangent(bsp.ctrlNodes[0] , bsp.ctrlNodes[1] ,
                        bsplines[ei].ctrlNodes[nodeIndex] , tangentThr))
                {
                    printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                        ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 2);
                }
            }
        }
        
        if (i + 1 < (int)bsp.ctrlNodes.size() - 1)
        {
            if (checkOrtho(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                    bsp.ctrlNodes[i + 2] , orthoThr))
            {
                printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                    bspIndex , i + 1);
                orthoSet.connect(bspIndex , i , bspIndex , i + 1 , 1);
            }
            else if (checkTangent(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                    bsp.ctrlNodes[i + 2] , tangentThr))
            {
                printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                    bspIndex , i + 1);
                orthoSet.connect(bspIndex , i , bspIndex , i + 1 , 2);
            }
        }
        else
        {
            int ni = getNodeIndex(bsp.ctrlNodes[i + 1]);
            for (int j = 0; j < edges[ni].size(); j++)
            {
                int ei = edges[ni][j].pli;
                if (ei == -1 || ei == bspIndex) continue;
                printf("(%d , %d)'s next: node = %d, line = %d\n" , numPolyLines - 1 ,
                    i , edges[ni][j].link , edges[ni][j].pli);
                
                int nodeIndex = (int)bsplines[ei].ctrlNodes.size() - 2;
                if (isEqual(bsp.ctrlNodes[i + 1] , bsplines[ei].ctrlNodes[0])) nodeIndex = 1;
                
                int curveIndex = (nodeIndex == 1 ? 0 : nodeIndex);
                if (checkOrtho(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                        bsplines[ei].ctrlNodes[nodeIndex] , orthoThr))
                {
                    printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                        ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 1);
                }
                else if (checkTangent(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                        bsplines[ei].ctrlNodes[nodeIndex] , tangentThr))
                {
                    printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                        ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 2);
                }
            }
        }
    }
}

void CurveNet::addSymmetryConstraint(int bspIndex, bool add)
{
	if (curveType[bspIndex] == 1)
	{
		vec3d &x1 = bsplines[bspIndex].ctrlNodes[0];
		vec3d &x2 = bsplines[bspIndex].ctrlNodes[1];
		vec3d n1 = x1 - x2;
		n1.normalize();
		for (int i = 0; i < numPolyLines; ++ i)
		{
			if (i == bspIndex) continue;
			if (curveType[i] == 1)
			{
				vec3d &x3 = bsplines[i].ctrlNodes[0];
				vec3d &x4 = bsplines[i].ctrlNodes[1];
				vec3d n2 = x3 - x4;
				n2.normalize();
				if (abs((x1 - x2).length() - (x3 - x4).length()) > 1e-6) continue;
				if (checkSymmetry(x1, n1, x3, n2, symmetryThr))
				{
					Plane p1((x1 + x3) / 2, x1 - x3);
					Plane p2((x2 + x4) / 2, x2 - x4);
					if (p1.dist(p2) < symmetryThr)
					{
						p1.add(p2);
						p1.weight = (x1 - x2).length();
						addSymmetryPlane(p1, add, bspIndex, i);
						printf("add symmetry : (%d, %d)\n" , bspIndex, i);
					}
				}
				if (checkSymmetry(x1, n1, x4, -n2, symmetryThr))
				{
					Plane p1((x1 + x4) / 2, x1 - x4);
					Plane p2((x2 + x3) / 2, x2 - x3);
					if (p1.dist(p2) < symmetryThr)
					{
						p1.add(p2);
						p1.weight = (x1 - x2).length();
						addSymmetryPlane(p1, add, bspIndex, i);
						printf("add symmetry : (%d, %d)\n" , bspIndex, i);
					}
				}
			}
		}
	}
	else
	{
		int sampleNum = 10;
		int pointNum = polyLines[bspIndex].size();
		if (checkCycleSpline(bspIndex)) --pointNum;
		int step = pointNum / sampleNum;
		double length = 0;
		Path &c1 = polyLines[bspIndex];

		for (int i = 1; i < pointNum; ++ i) length += (c1[i] - c1[i-1]).length();
		
		for (int i = 0; i < numPolyLines; ++ i)
		{
			if (i == bspIndex) continue;
			if (curveType[i] == curveType[bspIndex] && (!checkCycleSpline(bspIndex)^checkCycleSpline(i)))
			{
				bool flg = false;
				Path &c2 = polyLines[i];
				for (int j = 0; j < pointNum; ++ j)
				{
					Plane p(0);
					for (int k = 0; k < pointNum; k += step)
					{
						Plane nextPlane((c1[k]+c2[(j+k)%pointNum])/2, c1[k]-c2[(j+k)%pointNum], (c1[k]-c2[(j+k)%pointNum]).length());
						p.add(nextPlane);
					}
					double dist = 0;
					for (int k = 0; k < pointNum; k += step)
					{
						dist += (p.reflect(c1[k]) - c2[(j+k)%pointNum]).length();
					}
					if (dist < symmetryThr)
					{
						p.weight = length;
						addSymmetryPlane(p, add, bspIndex, i);
						bsplines[bspIndex].copyBSP(bsplines[i]);
						for (int t = 0; t < bsplines[i].ctrlNodes.size(); ++ t)
						{
							bsplines[bspIndex].ctrlNodes[t] = p.reflect(bsplines[i].ctrlNodes[t]);
						}
						flg = true;
						break;
					}
					if (!checkCycleSpline(bspIndex))
					{
						if (flg)
						{
							bsplines[bspIndex].ctrlNodes[0] = c1[0];
							bsplines[bspIndex].ctrlNodes[bsplines[bspIndex].N-1] = c1[pointNum-1];
						}
						break;
					}
				}
				if (flg) break;
				for (int j = 0; j < pointNum; ++ j)
				{
					Plane p(0);
					for (int k = 0; k < pointNum; k += step)
					{
						Plane nextPlane((c1[pointNum-1-k]+c2[(j+k)%pointNum])/2, c1[pointNum-1-k]-c2[(j+k)%pointNum], (c1[pointNum-1-k]-c2[(j+k)%pointNum]).length());
						p.add(nextPlane);
					}
					double dist = 0;
					for (int k = 0; k < pointNum; k += step)
					{
						dist += (p.reflect(c1[pointNum-1-k]) - c2[(j+k)%pointNum]).length();
					}
					if (dist < symmetryThr)
					{
						p.weight = length;
						addSymmetryPlane(p, add, bspIndex, i);
						bsplines[bspIndex].copyBSP(bsplines[i]);
						for (int t = 0; t < bsplines[i].ctrlNodes.size(); ++ t)
						{
							bsplines[bspIndex].ctrlNodes[t] = p.reflect(bsplines[i].ctrlNodes[t]);
						}
						flg = true;
						break;
					}
					if (!checkCycleSpline(bspIndex))
					{
						if (flg)
						{
							bsplines[bspIndex].ctrlNodes[0] = c1[pointNum-1];
							bsplines[bspIndex].ctrlNodes[bsplines[bspIndex].N-1] = c1[0];
						}
						break;
					}
				}
				if (flg) break;
			}
		}
	}
}

int CurveNet::addSymmetryPlane(Plane &p, bool add, int a, int b)
{
	for (int i = 0; i < symmetricPlanes.size(); ++ i)
	{
		if (symmetricPlanes[i].dist(p) < planeDiffThr)
		{
			if (add)
			{
				symmetricPlanes[i].add(p);
				if (a > -1 && b > -1)
				{
					symmLines[i].push_back(std::make_pair(a, b));
				}
			}
			else
			{
				symmetricPlanes[i].remove(p);
			}
			return i;
		}
	}
	if (add)
	{
		symmetricPlanes.push_back(p);
		std::vector<std::pair<int, int> > tmp;
		if (a > -1 && b > -1)
			tmp.push_back(std::make_pair(a, b));
		symmLines.push_back(tmp);
		symmPoints.push_back(std::vector<SelfSymmIdx>());
	}
	return (int)symmetricPlanes.size() - 1;
}

void CurveNet::addSelfSymmetryConstraint(int bspIndex)
{
	int sampleNum = 10;
	int pointNum = polyLines[bspIndex].size();
	if (checkCycleSpline(bspIndex)) --pointNum;
	int step = pointNum / sampleNum / 2;
	double length = 0;
	Path &c = polyLines[bspIndex];
	for (int i = 0; i < pointNum; ++ i)
	{
		Plane p(0);
		for (int j = 0; j < sampleNum; ++ j)
		{
			int l = (i + j * step) % pointNum;
			int r = (i - 1 - j * step + pointNum) % pointNum;
			Plane nextp((c[l]+c[r]) / 2, c[l] - c[r], (c[1] - c[r]).length());
			p.add(nextp);
		}
		double dist = 0;
		for (int j = 0; j < sampleNum; ++ j)
		{
			int l = (i + j * step) % pointNum;
			int r = (i - 1 - j * step + pointNum) % pointNum;
			dist += (p.reflect(c[l]) - c[r]).length();
		}
		if (dist < symmetryThr)
		{
			for (int j = 0; j < sampleNum; ++ j)
			{
				int l = (i + j * step) % pointNum;
				int r = (i - 1 - j * step + pointNum) % pointNum;
				addSelfSymmPlane(p, true, bspIndex, l, r);
			}
		}
		if (!checkCycleSpline(bspIndex))
		{
			break;
		}
	}
}

int CurveNet::addSelfSymmPlane(Plane &p, bool add, int l, int a, int b)
{
	for (int i = 0; i < symmetricPlanes.size(); ++ i)
	{
		if (symmetricPlanes[i].dist(p) < planeDiffThr)
		{
			if (add)
			{
				symmetricPlanes[i].add(p);
				symmPoints[i].push_back(SelfSymmIdx(l, a, b));
			}
			else
			{
				symmetricPlanes[i].remove(p);
			}
			return i;
		}
	}
	if (add)
	{
		symmetricPlanes.push_back(p);
		std::vector<SelfSymmIdx> tmp;
		tmp.push_back(SelfSymmIdx(l, a, b));
		symmPoints.push_back(tmp);
		symmLines.push_back(std::vector<std::pair>());
	}
	return (int)symmetricPlanes.size() - 1;
}

void CurveNet::addTransformConstraint(int bspIndex)
{

}

void CurveNet::mapOrigin2polyLines(int bspIndex)
{
	std::vector<int> tmp;
	int sampleNum = polyLines[bspIndex].size();
	int originNum = originPolyLines[bspIndex].size();
	int range = sampleNum / originNum * 2 + 1;
	if (checkCycleSpline(bspIndex))
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
		tmp.push_back(s)
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

void CurveNet::addCycle(const Cycle& cycle)
{
    cycles.push_back(cycle);
    std::vector<Path> cyclePts;
    vec3d center;
    calcDispCyclePoints(cycle , cyclePts , center);
    cycleCenters.push_back(center);
    cyclePoints.push_back(cyclePts);
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
    extendPath(vec3d(0 , 0 , 0) , vec3d(10 , 0 , 0) , path , true , bsp , originPath);
    
    startPath(vec3d(5 , 2 , 0));
    path.clear();
    for (int i = 2; i >= 0; i--)
    {
        path.push_back(vec3d(5 , i , 0));
    }
    extendPath(vec3d(5 , 2 , 0) , vec3d(5 , 0 , 0) , path , true , bsp , originPath);
    breakPath(0 , 5);
    
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