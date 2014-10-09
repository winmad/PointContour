#include "curveNet.h"

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
}

void CurveNet::startPath(const vec3d& st)
{
    nodes.push_back(st);
    nodesStat.push_back(true);
    numNodes++;
    edges.resize(numNodes);
    edges[numNodes - 1].clear();
}

void CurveNet::extendPath(const vec3d& st , const vec3d& ed ,
    const Path& path , bool newNode)
{
    BSpline bsp;
    extendPath(st , ed , path , newNode , bsp);
}

void CurveNet::extendPath(const vec3d& st , const vec3d& ed ,
    const Path& path , bool newNode , const BSpline& bsp)
{
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
    polyLinesIndex.push_back(PolyLineIndex(st_ni , edges[st_ni].size() ,
            ed_ni , edges[ed_ni].size()));
    // bsplines
    bsplines.push_back(bsp);
    // edges
    edges[st_ni].push_back(CurveEdge(ed_ni , numPolyLines));
    edges[ed_ni].push_back(CurveEdge(st_ni , numPolyLines));
    numPolyLines++;

    if (bsp.ctrlNodes.size() == 0) return;
    
    printf("===== merge extend =====\n");
    for (int i = 0; i < (int)bsp.ctrlNodes.size() - 1; i++)
    {
        collinearSet.makeSet(numPolyLines - 1 , i);
        for (int j = 0; j < numPolyLines; j++)
        {
            for (int k = 0; k < (int)bsplines[j].ctrlNodes.size() - 1; k++)
            {
                if (j == numPolyLines - 1 && k >= i) break;
                if (collinearSet.sameRoot(numPolyLines - 1 , i , j , k)) continue;
                if (checkCollinear(bsp.ctrlNodes[i] , bsp.ctrlNodes[i + 1] ,
                        bsplines[j].ctrlNodes[k] , bsplines[j].ctrlNodes[k + 1] , 0.05))
                {
                    printf("merge: (%d , %d) , (%d , %d)\n" , numPolyLines - 1 , i ,
                        j , k);
                    /*
                    vec3d v1 = bsp.ctrlNodes[i + 1] - bsp.ctrlNodes[i];
                    v1 /= v1.length();
                    vec3d v2 = bsplines[j].ctrlNodes[k + 1] - bsplines[j].ctrlNodes[k];
                    v2 /= v2.length();
                    printf("(%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n(%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" , bsp.ctrlNodes[i].x , bsp.ctrlNodes[i].y , bsp.ctrlNodes[i].z , bsp.ctrlNodes[i + 1].x , bsp.ctrlNodes[i + 1].y , bsp.ctrlNodes[i + 1].z , v1.x , v1.y , v1.z , bsplines[j].ctrlNodes[k].x , bsplines[j].ctrlNodes[k].y , bsplines[j].ctrlNodes[k].z , bsplines[j].ctrlNodes[k + 1].x , bsplines[j].ctrlNodes[k + 1].y , bsplines[j].ctrlNodes[k + 1].z , v2.x , v2.y , v2.z);
                    */
                    collinearSet.merge(numPolyLines - 1 , i , j , k);
                }
            }
        }
    }
}

void CurveNet::breakPath(const int& breakLine , const int& breakPoint)
{
    PolyLineIndex index = polyLinesIndex[breakLine];
    int st_ni = index.ni[0] , st_ei = index.ei[0];
    int ed_ni = index.ni[1] , ed_ei = index.ei[1];
    
    vec3d breakPos = polyLines[breakLine][breakPoint];
    int mid_ni = getNodeIndex(breakPos);

    if (!isEqual(nodes[st_ni] , polyLines[breakLine][0]))
    {
        std::swap(st_ni , ed_ni);
        std::swap(st_ei , ed_ei);
    }
    
    Path path = polyLines[breakLine];
    
    // "left" part
    polyLines[breakLine].erase(polyLines[breakLine].begin() + breakPoint + 1 ,
                               polyLines[breakLine].end());
    edges[st_ni][st_ei] = CurveEdge(mid_ni , breakLine);
    edges[mid_ni].push_back(CurveEdge(st_ni , breakLine));
    polyLinesIndex[breakLine].ni[0] = st_ni;
    polyLinesIndex[breakLine].ei[0] = st_ei;
    polyLinesIndex[breakLine].ni[1] = mid_ni;
    polyLinesIndex[breakLine].ei[1] = edges[mid_ni].size();
    
    // "right" part
    path.erase(path.begin() , path.begin() + breakPoint);
    polyLines.push_back(path);
    edges[mid_ni].push_back(CurveEdge(ed_ni , numPolyLines));
    edges[ed_ni][ed_ei] = CurveEdge(mid_ni , numPolyLines);
    polyLinesIndex.push_back(PolyLineIndex(mid_ni , edges[mid_ni].size() - 1 ,
                                           ed_ni , ed_ei));
    numPolyLines++;

    // recalculate both B-Splines
    if (bsplines[breakLine].ctrlNodes.size() > 0)
    {
        convert2Spline(polyLines[breakLine] , bsplines[breakLine]);
        BSpline bsp;
        convert2Spline(polyLines[numPolyLines - 1] , bsp);
        bsplines.push_back(bsp);
        
        printf("===== merge split ======\n");
        for (int t = 0; t < 2; t++)
        {
            BSpline bsp;
            int bspIndex;
            if (t == 0)
            {
                bsp = bsplines[breakLine];
                bspIndex = breakLine;
            }
            else
            {
                bsp = bsplines[numPolyLines - 1];
                bspIndex = numPolyLines - 1;
            }
            for (int i = 0; i < (int)bsp.ctrlNodes.size() - 1; i++)
            {
                collinearSet.makeSet(bspIndex , i);
                for (int j = 0; j < numPolyLines; j++)
                {
                    for (int k = 0; k < (int)bsplines[j].ctrlNodes.size() - 1; k++)
                    {
                        if (j == bspIndex && k >= i) break;
                        if (t == 0 && j == numPolyLines - 1) break;
                        if (collinearSet.sameRoot(numPolyLines - 1 , i , j , k)) continue;
                        if (checkCollinear(bsp.ctrlNodes[i] , bsp.ctrlNodes[i + 1] ,
                                bsplines[j].ctrlNodes[k] , bsplines[j].ctrlNodes[k + 1] , 0.05))
                        {
                            printf("merge: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                                j , k);
                            /*
                            vec3d v1 = bsp.ctrlNodes[i + 1] - bsp.ctrlNodes[i];
                            v1 /= v1.length();
                            vec3d v2 = bsplines[j].ctrlNodes[k + 1] - bsplines[j].ctrlNodes[k];
                            v2 /= v2.length();
                            printf("(%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n(%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" , bsp.ctrlNodes[i].x , bsp.ctrlNodes[i].y , bsp.ctrlNodes[i].z , bsp.ctrlNodes[i + 1].x , bsp.ctrlNodes[i + 1].y , bsp.ctrlNodes[i + 1].z , v1.x , v1.y , v1.z , bsplines[j].ctrlNodes[k].x , bsplines[j].ctrlNodes[k].y , bsplines[j].ctrlNodes[k].z , bsplines[j].ctrlNodes[k + 1].x , bsplines[j].ctrlNodes[k + 1].y , bsplines[j].ctrlNodes[k + 1].z , v2.x , v2.y , v2.z);
                            */
                            collinearSet.merge(bspIndex , i , j , k);
                        }
                    }
                }
            }
        }
    }
}

void CurveNet::deletePath(const int& deleteLine)
{
    polyLines[deleteLine].clear();
    bsplines[deleteLine].clear();
    
    PolyLineIndex& index = polyLinesIndex[deleteLine];
    int st_ni = index.ni[0] , st_ei = index.ei[0];
    int ed_ni = index.ni[1] , ed_ei = index.ei[1];
    edges[st_ni][st_ei] = CurveEdge(-1 , -1);
    edges[ed_ni][ed_ei] = CurveEdge(-1 , -1);

    index.ni[0] = index.ni[1] = index.ei[0] = index.ei[1] = -1;

    if (linkNoEdges(st_ni)) nodesStat[st_ni] = false;
    if (linkNoEdges(ed_ni)) nodesStat[ed_ni] = false;
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

bool CurveNet::checkCollinear(const vec3d& x1 , const vec3d& y1 ,
    const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    vec3d v1 , v2;
    /*
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
    */
    v1 = y1 - x1;
    v1.normalize();
    v2 = y2 - x2;
    v2.normalize();
    if (1.0 - std::abs(v1.dot(v2)) > threshold) return false;
    
    vec3d denom = (y1 - x1).cross(y2 - x2);
    vec3d numer = (x2 - x1).dot(denom);
    double d = numer.length() / denom.length();
    // printf("dist = %.6f\n" , d);
    if (d > 0.01) return false;
    
    return true;
}

void CurveNet::test()
{
    clear();
    startPath(vec3d(0 , 0 , 0));
    Path path;
    for (int i = 0; i <= 10; i++)
    {
        path.push_back(vec3d(i , 0 , 0));
    }
    extendPath(vec3d(0 , 0 , 0) , vec3d(10 , 0 , 0) , path , true);
    
    startPath(vec3d(5 , 2 , 0));
    path.clear();
    for (int i = 2; i >= 0; i--)
    {
        path.push_back(vec3d(5 , i , 0));
    }
    extendPath(vec3d(5 , 2 , 0) , vec3d(5 , 0 , 0) , path , true);
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