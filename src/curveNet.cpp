#include "curveNet.h"

void CurveNet::clear()
{
    numNodes = numPolyLines = 0;
    nodes.clear();
    edges.clear();
    polyLines.clear();
    polyLinesIndex.clear();
}

void CurveNet::startPath(const vec3f& st)
{
    nodes.push_back(st);
    numNodes++;
    edges.resize(numNodes);
    edges[numNodes - 1].clear();
}

void CurveNet::extendPath(const vec3f& st , const vec3f& ed ,
                          const Path& path)
{
    int st_ni = getNodeIndex(st);
    int ed_ni = numNodes;
    
    // nodes
    nodes.push_back(ed);
    numNodes++;
    edges.resize(numNodes);
    edges[numNodes - 1].clear();
    
    // polyLines
    polyLines.push_back(path);
    polyLinesIndex.push_back(PolyLineIndex(st_ni , edges[st_ni].size() ,
                                      ed_ni , edges[ed_ni].size()));
    
    // edges
    edges[st_ni].push_back(CurveEdge(ed_ni , numPolyLines));
    edges[ed_ni].push_back(CurveEdge(st_ni , numPolyLines));
    numPolyLines++;
}

void CurveNet::breakPath(const int& breakLine , const int& breakPoint)
{
    PolyLineIndex index = polyLinesIndex[breakLine];
    int st_ni = index.ni[0] , st_ei = index.ei[0];
    int ed_ni = index.ni[1] , ed_ei = index.ei[1];
    
    vec3f breakPos = polyLines[breakLine][breakPoint];
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
}

int CurveNet::getNodeIndex(const vec3f& pos)
{
    for (int i = 0; i < numNodes; i++)
        if (isEqual(pos , nodes[i]))
            return i;
    return -1;
}

void CurveNet::test()
{
    clear();
    startPath(vec3f(0 , 0 , 0));
    Path path;
    for (int i = 0; i <= 10; i++)
    {
        path.push_back(vec3f(i , 0 , 0));
    }
    extendPath(vec3f(0 , 0 , 0) , vec3f(10 , 0 , 0) , path);
    
    startPath(vec3f(5 , 2 , 0));
    path.clear();
    for (int i = 2; i >= 0; i--)
    {
        path.push_back(vec3f(5 , i , 0));
    }
    extendPath(vec3f(5 , 2 , 0) , vec3f(5 , 0 , 0) , path);
    breakPath(0 , 5);
    
    debugLog();
}

void CurveNet::debugPrint()
{
    printf("============ nodes list =============\n");
    for (int i = 0; i < numNodes; i++)
    {
        printf("node %d: (%.6f,%.6f,%.6f)\n" , i , nodes[i].x , nodes[i].y ,
                 nodes[i].z);
    }
    printf("========== poly lines list ==========\n");
    for (int i = 0; i < numPolyLines; i++)
    {
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
        writeLog("node %d: (%.6f,%.6f,%.6f)\n" , i , nodes[i].x , nodes[i].y ,
                 nodes[i].z);
    }
    writeLog("========== poly lines list ==========\n");
    for (int i = 0; i < numPolyLines; i++)
    {
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
