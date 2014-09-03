#ifndef CURVE_NET_H
#define CURVE_NET_H

#include "smallUtils.h"

struct PolyLineIndex
{
    PolyLineIndex() {}
    PolyLineIndex(int _ni0 , int _ei0 , int _ni1 , int _ei1)
    {
        ni[0] = _ni0; ei[0] = _ei0;
        ni[1] = _ni1; ei[1] = _ei1;
    }
    int ni[2] , ei[2];
};

class CurveEdge
{
public:
    CurveEdge() {}
    CurveEdge(int _link , int _pli) :
        link(_link) , pli(_pli) {}
    /*
    CurveEdge(const CurveEdge& edge)
    {
        link = edge.link;
        polyLine = edge.polyLine;
    }
    
    CurveEdge& operator =(const CurveEdge& edge)
    {
        link = edge.link;
        polyLine = edge.polyLine;
    }
    */
    int link;
    int pli;
};

class CurveNet
{
public:
    CurveNet() { clear(); }
    
    void clear();
    void startPath(const vec3f& st);
    void extendPath(const vec3f& st , const vec3f& ed , const Path& path);
    void breakPath(const int& breakLine , const int& breakPoint);
    
    int getNodeIndex(const vec3f& pos);
    
    void test();
    void debugPrint();
    void debugLog();
    
    int numNodes , numPolyLines;
    std::vector<vec3f> nodes;
    std::vector<std::vector<CurveEdge> > edges;
    std::vector<Path> polyLines;
    std::vector<PolyLineIndex> polyLinesIndex;
};

#endif