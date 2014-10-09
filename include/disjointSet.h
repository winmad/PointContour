#ifndef DISJOINT_SET_H
#define DISJOINT_SET_H

#include "smallUtils.h"
#include <map>
#include <utility>
#include <vector>

class DisjointSet
{
public:
    void clear();
    void makeSet(int bspIndex , int curveIndex);
    std::pair<int , int> find(int bspIndex , int curveIndex);
    bool sameRoot(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2);
    void merge(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2);

    int forHash(int bspIndex , int curveIndex);
    std::pair<int , int> backHash(int index);

    int numCurves;
    std::map<int , int> curveDict;
    std::vector<int> curveId;
    std::vector<int> curveColor;
    std::vector<int> rank;

    // for debug
    void printLog();
    void test();
};

#endif