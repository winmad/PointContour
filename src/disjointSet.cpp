#include "disjointSet.h"

void DisjointSet::clear()
{
    numCurves = 0;
    curveDict.clear();
    curveId.clear();
    curveColor.clear();
}

void DisjointSet::makeSet(int bspIndex , int curveIndex)
{
    int x = bspIndex * 1000 + curveIndex;
    curveDict[x] = numCurves++;
    curveId.push_back(x);
    curveColor.push_back(numCurves - 1);
    rank.push_back(0);
}

std::pair<int , int> DisjointSet::find(int bspIndex , int curveIndex)
{
    int x = forHash(bspIndex , curveIndex);
    std::pair<int , int> paCurve(bspIndex , curveIndex);
    if (x != curveColor[x])
    {
        paCurve = backHash(curveColor[x]);
        paCurve = find(paCurve.first , paCurve.second);
        curveColor[x] = forHash(paCurve.first , paCurve.second);
    }
    return paCurve;
}

bool DisjointSet::sameRoot(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2)
{
    std::pair<int , int> xRootCurve = find(bspIndex1 , curveIndex1);
    std::pair<int , int> yRootCurve = find(bspIndex2 , curveIndex2);
    int xRoot = forHash(xRootCurve.first , xRootCurve.second);
    int yRoot = forHash(yRootCurve.first , yRootCurve.second);
    
    if (xRoot == yRoot)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void DisjointSet::merge(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2)
{
    std::pair<int , int> xRootCurve = find(bspIndex1 , curveIndex1);
    std::pair<int , int> yRootCurve = find(bspIndex2 , curveIndex2);
    int xRoot = forHash(xRootCurve.first , xRootCurve.second);
    int yRoot = forHash(yRootCurve.first , yRootCurve.second);
    
    if (xRoot == yRoot) return;

    if (rank[xRoot] < rank[yRoot])
    {
        curveColor[xRoot] = yRoot;
    }
    else if (rank[xRoot] > rank[yRoot])
    {
        curveColor[yRoot] = xRoot;
    }
    else
    {
        curveColor[yRoot] = xRoot;
        rank[xRoot]++;
    }
}

int DisjointSet::forHash(int bspIndex , int curveIndex)
{
    return curveDict[bspIndex * 1000 + curveIndex];
}

std::pair<int , int> DisjointSet::backHash(int index)
{
    int x = curveId[index];
    return std::make_pair(x / 1000 , x % 1000);
}

void DisjointSet::printLog()
{
    writeLog("\n--------- num of curves = %d ---------\n" , numCurves);
    for (int i = 0; i < curveId.size(); i++)
    {
        std::pair<int , int> pa = backHash(curveColor[i]);
        pa = find(pa.first , pa.second);
        curveColor[i] = forHash(pa.first , pa.second);
        writeLog("curveId = %d, root = %d, rank = %d\n" , curveId[i] , curveColor[i] , rank[i]);
    }
}

void DisjointSet::test()
{
    clear();
    makeSet(0 , 0); makeSet(0 , 1); makeSet(0 , 2);
    makeSet(1 , 0); makeSet(1 , 1);
    printLog();

    merge(0 , 0 , 0 , 1);
    printLog();

    merge(1 , 0 , 1 , 1);
    printLog();

    merge(1 , 1 , 0 , 0);
    printLog();

    merge(1 , 1 , 0 , 2);
    printLog();
}