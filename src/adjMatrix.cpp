#include "adjMatrix.h"

AdjMatrix::AdjMatrix()
{
    maxN = 100;
    mat = new int*[maxN];
    for (int i = 0; i < maxN; i++) mat[i] = new int[maxN];
    numCurves = 0;
}

void AdjMatrix::clear()
{
    numCurves = 0;
    curveDict.clear();
    curveId.clear();
    for (int i = 0; i < maxN; i++)
    {
        for (int j = 0; j < maxN; j++) mat[i][j] = 0;
    }
}

void AdjMatrix::newCurve(int bspIndex , int curveIndex)
{
    int x = bspIndex * 1000 + curveIndex;
    curveDict[x] = numCurves++;
    curveId.push_back(x);
}

void AdjMatrix::connect(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2 ,
    int mark)
{
    int x = forHash(bspIndex1 , curveIndex1);
    int y = forHash(bspIndex2 , curveIndex2);
    mat[x][y] = mat[y][x] = mark;
}

int AdjMatrix::getMark(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2)
{
    int x = forHash(bspIndex1 , curveIndex1);
    int y = forHash(bspIndex2 , curveIndex2);
    return mat[x][y];
}

int AdjMatrix::forHash(int bspIndex , int curveIndex)
{
    return curveDict[bspIndex * 1000 + curveIndex];
}

std::pair<int , int> AdjMatrix::backHash(int index)
{
    int x = curveId[index];
    return std::make_pair(x / 1000 , x % 1000);
}
