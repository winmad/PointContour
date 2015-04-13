#include "adjMatrix.h"

AdjMatrix::AdjMatrix()
{
    maxN = 800;
    // mat = new int*[maxN];
    // for (int i = 0; i < maxN; i++) mat[i] = new int[maxN];
    mat.resize(maxN);
    for (int i = 0; i < maxN; i++) mat[i].resize(maxN);
    clear();
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
    int x = bspIndex * 10000 + curveIndex;
    curveDict[x] = numCurves++;
    curveId.push_back(x);
}

void AdjMatrix::connect(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2 ,
    int mark)
{
    int x = forHash(bspIndex1 , curveIndex1);
    int y = forHash(bspIndex2 , curveIndex2);
    /*
    printf("(%d,%d), %d<->(%d,%d), %d=%d\n" , bspIndex1 , curveIndex1 , x ,
        bspIndex2 , curveIndex2 , y ,
        mark);
    */
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
    return curveDict[bspIndex * 10000 + curveIndex];
}

std::pair<int , int> AdjMatrix::backHash(int index)
{
    int x = curveId[index];
    return std::make_pair(x / 10000 , x % 10000);
}

void AdjMatrix::printLog()
{
    writeLog("====================\n");
    for (int i = 0; i < curveId.size(); i++)
    {
        writeLog("%d " , curveId[i]);
    }
    writeLog("\n");
    for (std::map<int , int>::iterator it = curveDict.begin(); it != curveDict.end(); it++)
    {
        writeLog("%d %d\n" , it->first , it->second);
    }
    for (int i = 0; i < 40; i++)
    {
        for (int j = 0; j < 40; j++) writeLog("%d " , mat[i][j]);
        writeLog("\n");
    }
    writeLog("\n");
}