#include "splineUtils.h"

void BSpline::calcCoefs()
{
    if (ctrlNodes.size() <= 2) return;

    newCoefs();
    double *tmp = new double[N * K];
    
    mxArray *m_knots = NULL;
    m_knots = mxCreateDoubleMatrix(1 , knots.size() , mxREAL);
    for (int i = 0; i < knots.size(); i++)
    {
        tmp[i] = knots[i];
    }
    memcpy((double*)mxGetPr(m_knots) , tmp , sizeof(double) * knots.size());
    engPutVariable(ep , "knots" , m_knots);

    mxArray *m_t = NULL;
    m_t = mxCreateDoubleMatrix(1 , t.size() , mxREAL);
    for (int i = 0; i < t.size(); i++)
    {
        tmp[i] = t[i];
    }
    memcpy((double*)mxGetPr(m_t) , tmp , sizeof(double) * t.size());
    engPutVariable(ep , "t" , m_t);

    mxArray *m_coefs = NULL;
    engEvalString(ep , "coefs = calcCoefs(knots , t);");
    m_coefs = engGetVariable(ep , "coefs");
    memcpy(tmp , (double*)mxGetPr(m_coefs) , sizeof(double) * N * K);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < K; j++)
        {
            coefs[i][j] = tmp[K * i + j];
        }
    }
    /*
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < K; j++)
        {
            writeLog("%.6f " , coefs[i][j]);
        }
        writeLog("\n");
    }
    */
    delete[] tmp;
    mxDestroyArray(m_knots);
    mxDestroyArray(m_coefs);
    mxDestroyArray(m_t);
}

int BSpline::getCtrlNodeIndex(const vec3d& pos)
{
    for (int i = 0; i < ctrlNodes.size(); i++)
    {
        if (isEqual(pos , ctrlNodes[i])) return i;
    }
    return -1;
}

void BSpline::updateBSpline(int index , const vec3d& newPos)
{
    // if (index < 0 || index >= ctrlNodes.size())
    // {
        // printf("update BSpline error: index out of range\n");
        // return;
    // }
    ctrlNodes[index] = newPos;
}

void convert2Spline(Path& path , BSpline& bsp)
{
    int N = path.size();
    if (N <= 2) return;
    mxArray *pts = NULL;
    pts = mxCreateDoubleMatrix(3 , N , mxREAL);
    double *coors = NULL;
    coors = new double[N * 3];
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            coors[3 * i + j] = path[i][j];
        }
    }

    memcpy((double*)mxGetPr(pts) , coors , sizeof(double) * N * 3);
    engPutVariable(ep , "pts" , pts);
    engEvalString(ep , "[bsp , p] = convert2Spline(pts);");
    mxArray *ctrlNodes = NULL;
    engEvalString(ep , "ctrlNodes = bsp.coefs;");
    ctrlNodes = engGetVariable(ep , "ctrlNodes");
    int numCtrlNodes;
    numCtrlNodes = mxGetN(ctrlNodes);
    mxArray *res = NULL;
    res = engGetVariable(ep , "p");
    
    memcpy(coors , (double*)mxGetPr(res) , sizeof(double) * N * 3);
    for (int i = 1; i < N - 1; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            path[i][j] = coors[3 * i + j];
        }
    }

    bsp.clear();
    double totLen = 0;
    bsp.t.push_back(0);
    for (int i = 1; i < N; i++)
    {
        double len = (path[i] - path[i - 1]).length();
        totLen += len;
        bsp.t.push_back(totLen);
    }
    for (int i = 0; i < N; i++) bsp.t[i] /= totLen;
    
    memcpy(coors , (double*)mxGetPr(ctrlNodes) , sizeof(double) * numCtrlNodes * 3);
    for (int i = 0; i < numCtrlNodes; i++)
    {
        vec3d p(coors[3 * i] , coors[3 * i + 1] , coors[3 * i + 2]);
        bsp.ctrlNodes.push_back(p);
    }

    mxArray *knots = NULL;
    engEvalString(ep , "knots = bsp.knots;");
    knots = engGetVariable(ep , "knots");
    int numKnots = mxGetN(knots);
    memcpy(coors , (double*)mxGetPr(knots) , sizeof(double) * numKnots);
    for (int i = 0; i < numKnots; i++)
    {
        bsp.knots.push_back(coors[i]);
    }

    bsp.N = path.size();
    bsp.K = numCtrlNodes;
    
    delete[] coors;
    mxDestroyArray(pts);
    mxDestroyArray(res);
    mxDestroyArray(ctrlNodes);
    mxDestroyArray(knots);
}

void resampleBsp(BSpline& bsp , Path& path)
{
    if (bsp.ctrlNodes.size() == 2)
    {
        vec3d x1 = bsp.ctrlNodes[0];
        vec3d x2 = bsp.ctrlNodes[1];
        vec3d v = x2 - x1;
        double len = v.length();
        //v /= len;
        // printf("path size = %lu, t size = %lu\n" , path.size() , bsp.t.size());
        path[0] = x1;
        path[(int)path.size() - 1] = x2;
        for (int j = 1; j < (int)path.size() - 1; j++)
        {
            /*
            vec3d u = path[j] - x1;
            double proj = u.dot(v);
            proj = std::min(std::max(proj , 1e-5) , len - 1e-5);
            path[j] = x1 + v * proj;
            */
            path[j] = x1 + v * bsp.t[j];
        }
        return;
    }

    // call matlab
    double *coors = NULL;
    coors = new double[path.size() * 3];
    
    mxArray *ctrlNodes = NULL;
    ctrlNodes = mxCreateDoubleMatrix(3 , bsp.ctrlNodes.size() , mxREAL);
    for (int i = 0; i < bsp.ctrlNodes.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            coors[3 * i + j] = bsp.ctrlNodes[i][j];
        }
    }
    memcpy((double*)mxGetPr(ctrlNodes) , coors , sizeof(double) * bsp.ctrlNodes.size() * 3);
    engPutVariable(ep , "ctrlNodes" , ctrlNodes);
    
    mxArray *knots = NULL;
    knots = mxCreateDoubleMatrix(1 , bsp.knots.size() , mxREAL);
    for (int i = 0; i < bsp.knots.size(); i++)
    {
        coors[i] = bsp.knots[i];
    }
    memcpy((double*)mxGetPr(knots) , coors , sizeof(double) * bsp.knots.size());
    engPutVariable(ep , "knots" , knots);

    mxArray *t = NULL;
    t = mxCreateDoubleMatrix(1 , bsp.t.size() , mxREAL);
    for (int i = 0; i < bsp.t.size(); i++)
    {
        coors[i] = bsp.t[i];
    }
    memcpy((double*)mxGetPr(t) , coors , sizeof(double) * bsp.t.size());
    engPutVariable(ep , "t" , t);

    engEvalString(ep , "pts = resampleBsp(ctrlNodes , knots , t);");
    mxArray *pts = NULL;
    pts = engGetVariable(ep , "pts");
    memcpy(coors , (double*)mxGetPr(pts) , sizeof(double) * path.size() * 3);
    for (int i = 0; i < path.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            path[i][j] = coors[3 * i + j];
        }
    }

    delete[] coors;
    mxDestroyArray(ctrlNodes);
    mxDestroyArray(knots);
    mxDestroyArray(pts);
    mxDestroyArray(t);
}

void convert2Line(Path& path , BSpline& bsp)
{
    bsp.clear();
    bsp.ctrlNodes.push_back(path.front());
    bsp.ctrlNodes.push_back(path.back());

    double totLen = 0;
    bsp.t.push_back(0);
    for (int i = 1; i < path.size(); i++)
    {
        double len = (path[i] - path[i - 1]).length();
        totLen += len;
        bsp.t.push_back(totLen);
    }
    for (int i = 0; i < path.size(); i++) bsp.t[i] /= totLen;

    resampleBsp(bsp , path);
}