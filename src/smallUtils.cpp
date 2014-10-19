#include "smallUtils.h"

Engine *ep;

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

    delete[] coors;
    mxDestroyArray(pts);
    mxDestroyArray(res);
    mxDestroyArray(ctrlNodes);
    mxDestroyArray(knots);
}

bool restartLog(std::string fileName)
{
    strcpy(logFileName , fileName.c_str());
    FILE *fp = fopen(logFileName , "w");
    if (!fp)
    {
        fprintf(stderr , "ERROR: could not open LOG_FILE %s\n" , logFileName);
        return false;
    }
    fclose(fp);
    return true;
}

bool writeLog(const char* msg , ...)
{
    va_list argptr;
    FILE *fp = fopen(logFileName , "a");
    if (!fp)
    {
        fprintf(stderr , "ERROR: could not open LOG_FILE %s\n" , logFileName);
        return false;
    }
    va_start(argptr , msg);
    vfprintf(fp , msg , argptr);
    va_end(argptr);
    fclose(fp);
    return true;
}

double point2double(const vec3d& p)
{
	double res = 0;
	for (int i = 0; i < 3; i++) 
	{
		res += ((double)p[i] * 50.0 + 500.0) * pow2[i];
	}
	return res;
}

bool isEqual(const vec3d& lhs , const vec3d& rhs)
{
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(lhs[i] - rhs[i]) > EPS)
            return false;
    }
    return true;
}

void writeBMP(const char* filename , int w , int h ,
              unsigned char* rgb)
{
    FILE *f;
    int filesize = 54 + 3 * w * h;  //w is your image width, h is image height, both int

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(w);
    bmpinfoheader[ 5] = (unsigned char)(w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(w>>16);
    bmpinfoheader[ 7] = (unsigned char)(w>>24);
    bmpinfoheader[ 8] = (unsigned char)(h);
    bmpinfoheader[ 9] = (unsigned char)(h>> 8);
    bmpinfoheader[10] = (unsigned char)(h>>16);
    bmpinfoheader[11] = (unsigned char)(h>>24);

    f = fopen(filename , "wb");
    fwrite(bmpfileheader,1,14,f);
    fwrite(bmpinfoheader,1,40,f);
    for(int i=0;i<w*h*3;i++)
    {
        fputc(rgb[i], f);
    }
    fclose(f);
}