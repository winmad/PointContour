#include "smallUtils.h"

Engine *ep;

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
		res += ((double)p[i] * 49.9 + 50.0) * pow2[i];
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

bool isValid(const vec3d& pos)
{
    return pos.x > -1e5;
}

void setNull(vec3d& pos)
{
    pos.x = -1e10;
}

bool isSameCycle(std::vector<unsigned>& c1 , std::vector<unsigned>& c2)
{
    if (c1.size() != c2.size()) return false;
    for (int i = 0; i < c1.size(); i++)
    {
        if (c1[i] != c2[i]) return false;
    }
    return true;
}

bool isSameCycleGroup(CycleGroup& c1 , CycleGroup& c2)
{
    if (c1.size() != c2.size()) return false;
    for (int i = 0; i < c1.size(); i++)
    {
        if (!isSameCycle(c1[i] , c2[i])) return false;
    }
    return true;
}

vec3d pointScaling(const vec3d& pos , const vec3d& origin , int chosenAxis , double scale)
{
    vec3d res;
    vec3d dir = pos - origin;
    for (int i = 0; i < 3; i++)
    {
        if (chosenAxis == i)
        {
            res[i] = origin[i] + dir[i] * scale;
        }
        else
        {
            res[i] = pos[i];
        }
    }
    return res;
}

double weightBetweenSegs(const vec3d& x1 , const vec3d& y1 ,
	const vec3d& x2 , const vec3d& y2)
{
	return std::exp(-SQR(((x1 + y1) * 0.5 - (x2 + y2) * 0.5).length() * 3));
}

bool isIntersected2D(const vec2d& u1 , const vec2d& u2 , const vec2d& v1 , const vec2d& v2)
{
    return (std::max(u1.x , u2.x) >= std::min(v1.x , v2.x) &&
        std::max(v1.x , v2.x) >= std::min(u1.x , u2.x) &&
        std::max(u1.y , u2.y) >= std::min(v1.y , v2.y) &&
        std::max(v1.y , v2.y) >= std::min(u1.y , u2.y) &&
        (v1 - u1).cross(u2 - u1) * (v2 - u1).cross(u2 - u1) < 0 &&
        (u1 - v1).cross(v2 - v1) * (u2 - v1).cross(v2 - v1) < 0);
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