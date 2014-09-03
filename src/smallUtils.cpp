#include "smallUtils.h"

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
		res += ((double)p[i] + 500.0) * pow2[i];
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