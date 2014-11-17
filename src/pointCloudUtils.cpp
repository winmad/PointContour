#include "pointCloudUtils.h"
#include "nvVector.h"
#include "TimeManager.h"
#include "curveNet.h"
#include <omp.h>
#include <queue>
#include <algorithm>
#include <wx/filename.h>

PointCloudUtils::PointCloudUtils()
{
	pcRenderer = new PointCloudRenderer();
    curveNet = new CurveNet();
    pcRenderer->pcUtils = this;
    pcRenderer->dispCurveNet = this->curveNet;
	isPointInside = NULL;
	originF = NULL;
	f = NULL;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ddf[i][j] = NULL;
	tensor = NULL;

	tree = NULL;

    KNN = 50;

    restartLog("debug.txt");
    if (!(ep = engOpen("")))
    {
        fprintf(stderr , "\nCan't start MATLAB engine\n");
    }
    else
    {
        fprintf(stderr , "\nStart MATLAB successfully\n");
    }
#ifdef _WIN32
	//engEvalString(ep , "addpath D:\\fz\\point_cloud\\PointContour\\matlab");
	engEvalString(ep , "addpath Y:\\Projects\\PointContour\\matlab");
#else
    engEvalString(ep , "addpath ~/Projects/PointContour/matlab/");
#endif

    //cycle::cycleTest();
    /*
    if (!mclInitializeApplication(NULL,0))
    {
        fprintf(stderr , "\nmcl initialized error\n");
    }
    else
    {
        fprintf(stderr , "\nmcl init successfully");
    }
    
    if (!libbsInitialize())
    {
        fprintf(stderr , "\nlibbs initialized error\n");
    }
    else
    {
        fprintf(stderr , "\nlibbs init successfully");
    }
    */
}

PointCloudUtils::~PointCloudUtils()
{
    //libbsTerminate();
    //mclTerminateApplication();
    if (pcRenderer != NULL)
		delete pcRenderer;
	if (tree != NULL)
		delete tree;
}

void PointCloudUtils::allocateMemory(vec3i resol , int extra)
{
	timer.PushCurrentTime();
	vec3i resolX = resol + vec3i(1);
	vec3i resolExt = resol + vec3i(1) + vec3i(extra) * 2;
	allocate3(isPointInside , resolX);

	allocate3(originF , resolExt);

	for (int i = 0; i < 3; i++)
		for (int j = i; j < 3; j++)
			allocate3(ddf[i][j] , resolExt);

	allocate3(tensor , resolExt);

	timer.PopAndDisplayTime("\nAllocate memory: %.6lf\n");
}

void PointCloudUtils::deallocateMemory(vec3i resol , int extra)
{
	timer.PushCurrentTime();

	vec3i resolX = resol + vec3i(1);
	vec3i resolExt = resol + vec3i(1) + vec3i(extra) * 2;
        
	deallocate3(isPointInside , resolX);

	deallocate3(originF , resolExt);
	deallocate3(f , resolExt);

	for (int i = 0; i < 3; i++)
		for (int j = i; j < 3; j++)
			deallocate3(ddf[i][j] , resolExt);

	deallocate3(tensor , resolExt);

	timer.PopAndDisplayTime("\nDeallocate memory: %.6lf\n");
}

void PointCloudUtils::init()
{
	timer.PushCurrentTime();
    
    initialized = true;
	getBBox();
	pcRenderer->init();
	if (tree != NULL)
		delete tree;
	tree = new PointKDTree<Data>(pcData);

	graphType = POINT_GRAPH;
    metricType = MIN_CURVATURE;

	nodes = edges = 0;

	timer.PopAndDisplayTime("\nBuild kdtree: %.6lf\n");
    
    curveNet->clear();
}

void PointCloudUtils::preprocess(const int& _gridResX , const int& _gridResY , const int& _gridResZ , 
								 const int& _extNum , const int& _filterRadius , 
								 const double& _alpha1 , const double& _alpha2 ,
                                 const double& _alphaN)
{
	// phase 0
	deallocateMemory(gridRes , extNum);

	extNum = _extNum;
	buildUniformGrid(vec3i(_gridResX , _gridResY , _gridResZ));

    allocateMemory(gridRes , extNum);

	// phase 1
	calcDistField();

	// phase 2
	gaussianSmooth(originF , f , _filterRadius / 3.0);

	// phase 3
	calcHessian(f);
	alpha1 = _alpha1; alpha2 = _alpha2;
    alphaN = _alphaN;
	calcMetric(f);

	// phase 4
    if (graphType == POINT_GRAPH)
	{
		calcPointTensor();
		buildGraphFromPoints();
	}
}

void PointCloudUtils::getBBox()
{
	box.lb = vec3d(1e20 , 1e20 , 1e20);
	box.rt = -box.lb;
	for (int i = 0; i < pcData.size(); i++)
	{
		box.lb.x = std::min(box.lb.x , pcData[i].pos.x);
		box.lb.y = std::min(box.lb.y , pcData[i].pos.y);
		box.lb.z = std::min(box.lb.z , pcData[i].pos.z);

		box.rt.x = std::max(box.rt.x , pcData[i].pos.x);
		box.rt.y = std::max(box.rt.y , pcData[i].pos.y);
		box.rt.z = std::max(box.rt.z , pcData[i].pos.z);
	}

    // rescale
    vec3d diag = box.rt - box.lb;
    double scale = 0.8 / std::min(diag.x , std::min(diag.y , diag.z));
    box.lb *= scale;
    box.rt *= scale;
    for (int i = 0; i < pcData.size(); i++)
    {
        pcData[i].pos *= scale;
    }
}

void PointCloudUtils::buildUniformGrid(vec3i size)
{
	gridRes = size;
	gridResx = gridRes + vec3i(1 , 1 , 1);
	vec3d diag = box.rt - box.lb;

	gridSize.x = diag.x / size.x;
	gridSize.y = diag.y / size.y;
	gridSize.z = diag.z / size.z;

	xval.resize(gridResx.x); yval.resize(gridResx.y); zval.resize(gridResx.z);
	xval[0] = box.lb.x;
	yval[0] = box.lb.y;
	zval[0] = box.lb.z;
	for (int i = 1; i <= gridRes.x; i++)
		xval[i] = xval[i - 1] + gridSize.x;
	for (int i = 1; i <= gridRes.y; i++)
		yval[i] = yval[i - 1] + gridSize.y;
	for (int i = 1; i <= gridRes.z; i++)
		zval[i] = zval[i - 1] + gridSize.z;
}

void PointCloudUtils::calcDistField()
{
	timer.PushCurrentTime();

	sizeOriginF = gridResx + vec3i(extNum , extNum , extNum) * 2;
	extXval.resize(sizeOriginF.x); extYval.resize(sizeOriginF.y); extZval.resize(sizeOriginF.z);
	extXval[0] = box.lb.x - extNum * gridSize.x;
	extYval[0] = box.lb.y - extNum * gridSize.y;
	extZval[0] = box.lb.z - extNum * gridSize.z;
	for (int i = 1; i < sizeOriginF.x; i++)
		extXval[i] = extXval[i - 1] + gridSize.x;
	for (int i = 1; i < sizeOriginF.y; i++)
		extYval[i] = extYval[i - 1] + gridSize.y;
	for (int i = 1; i < sizeOriginF.z; i++)
		extZval[i] = extZval[i - 1] + gridSize.z;

#pragma omp parallel for
	for (int i = 0; i < sizeOriginF.x; i++)
	{
		for (int j = 0; j < sizeOriginF.y; j++)
		{
			for (int k = 0; k < sizeOriginF.z; k++)
			{
				DistQuery q;
				q.maxSqrDis = 1e30;
				vec3d pos(extXval[i] , extYval[j] , extZval[k]);
				tree->searchKnn(0 , pos , q);
				originF[i][j][k] = sqrt(q.maxSqrDis);
			}
		}
	}

	timer.PopAndDisplayTime("\nCalculate distance field: %.6lf\n");
	/*
	for (int i = 0; i < sizeOriginF.x; i++)
		for (int j = 0; j < sizeOriginF.y; j++)
			for (int k = 0; k < sizeOriginF.z; k++)
				fprintf(fp , "(%.6lf,%.6lf,%.6lf) , f = %.6lf\n" , extXval[i] , extYval[j] , extZval[k] , originF[i][j][k]);
	*/
	//extXval.clear(); extYval.clear(); extZval.clear();
}

void PointCloudUtils::gaussianSmooth(double*** &origin , double*** &f , double stddev)
{
	if (std::abs(stddev) < 1e-6)
	{
		allocate3(f , sizeOriginF);
		for (int i = 0; i < sizeOriginF.x; i++)
			for (int j = 0; j < sizeOriginF.y; j++)
				for (int k = 0; k < sizeOriginF.z; k++)
					f[i][j][k] = originF[i][j][k];
		return;
	}

	timer.PushCurrentTime();

	int diameter = ceil(stddev * 6.0);
	int radius = std::max(diameter / 2 , 1);

	std::vector<double> w;
	double sumw = 0;
	for (int i = 0; i <= radius; i++)
	{
		w.push_back(exp(-(i * i) / (2 * stddev * stddev)) / sqrt(2 * PI * stddev * stddev));
		sumw += w[i];
	}
	sumw = 2 * sumw + w[0];
	for (int i = 0; i <= radius; i++)
		w[i] /= sumw;
	
	double*** sf[2];
	sf[0] = NULL; sf[1] = NULL;
	allocate3(sf[0] , sizeOriginF);
	allocate3(sf[1] , sizeOriginF);

	int now = 1;
	for (int i = 0; i < sizeOriginF.x; i++)
		for (int j = 0; j < sizeOriginF.y; j++)
			for (int k = 0; k < sizeOriginF.z; k++)
				sf[1 - now][i][j][k] = originF[i][j][k];

#pragma omp parallel for
	for (int i = 0; i < sizeOriginF.x; i++)
	{
		for (int j = 0; j < sizeOriginF.y; j++)
		{
			for (int k = 0; k < sizeOriginF.z; k++)
			{
				sf[now][i][j][k] = 0;
				for (int ii = i - radius; ii <= i + radius; ii++)
				{
					int ai = std::max(ii , 0);
					ai = std::min(ai , sizeOriginF.x - 1);

					sf[now][i][j][k] += sf[1 - now][ai][j][k] * w[std::abs(ii - i)];
				}
			}
		}
	}
	now ^= 1;
#pragma omp parallel for
	for (int i = 0; i < sizeOriginF.x; i++)
	{
		for (int j = 0; j < sizeOriginF.y; j++)
		{
			for (int k = 0; k < sizeOriginF.z; k++)
			{
				sf[now][i][j][k] = 0;
				for (int jj = j - radius; jj <= j + radius; jj++)
				{
					int aj = std::max(jj , 0);
					aj = std::min(aj , sizeOriginF.y - 1);

					sf[now][i][j][k] += sf[1 - now][i][aj][k] * w[std::abs(jj - j)];
				}
			}
		}
	}
	now ^= 1;
#pragma omp parallel for
	for (int i = 0; i < sizeOriginF.x; i++)
	{
		for (int j = 0; j < sizeOriginF.y; j++)
		{
			for (int k = 0; k < sizeOriginF.z; k++)
			{
				sf[now][i][j][k] = 0;
				for (int kk = k - radius; kk <= k + radius; kk++)
				{
					int ak = std::max(kk , 0);
					ak = std::min(ak , sizeOriginF.z - 1);

					sf[now][i][j][k] += sf[1 - now][i][j][ak] * w[std::abs(kk - k)];
				}
			}
		}
	}
	f = sf[now];

	deallocate3(sf[1 - now] , sizeOriginF);

	timer.PopAndDisplayTime("\nGaussian smooth: %.6lf\n");
	/*
	for (int i = 0; i < sizeOriginF.x; i++)
		for (int j = 0; j < sizeOriginF.y; j++)
			for (int k = 0; k < sizeOriginF.z; k++)
				fprintf(fp , "(%.6lf,%.6lf,%.6lf) , f = %.6lf\n" , extXval[i] , extYval[j] , extZval[k] , f[i][j][k]);
	*/
}

void PointCloudUtils::calcSecondOrderDerivative(double*** &f , int d1 , int d2 , 
											    double*** &ddf)
{
	if (d1 > d2)
		std::swap(d1 , d2);

	if (d1 == 0 && d2 == 0)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i + 1][j][k] - 2 * f[i][j][k] + f[i - 1][j][k]) / (gridSize.x * gridSize.x);
}
	else if (d1 == 0 && d2 == 1)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i + 1][j + 1][k] - f[i + 1][j][k] - f[i][j + 1][k] + 2 * f[i][j][k] -
						f[i - 1][j][k] - f[i][j - 1][k] + f[i - 1][j - 1][k]) / (2.0 * gridSize.x * gridSize.y);
	}
	else if (d1 == 0 && d2 == 2)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i + 1][j][k + 1] - f[i + 1][j][k] - f[i][j][k + 1] + 2 * f[i][j][k] -
						f[i - 1][j][k] - f[i][j][k - 1] + f[i - 1][j][k - 1]) / (2.0 * gridSize.x * gridSize.z);
	}
	else if (d1 == 1 && d2 == 1)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i][j + 1][k] - 2 * f[i][j][k] + f[i][j - 1][k]) / (gridSize.y * gridSize.y);
	}
	else if (d1 == 1 && d2 == 2)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i][j + 1][k + 1] - f[i][j + 1][k] - f[i][j][k + 1] + 2 * f[i][j][k] -
						f[i][j - 1][k] - f[i][j][k - 1] + f[i][j - 1][k - 1]) / (2 * gridSize.y * gridSize.z);
	}
	else if (d1 == 2 && d2 == 2)
	{
#pragma omp parallel for
		for (int i = 1; i < sizeOriginF.x - 1; i++)
			for (int j = 1; j < sizeOriginF.y - 1; j++)
				for (int k = 1; k < sizeOriginF.z - 1; k++)
					ddf[i][j][k] = (f[i][j][k + 1] - 2 * f[i][j][k] + f[i][j][k - 1]) / (gridSize.z * gridSize.z);
	}
}

void PointCloudUtils::calcTensorDecomposition(Tensor& ts)
{
	Matrix3d hesMat = ts.hessian;
	SelfAdjointEigenSolver<Matrix3d> vd(hesMat);
	Vector3d egval = vd.eigenvalues();
	Matrix3d egvec = vd.eigenvectors();
	int t[3];
	t[0] = 0; t[1] = 1; t[2] = 2;

	for (int a = 0; a < 3; a++)
		for (int b = a + 1; b < 3; b++)
			if (std::abs(egval(t[a])) < std::abs(egval(t[b])))
				std::swap(t[a] , t[b]);

	for (int a = 0; a < 3; a++)
	{
		ts.eigenVal[a] = egval(t[a]);
		ts.axis[a] = vec3d(egvec(0 , t[a]) , egvec(1 , t[a]) , egvec(2 , t[a]));
	}
}

void PointCloudUtils::calcTensorMetric(Tensor& ts)
{
    double ev[3];
    ev[0] = std::abs(ts.eigenVal[0]);
    ev[1] = std::abs(ts.eigenVal[1]);
    ev[2] = std::abs(ts.eigenVal[2]);
    
	double s1 = ev[0] - ev[2];
	double s2 = ev[1] - ev[2];
	if (ev[0] < -EPS)
		s1 = 0.0;
	if (ev[1] < -EPS)
		s2 = 0.0;

    double scale = 1.0 / (1.0 + alphaN * ev[0] * ev[1] * ev[2]);

	ts.axisLen[0] = (1.0 + alpha1 * s1) * (1.0 + alpha2 * s2) * scale;
	ts.axisLen[1] = (1.0 + alpha2 * s2) / (1.0 + alpha1 * s1) * scale;
	ts.axisLen[2] = 1.0 / ((1.0 + alpha1 * s1) * (1.0 + alpha2 * s2)) * scale;
    //ts.axisLen[0] = 1.0 * scale;
    //ts.axisLen[1] = 1.0 / (1 + alpha1 * s1) * scale;
    //ts.axisLen[2] = 1.0 / ((1 + alpha1 * s1) * (1 + alpha2 * s2)) * scale;
    //ts.axisLen[0] = (1.0 + alpha1 * s1) * scale;
    //ts.axisLen[1] = (1.0 + alpha2 * s2) * scale;
    //ts.axisLen[2] = 1.0 * scale;

    if (metricType == MAX_CURVATURE)
    {
        std::swap(ts.axisLen[1] , ts.axisLen[2]);
    }
    
    Matrix3d vec , d;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            vec(j , i) = ts.axis[i][j];
            d(i , j) = 0;
        }
        d(i , i) = ts.axisLen[i];
    }
    ts.tensor = vec * d * vec.inverse();
    
    /*
    SelfAdjointEigenSolver<Matrix3d> vd(ts.tensor);
	Vector3d egval = vd.eigenvalues();
	Matrix3d egvec = vd.eigenvectors();
    for (int i = 0; i < 3; i++)
    {
        if (egval(i) < -EPS)
        {
            printf("error: %.8f\n" , egval(i));
        }
    }
    */
    /*
    Matrix3d mat = vec * d * vec.inverse();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            ts.tensor(i , j) = mat(i , j);
    */
    /*
    writeLog("================\n");
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            writeLog("%.6lf " , ts.tensor(i , j));
        writeLog("\n");
    }
    */
}

void PointCloudUtils::calcHessian(double*** &f)
{
	timer.PushCurrentTime();

#pragma omp parallel for
	for (int i = 0; i < 3; i++)
	{
		for (int j = i; j < 3; j++)
			calcSecondOrderDerivative(f , i , j , ddf[i][j]);
	}
/*
#pragma omp parallel for
	for (int i = 0; i <= gridRes.x; i++)
	{
		for (int j = 0; j <= gridRes.y; j++)
		{
			for (int k = 0; k <= gridRes.z; k++)
			{
				for (int r = 0; r < 3; r++)
					for (int c = 0; c < 3; c++)
						tensor[i][j][k].hessian(r , c) = ddf[std::min(r , c)][std::max(r , c)][i + extNum][j + extNum][k + extNum];

				calcTensorDecomposition(tensor[i][j][k]);
			}
		}
	}
*/
#pragma omp parallel for
    for (int i = 1; i < sizeOriginF.x - 1; i++)
    {
        for (int j = 1; j < sizeOriginF.y - 1; j++)
        {
            for (int k = 1; k < sizeOriginF.z - 1; k++)
            {
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        tensor[i][j][k].hessian(r , c) = ddf[std::min(r , c)][std::max(r , c)][i][j][k];
                
                calcTensorDecomposition(tensor[i][j][k]);
            }
        }
    }
	timer.PopAndDisplayTime("\nCalculate hessian: %.6lf\n");
}

void PointCloudUtils::calcMetric(double*** &f)
{
	timer.PushCurrentTime();
/*
#pragma omp parallel for
	for (int i = 0; i <= gridRes.x; i++)
	{
		for (int j = 0; j <= gridRes.y; j++)
		{
			for (int k = 0; k <= gridRes.z; k++)
			{
				calcTensorMetric(tensor[i][j][k]);
			}
		}
	}
*/
    
#pragma omp parallel for
	for (int i = 1; i < sizeOriginF.x - 1; i++)
	{
		for (int j = 1; j < sizeOriginF.y - 1; j++)
		{
			for (int k = 1; k < sizeOriginF.z - 1; k++)
			{
				calcTensorMetric(tensor[i][j][k]);
			}
		}
	}

	timer.PopAndDisplayTime("\nCalculate metric: %.6lf\n");

    /*
	int step = 1;
    int x = 4;//gridRes.x;
    int y = 4;//gridRes.y;
    int z = 4;//gridRes.z;
	for (int i = 0; i <= x; i+=step)
		for (int j = 0; j <= y; j+=step)
			for (int k = 0; k <= z; k+=step)
			{
				//if (!isPointInside[i][j][k])
				//	continue;
                //writeLog("------(%.6lf,%.6lf,%.6lf)------\n" , xval[i] , yval[j] , zval[k]);
				for (int a = 0; a < 3; a++)
				{
					//fprintf(fp , "(%.6lf,%.6lf,%.6lf), eigenVal = %.6lf, length = %.6lf\n" ,
						//tensor[i][j][k].axis[a].x , tensor[i][j][k].axis[a].y , tensor[i][j][k].axis[a].z ,
						//tensor[i][j][k].eigenVal[a] , tensor[i][j][k].axisLen[a]);
				}
			}
     */
}
/*
vec3i PointCloudUtils::nearestGridPoint(const vec3d& pos)
{
	int xi = clampx((int)((pos.x - box.lb.x) / gridSize.x) , 0 , gridRes.x);
	int yi = clampx((int)((pos.y - box.lb.y) / gridSize.y) , 0 , gridRes.y);
	int zi = clampx((int)((pos.z - box.lb.z) / gridSize.z) , 0 , gridRes.z);

	double min_d = 1e30;
	vec3i res(xi , yi , zi);
	for (int i = 0; i <= 1; i++)
	{
		for (int j = 0; j <= 1; j++)
		{
			for (int k = 0; k <= 1; k++)
			{
				if (xi + i > gridRes.x || yi + j > gridRes.y || zi + k > gridRes.z)
					continue;

				double d = (vec3d(xval[xi + i] , yval[yi + j] , zval[zi + k]) - pos).length();
				if (d < min_d)
				{
					min_d = d;
					res.x = xi + i;
					res.y = yi + j;
					res.z = zi + k;
				}
			}
		}
	}

	//fprintf(fp , "(%.6lf,%.6lf,%.6lf)->(%d,%d,%d)\n" , pos.x , pos.y , pos.z , res.x , res.y , res.z);
	return res;
}
*/
double PointCloudUtils::calcEdgeWeight(const vec3d& _v , 
	const Tensor& st , const Tensor& ed)
{
    Vector3d dv(_v.x , _v.y , _v.z);
    
    double w = 0.5 * (sqrt(dv.transpose() * st.tensor * dv) +
                      sqrt(dv.transpose() * ed.tensor * dv));
    /*
	double cos_theta = std::min(1.f , std::abs(v.dot(st.axis[0])));
	double sin_theta = sqrt(1.f - cos_theta * cos_theta);
	vec3d v_proj = v - st.axis[0] * cos_theta;
	v_proj.normalize();

	double cos_phi = std::min(1.f , std::abs(v_proj.dot(st.axis[1])));
	double sin_phi = sqrt(1.f - cos_phi * cos_phi);
	double w = sqrt(st.axisLen[0] * SQR(cos_theta) + 
		st.axisLen[1] * SQR(sin_theta * cos_phi) + 
		st.axisLen[2] * SQR(sin_theta * sin_phi));
    
	v = -v;
	cos_theta = std::min(1.f , std::abs(v.dot(ed.axis[0])));
	sin_theta = sqrt(1.f - cos_theta * cos_theta);
	v_proj = v - ed.axis[0] * cos_theta;
	v_proj.normalize();

	cos_phi = std::min(1.f , std::abs(v_proj.dot(ed.axis[1])));
	sin_phi = sqrt(1.f - cos_phi * cos_phi);
	w += sqrt(ed.axisLen[0] * SQR(cos_theta) + 
		ed.axisLen[1] * SQR(sin_theta * cos_phi) + 
		ed.axisLen[2] * SQR(sin_theta * sin_phi));

	w *= len_v * 0.5f;
    
    writeLog("%.6lf %.6lf\n" , w , ww);
    */
	return w;
}

Matrix3d PointCloudUtils::lerpHessian(const vec3d& pos)
{
    vec3d leftBottom(extXval[0] , extYval[0] , extZval[0]);
    vec3i maxSize(sizeOriginF);
    int xi = clampx((int)((pos.x - leftBottom.x) / gridSize.x) , 1 , maxSize.x - 2);
    int yi = clampx((int)((pos.y - leftBottom.y) / gridSize.y) , 1 , maxSize.y - 2);
    int zi = clampx((int)((pos.z - leftBottom.z) / gridSize.z) , 1 , maxSize.z - 2);
    
    vec3d v;
    v.x = (pos.x - extXval[xi]) / (extXval[xi + 1] - extXval[xi]);
    v.y = (pos.y - extYval[yi]) / (extYval[yi + 1] - extYval[yi]);
    v.z = (pos.z - extZval[zi]) / (extZval[zi + 1] - extZval[zi]);
    v.x = clampx(v.x , 0.0 , 1.0);
    v.y = clampx(v.y , 0.0 , 1.0);
    v.z = clampx(v.z , 0.0 , 1.0);

    Matrix3d hesMat;
    hesMat = (1 - v.x) * (1 - v.y) * (1 - v.z) * tensor[xi][yi][zi].hessian +
        (1 - v.x) * (1 - v.y) * v.z * tensor[xi][yi][zi + 1].hessian +
        (1 - v.x) * v.y * (1 - v.z) * tensor[xi][yi + 1][zi].hessian +
        (1 - v.x) * v.y * v.z * tensor[xi][yi + 1][zi + 1].hessian +
        v.x * (1 - v.y) * (1 - v.z) * tensor[xi + 1][yi][zi].hessian +
        v.x * (1 - v.y) * v.z * tensor[xi + 1][yi][zi + 1].hessian +
        v.x * v.y * (1 - v.z) * tensor[xi + 1][yi + 1][zi].hessian +
        v.x * v.y * v.z * tensor[xi + 1][yi + 1][zi + 1].hessian;
    
    return hesMat;
}

Matrix3d PointCloudUtils::lerpTensor(const vec3d &pos)
{
    vec3d leftBottom(extXval[0] , extYval[0] , extZval[0]);
    vec3i maxSize(sizeOriginF);
    int xi = clampx((int)((pos.x - leftBottom.x) / gridSize.x) , 0 , maxSize.x - 1);
    int yi = clampx((int)((pos.y - leftBottom.y) / gridSize.y) , 0 , maxSize.y - 1);
    int zi = clampx((int)((pos.z - leftBottom.z) / gridSize.z) , 0 , maxSize.z - 1);
    
    vec3d v;
    v.x = (pos.x - extXval[xi]) / (extXval[xi + 1] - extXval[xi]);
    v.y = (pos.y - extYval[yi]) / (extYval[yi + 1] - extYval[yi]);
    v.z = (pos.z - extZval[zi]) / (extZval[zi + 1] - extZval[zi]);
    v.x = clampx(v.x , 0.0 , 1.0);
    v.y = clampx(v.y , 0.0 , 1.0);
    v.z = clampx(v.z , 0.0 , 1.0);
    /*
    if (v.x < -EPS || v.x > 1 + EPS ||
        v.y < -EPS || v.y > 1 + EPS ||
        v.z < -EPS || v.y > 1 + EPS)
    {
        printf("lerp tensor error: (%.8lf,%.8lf,%.8lf)\n" ,
            v.x , v.y , v.z);
        printf("x: %.8lf, %.8lf; y: %.8lf, %.8lf; z: %.8lf, %.8lf\n" ,
            pos.x , extXval[xi] , pos.y , extYval[yi] , pos.z , extZval[zi]);
    }
    */
    
    Matrix3d tensorMat;
    tensorMat = (1 - v.x) * (1 - v.y) * (1 - v.z) * tensor[xi][yi][zi].tensor +
        (1 - v.x) * (1 - v.y) * v.z * tensor[xi][yi][zi + 1].tensor +
        (1 - v.x) * v.y * (1 - v.z) * tensor[xi][yi + 1][zi].tensor +
        (1 - v.x) * v.y * v.z * tensor[xi][yi + 1][zi + 1].tensor +
        v.x * (1 - v.y) * (1 - v.z) * tensor[xi + 1][yi][zi].tensor +
        v.x * (1 - v.y) * v.z * tensor[xi + 1][yi][zi + 1].tensor +
        v.x * v.y * (1 - v.z) * tensor[xi + 1][yi + 1][zi].tensor +
        v.x * v.y * v.z * tensor[xi + 1][yi + 1][zi + 1].tensor;
    
    /*
    SelfAdjointEigenSolver<Matrix3d> vd(tensorMat);
    Vector3d egval = vd.eigenvalues();
    Matrix3d egvec = vd.eigenvectors();
    for (int i = 0; i < 3; i++)
    {
        if (egval(i) < -EPS)
        {
            printf("error: %.8f\n" , egval(i));
            printf("(%d,%d,%d)\n" , xi , yi , zi);
            printf("(%.6lf,%.6lf,%.6lf)\n" , v.x , v.y , v.z);
        }
    }
    */
    return tensorMat;
}

void PointCloudUtils::calcPointTensor()
{
	timer.PushCurrentTime();

	nodes = 0;
    index2Point.clear();
    point2Index.clear();
    pointTensor.clear();
    index2Tensor.clear();
	for (int i = 0; i < pcData.size(); i++)
	{
		vec3d pos = pcData[i].pos;

		double hashVal = point2double(pos);
		if (point2Index.find(hashVal) != point2Index.end())
        {
            /*
            int ind = point2Index[hashVal];
            if (!isEqual(index2Point[ind] , pos))
            {
                printf("(%.6f,%.6f,%.6f) ~= (%.6f,%.6f,%.6f)\n" , pos.x , pos.y , pos.z ,
                    index2Point[ind].x , index2Point[ind].y , index2Point[ind].z);
            }
            */
            continue;
        }

		index2Point.push_back(pos);
		point2Index[hashVal] = nodes;
        
		Matrix3d hesMat;
		hesMat = lerpHessian(pos);

        /*
		Tensor ts;
		ts.hessian = hesMat;
		calcTensorDecomposition(ts);
		calcTensorMetric(ts);
        */
        Tensor ts;
        ts.tensor = lerpTensor(pos);
        
        /*
        writeLog("=================\n");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                writeLog("%.6lf " , ts.tensor(i , j));
            }
            writeLog("\n");
        }
        writeLog("-----------------\n");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                writeLog("%.6lf " , tsx.tensor(i , j));
            }
            writeLog("\n");
        }
        writeLog("\n");
        */
		pointTensor.push_back(ts);

		nodes++;
	}

    for (int i = 0; i < nodes; i++)
    {
        index2Tensor.push_back(&pointTensor[i]);
    }
	timer.PopAndDisplayTime("\nInterpolate point tensors: %.6lf\n");

    /*
	for (int i = 0; i < nodes; i++)
	{
		writeLog("========%d=========\n" , i);
		vec3d pos = index2Point[i];
        double hashVal = point2double(pos);
		writeLog("------index = %d , (%.6lf,%.6lf,%.6lf)------\n" , point2Index[hashVal] ,
            pos.x , pos.y , pos.z);
        
		Tensor ts = pointTensor[i];
		for (int a = 0; a < 3; a++)
		{
			writeLog("(%.6lf,%.6lf,%.6lf), eigenVal = %.6lf, renderLen = %.6lf, length = %.6lf, renderLen = %.6lf\n" ,
				ts.axis[a].x , ts.axis[a].y , ts.axis[a].z , 
				ts.eigenVal[a] , pcRenderer->calcHessianRenderLength(ts.eigenVal[a]) ,
				ts.axisLen[a] , pcRenderer->calcMetricRenderLength(ts.axisLen[a]));
		}
        writeLog("------------------\n");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                writeLog("%.6lf " , ts.tensor(i , j));
            }
            writeLog("\n");
        }
	}
    */
}

void PointCloudUtils::buildGraphFromPoints()
{
	timer.PushCurrentTime();

	pointGraph.clear();
	pointGraph.resize(nodes);
#pragma omp parallel for
	for (int i = 0; i < nodes; i++)
	{
		KnnQuery query(KNN);
		tree->searchKnn(0 , index2Point[i] , query);
        pointGraph[i].clear();
		for (int k = 0; k < query.knnPoints.size(); k++)
		{
			double hashVal = point2double(query.knnPoints[k].point->pos);
			int j = point2Index[hashVal];
			if (i == j)
				continue;
			vec3d v = index2Point[j] - index2Point[i];
			double w;
            w = calcEdgeWeight(v , pointTensor[i] , pointTensor[j]);
			pointGraph[i].push_back(Edge(j , w));

            /*
            if (i == 27330 || i == 28171)
            {
                writeLog("(%.6f,%.6f,%.6f) <-> (%.6f,%.6f,%.6f) , w=%.6f\n" ,
                    pcData[i].pos.x , pcData[i].pos.y , pcData[i].pos.z ,
                    query.knnPoints[k].point->pos.x , query.knnPoints[k].point->pos.y ,
                    query.knnPoints[k].point->pos.z , w);
            }
            */
		}
	}

	timer.PopAndDisplayTime("\nBuild point graph: %.6lf\n");
    /*
	for (int i = 0; i < nodes; i++)
	{
		writeLog("==============\n");
		for (int k = 0; k < pointGraph[i].size(); k++)
		{
			Edge e = pointGraph[i][k];
			writeLog("(%d->%d), (%.6lf,%.6lf,%.6lf)->(%.6lf,%.6lf,%.6lf): %.6lf\n" ,
				i , e.y , index2Point[i].x , index2Point[i].y , index2Point[i].z ,
				index2Point[e.y].x , index2Point[e.y].y , index2Point[e.y].z , e.w);
		}
	}
    */
}

bool PointCloudUtils::addPointToGraph(const vec3d& pos)
{
    // new node
    double hashVal = point2double(pos);
    if (point2Index.find(hashVal) != point2Index.end())
    {
        return false;
    }
    index2Point.push_back(pos);
    point2Index[hashVal] = nodes;
    Tensor ts;
    ts.tensor = lerpTensor(pos);
    pointTensor.push_back(ts);
    index2Tensor.push_back(&pointTensor[nodes]);
    nodes++;
    // new edges
    pointGraph.push_back(std::vector<Edge>());
    KnnQuery query(KNN);
    tree->searchKnn(0 , pos , query);
    for (int k = 0; k < query.knnPoints.size(); k++)
    {
        double hashVal = point2double(query.knnPoints[k].point->pos);
        int j = point2Index[hashVal];
        if (nodes - 1 == j) continue;
        vec3d v = index2Point[j] - pos;
        double w = calcEdgeWeight(v , pointTensor[nodes - 1] , pointTensor[j]);
        pointGraph[nodes - 1].push_back(Edge(j , w));
        pointGraph[j].push_back(Edge(nodes - 1 , w));
    }
    printf("add new point (%.6f,%.6f,%.6f)\n" , pos.x , pos.y , pos.z);
    return true;
}

bool PointCloudUtils::dijkstra(const Graph& g , const int& source ,
							   DijkstraInfo& info , int *sink)
{
	int maxNodeNum = nodes;
	info.init(maxNodeNum);
	Heap q(maxNodeNum);

	info.dist[source] = 0.0;
	q.push(source , 0.0);

    // bool first = true;

	while (!q.empty())
	{
		int now = q.top().index;
		double d = q.top().key;

		q.pop();

		if (sink != NULL && now == *sink)
			return 1;

		for (int k = 0; k < g[now].size(); k++)
		{
			const Edge& e = g[now][k];
            /*
            if (first)
            {
                printf("(%.6f,%.6f,%.6f) , w=%.6f\n" , index2Point[e.y].x ,
                    index2Point[e.y].y , index2Point[e.y].z , e.w);
            }
            */
			if (info.dist[e.y] > d + e.w)
			{
				info.dist[e.y] = d + e.w;
				info.prev[e.y] = now;

				q.push(e.y , info.dist[e.y]);
			}
		}

        // first = false;
	}

	if (sink != NULL && info.dist[*sink] > 1e20)
		return 0;

	return 1;
}

void PointCloudUtils::traceBack(const DijkstraInfo& info , const int& sink ,
								Path& pathVertex)
{
	//std::vector<int> seq;
	pathVertex.clear();
	int now = sink;
	while (info.prev[now] != -1)
	{
		pathVertex.push_back(index2Point[now]);
		//seq.push_back(now);
		now = info.prev[now];
	}
	pathVertex.push_back(index2Point[now]);
    /*
    seq.push_back(now);
	
	fprintf(stdout , "===========%.6lf=========\n" , info.dist[sink]);
	
	for (int i = (int)pathVertex.size() - 1; i >= 1; i--)
	{
		fprintf(stdout , "(%.6lf,%.6lf,%.6lf) -> (%.6lf,%.6lf,%.6lf) , %.6lf\n" , pathVertex[i].x , pathVertex[i].y , pathVertex[i].z ,
			pathVertex[i - 1].x , pathVertex[i - 1].y , pathVertex[i - 1].z , info.dist[seq[i - 1]] - info.dist[seq[i]]);
	}
    */
}

void PointCloudUtils::laplacianSmooth(Path &path)
{
    if (path.size() <= 2)
        return;
    Path oldPath(path);
#pragma omp parallel for
    for (int i = 1; i < path.size() - 1; i++)
    {
        vec3d p = oldPath[i - 1] + (oldPath[i + 1] - oldPath[i - 1]) * 0.5;
        vec3d v = (p - oldPath[i]) * pcRenderer->smoothScale;
        path[i] = oldPath[i] + v;
    }
    path[0] = oldPath[0];
    path[path.size() - 1] = oldPath[path.size() - 1];
}

vec3d calcGradient(const vec3d& v , const Matrix3d& tensor)
{
    Vector3d dv(v.x , v.y , v.z);
    Vector3d grad = tensor * dv;
    vec3d res(grad(0) , grad(1) , grad(2));
    res *= 0.5;
    double denom = sqrt(dv.transpose() * tensor * dv);
    res /= denom;
    return res;
}

double calcAnisDist(const vec3d& v , const Matrix3d& ts)
{
    Vector3d dv(v.x , v.y , v.z);
    return dv.transpose() * ts * dv;
}

double calcAnisDist(const vec3d& pos1 , const Matrix3d& t1 , const vec3d& pos2 ,
                   const Matrix3d& t2 , const vec3d& pos3 , const Matrix3d& t3)
{
    double res = 0.0;
    vec3d v = pos2 - pos1;
    Vector3d dv(v.x , v.y , v.z);
    double tp = dv.transpose() * t1 * dv;
    res += sqrt(tp);
    tp = dv.transpose() * t2 * dv;
    res += sqrt(tp);
    v = pos2 - pos3;
    for (int i = 0; i < 3; i++)
        dv(i) = v[i];
    tp = dv.transpose() * t2 * dv;
    res += sqrt(tp);
    tp = dv.transpose() * t3 * dv;
    res += sqrt(tp);
    return res * 0.5;
}

Matrix3d lineSearchHessian(const vec3d& v , const Matrix3d& tensor)
{
    Matrix3d res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res(i , j) = 0.0;
    Vector3d dv(v.x , v.y , v.z);
    double denom = dv.transpose() * tensor * dv;
    double upscale = 1e5;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            double t1 = 0.0 , t2 = 0.0;
            for (int k = 0; k < 3; k++)
            {
                t1 += tensor(i , k) * dv(k);
                t2 += tensor(j , k) * dv(k);
            }
            res(i , j) = (0.5 * tensor(i , j) * upscale) / (sqrt(denom) * upscale) -
                (0.5 * t1 * upscale * t2) / (denom * upscale * sqrt(denom));
        }
    }
    /*
    if (std::abs(denom) < EPS)
    {
        printf("======= ill! line search hessian: %.8lf =======\n" , denom);
        printf("v=(%.8lf,%.8lf,%.8lf)\n" , v.x , v.y , v.z);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                printf("%.8lf " , tensor(i , j));
            }
            printf("\n");
        }
    }
    else
    {
        printf("======= normal! line search hessian: %.8lf =======\n" , denom);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                printf("%.8lf " , res(i , j));
            }
            printf("\n");
        }
    }
    */
    return res;
}

double lineSearch(const vec3d& d , const vec3d& pos1 , const Matrix3d& t1 ,
                 const vec3d& pos2 , const Matrix3d& t2 ,
                 const vec3d& pos3 , const Matrix3d& t3)
{
    vec3d grad = calcGradient(pos2 - pos1 , t1) +
        calcGradient(pos2 - pos1 , t2) + calcGradient(pos2 - pos3 , t2) +
        calcGradient(pos2 - pos3 , t3);
    Vector3d f1(grad.x , grad.y , grad.z);
    Matrix3d f2 = lineSearchHessian(pos2 - pos1 , t1) +
        lineSearchHessian(pos2 - pos1 , t2) +
        lineSearchHessian(pos2 - pos3 , t2) +
        lineSearchHessian(pos2 - pos3 , t3);
    Vector3d vd(d.x , d.y , d.z);
    double denom = vd.transpose() * f2 * vd;
    double res = -f1.dot(vd) / denom;
    /*
    if (abs(denom) < 1.0)
    {
        //return 0.0;
        printf("line search: %.8lf / %.8lf = %.8lf\n" , -f1.dot(vd) ,
            denom , res);
        printf("grad = (%.8lf, %.8lf, %.8lf)\n" , grad.x , grad.y , grad.z);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                printf("%.8lf " , f2(i , j));
            }
            printf("\n");
        }
    }
    */
    return res;
}

double lineSearchSecant(const vec3d& d ,
    const vec3d& pos1 , const Matrix3d& t1 ,
    const vec3d& pos2 , const Matrix3d& t2 ,
    const vec3d& pos3 , const Matrix3d& t3)
{
    vec3d grad = calcGradient(pos2 - pos1 , t1) +
        calcGradient(pos2 - pos1 , t2) + calcGradient(pos2 - pos3 , t2) +
        calcGradient(pos2 - pos3 , t3);
    double sigma = 1e-4;
    vec3d pos_prime = pos2 + d * sigma;
    double res;
    vec3d grad_prime = calcGradient(pos_prime - pos1 , t1) +
        calcGradient(pos_prime - pos1 , t2) +
        calcGradient(pos_prime - pos3 , t2) +
        calcGradient(pos_prime - pos3 , t3);
    res = -sigma * grad.dot(d) / (grad_prime.dot(d) - grad.dot(d));
    return res;
}

void PointCloudUtils::gradientDescentSmooth(Path& path)
{
    if (path.size() <= 2)
        return;
    Path oldPath(path);
    std::vector<Matrix3d> ts;
    ts.resize(path.size());
#pragma omp parallel for
    for (int i = 0; i < path.size(); i++)
    {
        ts[i] = lerpTensor(path[i]);
    }
    
#pragma omp parallel for
    for (int i = 1; i < path.size() - 1; i++)
    {
        vec3d grad(0.0);
        vec3d v , p , mid;
        v = oldPath[i] - oldPath[i - 1];
        if (v.length() < 1e-3)
            continue;
        grad += calcGradient(v , ts[i - 1]) +
            calcGradient(v , ts[i]);

        v = oldPath[i] - oldPath[i + 1];
        if (v.length() < 1e-3)
            continue;
        grad += calcGradient(v , ts[i]) +
            calcGradient(v , ts[i + 1]);

        /*
        vec3d _p0 = oldPath[i] - grad / grad.length() * 1e-5;
        double _f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p0 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3d _p1 = oldPath[i];
        double _f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p1 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3d _p2 = oldPath[i] + grad / grad.length() * 1e-5;
        double _f2 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p2 , ts[i] , oldPath[i + 1] , ts[i + 1]);

        if (_f0 > _f1 || _f2 < _f1)
            continue;
        
        if (_f0 <= _f1 && _f1 <= _f2)
        {
            printf("=================\nyes ");
            printf("f(-1)=%.8f, f(0)=%.8f, f(1)=%.8f\n" ,
                   _f0 , _f1 , _f2);
            printf("grad = (%.8f,%.8f,%.8f), %.8f\n" , grad.x , grad.y , grad.z , grad.length());
        }
        else
        {
            printf("=================\nno ");
            printf("f(-1)=%.8lf, f(0)=%.8lf, f(1)=%.8lf\n" ,
                   _f0 , _f1 , _f2);
            printf("grad = (%.8lf,%.8lf,%.8lf), %.8lf\n" , grad.x , grad.y , grad.z , grad.length());
        }
        */
        p = oldPath[i - 1] + (oldPath[i + 1] - oldPath[i - 1]) * 0.5;
        mid = p - oldPath[i];
        double len = mid.length();
        //grad /= grad.length();
        vec3d d = -grad;
        double alpha = lineSearch(d , oldPath[i - 1] , ts[i - 1] ,
            oldPath[i] , ts[i] , oldPath[i + 1] , ts[i + 1]);
        //double alpha = lineSearchSecant(d , oldPath[i - 1] , ts[i - 1] ,
        //    oldPath[i] , ts[i] , oldPath[i + 1] , ts[i + 1]);
        /*
        vec3d p0 = oldPath[i] - grad * alpha + grad * 1e-6;
        double f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p0 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3d p1 = oldPath[i] - grad * alpha;
        double f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p1 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3d p2 = oldPath[i] - grad * alpha - grad * 1e-6;
        double f2 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p2 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        if (f0 > f1 && f1 < f2)
        {
            printf("yes, alpha = %.8lf, d1 = %.8lf, d2 = %.8lf\n" , alpha , f1 - f0 , f1 - f2);
        }
        else
        {
            printf("no, alpha = %.8lf, d1 = %.8lf, d2 = %.8lf\n" , alpha , f1 - f0 , f1 - f2);
        }
        */
        alpha = std::min(alpha , len);
        
        path[i] = oldPath[i] + d * alpha * pcRenderer->smoothScale;
    }
    path[0] = oldPath[0];
    path[path.size() - 1] = oldPath[path.size() - 1];
    
    /*
    printf("========================\n");
    for (int i = 1; i < path.size() - 1; i++)
    {
        vec3d grad(0.f) , v , p , mid;
        v = path[i] - path[i - 1];
        grad += calcGradient(v , ts[i - 1]) +
            calcGradient(v , ts[i]);
        v = path[i + 1] - path[i];
        grad += calcGradient(v , ts[i]) +
            calcGradient(v , ts[i + 1]);
        printf("pos = (%.6lf,%.6lf,%.6lf), grad = (%.6lf,%.6lf,%.6lf), norm = %.6lf\n" ,
               path[i].x , path[i].y , path[i].z ,
               grad.x , grad.y , grad.z , grad.length());
        
        double f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] , oldPath[i] ,
                                ts[i] , oldPath[i + 1] , ts[i + 1]);
        double f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] , path[i] ,
                                ts[i] , oldPath[i + 1] , ts[i + 1]);
        printf("prev f = %.8f , now f = %.8f\n" , f0 , f1);
    }
    */
}

double calcJuncF(const vec3d& pj , const Matrix3d& tsj ,
    const std::vector<vec3d>& pts , const std::vector<Matrix3d>& ts)
{
    double res = 0;
    for (int i = 0; i < pts.size(); i++)
    {
        vec3d v = pj - pts[i];
        res += calcAnisDist(v , tsj) + calcAnisDist(v , ts[i]);
    }
    return res;
}

vec3d calcJuncGrad(const vec3d& pj , const Matrix3d& tsj ,
    const std::vector<vec3d>& pts , const std::vector<Matrix3d>& ts)
{
    vec3d grad(0.0);
    for (int i = 0; i < pts.size(); i++)
    {
        vec3d v = pj - pts[i];
        grad += calcGradient(v , tsj) + calcGradient(v , ts[i]);
    }
    return grad;
}

double calcJuncAlpha(const vec3d& d , const vec3d& pj , const Matrix3d& tsj ,
    const std::vector<vec3d>& pts , const std::vector<Matrix3d>& ts)
{
    vec3d grad(0.0);
    Matrix3d f2;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++) f2(i , j) = 0;
    }
    for (int i = 0; i < pts.size(); i++)
    {
        vec3d v = pj - pts[i];
        grad += calcGradient(v , tsj) + calcGradient(v , ts[i]);
        f2 += lineSearchHessian(v , tsj) + lineSearchHessian(v , ts[i]);
    }
    Vector3d f1(grad.x , grad.y , grad.z);
    Vector3d vd(d.x , d.y , d.z);
    double res = -f1.dot(vd);
    double denom = vd.transpose() * f2 * vd;
    res /= denom;
    return res;
}

void PointCloudUtils::optimizeJunction(CurveNet* cn , const vec3d& pos)
{
    std::vector<vec3d> pts;
    std::vector<Matrix3d> ts;
    int ni = cn->getNodeIndex(pos);
    if (!cn->nodesStat[ni]) return;
    double thr = 1e10;
    for (int i = 0; i < cn->edges[ni].size(); i++)
    {
        Path& path = cn->polyLines[cn->edges[ni][i].pli];
        if (path.size() <= 2) continue;
        // PAOS's index is either 0 or L-1
        int nextId = 1;
        if (!isEqual(pos , path[0]))
        {
            nextId = path.size() - 2;
            thr = std::min(thr , (path[nextId] - path[nextId + 1]).length());
        }
        else
        {
            thr = std::min(thr , (path[nextId] - path[nextId - 1]).length());
        }
        pts.push_back(path[nextId]);
        ts.push_back(lerpTensor(path[nextId]));
    }
    vec3d p(pos);
    for (int iter = 0; iter < 1; iter++)
    {
        Matrix3d tsj = lerpTensor(p);
        vec3d grad = calcJuncGrad(p , tsj , pts , ts);
        vec3d d = -grad;
        double alpha = calcJuncAlpha(d , p , tsj , pts , ts);
        d *= alpha;
        if (d.length() <= thr)
            p = p + d;
        // else
            // printf("too much\n");
        //printf("prev = (%.6f,%.6f,%.6f), now = (%.6f,%.6f,%.6f)\n" , pos.x , pos.y , pos.z ,
        //p.x , p.y , p.z);
    }
    cn->nodes[ni] = p;
    for (int i = 0; i < cn->edges[ni].size(); i++)
    {
        Path& path = cn->polyLines[cn->edges[ni][i].pli];
        if (path.size() <= 2) continue;
        if (isEqual(pos , path[0]))
        {
            path[0] = p;
        }
        else
        {
            path[path.size() - 1] = p;
        }
    }
    // double f = calcJuncF(pos , tsj , pts , ts);
    // printf("===================\n");
    /*
    printf("f = %.6f, grad = (%.6f,%.6f,%.6f)\n" , f , grad.x , grad.y , grad.z);
    vec3d p0 = pos + d * 1e-5;
    double f0 = calcJuncF(p0 , tsj , pts , ts);
    vec3d p1 = pos - d * 1e-5;
    double f1 = calcJuncF(p1 , tsj , pts , ts);
    if (f0 <= f && f <= f1)
    {
        printf("yes ");
    }
    else
    {
        printf("no ");
    }
    printf("f0 = %.8lf , f = %.8lf , f1 = %.8lf\n" , f0 , f , f1);
    */
    /*
    vec3d q0 = pos + d * alpha - d * 1e-4;
    double g0 = calcJuncF(q0 , tsj , pts , ts);
    vec3d q1 = pos + d * alpha;
    double g1 = calcJuncF(q1 , tsj , pts , ts);
    vec3d q2 = pos + d * alpha + d * 1e-4;
    double g2 = calcJuncF(q2 , tsj , pts , ts);
    if (g0 >= g1 && g1 <= g2)
    {
        printf("yes ");
    }
    else
    {
        printf("no ");
    }
    printf("alpha = %.8lf , g0 = %.8lf , g1 = %.8lf , g2 = %.8lf\n" , alpha ,
        g0 , g1 , g2);
    */
    /*
    printf("======== (%.6f,%.6f,%.6f) =========\n" , pos.x , pos.y , pos.z);
    printf("%d\n" , ni);

    for (int i = 0; i < pts.size(); i++)
    {
        printf("adj pt = (%.6f,%.6f,%.6f)\n" , pts[i].x , pts[i].y , pts[i].z);
    }
    */
}

void PointCloudUtils::loadCurveNet()
{
}

void PointCloudUtils::saveCurveNet()
{
    char* fileName = new char[strlen(m_fileName) + 20];
	strcpy(fileName , m_fileName);
	for (int i = strlen(fileName) - 1; i >= 0; i--)
    {
		if (fileName[i] == '.')
		{
			fileName[i]='\0';
			break;
		}
	}
	wxFileName dirname;
	if(!dirname.DirExists(wxString(fileName)))
		dirname.Mkdir(wxString(fileName));

	strcat(fileName,"/curve.txt");
	printf("save curve file name = %s\n" , fileName);
}