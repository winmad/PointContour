#include "pointCloudUtils.h"
#include "nvVector.h"
#include "TimeManager.h"
#include "curveNet.h"
#include <omp.h>
#include <queue>

PointCloudUtils::PointCloudUtils()
{
	pcRenderer = new PointCloudRenderer();
	isPointInside = NULL;
	originF = NULL;
	f = NULL;
	//df[0] = df[1] = df[2] = NULL;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ddf[i][j] = NULL;
	tensor = NULL;

	tree = NULL;

    restartLog("debug.txt");
}

PointCloudUtils::~PointCloudUtils()
{
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

	timer.PopAndDisplayTime("\nAllocate memory: %.6f\n");
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

	timer.PopAndDisplayTime("\nDeallocate memory: %.6f\n");
}

void PointCloudUtils::init()
{
	timer.PushCurrentTime();
    
	getBBox();
	pcRenderer->pcUtils = this;
	pcRenderer->init();
	if (tree != NULL)
		delete tree;
	tree = new PointKDTree<Data>(pcData);

	graphType = POINT_GRAPH;

	nodes = edges = subdivNodes = 0;

	timer.PopAndDisplayTime("\nBuild kdtree: %.6f\n");
    
    curveNet.clear();
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

    if (graphType != POINT_GRAPH)
        buildAdaptiveGrid();

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
	if (graphType == ADAPTIVE_GRAPH)
	{
		subdivision();
		buildGraphFromAdaptiveGrid();
		addSubdivisionEdges(adapGraph);
	}
	else if (graphType == UNIFORM_GRAPH)
	{
		buildGraphFromUniformGrid();
	}
	else if (graphType == POINT_GRAPH)
	{
		calcPointTensor();
		buildGraphFromPoints();
	}
}

void PointCloudUtils::getBBox()
{
	box.lb = vec3f(1e20f , 1e20f , 1e20f);
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
}

void PointCloudUtils::buildUniformGrid(vec3i size)
{
	gridRes = size;
	gridResx = gridRes + vec3i(1 , 1 , 1);
	vec3f diag = box.rt - box.lb;

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

void PointCloudUtils::buildAdaptiveGrid()
{
	if (graphType == POINT_GRAPH)
		return;

	bandwidth = 2;

	timer.PushCurrentTime();

	for (int i = 0; i < gridRes.x; i++)
		for (int j = 0; j < gridRes.y; j++)
			for (int k = 0; k < gridRes.z; k++)
				isPointInside[i][j][k] = 0;
	
	for (int i = 0; i < pcData.size(); i++)
	{
		vec3f& pos = pcData[i].pos;
		int xi = std::min((int)((pos.x - box.lb.x) / gridSize.x) , gridRes.x - 1);
		int yi = std::min((int)((pos.y - box.lb.y) / gridSize.y) , gridRes.y - 1);
		int zi = std::min((int)((pos.z - box.lb.z) / gridSize.z) , gridRes.z - 1);

		isPointInside[xi][yi][zi] = 1;
	}
	
	if (graphType == ADAPTIVE_GRAPH)
		genNarrowBand();
	/*
	for (int i = 0; i < gridRes.x; i++)
	{
		for (int j = 0; j < gridRes.y; j++)
		{
			for (int k = 0; k < gridRes.z; k++)
			{
				if (isPointInside[i][j][k] > 0)
				{
					fprintf(fp , "(%.6f,%.6f,%.6f), dist = %d\n" , 
						xval[i] , yval[j] , zval[k] , isPointInside[i][j][k]);
				}
			}
		}
	}
	*/
	// statistics
	printf("\n/**** Grid statistics ***/\n");
	int cntPointInside = 0;
	int cntBandPoints = 0;
	for (int i = 0; i < gridRes.x; i++)
	{
		for (int j = 0; j < gridRes.y; j++)
		{
			for (int k = 0; k < gridRes.z; k++)
			{
				if (isPointInside[i][j][k] == 1)
					cntPointInside++;
				if (isPointInside[i][j][k] >= 1)
					cntBandPoints++;
			}
		}
	}
	printf("Grid with point %d / %d\n" , cntPointInside , gridResx.x * gridResx.y * gridResx.z);
	printf("Narrow band points %d / %d\n" , cntBandPoints , gridResx.x * gridResx.y * gridResx.z);
	printf("\n");
	// end of statistics
	
	nodes = 0;

	point2Index.clear();
	index2Tensor.clear();
	index2Point.clear();
	for (int i = 0; i <= gridRes.x; i++)
	{
		for (int j = 0; j <= gridRes.y; j++)
		{
			for (int k = 0; k <= gridRes.z; k++)
			{
				if (graphType == ADAPTIVE_GRAPH)
				{
					if (isPointInside[i][j][k] == 0)
						continue;
					for (int ii = i; ii <= i + 1; ii++)
					{
						if (ii < 0 || ii > gridRes.x)
							continue;
						for (int jj = j; jj <= j + 1; jj++)
						{
							if (jj < 0 || jj > gridRes.y)
								continue;
							for (int kk = k; kk <= k + 1; kk++)
							{
								if (kk < 0 || kk > gridRes.z)
									continue;
								if (ii == i && jj == j && kk == k)
									continue;
								vec3f pos = vec3f(xval[ii] , yval[jj] , zval[kk]);
								double hashVal = point2double(pos);
								if (point2Index.find(hashVal) == point2Index.end())
								{
									index2Point.push_back(pos);
									index2Tensor.push_back(&tensor[ii][jj][kk]);
									point2Index[hashVal] = nodes;
									nodes++;
								}
							}
						}
					}
				}
				else if (graphType == UNIFORM_GRAPH)
				{
					vec3f pos = vec3f(xval[i] , yval[j] , zval[k]);
					double hashVal = point2double(pos);

					index2Point.push_back(pos);
					index2Tensor.push_back(&tensor[i][j][k]);
					point2Index[hashVal] = nodes;
					nodes++;
				}
			}
		}
	}
	
	gridNodes = nodes;
	printf("Adaptive nodes: %d / %d\n" , gridNodes , gridResx.x * gridResx.y * gridResx.z);

	timer.PopAndDisplayTime("\nBuild adaptive grid: %.6lf\n");
	/*
	for (int i = 0; i < gridNodes; i++)
	{
		fprintf(fp , "#%d: (%.6f,%.6f,%.6f)\n" , i , index2Point[i].x , 
			index2Point[i].y , index2Point[i].z);
	}
	*/
}

void PointCloudUtils::genNarrowBand()
{
	std::queue<int> q;
	for (int i = 0; i < gridRes.x; i++)
	{
		for (int j = 0; j < gridRes.y; j++)
		{
			for (int k = 0; k < gridRes.z; k++)
			{
				if (isPointInside[i][j][k] == 1)
				{
					int index = i * (gridResx.y * gridResx.z) + 
						j * gridResx.z + k;
					q.push(index);
				}
			}
		}
	}

	const vec3i dir[] = {vec3i(0,0,-1), vec3i(0,0,1), vec3i(0,1,0), vec3i(0,-1,0), vec3i(-1,0,0), vec3i(1,0,0)};
	while (!q.empty())
	{
		int now = q.front();
		q.pop();
		vec3i p = index2Grid(now);

		if (isPointInside[p.x][p.y][p.z] >= bandwidth)
			return;

		for (int d = 0; d < 6; d++)
		{
			int i = p.x + dir[d].x;
			int j = p.y + dir[d].y;
			int k = p.z + dir[d].z;
			if (i < 0 || i >= gridRes.x || j < 0 || j >= gridRes.y || k < 0 || k >= gridRes.z)
				continue;
			if (isPointInside[i][j][k] == 0)
			{
				isPointInside[i][j][k] = isPointInside[p.x][p.y][p.z] + 1;
				int index = i * (gridResx.y * gridResx.z) + 
					j * gridResx.z + k;
				q.push(index);
			}
		}
		/*
		for (int i = p.x - 1; i <= p.x + 1; i++)
		{
			if (i < 0 || i >= gridRes.x)
				continue;
			for (int j = p.y - 1; j <= p.y + 1; j++)
			{
				if (j < 0 || j >= gridRes.y)
					continue;
				for (int k = p.z - 1; k <= p.z + 1; k++)
				{
					if (k < 0 || k >= gridRes.z)
						continue;
					if (i == p.x && j == p.y && k == p.z)
						continue;
					if (isPointInside[i][j][k] == 0)
					{
						isPointInside[i][j][k] = isPointInside[p.x][p.y][p.z] + 1;
						int index = grid2Index(vec3i(i , j , k));
						q.push(index);
					}
				}
			}
		}
		*/
	}
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
				q.maxSqrDis = 1e30f;
				vec3f pos(extXval[i] , extYval[j] , extZval[k]);
				tree->searchKnn(0 , pos , q);
				originF[i][j][k] = sqrt(q.maxSqrDis);
			}
		}
	}

	timer.PopAndDisplayTime("\nCalculate distance field: %.6f\n");
	/*
	for (int i = 0; i < sizeOriginF.x; i++)
		for (int j = 0; j < sizeOriginF.y; j++)
			for (int k = 0; k < sizeOriginF.z; k++)
				fprintf(fp , "(%.6f,%.6f,%.6f) , f = %.6f\n" , extXval[i] , extYval[j] , extZval[k] , originF[i][j][k]);
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

	timer.PopAndDisplayTime("\nGaussian smooth: %.6f\n");
	/*
	for (int i = 0; i < sizeOriginF.x; i++)
		for (int j = 0; j < sizeOriginF.y; j++)
			for (int k = 0; k < sizeOriginF.z; k++)
				fprintf(fp , "(%.6f,%.6f,%.6f) , f = %.6lf\n" , extXval[i] , extYval[j] , extZval[k] , f[i][j][k]);
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
		ts.axis[a] = vec3f(egvec(0 , t[a]) , egvec(1 , t[a]) , egvec(2 , t[a]));
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
	if (ev[0] < -1e-6f)
		s1 = 0.0;
	if (ev[1] < -1e-6f)
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
            writeLog("%.6f " , ts.tensor(i , j));
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
	timer.PopAndDisplayTime("\nCalculate hessian: %.6f\n");
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
		for (int j = 1; j < sizeOriginF.y; j++)
		{
			for (int k = 1; k < sizeOriginF.z - 1; k++)
			{
				calcTensorMetric(tensor[i][j][k]);
			}
		}
	}

	timer.PopAndDisplayTime("\nCalculate metric: %.6f\n");

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

int PointCloudUtils::grid2Index(const vec3i& gridPos)
{
	if (graphType != UNIFORM_GRAPH)
	{
		vec3f pos(xval[gridPos.x] , yval[gridPos.y] , zval[gridPos.z]);
		return point2Index[point2double(pos)];
	}
	else
	{
		return gridPos.x * (gridResx.y * gridResx.z) + 
			gridPos.y * gridResx.z + gridPos.z;
	}
}

vec3i PointCloudUtils::index2Grid(const int& index)
{
	vec3i res;
	res.x = index / (gridResx.y * gridResx.z);
	res.y = (index % (gridResx.y * gridResx.z)) / gridResx.z;
	res.z = index % gridResx.z;
	return res;
}

vec3i PointCloudUtils::nearestGridPoint(const vec3f& pos)
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

				double d = (vec3f(xval[xi + i] , yval[yi + j] , zval[zi + k]) - pos).length();
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

float PointCloudUtils::calcEdgeWeight(const vec3f& _v , 
	const Tensor& st , const Tensor& ed)
{
	//float len_v = _v.length();
	//vec3f v = _v / len_v;

    Vector3d dv(_v.x , _v.y , _v.z);
    
    float w = 0.5f * (sqrt(dv.transpose() * st.tensor * dv) +
                      sqrt(dv.transpose() * ed.tensor * dv));
    /*
	float cos_theta = std::min(1.f , std::abs(v.dot(st.axis[0])));
	float sin_theta = sqrt(1.f - cos_theta * cos_theta);
	vec3f v_proj = v - st.axis[0] * cos_theta;
	v_proj.normalize();

	float cos_phi = std::min(1.f , std::abs(v_proj.dot(st.axis[1])));
	float sin_phi = sqrt(1.f - cos_phi * cos_phi);
	float w = sqrt(st.axisLen[0] * SQR(cos_theta) + 
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
    
    writeLog("%.6f %.6f\n" , w , ww);
    */
	return w;
}

void PointCloudUtils::buildGraphFromUniformGrid()
{
	timer.PushCurrentTime();

	edges = 0;
	uniGraph.clear();
	uniGraph.resize(gridResx.x * gridResx.y * gridResx.z);
	for (int i = 0; i <= gridRes.x; i++)
	{
		for (int j = 0; j <= gridRes.y; j++)
		{
			for (int k = 0; k <= gridRes.z; k++)
			{
				for (int ii = i - 1; ii <= i + 1; ii++)
				{
					if (ii < 0 || ii > gridRes.x)
						continue;
					for (int jj = j - 1; jj <= j + 1; jj++)
					{
						if (jj < 0 || jj > gridRes.y)
							continue;
						for (int kk = k - 1; kk <= k + 1; kk++)
						{
							if (kk < 0 || kk > gridRes.z)
								continue;

							if (grid2Index(vec3i(i , j , k)) >= grid2Index(vec3i(ii , jj , kk)))
								continue;

							vec3f v = vec3f(xval[ii] , yval[jj] , zval[kk]) - 
									  vec3f(xval[i] , yval[j] , zval[k]);
							float w = calcEdgeWeight(v , tensor[i][j][k] , tensor[ii][jj][kk]);

							uniGraph[grid2Index(vec3i(i , j , k))].push_back(
								Edge(grid2Index(vec3i(ii , jj , kk)) , w));

							uniGraph[grid2Index(vec3i(ii , jj , kk))].push_back(
								Edge(grid2Index(vec3i(i , j , k)) , w));

							edges += 2;
						}
					}
				}
			}
		}
	}

	printf("Unigraph edges: %d\n" , edges);

	timer.PopAndDisplayTime("\nBuild uniform graph: %.6lf\n");
	/*
	int N = 2;
	for (int i = 0; i < N; i++)
	{
		fprintf(fp , "======== node #%d ========\n" , i);
		vec3i grid = index2Grid(i);
		vec3f st(xval[grid.x] , yval[grid.y] , zval[grid.z]);
		for (int j = 0; j < uniGraph[i].size(); j++)
		{
			grid = index2Grid(uniGraph[i][j].y);
			vec3f ed(xval[grid.x] , yval[grid.y] , zval[grid.z]);
			fprintf(fp , "(%d->%d), (%.6f,%.6f,%.6f)->(%.6f,%.6f,%.6f) %.6f\n" , 
				i , uniGraph[i][j].y ,
				st.x , st.y , st.z , ed.x , ed.y , ed.z , 
				uniGraph[i][j].w);
		}
	}
	*/
}

void PointCloudUtils::buildGraphFromAdaptiveGrid()
{
	timer.PushCurrentTime();

	edges = 0;
	adapGraph.clear();
	adapGraph.resize(nodes);

#pragma omp parallel for
	for (int i = 0; i < nodes; i++)
	{
		const Tensor* st = index2Tensor[i];
		for (int di = -1; di <= 1; di++)
		{
			for (int dj = -1; dj <= 1; dj++)
			{
				for (int dk = -1; dk <= 1; dk++)
				{
					if (di == 0 && dj == 0 && dk == 0)
						continue;
					vec3f pos = index2Point[i] + gridSize * vec3f(di , dj , dk);
					double hashVal = point2double(pos);
					if (point2Index.find(hashVal) == point2Index.end())
						continue;
					int j = point2Index[hashVal];
					const Tensor* ed = index2Tensor[j];
					vec3f v = pos - index2Point[i];
					float w = calcEdgeWeight(v , *st , *ed);

					adapGraph[i].push_back(Edge(j , w));
					edges++;
				}
			}
		}
	}
	
	printf("Adaptive graph edges: %d / %d\n" , edges , gridResx.x * gridResx.y * gridResx.z * 26);

	timer.PopAndDisplayTime("\nBuild adaptive graph: %.6lf\n");
	/*
	for (int i = 0; i < nodes; i++)
	{
		for (int j = 0; j < adapGraph[i].size(); j++)
		{
			int y = adapGraph[i][j].y;
			fprintf(fp , "(%d->%d): (%.6f,%.6f,%.6f)->(%.6f,%.6f,%.6f) %.6f\n" , i , adapGraph[i][j].y , 
				index2Point[i].x , index2Point[i].y , index2Point[i].z ,
				index2Point[y].x , index2Point[y].y , index2Point[y].z ,
				adapGraph[i][j].w);
		}
	}
	*/
}

void PointCloudUtils::subdivision()
{
	timer.PushCurrentTime();

	const vec3f dir[] = {vec3f(0,0,0.5), vec3f(0,0.5,0), vec3f(0,0.5,0.5),
		vec3f(0,0.5,1), vec3f(0,1,0.5), vec3f(0.5,0,0), vec3f(0.5,0,0.5),
		vec3f(0.5,0,1), vec3f(0.5,0.5,0), vec3f(0.5,0.5,0.5), vec3f(0.5,0.5,1),
		vec3f(0.5,1,0), vec3f(0.5,1,0.5), vec3f(0.5,1,1), vec3f(1,0,0.5),
		vec3f(1,0.5,0), vec3f(1,0.5,0.5), vec3f(1,0.5,1), vec3f(1,1,0.5)};

	subdivNodes = 0;
	subdivTensor.clear();
	for (int i = 0; i < gridRes.x; i++)
	{
		for (int j = 0; j < gridRes.y; j++)
		{
			for (int k = 0; k < gridRes.z; k++)
			{
				// subdivision if grid contains points
				if (isPointInside[i][j][k] >= 1)
				{
					for (int p = 0; p < 19; p++)
					{
						const vec3f& v = dir[p];

						// store point
						vec3f pos = vec3f(xval[i] , yval[j] , zval[k]) + v * gridSize;
						double hashVal = point2double(pos);
						if (point2Index.find(hashVal) != point2Index.end())
							continue;

						index2Point.push_back(pos);
						point2Index[hashVal] = nodes;

						// interpolate hessian
						Matrix3d hesMat;
						hesMat = lerpHessian(pos);
                        
						// calculate metric
						subdivTensor.push_back(Tensor());

						Tensor& ts = subdivTensor[subdivNodes];

						ts.hessian = hesMat;
						
						calcTensorDecomposition(ts);
						calcTensorMetric(ts);

						nodes++;
						subdivNodes++;
					}
				}
			}
		}
	}
	
	for (int i = 0; i < subdivNodes; i++)
		index2Tensor.push_back(&subdivTensor[i]);
	
	printf("Subdivision nodes: %d\n" , subdivNodes);

	timer.PopAndDisplayTime("\nSubdivision nodes and tensor: %.6f\n");
	/*
	for (int i = gridNodes; i < gridNodes + subdivNodes; i++)
	{
		fprintf(fp , "========%d=========\n" , i);
		const vec3f& pos = index2Point[i];
		fprintf(fp , "------(%.6lf,%.6lf,%.6lf)------\n" , pos.x , pos.y , pos.z);

		Tensor& ts = subdivTensor[i - gridNodes];
		for (int a = 0; a < 3; a++)
		{
			fprintf(fp , "(%.6lf,%.6lf,%.6lf), eigenVal = %.6lf, renderLen = %.6lf, length = %.6lf, renderLen = %.6lf\n" , 
				ts.axis[a].x , ts.axis[a].y , ts.axis[a].z , 
				ts.eigenVal[a] , pcRenderer->calcHessianRenderLength(ts.eigenVal[a]) ,
				ts.axisLen[a] , pcRenderer->calcMetricRenderLength(ts.axisLen[a]));
		}
		fprintf(fp , "-------\n");
		ts = *index2Tensor[i];
		for (int a = 0; a < 3; a++)
		{
			fprintf(fp , "(%.6lf,%.6lf,%.6lf), eigenVal = %.6lf, renderLen = %.6lf, length = %.6lf, renderLen = %.6lf\n" , 
				ts.axis[a].x , ts.axis[a].y , ts.axis[a].z , 
				ts.eigenVal[a] , pcRenderer->calcHessianRenderLength(ts.eigenVal[a]) ,
				ts.axisLen[a] , pcRenderer->calcMetricRenderLength(ts.axisLen[a]));
		}
	}
	*/
}

void PointCloudUtils::addSubdivisionEdges(Graph& g)
{
	timer.PushCurrentTime();

	g.resize(nodes);

	for (int i = gridNodes; i < nodes; i++)
	{
		const Tensor* st = index2Tensor[i];
		for (int di = -1; di <= 1; di++)
		{
			for (int dj = -1; dj <= 1; dj++)
			{
				for (int dk = -1; dk <= 1; dk++)
				{
					if (di == 0 && dj == 0 && dk == 0)
						continue;
					vec3f pos = index2Point[i] + gridSize * vec3f(di , dj , dk) * 0.5f;
					double hashVal = point2double(pos);
					if (point2Index.find(hashVal) == point2Index.end())
						continue;
					int j = point2Index[hashVal];
					if (j >= i)
						continue;
					const Tensor* ed = index2Tensor[j];
					vec3f v = pos - index2Point[i];
					float w = calcEdgeWeight(v , *st , *ed);

					g[i].push_back(Edge(j , w));
					g[j].push_back(Edge(i , w));

					edges += 2;
				}
			}
		}
	}
	printf("After subdivision, edges = %d\n" , edges);

	timer.PopAndDisplayTime("\nAdd subdivision edges: %.6f\n");
	/*
	for (int i = gridNodes; i < gridNodes + 10; i++)
	{
		fprintf(fp , "=====================\n");
		for (int j = 0; j < g[i].size(); j++)
		{
			fprintf(fp , "(%d <-> %d), (%.6f,%.6f,%.6f) <-> (%.6f,%.6f,%.6f) : %.6f\n" , i , g[i][j].y , 
				index2Point[i].x , index2Point[i].y , index2Point[i].z ,
				index2Point[g[i][j].y].x , index2Point[g[i][j].y].y , index2Point[g[i][j].y].z ,
				g[i][j].w);
		}
	}
	*/
}

Matrix3d PointCloudUtils::lerpHessian(const vec3f& pos)
{
    vec3f leftBottom(extXval[0] , extYval[0] , extZval[0]);
    vec3i maxSize(sizeOriginF);
    int xi = clampx((int)((pos.x - leftBottom.x) / gridSize.x) , 1 , maxSize.x - 2);
    int yi = clampx((int)((pos.y - leftBottom.y) / gridSize.y) , 1 , maxSize.y - 2);
    int zi = clampx((int)((pos.z - leftBottom.z) / gridSize.z) , 1 , maxSize.z - 2);
    
    vec3d v;
    /*
    v.x = ((double)pos.x - xval[xi]) / (xval[xi + 1] - xval[xi]);
    v.y = ((double)pos.y - yval[yi]) / (yval[yi + 1] - yval[yi]);
    v.z = ((double)pos.z - zval[zi]) / (zval[zi + 1] - zval[zi]);
    */
    v.x = clampx(((double)pos.x - extXval[xi]) / (extXval[xi + 1] - extXval[xi]) , 0.0 , 1.0);
    v.y = clampx(((double)pos.y - extYval[yi]) / (extYval[yi + 1] - extYval[yi]) , 0.0 , 1.0);
    v.z = clampx(((double)pos.z - extZval[zi]) / (extZval[zi + 1] - extZval[zi]) , 0.0 , 1.0);

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

Matrix3d PointCloudUtils::lerpTensor(const vec3f &pos)
{
    vec3f leftBottom(extXval[0] , extYval[0] , extZval[0]);
    vec3i maxSize(sizeOriginF);
    int xi = clampx((int)((pos.x - leftBottom.x) / gridSize.x) , 0 , maxSize.x - 1);
    int yi = clampx((int)((pos.y - leftBottom.y) / gridSize.y) , 0 , maxSize.y - 1);
    int zi = clampx((int)((pos.z - leftBottom.z) / gridSize.z) , 0 , maxSize.z - 1);
    
    vec3d v;
    /*
    v.x = ((double)pos.x - xval[xi]) / (xval[xi + 1] - xval[xi]);
    v.y = ((double)pos.y - yval[yi]) / (yval[yi + 1] - yval[yi]);
    v.z = ((double)pos.z - zval[zi]) / (zval[zi + 1] - zval[zi]);
    */
    v.x = clampx(((double)pos.x - extXval[xi]) / (extXval[xi + 1] - extXval[xi]) , 0.0 , 1.0);
    v.y = clampx(((double)pos.y - extYval[yi]) / (extYval[yi + 1] - extYval[yi]) , 0.0 , 1.0);
    v.z = clampx(((double)pos.z - extZval[zi]) / (extZval[zi + 1] - extZval[zi]) , 0.0 , 1.0);
    
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
            printf("(%.6f,%.6f,%.6f)\n" , v.x , v.y , v.z);
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
	for (int i = 0; i < pcData.size(); i++)
	{
		vec3f pos = pcData[i].pos;

		double hashVal = point2double(pos);
		if (point2Index.find(hashVal) != point2Index.end())
        {
            continue;
        }

		index2Point.push_back(pos);
		point2Index[hashVal] = nodes;
        
		Matrix3d hesMat;
		hesMat = lerpHessian(pos);

		Tensor ts;
		ts.hessian = hesMat;

		calcTensorDecomposition(ts);
		calcTensorMetric(ts);
        
        //Tensor tsx;
        //tsx.tensor = lerpTensor(pos);
        
        /*
        writeLog("=================\n");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                writeLog("%.6f " , ts.tensor(i , j));
            }
            writeLog("\n");
        }
        writeLog("-----------------\n");
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                writeLog("%.6f " , tsx.tensor(i , j));
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
	timer.PopAndDisplayTime("\nInterpolate point tensors: %.6f\n");
	
    /*
	for (int i = 0; i < nodes; i++)
	{
		writeLog("========%d=========\n" , i);
		vec3f pos = index2Point[i];
		writeLog("------(%.6lf,%.6lf,%.6lf)------\n" , pos.x , pos.y , pos.z);

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
                writeLog("%.6f " , ts.tensor(i , j));
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
		KnnQuery query(50);
		tree->searchKnn(0 , pcData[i].pos , query);
        //Tensor& st = pointTensor[i];
		for (int k = 0; k < query.knnPoints.size(); k++)
		{
			double hashVal = point2double(query.knnPoints[k].point->pos);
			int j = point2Index[hashVal];
			if (i == j)
				continue;
            //Tensor& ed = pointTensor[j];
			vec3f v = index2Point[j] - index2Point[i];
			float w;
            w = calcEdgeWeight(v , pointTensor[i] , pointTensor[j]);
            //w = calcEdgeWeight(v , st , ed);
			pointGraph[i].push_back(Edge(j , w));
		}
	}

	timer.PopAndDisplayTime("\nBuild point graph: %.6f\n");

/*
	for (int i = 0; i < nodes; i++)
	{
		fprintf(fp , "==============\n");
		for (int k = 0; k < pointGraph[i].size(); k++)
		{
			Edge e = pointGraph[i][k];
			fprintf(fp , "(%d->%d), (%.6f,%.6f,%.6f)->(%.6f,%.6f,%.6f): %.6f\n" ,
				i , e.y , index2Point[i].x , index2Point[i].y , index2Point[i].z ,
				index2Point[e.y].x , index2Point[e.y].y , index2Point[e.y].z , e.w);
		}
	}
*/
}

bool PointCloudUtils::dijkstra(const Graph& g , const int& source ,
							   DijkstraInfo& info , int *sink)
{
	int maxNodeNum = nodes;
	info.init(maxNodeNum);
	Heap q(maxNodeNum);

	info.dist[source] = 0.f;
	q.push(source , 0.f);

	while (!q.empty())
	{
		int now = q.top().index;
		float d = q.top().key;

		q.pop();

		if (sink != NULL && now == *sink)
			return 1;

		for (int k = 0; k < g[now].size(); k++)
		{
			const Edge& e = g[now][k];

			if (info.dist[e.y] > d + e.w)
			{
				info.dist[e.y] = d + e.w;
				info.prev[e.y] = now;

				q.push(e.y , info.dist[e.y]);
			}
		}
	}

	if (sink != NULL && info.dist[*sink] > 1e20f)
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
	
	fprintf(fp , "===========%.6f=========\n" , info.dist[sink]);
	
	for (int i = (int)pathVertex.size() - 1; i >= 1; i--)
	{
		fprintf(fp , "(%.6f,%.6f,%.6f) -> (%.6f,%.6f,%.6f) , %.6f\n" , pathVertex[i].x , pathVertex[i].y , pathVertex[i].z ,
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
        vec3f p = oldPath[i - 1] + (oldPath[i + 1] - oldPath[i - 1]) * 0.5f;
        vec3f v = (p - oldPath[i]) * pcRenderer->smoothScale;
        path[i] = oldPath[i] + v;
    }
    path[0] = oldPath[0];
    path[path.size() - 1] = oldPath[path.size() - 1];
}

vec3d calcGradient(const vec3f& v , const Matrix3d& tensor)
{
    Vector3d dv(v.x , v.y , v.z);
    Vector3d grad = tensor * dv;
    vec3d res(grad(0) , grad(1) , grad(2));
    res *= 0.5;
    res /= sqrt(dv.transpose() * tensor * dv);
    
    /*
    double tp = (dv.transpose() * tensor * dv);
    Matrix<double , 1 , 1> tmp = dv.transpose() * tensor * dv;
    
    if (sqrt(tp) < EPS)
    {
        printf("calc gradient: %.8f, (%.8f,%.8f,%.8f)\n" , sqrt(tp) , res.x , res.y , res.z);
    }
    
    printf("(%.6f,%.6f,%.6f) , %.8f\n" , v.x , v.y , v.z , tp);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            printf("%.6f " , tensor(i , j));
        }
        printf("\n");
    }
    */
    return res;
}

double calcAnisDist(const vec3f& pos1 , const Matrix3d& t1 , const vec3f& pos2 ,
                   const Matrix3d& t2 , const vec3f& pos3 , const Matrix3d& t3)
{
    double res = 0.0;
    vec3f v = pos2 - pos1;
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

Matrix3d lineSearchHessian(const vec3f& v , const Matrix3d& tensor)
{
    Matrix3d res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res(i , j) = 0.0;
    Vector3d dv(v.x , v.y , v.z);
    double denom = dv.transpose() * tensor * dv;
    /*
    if (std::abs(denom) < EPS)
    {
        printf("line search hessian: %.8lf\n" , denom);
        return res;
    }
    */
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
            res(i , j) = 0.5 * tensor(i , j) / sqrt(denom) -
                         0.5 * t1 * t2 / pow(denom , 1.5);
        }
    }
    return res;
}

double lineSearch(const vec3d& d , const vec3f& pos1 , const Matrix3d& t1 ,
                 const vec3f& pos2 , const Matrix3d& t2 ,
                 const vec3f& pos3 , const Matrix3d& t3)
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
    double res = -f1.dot(vd) / (vd.transpose() * f2 * vd);
    /*
    double tmp = vd.transpose() * f2 * vd;
    if (abs(tmp) < EPS)
    {
        printf("line search: %.8lf, %.8lf\n" , res , tmp);
        return 0.f;
    }
    */
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
    
//#pragma omp parallel for
    for (int i = 1; i < path.size() - 1; i++)
    {
        vec3d grad_double(0.0);
        vec3f v , p , mid;
        v = oldPath[i] - oldPath[i - 1];
        grad_double += calcGradient(v , ts[i - 1]) +
            calcGradient(v , ts[i]);

        v = oldPath[i] - oldPath[i + 1];
        grad_double += calcGradient(v , ts[i]) +
            calcGradient(v , ts[i + 1]);
        
        p = oldPath[i - 1] + (oldPath[i + 1] - oldPath[i - 1]) * 0.5f;
        vec3f grad(grad_double.x , grad_double.y , grad_double.z);
        
        vec3f _p0 = oldPath[i] - grad / grad.length() * 0.0001f;
        double _f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p0 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3f _p1 = oldPath[i];
        double _f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p1 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3f _p2 = oldPath[i] + grad / grad.length() * 0.0001f;
        double _f2 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                _p2 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        /*
        double delta = 1e-4;
        _p0 = oldPath[i] - vec3f(0 , 0 , delta);
        _f0 = calcAnisDist(oldPath[i - 1], ts[i - 1], _p0, ts[i], oldPath[i + 1], ts[i + 1]);
        _p1 = oldPath[i] + vec3f(0 , 0 , delta);
        _f1 = calcAnisDist(oldPath[i - 1], ts[i - 1], _p1, ts[i], oldPath[i + 1], ts[i + 1]);
        double diff = (_f1 - _f0) / delta * 0.5;
        printf("%.8lf , %.8lf\n" , grad.z , diff);
        */
        /*
        if (_f0 < _f1 && _f1 < _f2)
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
        mid = p - oldPath[i];
        double len = mid.length();
        grad /= grad_double.length();
        vec3d d = -grad_double / grad_double.length();
        double alpha = lineSearch(d , oldPath[i - 1] , ts[i - 1] , oldPath[i] ,
                                 ts[i] , oldPath[i + 1] , ts[i + 1]);
        /*
        if (!((_f0 < _f1) && (_f1 < _f2)))
            alpha = 0.0;
        
        vec3f p0 = oldPath[i] - grad * alpha * 0.97f;
        double f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p0 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3f p1 = oldPath[i] - grad * alpha;
        double f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p1 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        vec3f p2 = oldPath[i] - grad * alpha * 1.03f;
        double f2 = calcAnisDist(oldPath[i - 1] , ts[i - 1] ,
                                p2 , ts[i] , oldPath[i + 1] , ts[i + 1]);
        if (f0 > f1 && f1 < f2)
            printf("yes ");
        else
            printf("no ");
        printf("alpha = %.8lf, f(-1)=%.8lf, f(0)=%.8lf, f(1)=%.8lf\n" , alpha ,
               f0 , f1 , f2);
        */
        alpha = std::min(alpha , len);
        
        path[i] = oldPath[i] - grad * alpha * pcRenderer->smoothScale;
    }
    path[0] = oldPath[0];
    path[path.size() - 1] = oldPath[path.size() - 1];
    
    /*
    printf("========================\n");
    for (int i = 1; i < path.size() - 1; i++)
    {
        vec3f grad(0.f) , v , p , mid;
        v = path[i] - path[i - 1];
        grad += calcGradient(v , ts[i - 1]) +
            calcGradient(v , ts[i]);
        v = path[i + 1] - path[i];
        grad += calcGradient(v , ts[i]) +
            calcGradient(v , ts[i + 1]);
        printf("pos = (%.6f,%.6f,%.6f), grad = (%.6f,%.6f,%.6f), norm = %.6f\n" ,
               path[i].x , path[i].y , path[i].z ,
               grad.x , grad.y , grad.z , grad.length());
        
        float f0 = calcAnisDist(oldPath[i - 1] , ts[i - 1] , oldPath[i] ,
                                ts[i] , oldPath[i + 1] , ts[i + 1]);
        float f1 = calcAnisDist(oldPath[i - 1] , ts[i - 1] , path[i] ,
                                ts[i] , oldPath[i + 1] , ts[i + 1]);
        printf("prev f = %.8f , now f = %.8f\n" , f0 , f1);
    }
    */
}

void PointCloudUtils::calcFirstOrderDerivative(double*** &f , const int& dir , double*** &df)
{
	/*
	if (df == NULL)
		allocate3(df , gridRes + vec3i(1 , 1 , 1));

	if (dir == 0)
	{
#pragma omp parallel for
		for (int i = 0; i <= gridRes.x; i++)
		{
			for (int j = 0; j <= gridRes.y; j++)
			{
				for (int k = 0; k <= gridRes.z; k++)
				{
					if (i == 0)
						df[i][j][k] = (f[i + 1][j][k] - f[i][j][k]) / gridSize.x;
					else if (i == gridRes.x)
						df[i][j][k] = (f[i][j][k] - f[i - 1][j][k]) / gridSize.x;
					else
						df[i][j][k] = (f[i + 1][j][k] - f[i - 1][j][k]) / (2.0 * gridSize.x);
				}
			}
		}
	}
	else if (dir == 1)
	{
#pragma omp parallel for
		for (int i = 0; i <= gridRes.x; i++)
		{
			for (int j = 0; j <= gridRes.y; j++)
			{
				for (int k = 0; k <= gridRes.z; k++)
				{
					if (j == 0)
						df[i][j][k] = (f[i][j + 1][k] - f[i][j][k]) / gridSize.y;
					else if (j == gridRes.y)
						df[i][j][k] = (f[i][j][k] - f[i][j - 1][k]) / gridSize.y;
					else
						df[i][j][k] = (f[i][j + 1][k] - f[i][j - 1][k]) / (2.0 * gridSize.y);
				}
			}
		}
	}
	else if (dir == 2)
	{
#pragma omp parallel for
		for (int i = 0; i <= gridRes.x; i++)
		{
			for (int j = 0; j <= gridRes.y; j++)
			{
				for (int k = 0; k <= gridRes.z; k++)
				{
					if (k == 0)
						df[i][j][k] = (f[i][j][k + 1] - f[i][j][k]) / gridSize.z;
					else if (k == gridRes.z)
						df[i][j][k] = (f[i][j][k] - f[i][j][k - 1]) / gridSize.z;
					else
						df[i][j][k] = (f[i][j][k + 1] - f[i][j][k - 1]) / (2.0 * gridSize.z);
				}
			}
		}
	}
	*/
}
