#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include <GL/glut.h>
#include "curveNet.h"
#include "pointCloudUtils.h"
#include "pointCloudRenderer.h"
#include "nvVector.h"

PointCloudRenderer::PointCloudRenderer()
{
    pcUtils = NULL;
    curveNet = NULL;
    dispCurveNet = new CurveNet();
    cos36.resize(36);
    sin36.resize(36);
    for(unsigned i=0; i<36; i++)
    {
        cos36[i] = cos(double(i) * PI / 18.);
        sin36[i] = sin(double(i) * PI / 18.);
    }
    init();
}

PointCloudRenderer::~PointCloudRenderer()
{
    delete dispCurveNet;
}

void PointCloudRenderer::init()
{
	isShowPoints = false;
	isShowUniformGrid = false;
	isShowAdaptiveGrid = false;
	isShowHessian = false;
	isShowMetric = false;
	isShowPointCloud = true;

	pathVertex.clear();
	
	axisU.clear();
	axisV.clear();

	discRenderBaseRadius = discRenderRadiusScale = 0.005;
	hessianRenderBaseLength = hessianRenderLengthScale = 0.001;
	metricRenderBaseLength = metricRenderLengthScale = 0.005;

	windowSizeX = windowSizeY = 0;
	rgbBuffer = NULL;
	pickedPoint = NULL;
    pickedDispPoint = NULL;
	lastPoint = NULL;
    lastDispPoint = NULL;
    pickedCurve = -1;
    
	isCtrlPress = false;

	if (pcUtils == NULL)
		return;
    
	isNormalInfo = true;
	for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		if (pcUtils->pcData[i].n.length() < 0.5)
		{
			isNormalInfo = false;
			break;
		}
		vec3d a , b;
		if (std::abs(pcUtils->pcData[i].n.x) < 0.999)
		{
			a.x = 0;
			a.y = -pcUtils->pcData[i].n.z;
			a.z = pcUtils->pcData[i].n.y;
		}
		else
		{
			a.y = 0;
			a.x = -pcUtils->pcData[i].n.z;
			a.z = pcUtils->pcData[i].n.x;
		}
		a.normalize();
		b = pcUtils->pcData[i].n.cross(a);
		b.normalize();
		axisU.push_back(a);
		axisV.push_back(b);
	}
	
	vec3d diag = pcUtils->box.rt - pcUtils->box.lb;
	selectionOffset = std::min(diag.x , std::min(diag.y , diag.z)) / 40.0;

	initSelectionBuffer();

	callListPoints();
	callListSurfelDisc();
    
    pathForComp.resize(3);
    for (int i = 0; i < 3; i++)
        pathForComp[i].clear();
    useBSpline = true;
    dispCurveNet->clear();
}

void PointCloudRenderer::drawPoint(const vec3d& pos)
{
	glBegin(GL_POINTS);
	glVertex3f(pos.x , pos.y , pos.z);
	glEnd();
}

void PointCloudRenderer::drawPoints()
{
	glBegin(GL_POINTS);
	for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		glNormal3f(pcUtils->pcData[i].n.x , pcUtils->pcData[i].n.y , pcUtils->pcData[i].n.z);
		glVertex3f(pcUtils->pcData[i].pos.x , pcUtils->pcData[i].pos.y , pcUtils->pcData[i].pos.z);
	}
	glEnd();
}

void PointCloudRenderer::drawCircle(const vec3d& origin , const vec3d& a , const vec3d& b , 
	const double& r)
{
	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(origin.x , origin.y , origin.z);
	for (int i = 0; i <= 36; i+=9)
	{
		vec3d p = origin + a * r * cos36[i % 36] + b * r * sin36[i % 36];
		glVertex3f(p.x , p.y , p.z);
	}
	glEnd();
}

void PointCloudRenderer::drawCircle(const vec3d& origin , const vec3d& a , const vec3d& b , 
	const double& r , const vec3d& n)
{
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(n.x , n.y , n.z);
	glVertex3f(origin.x , origin.y , origin.z);
	for (int i = 0; i <= 36; i+=9)
	{
		vec3d p = origin + a * r * cos36[i % 36] + b * r * sin36[i % 36];
		glNormal3f(n.x , n.y , n.z);
		glVertex3f(p.x , p.y , p.z);
	}
	glEnd();
}

void PointCloudRenderer::drawLines(const Path& v)
{
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < v.size(); i++)
	{
		glVertex3f(v[i].x , v[i].y , v[i].z);
	}
	glEnd();
}

void PointCloudRenderer::drawCube(const vec3d& lb , const vec3d& rt)
{
	glBegin(GL_LINES);
	glVertex3f(lb.x , lb.y , lb.z);
	glVertex3f(lb.x , lb.y , rt.z);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(lb.x , rt.y , lb.z);
	glVertex3f(lb.x , rt.y , rt.z);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(rt.x , lb.y , lb.z);
	glVertex3f(rt.x , lb.y , rt.z);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(rt.x , rt.y , lb.z);
	glVertex3f(rt.x , rt.y , rt.z);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3f(lb.x , lb.y , lb.z);
	glVertex3f(lb.x , rt.y , lb.z);
	glVertex3f(rt.x , rt.y , lb.z);
	glVertex3f(rt.x , lb.y , lb.z);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3f(lb.x , lb.y , rt.z);
	glVertex3f(lb.x , rt.y , rt.z);
	glVertex3f(rt.x , rt.y , rt.z);
	glVertex3f(rt.x , lb.y , rt.z);
	glEnd();
}

void PointCloudRenderer::drawEllipse(const vec3d& origin , 
									 const vec3d& majorAxis , const double& majorLen , 
									 const vec3d& minorAxis , const double& minorLen)
{
	vec3d n = majorAxis.cross(minorAxis);
	glBegin(GL_POLYGON);
	for (unsigned i = 0; i < 36; i++) 
	{
		vec3d p = origin + cos36[i] * majorAxis * majorLen + sin36[i] * minorAxis * minorLen;
		glNormal3d(n.x,n.y,n.z);
		glVertex3d(p.x,p.y,p.z);
	}
	glEnd();
}

void PointCloudRenderer::drawEllipsoid(const vec3d& origin , 
									   const vec3d& a , const double& la , 
									   const vec3d& b , const double& lb , 
									   const vec3d& c , const double& lc)
{
	for (int i = 0; i < 18; i+=2)
	{
		int t = i % 36;
		glBegin(GL_TRIANGLE_STRIP);
		for (int j = 0; j <= 36; j+=2)
		{
			int s = j % 36;
			vec3d p1 = origin + a * la * sin36[t] * cos36[s] + 
				b * lb * sin36[t] * sin36[s] + c * lc * cos36[t];
			vec3d p2 = origin + a * la * sin36[(t + 2) % 36] * cos36[s] +
				b * lb * sin36[(t + 2) % 36] * sin36[s] + c * lc * cos36[(t + 2) % 36];

			glVertex3f(p1.x , p1.y , p1.z);
			glVertex3f(p2.x , p2.y , p2.z);
		}
		glEnd();
	}
}

void PointCloudRenderer::callListPoints()
{
	glNewList(LIST_POINTS , GL_COMPILE);

	drawPoints();

	glEndList();
}

void PointCloudRenderer::callListSurfelDisc()
{
	glNewList(LIST_SURFEL_DISC , GL_COMPILE);

	for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		drawCircle(pcUtils->pcData[i].pos , axisU[i] , axisV[i] , discRenderRadiusScale , pcUtils->pcData[i].n);
	}

	glEndList();
}

void PointCloudRenderer::renderPoints()
{
	if (pcUtils == NULL)
		return;
	glColor3f(0.f , 0.f , 0.f);
	glPointSize(2.f);
	
	glCallList(LIST_POINTS);
    //callListPoints();
}

void PointCloudRenderer::renderSurfelDisc()
{
	if (pcUtils == NULL)
		return;
	glColor3f(0.5f , 0.539f , 0.527f);

	glCallList(LIST_SURFEL_DISC);
    //callListSurfelDisc();
}

void PointCloudRenderer::renderPointCloud()
{
	if (!isShowPointCloud)
		return;
	if (isShowPoints || !isNormalInfo)
		renderPoints();
	else 
		renderSurfelDisc();
}

void PointCloudRenderer::renderUniformGrid()
{
	if (pcUtils == NULL)
		return;
	if (!isShowUniformGrid)
		return;
	glColor3f(0.f , 0.f , 0.f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0x4444);
	glLineWidth(0.1f);
	for (int i = 0; i < pcUtils->gridRes.x; i++)
	{
		for (int j = 0; j < pcUtils->gridRes.y; j++)
		{
			for (int k = 0; k < pcUtils->gridRes.z; k++)
			{
				vec3d lb(pcUtils->xval[i] , pcUtils->yval[j] , pcUtils->zval[k]);
				vec3d rt = lb + pcUtils->gridSize;
				drawCube(lb , rt);
			}
		}
	}
	glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderHessian()
{
	if (pcUtils == NULL)
		return;
	if (!isShowHessian)
		return;
	glColor3f(0.1f , 0.1f , 1.f);
	glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);

	int st = 0;
	vec3i delta = pcUtils->gridRes / 8;
	delta.x = std::max(delta.x , 1);
	delta.y = std::max(delta.y , 1);
	delta.z = std::max(delta.z , 1);

	for (int i = st; i <= pcUtils->gridRes.x; i += delta.x)
	{
		for (int j = st; j <= pcUtils->gridRes.y; j += delta.y)
		{
			for (int k = st; k <= pcUtils->gridRes.z; k += delta.z)
			{
				vec3d origin(pcUtils->xval[i] , pcUtils->yval[j] , pcUtils->zval[k]);
				vec3d u , v , w;
				double lu , lv , lw;
				u = pcUtils->tensor[i][j][k].axis[0];
				v = pcUtils->tensor[i][j][k].axis[1];
				w = pcUtils->tensor[i][j][k].axis[2];
				lu = calcHessianRenderLength(pcUtils->tensor[i][j][k].eigenVal[0]);
				lv = calcHessianRenderLength(pcUtils->tensor[i][j][k].eigenVal[1]);
				lw = calcHessianRenderLength(pcUtils->tensor[i][j][k].eigenVal[2]);
				drawEllipsoid(origin , u , lu , v , lv , w , lw);
			}
		}
	}
}

void PointCloudRenderer::renderMetric()
{
	if (pcUtils == NULL)
		return;
	if (!isShowMetric)
		return;
	glColor3f(0.1f , 0.1f , 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);

	int st = 0;
	vec3i delta = pcUtils->gridRes / 8;
	delta.x = std::max(delta.x , 1);
	delta.y = std::max(delta.y , 1);
	delta.z = std::max(delta.z , 1);

	for (int i = st; i <= pcUtils->gridRes.x; i += delta.x)
	{
		for (int j = st; j <= pcUtils->gridRes.y; j += delta.y)
		{
			for (int k = st; k <= pcUtils->gridRes.z; k += delta.z)
			{
				vec3d origin(pcUtils->xval[i] , pcUtils->yval[j] , pcUtils->zval[k]);
				vec3d u , v , w;
				double lu , lv , lw;
				u = pcUtils->tensor[i][j][k].axis[0];
				v = pcUtils->tensor[i][j][k].axis[1];
				w = pcUtils->tensor[i][j][k].axis[2];
				lu = calcMetricRenderLength(pcUtils->tensor[i][j][k].axisLen[0]);
				lv = calcMetricRenderLength(pcUtils->tensor[i][j][k].axisLen[1]);
				lw = calcMetricRenderLength(pcUtils->tensor[i][j][k].axisLen[2]);
				drawEllipsoid(origin , u , lu , v , lv , w , lw);
			}
		}
	}
}

void PointCloudRenderer::renderSelectedPoints()
{
    if (dispCurveNet == NULL)
        return;

	glColor3f(1.f , 0.f , 0.f);
	glPointSize(12.f);
	for (int i = 0; i < (int)dispCurveNet->nodes.size(); i++)
	{
        if (!dispCurveNet->nodesStat[i]) continue;
		drawPoint(dispCurveNet->nodes[i]);
	}

	if (pickedDispPoint != NULL)
	{
		glColor3f(0.f , 1.f , 0.f);
        drawPoint(*pickedDispPoint);
	}

	if (lastPoint != NULL)
	{
		glColor3f(0.f , 1.f , 0.f);
		drawPoint(*lastDispPoint);
	}
}

void PointCloudRenderer::renderCurrentPath()
{
	glColor3f(0.f , 0.f , 1.f);
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
	drawLines(pathVertex);
}


void PointCloudRenderer::renderStoredPaths()
{
    if (dispCurveNet == NULL)
        return;
    
	glColor3f(0.f , 0.f , 1.f);
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
	for (int i = 0; i < dispCurveNet->polyLines.size(); i++)
	{
		drawLines(dispCurveNet->polyLines[i]);
	}
    
    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderPickedCurve()
{
    if (pickedCurve == -1) return;

    glColor3f(1.f , 0.f , 0.f);
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
	drawLines(dispCurveNet->polyLines[pickedCurve]);
    
    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderPathForComp()
{
    if (pathForComp.size() < 2)
        return;
    
    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(2.f);
    
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
	drawLines(pathForComp[0]);
	
    glColor3f(0.f , 1.f , 0.f);
    
    drawLines(pathForComp[1]);
    
    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::render()
{
	renderPointCloud();
	renderHessian();
	renderMetric();
	renderSelectedPoints();
	renderCurrentPath();
	renderStoredPaths();
    renderPickedCurve();
    //renderPathForComp();
}

void PointCloudRenderer::initSelectionBuffer()
{
	int pointSize = pcUtils->pcData.size();
	glObjColors.resize(pointSize);

	unsigned char r , g , b;

	for (int i = 0; i < pointSize; i++)
	{
		vec3uc& col = glObjColors[i];

		r = i / 65536;
		g = (i - (int)r * 65536) / 256;
		b = i - (int)r * 65536 - g * 256;

		col.x = r;
		col.y = g;
		col.z = b;
	}

	callListSelectionBuffer();
}

void PointCloudRenderer::callListSelectionBuffer()
{
	glNewList(LIST_SELECTION_BUFFER , GL_COMPILE);

	for (int i = 0; i < axisU.size(); i++)
	{
		vec3uc& col = glObjColors[i];
		glColor3ub(col.x , col.y , col.z);
		drawCircle(pcUtils->pcData[i].pos , axisU[i] , axisV[i] , selectionOffset);
	}

	glEndList();
}

void PointCloudRenderer::updateSelectionBuffer()
{
	glDrawBuffer(GL_BACK);

	glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
	
	if (isNormalInfo)
	{
		glCallList(LIST_SELECTION_BUFFER);
        //callListSelectionBuffer();
	}
	
	//fprintf(fr , "%.6f\n" , selectionOffset);

	int windowWidth = windowSizeX;
	int windowHeight = windowSizeY;

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT , viewport);
	windowSizeX = viewport[2]; windowSizeY = viewport[3];

	if (windowSizeX != windowWidth || windowSizeY != windowHeight)
	{
		if (rgbBuffer != NULL)
			delete[] rgbBuffer;
		rgbBuffer = new unsigned char[windowSizeX * windowSizeY * 3];
	}

	if (rgbBuffer != NULL)
	{
		glReadBuffer(GL_BACK);
		glReadPixels((GLint)0 , (GLint)0 , (GLint)windowSizeX , (GLint)windowSizeY ,
			GL_RGB , GL_UNSIGNED_BYTE , rgbBuffer);
	}

	// DEBUG: output buffer
    if (false)
	{
        printf("(%d,%d)\n" , windowSizeX , windowSizeY);
        for (int i=0;i<windowSizeX*windowSizeY*3;i+=3) 
        {
            if (rgbBuffer[i] == 255 && rgbBuffer[i + 1] == 255 &&
                rgbBuffer[i + 2] == 255)
            {
                continue;
            }

			writeLog("pixel num = %d, (%d,%d,%d)\n" , i ,
                rgbBuffer[i] , rgbBuffer[i + 1] , rgbBuffer[i + 2]);
	    }
        writeBMP("selectBuffer.bmp" , windowSizeX , windowSizeY , rgbBuffer);
	}
}

int PointCloudRenderer::selectionByColorMap(int mouseX , int mouseY)
{
	if (rgbBuffer == NULL)
		return -1;

	int pixelId = (mouseX + (windowSizeY - mouseY) * windowSizeX) * 3;
	int r = rgbBuffer[pixelId];
	int g = rgbBuffer[pixelId + 1];
	int b = rgbBuffer[pixelId + 2];

	int selectedObj = r * 65536 + g * 256 + b;
	if (selectedObj >= glObjColors.size() ||
        selectedObj < 0)
		return -1;
	return selectedObj;
}

void PointCloudRenderer::pickPoint(int mouseX , int mouseY , bool isStore)
{
	int selectedObj = selectionByColorMap(mouseX , mouseY);
    /*
	printf("%d, (%.6f,%.6f,%.6f)\n" , selectedObj ,
        pcUtils->pcData[selectedObj].pos.x ,
        pcUtils->pcData[selectedObj].pos.y ,
        pcUtils->pcData[selectedObj].pos.z);
    */
	if (selectedObj != -1)
	{
		pickedPoint = &pcUtils->pcData[selectedObj].pos;
        pickedDispPoint = pickedPoint;
		vec3d pos , dispPos;
		int edi;
        
        pos = *pickedPoint;
        dispPos = *pickedPoint;
		
        // snapping when it is near curve
        bool isSnap = false;
        int breakLine , breakPoint;
        double min_dist = 1e20;
        
        for (int i = 0; i < dispCurveNet->numPolyLines; i++)
        {
            for (int j = 0; j < dispCurveNet->polyLines[i].size(); j++)
            {
                double tmp = (pos - dispCurveNet->polyLines[i][j]).length();
                if (tmp < min_dist)
                {
                    min_dist = tmp;
                    breakLine = i;
                    breakPoint = j;
                }
            }
        }
        double snapOffset = selectionOffset;
        if (isSnap && (breakPoint == 0 || breakPoint == (int)dispCurveNet->polyLines[breakLine].size() - 1))
        {
            snapOffset *= 2.0;
        }
        
        if (min_dist < snapOffset)
        {
            pos = curveNet->polyLines[breakLine][breakPoint];
            dispPos = dispCurveNet->polyLines[breakLine][breakPoint];
            *pickedPoint = pos;
            *pickedDispPoint = dispPos;
            isSnap = true;
            //printf("snap!\n");
        }
        
		edi = pcUtils->point2Index[point2double(pos)];

        // path smoothing
		if (lastPoint != NULL)
		{
			if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
				pcUtils->traceBack(pcUtils->pointGraphInfo , edi , pathVertex);
            
            pathForComp[0] = pathVertex;
            pathForComp[1] = pathVertex;

            for (int i = 0; i < smoothIter; i++)
            {
                pcUtils->laplacianSmooth(pathForComp[1]);
                pcUtils->gradientDescentSmooth(pathVertex);
            }
            pathForComp[2] = pathVertex;
            
            if (pathChoice < 2)
                pathVertex = pathForComp[pathChoice];

            if (useBSpline)
            {
                pcUtils->convert2Spline(pathVertex);
            }
            pathVertex[0] = dispPos;
		}
        
		if (isStore)
		{
			int sti;
            bool newNode = true;
            if (lastPoint == NULL)
            {
                curveNet->startPath(pos);
                dispCurveNet->startPath(dispPos);
            }
            else
            {
                if (isSnap && (breakPoint == 0 || breakPoint == (int)curveNet->polyLines[breakLine].size() - 1))
                {
                    newNode = false;
                }
                curveNet->extendPath(*lastPoint , pos , pathForComp[0] , newNode);
                dispCurveNet->extendPath(*lastDispPoint , dispPos , pathVertex , newNode);
                if (isSnap && newNode)
                {
                    curveNet->breakPath(breakLine , breakPoint);
                    dispCurveNet->breakPath(breakLine , breakPoint);
                }
            }
            
            // dispCurveNet->debugLog();
            
			sti = edi;

            if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
				pcUtils->dijkstra(pcUtils->pointGraph , sti , pcUtils->pointGraphInfo);

            lastPoint = &curveNet->nodes[curveNet->getNodeIndex(pos)];
            lastDispPoint = &dispCurveNet->nodes[dispCurveNet->getNodeIndex(dispPos)];
		}
	}
	else
	{
		pickedPoint = NULL;
        pickedDispPoint = NULL;
	}
}

std::vector<vec3d> getRay(int mouseX,int mouseY)
{
	// ********* ray from eye to mouse location *********
	GLint aViewport[4];
	GLdouble matMV[16], matProj[16];
	GLdouble wx, wy, wz;  //  temp world x, y, z coords

	glGetIntegerv (GL_VIEWPORT, aViewport);
	glGetDoublev (GL_MODELVIEW_MATRIX, matMV);
	glGetDoublev (GL_PROJECTION_MATRIX, matProj);
	//  note viewport[3] is height of window in pixels
	mouseY = aViewport[3] - (GLint) mouseY - 1;
	//printf ("Coordinates at cursor are (%4d, %4d)\n", mouseX, mouseY);

	gluUnProject ((GLdouble) mouseX, (GLdouble) mouseY, 0.0,
		matMV, matProj, aViewport, &wx, &wy, &wz);
	vec3d lineP0(wx,wy,wz);
	//printf ("World coords at z=0.0 are (%f, %f, %f)\n", wx, wy, wz);

	gluUnProject ((GLdouble) mouseX, (GLdouble) mouseY, 1.0, matMV, matProj, aViewport, &wx, &wy, &wz);
	vec3d lineP1(wx,wy,wz);
	//printf ("World coords at z=1.0 are (%f, %f, %f)\n", wx, wy, wz);

	std::vector<vec3d> ray;
	ray.push_back(lineP0);ray.push_back(lineP1);
	return ray;
}

void PointCloudRenderer::pickCurve(int mouseX , int mouseY , bool isDelete)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
	pickedCurve = -1;
	double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < dispCurveNet->polyLines.size(); i++)
    {
		for (int j = 0; j < dispCurveNet->polyLines[i].size(); j++)
        {
			vec3d p = dispCurveNet->polyLines[i][j];
			double paramU = (
				((p[0]-rayStr[0])*(rayEnd[0]-rayStr[0])) +
				((p[1]-rayStr[1])*(rayEnd[1]-rayStr[1])) +
				((p[2]-rayStr[2])*(rayEnd[2]-rayStr[2]))
				)/p0p1LenSquared;

			vec3d lineP = rayStr + (rayEnd - rayStr) * paramU;
			double distance = (lineP - p).length();
			if (minDistance > distance)
            {
				minDistance = distance;
				pickedCurve = i;
			}
		}
	}
    if (minDistance > selectionOffset) pickedCurve = -1;
	if (minDistance <= selectionOffset && isDelete)
    {
        curveNet->deletePath(pickedCurve);
        dispCurveNet->deletePath(pickedCurve);
        
        //dispCurveNet->debugLog();
    }
}

void PointCloudRenderer::clearPaths()
{
	pathVertex.clear();
    for (int i = 0; i < 3; i++)
        pathForComp[i].clear();
	pickedPoint = NULL;
    pickedDispPoint = NULL;
	lastPoint = NULL;
    lastDispPoint = NULL;
    pickedCurve = -1;
    
    curveNet->clear();
    dispCurveNet->clear();
}
