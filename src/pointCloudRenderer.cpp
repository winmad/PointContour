#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include <GL/glut.h>
#include "curveNet.h"
#include "pointCloudUtils.h"
#include "pointCloudRenderer.h"
#include "nvVector.h"

PointCloudRenderer::PointCloudRenderer()
{
    pcUtils = NULL;
    // curveNet = NULL;
    // dispCurveNet = new CurveNet();
    dispCurveNet = NULL;
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
    // delete dispCurveNet;
}

void PointCloudRenderer::init()
{
	isShowPoints = false;
	isShowUniformGrid = false;
	isShowAdaptiveGrid = false;
	isShowHessian = false;
	isShowMetric = false;
	isShowPointCloud = true;
    isShowCtrlNodes = false;
    isShowCollinear = false;
    constraintsVisual = 0;
    bspIndex = 0; curveIndex = 0;

	pathVertex.clear();
    bsp.clear();
	
	axisU.clear();
	axisV.clear();

	discRenderBaseRadius = discRenderRadiusScale = 0.005;
	hessianRenderBaseLength = hessianRenderLengthScale = 0.001;
	metricRenderBaseLength = metricRenderLengthScale = 0.005;

	windowSizeX = windowSizeY = 0;
	rgbBuffer = NULL;
	// pickedPoint = NULL;
    setNull(pickedDispPoint);
	// lastPoint = NULL;
    setNull(lastDispPoint);
    pickedCurve = -1;
    pickedCycle = -1;
    
	isCtrlPress = false;
    isAltPress = false;
	isShiftPress = false;

#if defined(_WIN32)
	isAutoOpt = false;
#else
    isAutoOpt = true;
#endif
    
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
	selectionOffset = std::min(diag.x , std::min(diag.y , diag.z)) / 50.0;

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

void PointCloudRenderer::drawLine(const vec3d& st , const vec3d& ed)
{
    glBegin(GL_LINE_STRIP);
    glVertex3f(st.x , st.y , st.z);
    glVertex3f(ed.x , ed.y , ed.z);
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

	if (isValid(pickedDispPoint))
	{
        if (snapToNode)
        {
            glColor3f(1.f , 1.f , 0.f);
        }
        else if (snapToCurve)
        {
            glColor3f(0.8f , 0.f , 0.8f);
        }
        else
        {
            glColor3f(0.f , 1.f , 0.f);
        }
        drawPoint(pickedDispPoint);
	}

	if (isValid(lastDispPoint))
	{
		glColor3f(0.f , 1.f , 0.f);
		drawPoint(lastDispPoint);
	}
}

void PointCloudRenderer::renderCurrentPath()
{
    if (isShowCollinear) return;
    if (constraintsVisual != 0) return;
    
	glColor3f(0.f , 0.f , 1.f);
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
	drawLines(pathVertex);
}

void PointCloudRenderer::renderStoredPaths()
{
    if (dispCurveNet == NULL) return;
    if (isShowCollinear) return;
    if (constraintsVisual != 0) return;
    
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

void PointCloudRenderer::renderCtrlNodes()
{
    if (!isShowCtrlNodes) return;
    if (dispCurveNet == NULL) return;
    
    glColor3f(0.f , 1.f , 0.f);
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
    for (int i = 0; i < dispCurveNet->bsplines.size(); i++)
    {
        drawLines(dispCurveNet->bsplines[i].ctrlNodes);
    }
    drawLines(bsp.ctrlNodes);

    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderCollinearLines()
{
    if (constraintsVisual != 1) return;
    if (dispCurveNet->bsplines.size() == 0) return;

    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex] ,
        dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);

    glColor3f(0.f , 0.f , 1.f);
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        for (int j = 0; j < (int)dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            if (i == bspIndex && j == curveIndex) continue;
            if (!dispCurveNet->collinearSet.sameRoot(bspIndex , curveIndex , i , j)) continue;
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderParallelLines()
{
    if (constraintsVisual != 2) return;
    if (dispCurveNet->bsplines.size() == 0) return;

    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex] ,
        dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);

    if (dispCurveNet->curveType[bspIndex] != 1) return;
    
    glColor3f(0.f , 0.f , 1.f);
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        if (dispCurveNet->bsplines[i].ctrlNodes.size() == 0) continue;
        if (dispCurveNet->curveType[i] != 1) continue;
        if (i != bspIndex && !dispCurveNet->parallelSet.sameRoot(bspIndex , 0 , i , 0)) continue;
        for (int j = 0; j < (int)dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            if (i == bspIndex && j == curveIndex) continue;
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderCoplanarLines()
{
    if (constraintsVisual != 3) return;
    if (dispCurveNet->bsplines.size() == 0) return;

    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex] ,
        dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);

    if (dispCurveNet->curveType[bspIndex] == 2) return;

    glColor3f(0.f , 0.f , 1.f);

    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        if (dispCurveNet->bsplines[i].ctrlNodes.size() == 0) continue;
        if (dispCurveNet->curveType[i] == 2) continue;
        if (dispCurveNet->coplanarSet.getMark(bspIndex , 0 , i , 0) != 1) continue;
        for (int j = 0; j < (int)dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            if (i == bspIndex && j == curveIndex) continue;
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderOrthogonalLines()
{
    if (constraintsVisual != 4) return;
    if (dispCurveNet->bsplines.size() == 0) return;

    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex] ,
        dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);
    
    glColor3f(0.f , 0.f , 1.f);
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        if (dispCurveNet->bsplines[i].ctrlNodes.size() == 0) continue;
        
        for (int j = 0; j < (int)dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            if (i == bspIndex && j == curveIndex) continue;
            if (dispCurveNet->orthoSet.getMark(bspIndex , curveIndex , i , j) != 1) continue;
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderTangentLines()
{
    if (constraintsVisual != 5) return;
    if (dispCurveNet->bsplines.size() == 0) return;

    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex] ,
        dispCurveNet->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);

    // printf("/======= (%d , %d) ======/\n" , bspIndex , curveIndex);
    glColor3f(0.f , 0.f , 1.f);
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        if (dispCurveNet->bsplines[i].ctrlNodes.size() == 0) continue;
        
        for (int j = 0; j < (int)dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            if (i == bspIndex && j == curveIndex) continue;
            if (dispCurveNet->orthoSet.getMark(bspIndex , curveIndex , i , j) != 2) continue;
            // printf("(%d , %d) <==> (%d , %d), %d\n" , bspIndex , curveIndex , i , j ,
                // dispCurveNet->orthoSet.getMark(bspIndex , curveIndex , i , j));
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderUnsavedCycles()
{
    glColor3f(1.f , 0.f , 1.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < unsavedCyclePoints.size(); i++)
    {
        for (int j = 0; j < unsavedCyclePoints[i].size(); j++)
        {
            drawLines(unsavedCyclePoints[i][j]);
        }
    }
}

void PointCloudRenderer::renderPickedCycle()
{
    if (pickedCycle == -1) return;
    
    glColor3f(1.f , 1.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < unsavedCyclePoints[pickedCycle].size(); i++)
    {
        drawLines(unsavedCyclePoints[pickedCycle][i]);
    }
}

void PointCloudRenderer::renderSavedCycles()
{
    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < dispCurveNet->cyclePoints.size(); i++)
    {
        for (int j = 0; j < dispCurveNet->cyclePoints[i].size(); j++)
        {
            drawLines(dispCurveNet->cyclePoints[i][j]);
        }
    }
}

void PointCloudRenderer::renderUnsavedMeshes()
{
	for (int patchID = 0; patchID < unsavedMeshes.size(); patchID++)
	{
		const cycle::TriangleCycle &triangleCycle = unsavedMeshes[patchID];
		const cycle::TriangleCycle &triangleCycleNormal = unsavedNormals[patchID];
		if(true)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glBegin(GL_TRIANGLES);
		for(int i=0;i<triangleCycle.size();i++){
			const std::vector<vec3d> &triangle = triangleCycle[i];
			const std::vector<vec3d> &verticesNormal = triangleCycleNormal[i];

			for(int j=0;j<triangle.size();j++){
				const vec3d &position = triangle[j];
				const vec3d &norm = verticesNormal[j];
				glNormal3f(norm.x,norm.y,norm.z);
				glVertex3f(position.x,position.y,position.z);
			}
		}
		glEnd();
	}
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
    renderCtrlNodes();
    //renderPathForComp();
    renderCollinearLines();
    renderParallelLines();
    renderCoplanarLines();
    renderOrthogonalLines();
    renderTangentLines();
    renderUnsavedCycles();
    renderPickedCycle();
    renderSavedCycles();
	renderUnsavedMeshes();
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
    int ni = -1;
    int selectedCurve = curveSelectionByRay(mouseX , mouseY , ni);
    bool isSnap = false;
    if (selectedCurve != -1 && ni != -1) isSnap = true;
    
	int selectedObj = -1;
    if (!isSnap) selectedObj = selectionByColorMap(mouseX , mouseY);
    /*
	printf("%d, (%.6f,%.6f,%.6f)\n" , selectedObj ,
        pcUtils->pcData[selectedObj].pos.x ,
        pcUtils->pcData[selectedObj].pos.y ,
        pcUtils->pcData[selectedObj].pos.z);
    */
	if (isSnap || selectedObj != -1)
	{
        vec3d pos , dispPos;
        int edi , breakLine = -1 , breakPoint = -1;
		if (isSnap)
        {
            breakLine = selectedCurve;
            breakPoint = ni;
            dispPos = dispCurveNet->polyLines[breakLine][breakPoint];
            if (pcUtils->point2Index.find(point2double(dispPos)) !=
                pcUtils->point2Index.end())
            {
                pos = dispPos;
            }
            else
            {
                pos = dispCurveNet->originPolyLines[breakLine][breakPoint];
            }
            pickedDispPoint = dispCurveNet->polyLines[breakLine][breakPoint];
            
            if (breakPoint == 0 || breakPoint == dispCurveNet->polyLines[breakLine].size() - 1)
            {
                snapToNode = true;
                snapToCurve = false;
            }
            else
            {
                snapToNode = false;
                snapToCurve = true;
            }
        }
        else
        {
            pos = dispPos = pickedDispPoint = pcUtils->pcData[selectedObj].pos;
            snapToNode = snapToCurve = false;
        }
        /*
        pickedDispPoint = pcUtils->pcData[selectedObj].pos;
		vec3d pos = pcUtils->pcData[selectedObj].pos;
        vec3d dispPos;
		int edi;
        
        // pos = *pickedPoint;
        dispPos = pickedDispPoint;
		
        // snapping when it is near curve
        bool isSnap = false;
        int breakLine = -1 , breakPoint = -1;
        double min_dist = 1e20;
        
        for (int i = 0; i < dispCurveNet->numPolyLines; i++)
        {
            for (int j = 0; j < dispCurveNet->polyLines[i].size(); j++)
            {
                double tmp = (dispPos - dispCurveNet->polyLines[i][j]).length();
                if (tmp < min_dist)
                {
                    min_dist = tmp;
                    breakLine = i;
                    breakPoint = j;
                }
            }
        }
        double snapOffset = selectionOffset * 1.5;
        if (breakLine != -1 && (breakPoint == 0 || breakPoint == (int)dispCurveNet->polyLines[breakLine].size() - 1))
        {
            snapOffset *= 3.0;
        }
        
        if (min_dist < snapOffset)
        {
            // pos = curveNet->polyLines[breakLine][breakPoint];
            dispPos = dispCurveNet->polyLines[breakLine][breakPoint];
            if (pcUtils->point2Index.find(point2double(dispPos)) !=
                pcUtils->point2Index.end())
            {
                pos = dispPos;
            }
            // pickedPoint = &curveNet->polyLines[breakLine][breakPoint];
            pickedDispPoint = dispCurveNet->polyLines[breakLine][breakPoint];
            isSnap = true;
            // printf("snap!\n");
        }
        */
		edi = pcUtils->point2Index[point2double(pos)];
        if (!(edi >= 0 && edi < pcUtils->nodes))
        {
            printf("ed index error\n");
        }

        // path smoothing
		if (isValid(lastDispPoint))
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

            pathVertex[0] = dispPos;

            if (useBSpline)
            {
                if (!dispCurveNet->collinearTest(pathVertex , bsp))
                {
                    convert2Spline(pathVertex , bsp);
                }
            }
		}
        /*
        printf("pickedPos = (%.6f,%.6f,%.6f)\n" , pickedDispPoint.x ,
                pickedDispPoint.y , pickedDispPoint.z);
        */
        if (isStore)
		{
			int sti;
            bool newNode = true;
            
            if (isSnap && breakLine != -1 && (breakPoint == 0 || breakPoint == (int)dispCurveNet->polyLines[breakLine].size() - 1))
            {
                newNode = false;
            }

            if (isSnap && newNode)
            {
                // curveNet->breakPath(breakLine , breakPoint);
                dispCurveNet->breakPath(breakLine , breakPoint);
                newNode = false;
            }
            
            if (!isValid(lastDispPoint))
            {
                // curveNet->startPath(pos);
                if (!isSnap) dispCurveNet->startPath(dispPos);
            }
            else
            {
                // vec3d lastp(lastPoint->x , lastPoint->y , lastPoint->z);
                // vec3d lastDisp(lastDispPoint->x , lastDispPoint->y , lastDispPoint->z);

                // curveNet->extendPath(lastp , pos , pathForComp[0] , newNode);
				/*
				printf("========== origin path ==========\n");
				for (int i = 0; i < pathForComp[0].size(); i++)
				{
					printf("(%.6f,%.6f,%.6f)\n" , pathForComp[0][i].x , pathForComp[0][i].y , pathForComp[0][i].z);
				}
				*/
                dispCurveNet->extendPath(lastDispPoint , dispPos , pathVertex ,
                    newNode , bsp , pathForComp[0]);

                if (isAutoOpt)
                {
                    pcUtils->opt.init(dispCurveNet);
                    pcUtils->opt.run(dispCurveNet);
                    dispPos = dispCurveNet->polyLines[dispCurveNet->numPolyLines - 1][0];
                }
                // dispCurveNet->orthoSet.printLog();
                // dispCurveNet->collinearSet.printLog();
                // dispCurveNet->collinearSet.test();
                // pcUtils->optimizeJunction(dispCurveNet , lastDisp);
            }

            // dispCurveNet->debugLog();

            // printf("dispPos = (%.6f,%.6f,%.6f)\n" , dispPos.x , dispPos.y , dispPos.z);
            
            if (pcUtils->addPointToGraph(dispPos))
            {
                sti = pcUtils->point2Index[point2double(dispPos)];
            }
            else
            {
                sti = edi;
            }

            if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
				pcUtils->dijkstra(pcUtils->pointGraph , sti , pcUtils->pointGraphInfo);

            // lastPoint = &curveNet->nodes[curveNet->getNodeIndex(pos)];
            lastDispPoint = dispPos;

            // find cycle
            if (dispCurveNet->numPolyLines > 0)
            {
                unsavedCycles.clear();
                cycle::cycleDiscovery(dispCurveNet->polyLines , dispCurveNet->cycles ,
                    unsavedCycles , unsavedMeshes , unsavedNormals);

                unsavedCyclePoints.clear();
                unsavedCycleCenters.clear();
                for (int i = 0; i < unsavedCycles.size(); i++)
                {
                    std::vector<Path> cyclePts;
                    vec3d center;
                    dispCurveNet->calcDispCyclePoints(unsavedCycles[i] ,
                        cyclePts , center);
                    unsavedCyclePoints.push_back(cyclePts);
                    unsavedCycleCenters.push_back(center);
                }
                /*
                writeLog("*********** Input ***********\n");
                writeLog("%lu\n\n" , dispCurveNet->numPolyLines);
                for (int i = 0; i < dispCurveNet->numPolyLines; i++)
                {
                    writeLog("%lu\n" , dispCurveNet->polyLines[i].size());
                    for (int j = 0; j < dispCurveNet->polyLines[i].size(); j++)
                    {
                        writeLog("%.6f %.6f %.6f\n" , dispCurveNet->polyLines[i][j].x ,
                            dispCurveNet->polyLines[i][j].y , dispCurveNet->polyLines[i][j].z);
                    }
                    writeLog("\n");
                }
                */
                writeLog("!!!!!!!!!!! Output cycles !!!!!!!!!!\n");
                writeLog("%lu\n\n" , unsavedCycles.size());
                for (int i = 0; i < unsavedCycles.size(); i++)
                {
                    writeLog("%lu\n" , unsavedCycles[i].size());
                    for (int j = 0; j < unsavedCycles[i].size(); j++)
                    {
                        writeLog("%lu " , unsavedCycles[i][j]);
                    }
                    writeLog("\n");
                }
            }
		}
	}
	else
	{
		// pickedPoint = NULL;
        setNull(pickedDispPoint);
	}
}

void PointCloudRenderer::optUpdate()
{
    pcUtils->opt.init(dispCurveNet);
    pcUtils->opt.run(dispCurveNet);
    vec3d stPos = dispCurveNet->polyLines[dispCurveNet->numPolyLines - 1][0];
    pcUtils->addPointToGraph(stPos);
    int sti = pcUtils->point2Index[point2double(stPos)];
    if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
    {
        pcUtils->dijkstra(pcUtils->pointGraph , sti , pcUtils->pointGraphInfo);
    }
    lastDispPoint = stPos;
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

int PointCloudRenderer::curveSelectionByRay(int mouseX , int mouseY , int& nodeIndex)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;
    nodeIndex = -1;

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
				res = i;
                nodeIndex = j;
			}
		}
	}
    double snapOffset = selectionOffset;
    // printf("%d %d , %.6f > %.6f\n" , res , nodeIndex , minDistance , snapOffset);
    if (res == -1) return res;
    if (nodeIndex == 0 || nodeIndex == dispCurveNet->polyLines[res].size() - 1)
        snapOffset *= 2;
    if (minDistance > snapOffset) res = -1;
    return res;
}

void PointCloudRenderer::pickCurve(int mouseX , int mouseY , bool isDelete)
{
    int ni;
	pickedCurve = curveSelectionByRay(mouseX , mouseY , ni);
	if (pickedCurve != -1 && isDelete)
    {
        // curveNet->deletePath(pickedCurve);
        dispCurveNet->deletePath(pickedCurve);
        
        //dispCurveNet->debugLog();
    }
}

int PointCloudRenderer::cycleSelectionByRay(int mouseX , int mouseY)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;

    double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < unsavedCycleCenters.size(); i++)
    {
		vec3d p = unsavedCycleCenters[i];
        double paramU = (
            ((p[0]-rayStr[0])*(rayEnd[0]-rayStr[0])) +
            ((p[1]-rayStr[1])*(rayEnd[1]-rayStr[1])) +
            ((p[2]-rayStr[2])*(rayEnd[2]-rayStr[2])))/p0p1LenSquared;

        vec3d lineP = rayStr + (rayEnd - rayStr) * paramU;
        double distance = (lineP - p).length();
        if (minDistance > distance)
        {
            minDistance = distance;
            res = i;
        }
	}
    
    // printf("%d %d , %.6f > %.6f\n" , res , nodeIndex , minDistance , snapOffset);
    return res;
}

void PointCloudRenderer::pickCycle(int mouseX , int mouseY , bool isStore)
{
	pickedCycle = cycleSelectionByRay(mouseX , mouseY);
	if (pickedCycle != -1 && isStore)
    {
        // curveNet->deletePath(pickedCurve);
        dispCurveNet->addCycle(unsavedCycles[pickedCycle]);
        /*
        printf("===== cycle size = %lu =====\n" , dispCurveNet->cycles.size());
        for (int i = 0; i < dispCurveNet->cycles.size(); i++)
        {
            printf("cycle %d: " , i);
            for (int j = 0; j < dispCurveNet->cycles[i].size(); j++)
            {
                printf("%d " , dispCurveNet->cycles[i][j]);
            }
            printf("\n");
        }
        */
        //dispCurveNet->debugLog();
    }
}

void PointCloudRenderer::clearPaths()
{
	pathVertex.clear();
    for (int i = 0; i < 3; i++)
        pathForComp[i].clear();
	// pickedPoint = NULL;
    setNull(pickedDispPoint);
	// lastPoint = NULL;
    setNull(lastDispPoint);
    pickedCurve = -1;
    pickedCycle = -1;
    // curveNet->clear();
    dispCurveNet->clear();
}

void PointCloudRenderer::incBspCurveIndex()
{
    if (dispCurveNet->numPolyLines == 0) return;
    curveIndex++;
    while (curveIndex >= (int)dispCurveNet->bsplines[bspIndex].ctrlNodes.size() - 1)
    {
        curveIndex = 0;
        bspIndex = (bspIndex + 1) % dispCurveNet->numPolyLines;
    }
}

void PointCloudRenderer::decBspCurveIndex()
{
    if (dispCurveNet->numPolyLines == 0) return;
    curveIndex--;
    while (curveIndex < 0)
    {
        bspIndex = (bspIndex - 1 + dispCurveNet->numPolyLines) % dispCurveNet->numPolyLines;
        curveIndex = (int)dispCurveNet->bsplines[bspIndex].ctrlNodes.size() - 2;
    }
}