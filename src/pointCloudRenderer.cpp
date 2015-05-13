#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include <GL/glut.h>
#include <algorithm>
#include <cassert>
#include "curveNet.h"
#include "pointCloudUtils.h"
#include "pointCloudRenderer.h"
#include "nvVector.h"
#include "colormap.h"
#include "CoarseSuf.h"
#include "SufTune.h"
#include "pclUtils.h"
#include "macros.h"

#ifdef _WIN32
	#include "SmoothPatch.h"
#endif

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
	isHideDrawnPoints = true;
	isShowPoints = false;
	isShowUniformGrid = false;
	isShowAdaptiveGrid = false;
	isShowHessian = false;
	isShowMetric = false;
	isShowPointCloud = true;
    isShowCtrlNodes = false;
	isShowCoplanes = false;
    isShowFeaturePoints = false;
    isShowDebugPoints = false;

    isShowAxisWidget = false;
    chosenAxis = -1;
    axisWidget.init();

    constraintsVisual = 0;
    patchesVisual = 0;
    bspIndex = 0; curveIndex = 0;
    dragPlane.setNull();
    dragPlaneNormalIndex = 0;
    crossPlane.setNull();
    drawMode = 0;
    copyMode = 0;
    lt_x = lt_y = -1;

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
    pickedSavedCycle = -1;
    pickedBsp = pickedCtrlNode = -1;
    toExpandCurve = pickedAutoCurve = -1;

    isCurvesChosen.clear();
    
	isCtrlPress = false;
    isAltPress = false;
	isShiftPress = false;

	isAutoOpt = true;
    
	if (pcUtils == NULL)
		return;

    initFreeSketchMode();

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

void PointCloudRenderer::drawPointCloud()
{
    for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		if (pcUtils->pcColor[i] >= 0 && pcUtils->pcColor[i] < dispCurveNet->meshes.size())
		{
			if (isHideDrawnPoints) continue;
			int c = pcUtils->pcColor[i];
			glColor3f(pcUtils->colors[c].r , pcUtils->colors[c].g , pcUtils->colors[c].b);
		}
		else
		{
			glColor3f(0.f , 0.f , 0.f);
		}

        //if ((i < pcUtils->partSym.isSampled.size() &&
        //pcUtils->partSym.isSampled[i]))
        if (i < isChosen.size() && isChosen[i])
        {
            glColor3f(1.f , 0.f , 0.f);
            glPointSize(8.f);
        }
        else
        {
            glPointSize(1.f);
        }

        glBegin(GL_POINTS);
        glNormal3f(pcUtils->pcData[i].n.x , pcUtils->pcData[i].n.y , pcUtils->pcData[i].n.z);
		glVertex3f(pcUtils->pcData[i].pos.x , pcUtils->pcData[i].pos.y , pcUtils->pcData[i].pos.z);
        glEnd();
	}
}

void PointCloudRenderer::drawPoint(const vec3d& pos)
{
	glBegin(GL_POINTS);
	glVertex3f(pos.x , pos.y , pos.z);
	glEnd();
}

void PointCloudRenderer::drawPoints(const std::vector<vec3d>& points)
{
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); i++)
    {
        glVertex3f(points[i].x , points[i].y , points[i].z);
    }
	glEnd();
}

void PointCloudRenderer::drawPoints(const std::vector<bool>& mask)
{
    glBegin(GL_POINTS);
    for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
        if (!(i < mask.size() && mask[i])) continue;
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

void PointCloudRenderer::drawPlane(const Plane& plane , const double& r)
{
    vec3d a(1 , 0 , 0), b;
    if (std::abs(plane.n.dot(a)) > 0.99)
    {
        a = vec3d(0 , 1 , 0);
    }
    b = plane.n.cross(a);
    b.normalize();
    a = b.cross(plane.n);
    a.normalize();
    drawCircle(plane.p , b , a , r);
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

void PointCloudRenderer::drawPatch(const cycle::TriangleCycle& triangleCycle ,
    const cycle::TriangleCycle& triangleCycleNormal)
{
    glBegin(GL_TRIANGLES);
    for(int i=0;i<triangleCycle.size();i++){
        const std::vector<vec3d> &triangle = triangleCycle[i];
        const std::vector<vec3d> &verticesNormal = triangleCycleNormal[i];

        for(int j=0;j<triangle.size();j++){
            const vec3d &position = triangle[j];
            const vec3d &norm = verticesNormal[j];
            // glNormal3f(norm.x,norm.y,norm.z);
            glVertex3f(position.x,position.y,position.z);
        }
    }
    glEnd();
}

void PointCloudRenderer::drawString(const std::string& str)
{
	/*
	for (int i = 0; i < str.length(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24 , str[i]);
	}
	*/
}

void PointCloudRenderer::callListPoints()
{
	glNewList(LIST_POINTS , GL_COMPILE);

    drawPointCloud();

	glEndList();
}

void PointCloudRenderer::callListSurfelDisc()
{
	glNewList(LIST_SURFEL_DISC , GL_COMPILE);

	for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		if (isHideDrawnPoints && pcUtils->pcColor[i] >= 0 && pcUtils->pcColor[i] < dispCurveNet->meshes.size())
			continue;
		drawCircle(pcUtils->pcData[i].pos , axisU[i] , axisV[i] , discRenderRadiusScale , pcUtils->pcData[i].n);
	}

	glEndList();
}

void PointCloudRenderer::renderString()
{
	//drawString("cycle score = xxx\n");
}

void PointCloudRenderer::renderPoints()
{
	if (pcUtils == NULL)
		return;

    drawPointCloud();
	// glCallList(LIST_POINTS);
    
	//callListPoints();
}

void PointCloudRenderer::renderSurfelDisc()
{
	if (pcUtils == NULL)
		return;
	glColor3f(0.5f , 0.539f , 0.527f);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	/*
	for (int i = 0; i < pcUtils->pcData.size(); i++)
	{
		if (pcUtils->pcColor[i] >= 0 && pcUtils->pcColor[i] < dispCurveNet->meshes.size())
			continue;

		drawCircle(pcUtils->pcData[i].pos , axisU[i] , axisV[i] , discRenderRadiusScale , pcUtils->pcData[i].n);
	}
	*/
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

	glPointSize(12.f);
	for (int i = 0; i < (int)dispCurveNet->nodes.size(); i++)
	{
        if (!dispCurveNet->nodesStat[i]) continue;
        if (pickedBsp != -1 && pickedCtrlNode != -1 &&
            isEqual(dispCurveNet->nodes[i] , dispCurveNet->bsplines[bspIndex].ctrlNodes[pickedCtrlNode])) continue;
        if (dispCurveNet->isNodesFixed[i]) glColor3f(0.f , 0.f , 0.f);
        else glColor3f(1.f , 0.f , 0.f);

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
    if (constraintsVisual != 0) return;

    glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
	for (int i = 0; i < dispCurveNet->polyLines.size(); i++)
	{
        if (dispCurveNet->curveType[i] == -1) continue;

        if (isCurvesChosen[i]) glColor3f(1.f , 0.f , 1.f);
        else if (dispCurveNet->isCurvesFixed[i]) glColor3f(0.f , 0.f , 0.f);
        else glColor3f(0.f , 0.f , 1.f);
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
		if (pickedBsp != -1 && pickedBsp != i) continue;
        drawLines(dispCurveNet->bsplines[i].ctrlNodes);
    }
    drawLines(bsp.ctrlNodes);

    glPointSize(12.f);
    for (int i = 0; i < dispCurveNet->bsplines.size(); i++)
    {
		if (pickedBsp != -1 && pickedBsp != i) continue;
        for (int j = 0; j < dispCurveNet->bsplines[i].ctrlNodes.size(); j++)
        {
            if (pickedBsp != -1 && pickedCtrlNode != -1 &&
                isEqual(dispCurveNet->bsplines[i].ctrlNodes[j] , dispCurveNet->bsplines[pickedBsp].ctrlNodes[pickedCtrlNode]))
            {
                glColor3f(1.f , 1.f , 0.f);
            }
            else if (snapToNode && isEqual(dispCurveNet->bsplines[i].ctrlNodes[j] , pickedDispPoint))
            {
                glColor3f(1.f , 1.f , 0.f);
            }
            else
            {
                glColor3f(1.f , 0.f , 0.f);
            }
            drawPoint(dispCurveNet->bsplines[i].ctrlNodes[j]);
        }
    }
    glColor3f(1.f , 0.f , 0.f);
    for (int j = 1; j < (int)bsp.ctrlNodes.size() - 1; j++)
    {
        drawPoint(bsp.ctrlNodes[j]);
    }
    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderDragPlane()
{
    if (!isShowCtrlNodes) return;
    if (!dragPlane.isValid()) return;
    
    glPointSize(12.f);
    glColor4f(0.f , 0.f , 0.f , 0.7f);
    drawPoint(dragStartPoint);

    glColor4f(0.f , 0.f , 0.f , 0.7f);
	glLineWidth(2.5f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    drawLine(dragStartPoint , dragCurPoint);
    // double ratio = 0.1;
    // drawLine(dragStartPoint - dragPlane.n * ratio , dragStartPoint + dragPlane.n * ratio);
    glDisable(GL_LINE_STIPPLE);
    
    // glColor4f(1.f , 99.f / 255.f , 71.f / 255.f , 0.7f);
    glColor4f(176.f / 255.f , 226.f / 255.f , 1.f , 0.7f);
    drawPlane(dragPlane , 1);
    /*
    Plane np = dragPlane;
    if (dragPlaneNormalIndex == 0)
    {
        np.n = vec3d(0 , 0 , 1);
    }
    else
    {
        np.n = vec3d(0 , 1 , 0);
    }
    drawPlane(np , 0.3);
    */
}

void PointCloudRenderer::renderCoplanes()
{
	if (!isShowCoplanes) return;
	glColor4f(1.f , 99.f / 255.f , 71.f / 255.f , 0.7f);
	for (int i = 0; i < dispCurveNet->coplanes.size(); i++)
	{
		drawPlane(dispCurveNet->coplanes[i] , 1);
	}
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

	if (dispCurveNet->curveType[bspIndex] != 1) return;

    glColor3f(0.f , 0.f , 1.f);
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
		if (i == bspIndex) continue;
		if (dispCurveNet->curveType[i] != 1) continue;
        if (!dispCurveNet->conSet->collinearSet.sameRoot(bspIndex , 0 , i , 0)) continue;
        drawLine(dispCurveNet->bsplines[i].ctrlNodes[0] ,
            dispCurveNet->bsplines[i].ctrlNodes[1]);
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
        if (i != bspIndex && !dispCurveNet->conSet->parallelSet.sameRoot(bspIndex , 0 , i , 0)) continue;
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
        if (dispCurveNet->conSet->coplanarSet.getMark(bspIndex , 0 , i , 0) != 1) continue;
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
            if (dispCurveNet->conSet->orthoSet.getMark(bspIndex , curveIndex , i , j) != 1) continue;
            // printf("ortho: (%d , %d) <==> (%d , %d), %d\n" , bspIndex , curveIndex , i , j ,
                // dispCurveNet->conSet->orthoSet.getMark(bspIndex , curveIndex , i , j));
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
            if (dispCurveNet->conSet->orthoSet.getMark(bspIndex , curveIndex , i , j) != 2) continue;
            // printf("(%d , %d) <==> (%d , %d), %d\n" , bspIndex , curveIndex , i , j ,
                // dispCurveNet->orthoSet.getMark(bspIndex , curveIndex , i , j));
            drawLine(dispCurveNet->bsplines[i].ctrlNodes[j] ,
                dispCurveNet->bsplines[i].ctrlNodes[j + 1]);
        }
    }
}

void PointCloudRenderer::renderUnsavedCycles()
{
    if (patchesVisual != 0 && patchesVisual != 2) return;
    if (!isShiftPress) return;
    glColor3f(1.f , 0.f , 1.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < unsavedCyclePoints.size(); i++)
    {
        if (i == pickedCycle) continue;
		if (!unsavedStatus[i]) continue;

		if (!inGroup[i])
		{
			glColor3f(1.f , 0.f , 1.f);
		}
		else
		{
			glColor3f(1.f , 0.5f , 0.f);
		}

        for (int j = 0; j < unsavedCyclePoints[i].size(); j++)
        {
            drawLines(unsavedCyclePoints[i][j]);
        }
    }
}

void PointCloudRenderer::renderPickedCycle()
{
    if (patchesVisual != 0 && patchesVisual != 2) return;
    if (pickedCycle == -1) return;
    if (!isCtrlPress) return;
    glColor3f(0.f , 1.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < unsavedCyclePoints[pickedCycle].size(); i++)
    {
        drawLines(unsavedCyclePoints[pickedCycle][i]);
    }
}

void PointCloudRenderer::renderPickedSavedCycle()
{
    if (patchesVisual != 0 && patchesVisual != 1) return;
    if (pickedSavedCycle == -1) return;
    
    glColor3f(0.f , 1.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < dispCurveNet->cyclePoints[pickedSavedCycle].size(); i++)
    {
        for (int j = 0; j < dispCurveNet->cyclePoints[pickedSavedCycle][i].size(); j++)
        {
            drawLines(dispCurveNet->cyclePoints[pickedSavedCycle][i][j]);
        }
    }
}

void PointCloudRenderer::renderSavedCycles()
{
    if (patchesVisual != 0 && patchesVisual != 1) return;
    glColor3f(1.f , 0.f , 0.f);
    glLineWidth(3.f);
    glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    for (int i = 0; i < dispCurveNet->cyclePoints.size(); i++)
    {
        if (i == pickedSavedCycle) continue;
        for (int j = 0; j < dispCurveNet->cyclePoints[i].size(); j++)
        {
            for (int k = 0; k < dispCurveNet->cyclePoints[i][j].size(); k++)
            {
                drawLines(dispCurveNet->cyclePoints[i][j][k]);
            }
        }
    }
}

void PointCloudRenderer::renderUnsavedMeshes()
{
    if (patchesVisual != 0 && patchesVisual != 2) return;
	for (int patchID = 0; patchID < unsavedMeshes.size(); patchID++)
	{
		if (patchID == pickedCycle)
			glColor3f(0.f , 1.f , 0.f);
		else
			glColor3f(0.f , 0.f , 1.f);
		const cycle::TriangleCycle &triangleCycle = unsavedMeshes[patchID];
		const cycle::TriangleCycle &triangleCycleNormal = unsavedNormals[patchID];
		if (patchesDraw == 1)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else if (patchesDraw == 0)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		drawPatch(triangleCycle , triangleCycleNormal);
	}
}

void PointCloudRenderer::renderPickedMesh()
{
    if (patchesVisual != 0 && patchesVisual != 2) return;
    if (pickedCycle == -1) return;
    if (!toBeSurfacing[pickedCycle] || !unsavedStatus[pickedCycle]) return;
	if (isCtrlPress) return;
	int patchID = pickedCycle;
    glColor3f(0.f , 1.f , 0.f);
    const cycle::TriangleCycle &triangleCycle = unsavedMeshes[patchID];
    const cycle::TriangleCycle &triangleCycleNormal = unsavedNormals[patchID];
    if (patchesDraw == 1)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else if (patchesDraw == 0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    drawPatch(triangleCycle , triangleCycleNormal);
}

void PointCloudRenderer::renderPickedSavedMesh()
{
    if (patchesVisual != 0 && patchesVisual != 1) return;
    if (pickedSavedCycle == -1) return;
    int patchID = pickedSavedCycle;
    glColor3f(0.f , 1.f , 0.f);
    const cycle::TriangleCycle &triangleCycle = dispCurveNet->meshes[patchID];
    const cycle::TriangleCycle &triangleCycleNormal = dispCurveNet->meshNormals[patchID];
    if (patchesDraw == 1)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else if (patchesDraw == 0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    drawPatch(triangleCycle , triangleCycleNormal);
}

void PointCloudRenderer::renderSavedMeshes()
{
    if (patchesVisual != 0 && patchesVisual != 1) return;
    for (int patchID = 0; patchID < dispCurveNet->meshes.size(); patchID++)
	{
        if (patchID == pickedSavedCycle)
			glColor3f(0.f , 1.f , 0.f);
		else
			glColor3f(1.f , 0.f , 0.f);

        const cycle::TriangleCycle &triangleCycle = dispCurveNet->meshes[patchID];
		const cycle::TriangleCycle &triangleCycleNormal = dispCurveNet->meshNormals[patchID];
		if (patchesDraw == 1)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else if (patchesDraw == 0)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		drawPatch(triangleCycle , triangleCycleNormal);
	}

    /*
	for (int i = 0; i < dispCurveNet->cycleGroups.size(); i++)
	{
		for (int j = 0; j < dispCurveNet->cycleGroups[i].size(); j++)
		{
			if (dispCurveNet->cycleGroups[i][j] == pickedSavedCycle)
				glColor3f(0.f , 1.f , 0.f);
			else
				glColor3f(1.f , 0.f , 0.f);
			int cycleId = dispCurveNet->cycleGroups[i][j];
			for (int k = 0; k < dispCurveNet->cyclePoints[cycleId].size(); k++)
			{
				drawLines(dispCurveNet->cyclePoints[cycleId][k]);
			}
		}
	}
    */
}

void PointCloudRenderer::renderAutoGenPaths()
{
    if (dispCurveNet == NULL) return;
    
	glLineWidth(3.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    
	for (int i = 0; i < autoGenPaths.size(); i++)
	{
        if (pickedAutoCurve != i)
        	glColor3f(1.f , 0.5f , 36.f / 255.f);
        else
            glColor3f(1.f , 20.f / 255.f , 147.f / 255.f);

		drawLines(autoGenPaths[i]);
	}
    
    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderFreeSketches()
{
    glLineWidth(2.f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);

    glColor3f(1.f , 0.f , 0.f);
    drawLines(sketchLine);
    for (int i = 0; i < freeSketchLines.size(); i++)
    {
        drawLines(freeSketchLines[i]);
    }

    glDisable(GL_LINE_STIPPLE);
}

void PointCloudRenderer::renderFeaturePoints()
{
    if (!isShowFeaturePoints) return;
    glPointSize(8.f);
    glColor3f(0.f , 0.f , 1.f);
    drawPoints(pcUtils->isFeaturePoint);
}

void PointCloudRenderer::renderDebug()
{
    if (!isShowDebugPoints) return;
    glPointSize(4.f);
    glColor3f(1.f , 0.f , 1.f);
    drawPoints(debugPoints);
    /*
    std::vector<vec3d> rays = getRay(pcUtils->openGLView->getWidth() / 2 ,
        pcUtils->openGLView->getHeight() / 2);
    std::vector<vec3d> rays_d = getRay(pcUtils->openGLView->getWidth() / 2 ,
        pcUtils->openGLView->getHeight() / 2 - 20);
    vec3d dir = rays.back() - rays.front();
    dir.normalize();
    vec3d up = rays_d.front() - rays.front();
    up.normalize();
    LocalFrame frame = pcUtils->openGLView->cameraFrame;
    frame.n = dir;
    frame.t = up;
    frame.s = dir.cross(up);
    frame.s.normalize();
    
    vec3d origin(0.0);
    glColor3f(1.f , 0.f , 0.f);
    drawLine(origin , frame.n);
    glColor3f(0.f , 1.f , 0.f);
    drawLine(origin , frame.s);
    glColor3f(0.f , 0.f , 1.f);
    drawLine(origin , frame.t);
    */
}

void PointCloudRenderer::renderCrossPlane()
{
    if (!crossPlane.isValid()) return;
    glPointSize(12.f);
    glColor4f(0.f , 0.f , 0.f , 0.7f);
    drawPoint(crossPlane.p);

    glColor4f(0.f , 0.f , 0.f , 0.7f);
	glLineWidth(2.5f);
	glEnable(GL_LINE_STIPPLE);
	glLineStipple(2, 0xffff);
    double ratio = 0.3;
    drawLine(crossPlane.p - crossPlane.n * ratio , crossPlane.p + crossPlane.n * ratio);
    glDisable(GL_LINE_STIPPLE);

    glColor4f(176.f / 255.f , 226.f / 255.f , 1.f , 0.7f);
    drawPlane(crossPlane , 1.2);

    glPointSize(3.f);
    glColor4f(1.f , 0.f , 0.f , 0.7f);
    drawPoints(crossPoints3d);
}

void PointCloudRenderer::renderAxisWidget()
{
    if (!isShowAxisWidget) return;
    glLineWidth(2.5f);

    if (chosenAxis != -1)
    {
        // glColor4f(1.f , 0.f , 0.f , 1.f);
        glColor4f(axisWidget.axesColors[chosenAxis].x , axisWidget.axesColors[chosenAxis].y ,
                    axisWidget.axesColors[chosenAxis].z , 0.7f);
        drawLine(axisWidget.origin - axisWidget.axes[chosenAxis] * axisWidget.length * 2 ,
            axisWidget.origin + axisWidget.axes[chosenAxis] * axisWidget.length * 2);
        if (copyMode == 3)
        {
            for (int i = 0; i < axisWidget.axes.size(); i++)
            {
                if (i == chosenAxis)
                {
                    glPointSize(4.f);
                    // glColor4f(1.f , 0.f , 0.f , 0.7f);
                }
                else
                {
                    glPointSize(1.f);
                    // glColor4f(0.f , 0.f , 0.f , 0.7f);
                }
                glColor4f(axisWidget.axesColors[i].x , axisWidget.axesColors[i].y ,
                    axisWidget.axesColors[i].z , 0.7f);
                drawPoints(axisWidget.globePoints[i]);
            }
        }
    }
    else
    {
        if (copyMode != 3)
        {
            for (int i = 0; i < axisWidget.axes.size(); i++)
            {
                // glColor4f(0.f , 0.f , 0.f , 0.7f);
                glColor4f(axisWidget.axesColors[i].x , axisWidget.axesColors[i].y ,
                    axisWidget.axesColors[i].z , 0.7f);
                drawLine(axisWidget.origin - axisWidget.axes[i] * axisWidget.length ,
                    axisWidget.origin + axisWidget.axes[i] * axisWidget.length);
            }
        }
        else if (copyMode == 3)
        {
            glPointSize(1.f);
            for (int i = 0; i < axisWidget.axes.size(); i++)
            {
                // glColor4f(0.f , 0.f , 0.f , 0.7f);
                glColor4f(axisWidget.axesColors[i].x , axisWidget.axesColors[i].y ,
                    axisWidget.axesColors[i].z , 0.7f);
                drawPoints(axisWidget.globePoints[i]);
            }
        }
    }

    glPointSize(8.f);
    glColor4f(0.f , 0.f , 0.f , 0.7f);
    drawPoint(axisWidget.origin);
}

void PointCloudRenderer::renderChoosingRectangle()
{
    if (lt_x == -1) return;
    vec3d a , b , c , d;
    a = mouseLocationTo3D(std::min(lt_x , rb_x) , std::min(lt_y , rb_y));
    b = mouseLocationTo3D(std::min(lt_x , rb_x) , std::max(lt_y , rb_y));
    c = mouseLocationTo3D(std::max(lt_x , rb_x) , std::max(lt_y , rb_y));
    d = mouseLocationTo3D(std::max(lt_x , rb_x) , std::min(lt_y , rb_y));
    glLineWidth(2.5f);
    glColor4f(0.f , 0.7f , 0.f , 0.7f);
    drawLine(a , b);
    drawLine(b , c);
    drawLine(c , d);
    drawLine(d , a);
}

void PointCloudRenderer::render()
{
    renderFreeSketches();
    renderPointCloud();
	renderHessian();
	renderMetric();
	renderSelectedPoints();
	renderCurrentPath();
	renderStoredPaths();
    renderPickedCurve();
    renderCtrlNodes();
    renderDragPlane();
	renderCoplanes();
    //renderPathForComp();
    renderCollinearLines();
    renderParallelLines();
    renderCoplanarLines();
    renderOrthogonalLines();
    renderTangentLines();
    renderAutoGenPaths();
    renderCrossPlane();
// #ifdef _WIN32
    renderUnsavedCycles();
    renderPickedMesh();
	renderPickedCycle();
    renderPickedSavedMesh();
    renderSavedMeshes();
    renderAxisWidget();
    renderChoosingRectangle();
// #else
    // renderUnsavedCycles();
    // renderPickedCycle();
    // renderPickedSavedCycle();
    // renderSavedCycles();
// #endif
    renderFeaturePoints();
    renderDebug();
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
		if (isHideDrawnPoints && pcUtils->pcColor[i] >= 0 && pcUtils->pcColor[i] < dispCurveNet->meshes.size())
			continue;

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
		/*
		for (int i = 0; i < axisU.size(); i++)
		{
			if (pcUtils->pcColor[i] >= 0 && pcUtils->pcColor[i] < dispCurveNet->meshes.size())
				continue;

			vec3uc& col = glObjColors[i];
			glColor3ub(col.x , col.y , col.z);
			drawCircle(pcUtils->pcData[i].pos , axisU[i] , axisV[i] , selectionOffset);
		}
		*/
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

void PointCloudRenderer::afterCurveNetUpdate()
{
    // pcUtils->optimizeJunction(dispCurveNet , lastDispPoint);

    printf("start optimization\n");
    if (isAutoOpt)
    {
        printf("opt init\n");
        pcUtils->opt.init(dispCurveNet , pcUtils);
        printf("opt run\n");
        pcUtils->opt.run(dispCurveNet);
        printf("after opt\n");
    }
    // dispCurveNet->orthoSet.printLog();
    // dispCurveNet->collinearSet.printLog();
    // dispCurveNet->collinearSet.test();

    // find cycle
    printf("start cycle discovery\n");
    cycleDisc();

//#ifdef __APPLE__
    // reconstruct surface
    printf("surface reconstruction\n");
    surfacingUnsavedCycles();
//#endif
    evalUnsavedCycles();
}

void PointCloudRenderer::pickPoint(int mouseX , int mouseY , int op)
{
    int ni = -1;
    int selectedCurve = curveSelectionByRay(mouseX , mouseY , ni ,
        dispCurveNet->polyLines);
    bool isSnap = false;
    if (selectedCurve != -1 && ni != -1) isSnap = true;
    
	int selectedObj = -1;
    if (!isSnap) selectedObj = selectionByColorMap(mouseX , mouseY);

    bool dispPosExists = false;
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

            dispPos = dispCurveNet->polyLines[breakLine][breakPoint];
            if (pcUtils->point2Index.find(point2double(dispPos)) !=
                pcUtils->point2Index.end())
            {
                pos = dispPos;
                dispPosExists = true;
            }
            else
            {
                // pos = dispCurveNet->originPolyLines[breakLine][breakPoint];
                if (snapToNode)
                {
                    pos = dispPos;
                    pcUtils->addPointToGraph(dispPos);
                    int start = pcUtils->point2Index[point2double(lastDispPoint)];
                    if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
                    pcUtils->dijkstra(pcUtils->pointGraph , start , pcUtils->pointGraphInfo);
                }
                else
                {
                    pos = dispCurveNet->originPolyLines[breakLine][breakPoint];
                }
            }
            pickedDispPoint = dispCurveNet->polyLines[breakLine][breakPoint];
        }
        else
        {
            pos = dispPos = pickedDispPoint = pcUtils->pcData[selectedObj].pos;
            snapToNode = snapToCurve = false;
        }
        
		edi = pcUtils->point2Index[point2double(pos)];
        // printf("%d, pos = (%.6f,%.6f,%.6f), dispPos = (%.6f,%.6f,%.6f) %d , %d\n" ,
            // edi , pos.x , pos.y , pos.z , dispPos.x , dispPos.y , dispPos.z ,
            // snapToNode , snapToCurve);
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
                if (drawMode == 0)
                {
                    if (!ConstraintDetector::collinearTest(pathVertex , bsp))
                    {
                        convert2Spline(pathVertex , bsp);
                    }
                }
                else if (drawMode == 1)
                {
                    convert2Line(pathVertex , bsp);
                }
            }
		}
        /*
        printf("pickedPos = (%.6f,%.6f,%.6f)\n" , pickedDispPoint.x ,
                pickedDispPoint.y , pickedDispPoint.z);
        */
        if (op == 1)
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
                dispCurveNet->breakPath(breakLine , breakPoint , isAutoOpt);
                isCurvesChosen.resize(dispCurveNet->numPolyLines , false);
                newNode = false;
            }
            
            if (!isValid(lastDispPoint))
            {
                // curveNet->startPath(pos);
                if (!isSnap) dispCurveNet->startPath(dispPos);
            }
            else
            {
                /*
				printf("========== origin path ==========\n");
				for (int i = 0; i < pathForComp[0].size(); i++)
				{
					printf("(%.6f,%.6f,%.6f)\n" , pathForComp[0][i].x , pathForComp[0][i].y , pathForComp[0][i].z);
				}
				*/
                dispCurveNet->extendPath(lastDispPoint , dispPos , pathVertex ,
                    newNode , bsp , pathForComp[0] , isAutoOpt);

                isCurvesChosen.resize(dispCurveNet->numPolyLines , false);
                toExpandCurve = -1;
                afterCurveNetUpdate();
                if (isAutoOpt)
                {
                    dispPos = dispCurveNet->polyLines[dispCurveNet->numPolyLines - 1][0];
                }
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

            // autoGenBySymmetry();
		}
	}
	else
	{
		// pickedPoint = NULL;
        setNull(pickedDispPoint);
	}
}

void PointCloudRenderer::optUpdate(bool isRefreshConst)
{
    if (isRefreshConst)
        dispCurveNet->refreshAllConstraints();
    pcUtils->opt.init(dispCurveNet , pcUtils);
    pcUtils->opt.run(dispCurveNet);
	if (dispCurveNet->numPolyLines > 0)
	{
		vec3d stPos = dispCurveNet->polyLines[dispCurveNet->numPolyLines - 1][0];
		pcUtils->addPointToGraph(stPos);
		int sti = pcUtils->point2Index[point2double(stPos)];
		if (pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
		{
			pcUtils->dijkstra(pcUtils->pointGraph , sti , pcUtils->pointGraphInfo);
		}
		lastDispPoint = stPos;
	}
}

void PointCloudRenderer::cycleDisc()
{
    // std::vector<std::vector<vec3d> > inCurves;
    std::vector<std::vector<unsigned> > inCycleConstraints;
    std::vector<bool> inCycleToBeRemoved;
    if (dispCurveNet->numPolyLines > 0)
    {
        unsavedCycles.clear();
        unsavedMeshes.clear();
        unsavedNormals.clear();

        for (int i = 0; i < dispCurveNet->cycles.size(); i++)
        {
            for (int j = 0; j < dispCurveNet->cycles[i].size(); j++)
            {
                inCycleConstraints.push_back(dispCurveNet->cycles[i][j]);
                if (dispCurveNet->cycles[i].size() > 1)
                {
                    inCycleToBeRemoved.push_back(false);
                }
                else
                {
                    inCycleToBeRemoved.push_back(true);
                }
            }
        }

        unsavedInCurveNums.clear();
        for (int i = 0; i < unsavedInCurvePoints.size(); i++)
        {
            delete[] unsavedInCurvePoints[i];
        }
        for (int i = 0; i < unsavedInCurveNormals.size(); i++)
        {
            delete[] unsavedInCurveNormals[i];
        }
        unsavedInCurvePoints.clear();
        unsavedInCurveNormals.clear();
        /*
        writeLog("====== inCycleConstraints %lu ======\n" , dispCurveNet->cycles.size());
        for (int i = 0; i < dispCurveNet->cycles.size(); i++)
        {
            for (int j = 0; j < dispCurveNet->cycles[i].size(); j++)
            {
                int idx = dispCurveNet->cycles[i][j];
                vec3d st = dispCurveNet->polyLines[idx].front();
                vec3d ed = dispCurveNet->polyLines[idx].back();
                writeLog("curve id = %u, (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" , dispCurveNet->cycles[i][j] , st.x , st.y , st.z , ed.x , ed.y , ed.z);
            }
            writeLog("\n");
        }
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
        // output to screen
        /*
        inCurves = dispCurveNet->polyLines;
        inCycleConstraints = dispCurveNet->cycles;

        printf("====== inCycleConstraints %lu ======\n" , inCycleConstraints.size());
        for (int i = 0; i < inCycleConstraints.size(); i++)
        {
            for (int j = 0; j < inCycleConstraints[i].size(); j++)
            {
                int idx = inCycleConstraints[i][j];
                vec3d st = inCurves[idx].front();
                vec3d ed = inCurves[idx].back();
                printf("curve id = %u, (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" , inCycleConstraints[i][j] , st.x , st.y , st.z , ed.x , ed.y , ed.z);
            }
            printf("\n");
        }

        printf("*********** Input ***********\n");
        printf("%lu\n\n" , dispCurveNet->numPolyLines);
        for (int i = 0; i < dispCurveNet->numPolyLines; i++)
        {
            printf("%lu\n" , inCurves[i].size());
            for (int j = 0; j < inCurves[i].size(); j++)
            {
                printf("%.6f %.6f %.6f\n" , inCurves[i][j].x ,
                    inCurves[i][j].y , inCurves[i][j].z);
            }
            printf("\n");
        }
        cycle::cycleDiscovery(inCurves , inCycleConstraints ,
            unsavedCycles , unsavedMeshes , unsavedNormals);
        */
        cycle::cycleDiscovery(dispCurveNet->polyLines , inCycleConstraints ,
            inCycleToBeRemoved, unsavedCycles , toBeSurfacing ,
            unsavedInCurveNums , unsavedInCurvePoints , unsavedInCurveNormals ,
            unsavedMeshes , unsavedNormals);

        /*
        for (int i = 0; i < unsavedInCurveNums.size(); i++)
        {
            writeLog("===== cycle %d =====\n" , i);
            writeLog("---- position, %d ----\n" , unsavedInCurveNums[i]);
            for (int j = 0; j < unsavedInCurveNums[i]; j++)
            {
                writeLog("%.6f %.6f %.6f\n" , unsavedInCurvePoints[i][3 * j] ,
                    unsavedInCurvePoints[i][3 * j + 1] , unsavedInCurvePoints[i][3 * j + 2]);
            }
            writeLog("---- normal, %d ----\n" , unsavedInCurveNums[i]);
            for (int j = 0; j < unsavedInCurveNums[i]; j++)
            {
                writeLog("%.6f %.6f %.6f\n" , unsavedInCurveNormals[i][3 * j] ,
                    unsavedInCurveNormals[i][3 * j + 1] , unsavedInCurveNormals[i][3 * j + 2]);
            }
        }
        */
        /*
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
        */

        unsavedCyclePoints.clear();
        unsavedCycleCenters.clear();
        unsavedStatus.clear();
		inGroup.clear();
        for (int i = 0; i < unsavedCycles.size(); i++)
        {
            std::vector<Path> cyclePts;
            vec3d center;
            dispCurveNet->calcDispCyclePoints(unsavedCycles[i] ,
                cyclePts , center);
            unsavedCyclePoints.push_back(cyclePts);
            unsavedCycleCenters.push_back(center);
            unsavedStatus.push_back(true);
			inGroup.push_back(false);
        }
		/*
		for (int i = 0; i < unsavedMeshes.size(); i++)
		{
			writeLog("======= Patch %d =======\n" , i);
			for (int j = 0; j < unsavedMeshes[i].size(); j++)
			{
				writeLog("---- face %d ----\n" , j);
				for (int k = 0; k < unsavedMeshes[i][j].size(); k++)
				{
					writeLog("%.6f , %.6f , %.6f\n" , unsavedMeshes[i][j][k].x ,
						unsavedMeshes[i][j][k].y , unsavedMeshes[i][j][k].z);
				}
			}
		}
		*/
    }
}

void PointCloudRenderer::surfacingUnsavedCycles()
{
    for (int i = 0; i < unsavedCycles.size(); i++)
    {
        std::vector<std::vector<vec3d> > mesh;
		std::vector<std::vector<vec3d> > meshNorm;
        if (!toBeSurfacing[i])
        {
            unsavedMeshes.push_back(mesh);
            unsavedNormals.push_back(meshNorm);
            continue;
        }
        
        std::vector<int> numPoints;
        std::vector<double*> inCurves;
        std::vector<double*> inNorms;
        numPoints.push_back(unsavedInCurveNums[i]);
        inCurves.push_back(unsavedInCurvePoints[i]);
        inNorms.push_back(unsavedInCurveNormals[i]);

		surfaceBuilding(numPoints , inCurves , inNorms ,
			true , true , true , 0.f , 0.f , 1.f , 1.f , 
			mesh , meshNorm);
        
        unsavedMeshes.push_back(mesh);
        unsavedNormals.push_back(meshNorm);
        /*
        printf("write inCurves...\n");
        FILE *fout = fopen("inCurves.txt" , "w");
        fprintf(fout , "%d\n" , inCurves.size());
        for (int i = 0; i < inCurves.size(); i++)
        {
            fprintf(fout , "%d\n" , inCurves[i].size());
            for (int j = 0; j < inCurves[i].size(); j++)
            {
                fprintf(fout , "%.6f %.6f %.6f\n" , inCurves[i][j].x ,
                    inCurves[i][j].y , inCurves[i][j].z);
            }
        }
        fclose(fout);
        */
    }
}

void PointCloudRenderer::evalUnsavedCycles()
{
    if (unsavedMeshes.size() > 0)
    {
        printf("calculate cycle scores...\n");
        pcUtils->timer.PushCurrentTime();
        unsavedCycleScores.clear();
		unsavedCycleScoreRanks.clear();
        pcUtils->calcPatchScores(unsavedMeshes , unsavedCycleScores);

        for (int i = 0; i < unsavedMeshes.size(); i++)
        {
            //unsavedCycleScores.push_back(pcUtils->calcPatchScore(unsavedMeshes[i]));
			int cnt = 0;
			for (int j = 0; j < unsavedMeshes.size(); j++)
			{
				if (unsavedCycleScores[j] > unsavedCycleScores[i]) cnt++;
			}
			unsavedCycleScoreRanks.push_back(cnt + 1);

            printf("cycle %d: (" , i);
            for (int j = 0; j < unsavedCycles[i].size(); j++)
				printf(" %d " , unsavedCycles[i][j]);
            printf(")\n");
            printf("score = %.6f, rank = %d/%lu\n" , unsavedCycleScores[i] , unsavedCycleScoreRanks[i] , unsavedMeshes.size());
        }
        pcUtils->timer.PopAndDisplayTime("\nCycle scoring time: %.6f\n");
    }
}

void PointCloudRenderer::backup()
{
    pcUtils->timer.PushCurrentTime();
    backupCurveNet.copyFrom(*dispCurveNet);
    pcUtils->timer.PopAndDisplayTime("===== backup time: %.6fs =====\n");

    //backupCurveNet.debugLog();
}

void PointCloudRenderer::undo()
{
    CurveNet tmp;
    tmp.copyFrom(*dispCurveNet);
    dispCurveNet->copyFrom(backupCurveNet);
    backupCurveNet.copyFrom(tmp);
	if (isAutoOpt)
	{
		dispCurveNet->refreshAllConstraints();
	}
    //dispCurveNet->debugLog();
    clearTemp();
}

std::vector<vec3d> PointCloudRenderer::getRay(int mouseX , int mouseY)
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
	// printf ("World coords at z=0.0 are (%f, %f, %f)\n", wx, wy, wz);

	gluUnProject ((GLdouble) mouseX, (GLdouble) mouseY, 1.0, matMV, matProj, aViewport, &wx, &wy, &wz);
	vec3d lineP1(wx,wy,wz);
	// printf ("World coords at z=1.0 are (%f, %f, %f)\n", wx, wy, wz);

	std::vector<vec3d> ray;
	ray.push_back(lineP0);ray.push_back(lineP1);
	return ray;
}

int PointCloudRenderer::curveSelectionByRay(int mouseX , int mouseY , int& nodeIndex ,
    std::vector<Path>& polyLines)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;
    nodeIndex = -1;

    double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < polyLines.size(); i++)
    {
        //if (dispCurveNet->curveType[i] == -1) continue;
        for (int j = 0; j < polyLines[i].size(); j++)
        {
			vec3d p = polyLines[i][j];
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
    // printf("curve selection: %d %d , %.6f > %.6f\n" , res , nodeIndex , minDistance , snapOffset);
    if (res == -1) return res;
    if (nodeIndex == 0 || nodeIndex == polyLines[res].size() - 1)
        snapOffset *= 2;
    if (minDistance > snapOffset) res = -1;
    // printf("curve selection: choose node\n");
	if (res != -1)
	{
		if ((polyLines[res][nodeIndex] - polyLines[res].front()).length() < 1e-2)
		{
			nodeIndex = 0;
		}
		else if ((polyLines[res][nodeIndex] - polyLines[res].back()).length() < 1e-2)
		{
			nodeIndex = (int)polyLines[res].size() - 1;
		}
	}
    // printf("curve selection: success!\n");
    return res;
}

bool PointCloudRenderer::pickCurve(int mouseX , int mouseY , int op)
{
    int ni;
	pickedCurve = curveSelectionByRay(mouseX , mouseY , ni ,
        dispCurveNet->polyLines);
	if (pickedCurve != -1)
    {
        bool pickNode = (ni == 0 || ni == (int)dispCurveNet->polyLines[pickedCurve].size() - 1);
        if (pickNode)
        {
            pickedDispPoint = dispCurveNet->polyLines[pickedCurve][ni];
            pickedCurve = -1;
            snapToNode = true;
            snapToCurve = false;
        }
        else
        {
            setNull(pickedDispPoint);
            toExpandCurve = pickedCurve;
        }
        if (op == 2)
        {
            if (!pickNode)
            {
                // curveNet->deletePath(pickedCurve);
                dispCurveNet->deletePath(pickedCurve);
                //dispCurveNet->debugLog();
            }
            else
            {
                int ni = dispCurveNet->getNodeIndex(pickedDispPoint);
                dispCurveNet->deleteNode(ni);
            }
        }
        if (op == 3)
        {
            if (pickedCurve != -1) isCurvesChosen[pickedCurve] = !isCurvesChosen[pickedCurve];
        }
        if (op == 4)
        {
            if (!pickNode)
            {
                if (pickedCurve != -1)
                {
                    dispCurveNet->isCurvesFixed[pickedCurve] = !dispCurveNet->isCurvesFixed[pickedCurve];
                }
            }
            else
            {
                int ni = dispCurveNet->getNodeIndex(pickedDispPoint);
                if (ni != -1) dispCurveNet->isNodesFixed[ni] = !dispCurveNet->isNodesFixed[ni];
            }
        }
        return true;
    }
    if (op == 5)
    {
        pickedCurve = -1;
        setNull(pickedDispPoint);
        std::fill(isCurvesChosen.begin() , isCurvesChosen.end() , false);
        vec2d lt = spaceLocationTo2D(mouseLocationTo3D(lt_x , lt_y) , sketchPlane);
        vec2d rb = spaceLocationTo2D(mouseLocationTo3D(rb_x , rb_y) , sketchPlane);
        vec2d rec[4];
        rec[0] = vec2d(std::min(lt.x , rb.x) , std::min(lt.y , rb.y));
        rec[1] = vec2d(std::min(lt.x , rb.x) , std::max(lt.y , rb.y));
        rec[2] = vec2d(std::max(lt.x , rb.x) , std::max(lt.y , rb.y));
        rec[3] = vec2d(std::max(lt.x , rb.x) , std::min(lt.y , rb.y));
        for (int i = 0; i < dispCurveNet->bsplines.size(); i++)
        {
            if (dispCurveNet->curveType[i] == -1) continue;
            for (int j = 0; j < dispCurveNet->bsplines[i].ctrlNodes.size() - 1; j++)
            {
                vec3d st = pcUtils->openGLView->cameraPos;
                vec3d dir = dispCurveNet->bsplines[i].ctrlNodes[j] - st;
                dir.normalize();
                vec2d p1 = spaceLocationTo2D(sketchPlane.intersect(st , dir) , sketchPlane);
                dir = dispCurveNet->bsplines[i].ctrlNodes[j + 1] - st;
                dir.normalize();
                vec2d p2 = spaceLocationTo2D(sketchPlane.intersect(st , dir) , sketchPlane);
                if (isIntersected2D(rec[0] , rec[1] , p1 , p2) ||
                    isIntersected2D(rec[1] , rec[2] , p1 , p2) ||
                    isIntersected2D(rec[2] , rec[3] , p1 , p2) ||
                    isIntersected2D(rec[3] , rec[0] , p1 , p2) ||
                    (p1.x >= rec[0].x && p1.x <= rec[2].x && p1.y >= rec[0].y && p1.y <= rec[2].y) ||
                    (p2.x >= rec[0].x && p2.x <= rec[2].x && p2.y >= rec[0].y && p2.y <= rec[2].y))
                {
                    isCurvesChosen[i] = true;
                    break;
                }
            }
        }
    }
    return false;
}

bool PointCloudRenderer::pickAutoCurve(int mouseX , int mouseY , int op)
{
    int ni;
	pickedAutoCurve = curveSelectionByRay(mouseX , mouseY , ni ,
        autoGenPaths);
    if (pickedAutoCurve != -1)
    {
        if (op == 1)
        {
            printf("begin storing auto curve\n");
            dispCurveNet->storeAutoPath(autoGenPaths[pickedAutoCurve] ,
                autoGenBsp[pickedAutoCurve] , autoGenOriginPaths[pickedAutoCurve] ,
                selectionOffset * 1.5 , isAutoOpt);
            isCurvesChosen.resize(dispCurveNet->numPolyLines , false);

            toExpandCurve = -1;
            afterCurveNetUpdate();
            vec3d pos = dispCurveNet->polyLines.back().front();
            pcUtils->addPointToGraph(pos);
            pos = dispCurveNet->polyLines.back().back();
            pcUtils->addPointToGraph(pos);

            autoGenOriginPaths.erase(autoGenOriginPaths.begin() + pickedAutoCurve);
            autoGenPaths.erase(autoGenPaths.begin() + pickedAutoCurve);
            autoGenBsp.erase(autoGenBsp.begin() + pickedAutoCurve);
        }
        return true;
    }
    return false;
}

void PointCloudRenderer::pickAllAutoCurves(int op)
{
    printf("begin storing translated auto curves\n");
    if (op == 1)
    {
        for (int i = 0; i < autoPathOrigins.size(); i++)
        {
            int j = autoPathOrigins[i];
            dispCurveNet->deletePath(j);
        }
    }
    for (int i = 0; i < autoGenPaths.size(); i++)
    {
        pcUtils->gradientDescentSmooth(autoGenPaths[i] , false);
        if (!ConstraintDetector::collinearTest(autoGenPaths[i] , autoGenBsp[i]))
        {
            convert2Spline(autoGenPaths[i] , autoGenBsp[i]);
        }
        autoGenBsp[i].calcCoefs();

        dispCurveNet->storeAutoPath(autoGenPaths[i] ,
            autoGenBsp[i] , autoGenOriginPaths[i] ,
            selectionOffset * 1.5 , isAutoOpt);
    }
    isCurvesChosen.resize(dispCurveNet->numPolyLines , false);

    afterCurveNetUpdate();
    for (int i = dispCurveNet->polyLines.size() - autoGenPaths.size();
         i < dispCurveNet->polyLines.size(); i++)
    {
        vec3d pos = dispCurveNet->polyLines[i].front();
        pcUtils->addPointToGraph(pos);
        pos = dispCurveNet->polyLines[i].back();
        pcUtils->addPointToGraph(pos);
    }

    autoGenOriginPaths.clear();
    autoGenPaths.clear();
    autoGenBsp.clear();
    autoPathOrigins.clear();
}

int PointCloudRenderer::cycleSelectionByRay(int mouseX , int mouseY ,
    std::vector<vec3d>& cycleCenters)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;

    double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < cycleCenters.size(); i++)
    {
        vec3d p = cycleCenters[i];
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
    if (minDistance > 0.5) res = -1;
    return res;
}

int PointCloudRenderer::cycleGroupSelectionByRay(int mouseX , int mouseY ,
    std::vector<std::vector<vec3d> >& cycleCenters)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;

    double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < cycleCenters.size(); i++)
    {
        for (int j = 0; j < cycleCenters[i].size(); j++)
        {
            vec3d p = cycleCenters[i][j];
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
	}
    
    // printf("%d %d , %.6f > %.6f\n" , res , nodeIndex , minDistance , snapOffset);
    if (minDistance > 0.5) res = -1;
    return res;
}

void PointCloudRenderer::cycleStatusUpdate()
{
    unsavedStatus.resize(unsavedCycles.size());
    for (int i = 0; i < unsavedCycles.size(); i++)
    {
        bool flag = true;
        bool surfacingFlag = true;
        for (int j = 0; j < dispCurveNet->cycles.size(); j++)
        {
            for (int k = 0; k < dispCurveNet->cycles[j].size(); k++)
            {
                if (isSameCycle(unsavedCycles[i] , dispCurveNet->cycles[j][k]))
                {
                    if (dispCurveNet->cycles[j].size() == 1)
                    {
                        flag = false;
                    }
                    else
                    {
                        flag = true;
                        surfacingFlag = false;
                    }
                    break;
                }
            }
        }
        unsavedStatus[i] = flag;
        toBeSurfacing[i] = surfacingFlag;
    }
	/*
	printf("cycle num = %d ? %d: " , unsavedCycles.size() , unsavedStatus.size());
	for (int i = 0; i < unsavedStatus.size(); i++)
		printf("%d " , (int)unsavedStatus[i]);
	printf("\n");
	*/
}

void PointCloudRenderer::pickCycle(int mouseX , int mouseY , int op)
{
	pickedCycle = cycleSelectionByRay(mouseX , mouseY , unsavedCycleCenters);
    if (pickedCycle != -1 && !unsavedStatus[pickedCycle])
    {
        pickedCycle = -1;
    }
	if (pickedCycle != -1)
    {
		char buf[128];
		sprintf(buf , "cycle score = %.6f, rank = %d/%lu\n" , unsavedCycleScores[pickedCycle] ,
			unsavedCycleScoreRanks[pickedCycle] , unsavedMeshes.size());
		wxString str(buf);
		pcUtils->statusBar->SetStatusText(str);

        if (op == 1)
        {
            dispCurveNet->addCycle(unsavedCycles[pickedCycle] ,
				unsavedCyclePoints[pickedCycle] ,
				unsavedCycleCenters[pickedCycle]);
            dispCurveNet->meshes.push_back(unsavedMeshes[pickedCycle]);
            dispCurveNet->meshNormals.push_back(unsavedNormals[pickedCycle]);

            pcUtils->pcSegmentByPatches(dispCurveNet->meshes);

			cycleStatusUpdate();
        }
		else if (op == 3)
		{
			if (!inGroup[pickedCycle])
			{
				inGroup[pickedCycle] = true;
				group.push_back(pickedCycle);
			}
			/*
			printf("\ncycle group: (");
			for (int i = 0; i < group.size(); i++)
			{
				printf(" %d " , group[i]);
			}
			printf(")\n");
			*/
		}
		else if (op == 4)
		{
			if (inGroup[pickedCycle])
			{
				inGroup[pickedCycle] = false;
				group.erase(find(group.begin() , group.end() , pickedCycle));
			}
			/*
			printf("\ncycle group: (");
			for (int i = 0; i < group.size(); i++)
			{
				printf(" %d " , group[i]);
			}
			printf(")\n");
			*/
		}

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

void PointCloudRenderer::cycleGroupUpdate()
{
	std::vector<Cycle> unsavedCycleGroup;
	std::vector<std::vector<Path> > unsavedCycleGroupPoints;
	std::vector<vec3d> unsavedCycleGroupCenters;

    std::vector<int> numPoints;
    std::vector<double*> inCurves;
    std::vector<double*> inNorms;
	for (int i = 0; i < group.size(); i++)
	{
		unsavedCycleGroup.push_back(unsavedCycles[group[i]]);
		unsavedCycleGroupPoints.push_back(unsavedCyclePoints[group[i]]);
		unsavedCycleGroupCenters.push_back(unsavedCycleCenters[group[i]]);

        numPoints.push_back(unsavedInCurveNums[group[i]]);
        inCurves.push_back(unsavedInCurvePoints[group[i]]);
        inNorms.push_back(unsavedInCurveNormals[group[i]]);
	}

	std::vector<std::vector<vec3d> > mesh;
	std::vector<std::vector<vec3d> > meshNorm;
    
	surfaceBuilding(numPoints , inCurves , inNorms ,
		true , true , true , 1.f , 0.f , 0.f , 0.f , mesh , meshNorm);

    dispCurveNet->addCycleGroup(unsavedCycleGroup , unsavedCycleGroupPoints , unsavedCycleGroupCenters);
    dispCurveNet->meshes.push_back(mesh);
    dispCurveNet->meshNormals.push_back(meshNorm);
	group.clear();

	pcUtils->pcSegmentByPatches(dispCurveNet->meshes);

	cycleStatusUpdate();
}

void PointCloudRenderer::pickSavedCycle(int mouseX , int mouseY , int op)
{
	pickedSavedCycle = cycleGroupSelectionByRay(mouseX , mouseY , dispCurveNet->cycleCenters);
	if (pickedSavedCycle != -1)
    {
        if (op == 2)
        {
            dispCurveNet->deleteCycle(pickedSavedCycle);
            cycleStatusUpdate();
			pcUtils->pcSegmentByPatches(dispCurveNet->meshes);
        }
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

void PointCloudRenderer::surfaceBuilding(std::vector<int> &numPoints, std::vector<double*> &inCurves, 
	std::vector<double*> &inNorms, bool useDelaunay, bool useMinSet, bool useNormal, 
	float areaWeight, float edgeWeight, float dihedralWeight, float boundaryNormalWeight, 
	std::vector<std::vector<vec3d> > &mesh , std::vector<std::vector<vec3d> > &meshNormals)
{
	int numPositions , numFaces;
	float *_pPositions;
	float *_pNormals;
	int *_pFaceIndices;
	std::vector<int> sizeField(3 , 0);
	for (int i = 0; i < 3; i++) sizeField[i] = pcUtils->sizeOriginF[i];
	
// 	coarseSuf::coarseSuf(numPoints , inCurves , inNorms ,
// 		true , true , false ,
// 		0.f , 0.f , 0.f , 1.f , numPositions , numFaces ,
// 		&_pPositions , &_pNormals , &_pFaceIndices);
	
	coarseSuf::genInitTriangulation(numPoints , inCurves , inNorms ,
		sizeField , pcUtils->extXval , pcUtils->extYval , pcUtils->extZval ,
		pcUtils->f , true , true , true ,
		0.f , 0.f , 0.f , 0.f , 1.f , numPositions , numFaces ,
		&_pPositions , &_pNormals , &_pFaceIndices);
	mesh.clear();
	meshNormals.clear();

#if OUTPUT_MESH_IN_CURVE
    FILE* fout = fopen("inCurves.txt" , "w");
    fprintf(fout , "%d\n" , inCurves.size());
    for (int i = 0; i < inCurves.size(); i++)
    {
        fprintf(fout , "\n");
        fprintf(fout , "%d\n" , numPoints[i]);
        for (int j = 0; j < numPoints[i]; j++)
        {
            fprintf(fout , "%.6f %.6f %.6f %.6f %.6f %.6f\n" , inCurves[i][3 * j] ,
                inCurves[i][3 * j + 1] , inCurves[i][3 * j + 2] ,
                inNorms[i][3 * j] , inNorms[i][3 * j + 1] , inNorms[i][3 * j + 2]);
        }
    }
    fclose(fout);
#endif

    bool smoothing_Nathan = false;
	bool smoothing_Ming = true;

    if (smoothing_Nathan)
	{
#ifdef _WIN32
        SP::Mesh spMesh,outputMesh;
        spMesh.allocateVertices(numPositions);
        spMesh.allocateFaces(numFaces);

        float *pPositions(spMesh.getPositions()),
            *pNormals(spMesh.getNormals());

        for(int i=0;i<numPositions;++i){
            pPositions[3*i] = _pPositions[3*i];
            pPositions[3*i+1] = _pPositions[3*i+1];
            pPositions[3*i+2] = _pPositions[3*i+2];
            pNormals[3*i] = _pNormals[3*i];
            pNormals[3*i+1] = _pNormals[3*i+1];
            pNormals[3*i+2] = _pNormals[3*i+2];
        }

        int *pFaceIndices(spMesh.getFaceIndices());
        int nTempIndex(0);
        for(int i=0; i<numFaces*3; i++){
            pFaceIndices[i] = _pFaceIndices[i]; 
        }	

        SP::SmoothPatchSettings settings;
        settings.mbConstrainNormals=false;
        settings.mbRemesh=true;
        settings.mfTension=0.0;
        settings.mnNumSubdivisions=1;
        settings.mnNumLaplacianSmooths=3;

		FILE *fout = fopen("coarse_mesh.txt" , "w");
		fprintf(fout , "%d\n" , numPositions);
		for (int i = 0; i < numPositions; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				fprintf(fout , "%.6f " , pPositions[3 * i + j]);
			}
			fprintf(fout , "\n");
		}
		fprintf(fout , "%d\n" , numFaces);
        for (int i = 0; i < numFaces; i++)
        {
			for (int j = 0; j < 3; j++)
			{
				fprintf(fout , "%d " , pFaceIndices[3 * i + j]);
			}
			fprintf(fout , "\n");
			/*
            vec3i pi;
            for (int j = 0; j < 3; j++)
            {
                pi[j] = pFaceIndices[3 * i + j];
            }
            writeLog("===== face (%d %d %d) =====\n" , pi.x , pi.y , pi.z);
            for (int j = 0; j < 3; j++)
            {
                vec3d p;
                int index = pi[j];
                p.x = pPositions[3 * index];
                p.y = pPositions[3 * index + 1];
                p.z = pPositions[3 * index + 2];
                writeLog("(%.6f , %.6f , %.6f)\n" , p.x , p.y , p.z);
            }
			*/
        }
		fclose(fout);

        SP::SmoothPatchBuilder smoothPatchBuilder;

        smoothPatchBuilder.buildSmoothPatch(settings,spMesh,outputMesh);

		int outPosNum = outputMesh.getVertexCount();
        int outFaceNum = outputMesh.getFaceCount();
	
        int *pInd = outputMesh.getFaceIndices();
        float *pPoints=outputMesh.getPositions();
        float *pNewNormals = outputMesh.getNormals();
        for(int i=0; i<outFaceNum; i++){
            std::vector<vec3d> tri;
            std::vector<vec3d> triNorms;
            for (int j=0;j<3;j++){
                int pointID = pInd[i*3+j];
                vec3d p(0.0) , n(0.0);
                for(int k=0;k<3;k++){
                    p[k]=(double)pPoints[pointID*3+k];
                    if(useNormal)
					n[k]=(double)pNewNormals[pointID*3+k];
                }
                tri.push_back(p);
                triNorms.push_back(n);
            }
            mesh.push_back(tri);
            if (useNormal)
			meshNormals.push_back(triNorms);
        }

		fout = fopen("refined_mesh.txt" , "w");
		fprintf(fout , "%d\n" , outPosNum);
		for (int i = 0; i < outPosNum; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				fprintf(fout , "%.6f " , pPoints[3 * i + j]);
			}
			fprintf(fout , "\n");
		}
		fprintf(fout , "%d\n" , outFaceNum);
        for (int i = 0; i < outFaceNum; i++)
        {
			for (int j = 0; j < 3; j++)
			{
				fprintf(fout , "%d " , pInd[3 * i + j]);
			}
			fprintf(fout , "\n");
        }
		fclose(fout);
#endif
	}
	else
	{
		if (smoothing_Ming)
		{
			int _numPositions_smoothed;
			int _numFaces_smoothed;
			float* _pPositions_smoothed;
			int* _pFaceIndices_smoothed;

			SufTune::sufTune(numPositions , numFaces , _pPositions , _pFaceIndices ,
				sizeField , pcUtils->extXval , pcUtils->extYval , pcUtils->extZval ,
				pcUtils->f , 0 , 1.414 , 0.5 , 0.0025 , 10 , 50 , 10 , true ,
				_numPositions_smoothed , _numFaces_smoothed ,
				&_pPositions_smoothed , &_pFaceIndices_smoothed);
			delete[] _pPositions;
			delete[] _pFaceIndices;
			numPositions = _numPositions_smoothed;
			numFaces = _numFaces_smoothed;
			_pPositions = _pPositions_smoothed;
			_pFaceIndices = _pFaceIndices_smoothed;
		}
		for (int j = 0; j < numFaces; j++)
		{
			vec3i face(_pFaceIndices[3 * j] , _pFaceIndices[3 * j + 1] , _pFaceIndices[3 * j + 2]);
			std::vector<vec3d> triPos;
			std::vector<vec3d> triNorm;
			for (int k = 0; k < 3; k++)
			{
				vec3d p , n;
				p.x = _pPositions[3 * face[k]];
				p.y = _pPositions[3 * face[k] + 1];
				p.z = _pPositions[3 * face[k] + 2];
// 				n.x = _pNormals[3 * face[k]];
// 				n.y = _pNormals[3 * face[k] + 1];
// 				n.z = _pNormals[3 * face[k] + 2];
                triPos.push_back(p);
				triNorm.push_back(n);
			}
			mesh.push_back(triPos);
			meshNormals.push_back(triNorm);
		}
	}

#if (OUTPUT_MESH)
    FILE* fout = fopen("coarse_mesh.txt" , "w");
    fprintf(fout , "%d\n" , numPositions);
    for (int i = 0; i < numPositions; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            fprintf(fout , "%.6f " , _pPositions[3 * i + j]);
        }
        fprintf(fout , "\n");
    }
    fprintf(fout , "%d\n" , numFaces);
    for (int i = 0; i < numFaces; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            fprintf(fout , "%d " , _pFaceIndices[3 * i + j]);
        }
        fprintf(fout , "\n");
    }
    fclose(fout);
#endif

	delete[] _pPositions;
	delete[] _pNormals;
	delete[] _pFaceIndices;
}

int PointCloudRenderer::ctrlNodeSelectionByRay(int mouseX , int mouseY , int& nodeIndex)
{
    std::vector<vec3d> rays = getRay(mouseX , mouseY);
    vec3d& rayStr = rays.front();
    vec3d& rayEnd = rays.back();
    
    double minDistance = 1e20;
    int res = -1;
    nodeIndex = -1;

    double p0p1LenSquared = (rayEnd - rayStr).dot(rayEnd - rayStr);

	for (int i = 0; i < dispCurveNet->bsplines.size(); i++)
    {
        if (dispCurveNet->curveType[i] == -1) continue;
		for (int j = 0; j < dispCurveNet->bsplines[i].ctrlNodes.size(); j++)
        {
			vec3d p = dispCurveNet->bsplines[i].ctrlNodes[j];
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
    double snapOffset = selectionOffset * 1.5;
    // printf("%d %d , %.6f > %.6f\n" , res , nodeIndex , minDistance , snapOffset);
    if (res == -1) return res;
    //if (nodeIndex == 0 || nodeIndex == dispCurveNet->polyLines[res].size() - 1)
    //snapOffset *= 2;
    if (minDistance > snapOffset)
    {
        res = -1;
        nodeIndex = -1;
    }
    return res;
}

bool PointCloudRenderer::pickCtrlNode(int mouseX , int mouseY , int lastX , int lastY , int op)
{
    if (op == 0)
    {
        pickedBsp = ctrlNodeSelectionByRay(mouseX , mouseY , pickedCtrlNode);
    }
    else if (op == 1)
    {
        // printf("bspIndex = %d, ctrlNodeIndex = %d\n" , pickedBsp , pickedCtrlNode);
        if (pickedBsp == -1) return false;
        vec3d pos = dispCurveNet->bsplines[pickedBsp].ctrlNodes[pickedCtrlNode];
        std::vector<vec3d> rays = getRay(mouseX , mouseY);
        vec3d dir = rays.back() - rays.front();
        dir.normalize();
        // choose tangent plane
        double dx = std::abs(mouseX - lastX);
        double dy = std::abs(mouseY - lastY);
        /*
        dragPlane.p = dragStartPoint;
        dragPlane.n = dir;
        if (dragPlaneNormalIndex == 0)
            dragPlane.n = vec3d(0.0 , 1.0 , 0.0);
        else
            dragPlane.n = vec3d(0.0 , 0.0 , 1.0);
        dragPlane.d = -pos.dot(dragPlane.n);
        */
        dragCurPoint = pos;
        /*
        printf("p=(%.6f,%.6f,%.6f), n=(%.6f,%.6f,%.6f)\n" ,
            dragPlane.p.x , dragPlane.p.y , dragPlane.p.z , dragPlane.n.x , dragPlane.n.y ,
            dragPlane.n.z);
        */
        vec3d newPos = dragPlane.intersect(rays.front() , dir);
        // printf("newPos = (%.6f, %.6f, %.6f)\n", newPos.x , newPos.y , newPos.z);
        dispCurveNet->updatePath(pickedBsp , pickedCtrlNode , newPos , false);
    }
    if (pickedBsp == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void PointCloudRenderer::autoGenBySymmetry()
{
    autoGenOriginPaths.clear();
    autoGenPaths.clear();
    autoGenBsp.clear();
    Path originPath , path;
    BSpline bsp;
    int bspIndex;
    if (toExpandCurve != -1)
    {
        bspIndex = toExpandCurve;
    }
    else
    {
        bspIndex = dispCurveNet->numPolyLines - 1;
        for (;;)
        {
            if (bspIndex < 0 || dispCurveNet->curveType[bspIndex] != -1) break;
            bspIndex--;
        }
    }
    if (bspIndex < 0) return;
    for (int i = 0; i < pcUtils->partSym.symPlanes.size(); i++)
    {
        Plane& plane = pcUtils->partSym.symPlanes[i];
        bool flag = dispCurveNet->reflSymPath(bspIndex , plane ,
            originPath , path , bsp);
        if (!flag) continue;
        autoGenOriginPaths.push_back(originPath);
        autoGenPaths.push_back(path);
        autoGenBsp.push_back(bsp);
    }
}

void PointCloudRenderer::initTranslationMode(bool initAxisOrigin)
{
    axisPlane = crossPlane;
    autoGenOriginPaths.clear();
    autoGenPaths.clear();
    autoGenBsp.clear();
    autoPathOrigins.clear();
    bool isFirst = initAxisOrigin;
    for (int i = 0; i < dispCurveNet->numPolyLines; i++)
    {
        if (isCurvesChosen[i])
        {
            if (isFirst)
            {
                axisWidget.origin = dispCurveNet->polyLines[i][0];
                axisWidget.resamplePoints();
                /*
                for (int j = 0; j < axisWidget.axes.size(); j++)
                {
                    resampleLine(axisWidget.origin - axisWidget.axes[j] * 0.5 ,
                        axisWidget.origin + axisWidget.axes[j] * 0.5 , 500 ,
                        axisWidget.axesPoints[j]);
                }
                */
                isFirst = false;
            }
            autoGenOriginPaths.push_back(dispCurveNet->originPolyLines[i]);
            autoGenPaths.push_back(dispCurveNet->polyLines[i]);
            autoGenBsp.push_back(dispCurveNet->bsplines[i]);
            autoPathOrigins.push_back(i);
        }
    }
}

void PointCloudRenderer::autoGenByTranslation(double offset)
{
    Path originPath , path;
    BSpline bsp;
    for (int i = 0; i < autoGenPaths.size(); i++)
    {
        // printf("  translate path %d\n" , i);
        dispCurveNet->translatePath(autoGenOriginPaths[i] , autoGenPaths[i] ,
            autoGenBsp[i] , axisPlane , offset , originPath , path , bsp);

        autoGenOriginPaths[i] = originPath;
        autoGenPaths[i] = path;
        autoGenBsp[i] = bsp;
    }
    axisWidget.origin += axisPlane.n * offset;
    axisWidget.resamplePoints();
    /*
    for (int i = 0; i < axisWidget.axesPoints.size(); i++)
    {
        for (int j = 0; j < axisWidget.axesPoints[i].size(); j++)
        {
            axisWidget.axesPoints[i][j] += axisPlane.n * offset;
        }
    }
    */
}

void PointCloudRenderer::autoGenByScaling(double offset)
{
    Path originPath , path;
    BSpline bsp;
    for (int i = 0; i < autoGenPaths.size(); i++)
    {
        // printf("  translate path %d\n" , i);
        dispCurveNet->scalePath(autoGenOriginPaths[i] , autoGenPaths[i] ,
            autoGenBsp[i] , axisPlane , offset , originPath , path , bsp);

        autoGenOriginPaths[i] = originPath;
        autoGenPaths[i] = path;
        autoGenBsp[i] = bsp;
    }
}

void PointCloudRenderer::autoGenByRotation(double offset)
{
    Path originPath , path;
    BSpline bsp;
    for (int i = 0; i < autoGenPaths.size(); i++)
    {
        // printf("  translate path %d\n" , i);
        dispCurveNet->rotatePath(autoGenOriginPaths[i] , autoGenPaths[i] ,
            autoGenBsp[i] , axisPlane , offset , originPath , path , bsp);

        autoGenOriginPaths[i] = originPath;
        autoGenPaths[i] = path;
        autoGenBsp[i] = bsp;
    }
}

void PointCloudRenderer::autoGenByICP()
{
// #if defined(__APPLE__)
#if 0
    autoGenOriginPaths.clear();
    autoGenPaths.clear();
    autoGenBsp.clear();

    PointCloudT::Ptr cloud_source (new PointCloudT);
    PointCloudT::Ptr cloud_target (new PointCloudT);
    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    PointCloudT::Ptr cloud_result (new PointCloudT);
    Eigen::Matrix4f transMat;

    path2PointCloud(chosenPoints , cloud_target);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(200);
    // icp.setMaxCorrespondenceDistance(0.05);
    icp.setInputTarget(cloud_target);

    double minScore = 1e20;
    int matchedCurve = -1;
    if (toExpandCurve == -1)
    {
        for (int i = 0; i < dispCurveNet->numPolyLines; i++)
        {
            if (dispCurveNet->curveType[i] == -1) continue;
            path2PointCloud(dispCurveNet->polyLines[i] , cloud_source);
            icp.setInputSource(cloud_source);
            icp.align(*cloud_tmp);
            double score = icp.getFitnessScore();
            // printf("curve %d icp score: %.6f\n" , i , score);

            if (minScore > score)
            {
                minScore = score;
                matchedCurve = i;
                *cloud_result = *cloud_tmp;
                transMat = icp.getFinalTransformation();
            }
        }
    }
    else
    {
        path2PointCloud(dispCurveNet->polyLines[toExpandCurve] , cloud_source);
        icp.setInputSource(cloud_source);
        icp.align(*cloud_result);
        minScore = icp.getFitnessScore();
        matchedCurve = toExpandCurve;
        transMat = icp.getFinalTransformation();
    }
    printf("curve %d icp score: %.6f\n" , matchedCurve , minScore);
    // post-icp
    /*
    int iters = 0;
    double prevScore = 1e20;
    double distThreshold = 0.02;
    for (int it = 0; it < iters; it++)
    {
        debugPoints.clear();

        icp.setInputSource(cloud_result);

        // cloud_target->width = cloud_result->width;
        // cloud_target->points.resize(cloud_target->width * cloud_target->height);
        std::vector<vec3d> closePoints;
        for (int i = 0; i < cloud_result->width; i++)
        {
            DistQuery q;
            q.maxSqrDis = 1e30;
            vec3d pos(cloud_result->points[i].x , cloud_result->points[i].y ,
                cloud_result->points[i].z);
            pcUtils->tree->searchKnn(0 , pos , q);
            // if (q.maxSqrDis > distThreshold * distThreshold) continue;
            pos = pcUtils->pcData[q.pointIndex].pos;
            closePoints.push_back(pos);
        }

        cloud_target->width = closePoints.size();
        cloud_target->height = 1;
        cloud_target->points.resize(cloud_target->width * cloud_target->height);
        for (int i = 0; i < closePoints.size(); i++)
        {
            vec3d pos = closePoints[i];
            debugPoints.push_back(pos);

            cloud_target->points[i].x = pos.x;
            cloud_target->points[i].y = pos.y;
            cloud_target->points[i].z = pos.z;
        }
        icp.setInputTarget(cloud_target);
        icp.align(*cloud_tmp);
        printf("post opt iter %d score: %.6f\n" , it , icp.getFitnessScore());
        if (icp.getFitnessScore() < prevScore)
            prevScore = icp.getFitnessScore();
        else
            break;
        *cloud_result = *cloud_tmp;
    }
    */
    Path originPath , path;
    BSpline bsp;

    if (minScore > 1e-2)
    {
    }
    // pointCloud2Path(cloud_result , originPath);
    /*
    for (int i = 0; i < dispCurveNet->polyLines[matchedCurve].size(); i++)
    {
        Eigen::Vector4f p;
        for (int j = 0; j < 3; j++) p(j) = dispCurveNet->polyLines[matchedCurve][i][j];
        p(3) = 1.f;
        p = transMat * p;
        originPath.push_back(vec3d(p(0) , p(1) , p(2)));
    }
    path = originPath;
    if (!ConstraintDetector::collinearTest(path , bsp))
    {
        convert2Spline(path , bsp);
    }
    bsp.calcCoefs();
    */
    dispCurveNet->transformPath(matchedCurve , transMat , originPath , path , bsp);

    pcUtils->gradientDescentSmooth(path , true);
    if (!ConstraintDetector::collinearTest(path , bsp))
    {
        convert2Spline(path , bsp);
    }
    bsp.calcCoefs();

    autoGenOriginPaths.push_back(originPath);
    autoGenPaths.push_back(path);
    autoGenBsp.push_back(bsp);
#endif
}

void PointCloudRenderer::autoGenByPclNurbsFitting()
{
#if defined(__MACH__) && defined(__APPLE__)
    autoGenOriginPaths.clear();
    autoGenPaths.clear();
    autoGenBsp.clear();

    pcl::on_nurbs::NurbsDataCurve2d inputPoints2d;
    LocalFrame frame;
    frame.buildFromNormal(crossPlane.n);
    for (int i = 0; i < chosenPoints.size(); i++)
    {
        vec3d posLocal = frame.toLocal(chosenPoints[i] - crossPlane.p);
        inputPoints2d.interior.push_back(Eigen::Vector2d(posLocal.x , posLocal.z));
    }
    pcl::on_nurbs::FittingCurve2d::Parameter curve_params;
    curve_params.smoothness = 0.000001;
    curve_params.rScale = 1.0;
    unsigned order = 3;
    unsigned numCtrlPoints = std::max((unsigned)6 , (unsigned)(chosenPoints.size() / 4 + 1));
    printf("num_ctrl_points = %d\n" , numCtrlPoints);

    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsPCA(order ,
        &inputPoints2d , numCtrlPoints);
    pcl::on_nurbs::FittingCurve2d fit(&inputPoints2d , curve);
    fit.assemble(curve_params);
    fit.solve();

    // resample on nurbs
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs , cloud , 8);

    // bspline fitting by my matlab code
    Path originPath , path;
    BSpline bsp;
    // printf("cloud_size = %lu\n" , cloud->size());
    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGB &p = cloud->at(i);
        // printf("%.6f %.6f %.6f\n" , p.x , p.y , p.z);
        vec3d posLocal(p.x , 0 , p.y);
        vec3d pos = crossPlane.p + frame.toWorld(posLocal);
        originPath.push_back(pos);
    }
    path = originPath;

    /*
    pcUtils->gradientDescentSmooth(path , true);
    */
    if (!ConstraintDetector::collinearTest(path , bsp))
    {
        convert2Spline(path , bsp);
    }
    bsp.calcCoefs();

    autoGenOriginPaths.push_back(originPath);
    autoGenPaths.push_back(path);
    autoGenBsp.push_back(bsp);
#endif
}

void PointCloudRenderer::clearAutoGen()
{
    autoGenPaths.clear();
    autoGenOriginPaths.clear();
    autoGenBsp.clear();
}

void PointCloudRenderer::initFreeSketchMode()
{
    chosenPoints.clear();
    isChosen.resize(pcUtils->pcData.size());
    for (int i = 0; i < pcUtils->pcData.size(); i++) isChosen[i] = false;
}

void PointCloudRenderer::freeSketchOnPointCloud(std::vector<std::pair<unsigned , unsigned> > & seq)
{
    // printf("----- seq size = %lu -----\n" , seq.size());
    for (int i = 0; i < seq.size(); i++)
    {
        int x = seq[i].first , y = seq[i].second;
        // printf("(%d , %d)\n" , x , y);
        int pointIndex = selectionByColorMap(x , y);
        if (pointIndex != -1 && !isChosen[pointIndex])
        {
            if (crossPlane.isValid() && pointIndex < isCrossing.size() &&
                !isCrossing[pointIndex]) continue;

            isChosen[pointIndex] = true;
            chosenPoints.push_back(pcUtils->pcData[pointIndex].pos);

            std::vector<vec3d> ray = getRay(x , y);
            vec3d dir = ray.back() - ray.front();
            dir.normalize();
            sketchLine.push_back(sketchPlane.intersect(ray.front() , dir));
            double t = (-sketchPlane.n.dot(ray.front()) - sketchPlane.d) /
                (sketchPlane.n.dot(dir));
            // printf("st = (%.6f,%.6f,%.6f), dir = (%.6f,%.6f,%.6f),\nt=%.6f, p = (%.6f,%.6f,%.6f)\n" ,
                // ray.front().x , ray.front().y , ray.front().z ,
                // dir.x , dir.y , dir.z , t ,
                // sketchLine.back().x , sketchLine.back().y , sketchLine.back().z);
        }
    }
}

void PointCloudRenderer::calcPointsNearCrossPlane()
{
    if (!crossPlane.isValid()) return;
    crossPoints3d.clear();
    crossPoints2d.clear();
    double threshold = 0.01;
    isCrossing.resize(pcUtils->pcData.size());
    for (int i = 0; i < pcUtils->pcData.size(); i++)
    {
        vec3d pos = pcUtils->pcData[i].pos;
        double d = std::abs(crossPlane.dist(pos));
        if (d < threshold)
        {
            crossPoints3d.push_back(pos);
            LocalFrame frame;
            frame.buildFromNormal(crossPlane.n);
            vec3d posLocal = frame.toLocal(pos - crossPlane.p);
            /*
            if (std::abs(posLocal.y) > threshold)
            {
                printf("proj error: %.6f\n" , posLocal.y);
            }
            */
            crossPoints2d.push_back(vec2d(posLocal.x , posLocal.z));
            isCrossing[i] = true;
        }
        else
        {
            isCrossing[i] = false;
        }
    }
}

vec3d PointCloudRenderer::mouseLocationTo3D(int mouseX , int mouseY)
{
    std::vector<vec3d> ray = getRay(mouseX , mouseY);
    vec3d dir = ray.back() - ray.front();
    dir.normalize();
    return sketchPlane.intersect(ray.front() , dir);
}

vec2d PointCloudRenderer::spaceLocationTo2D(vec3d pos , Plane& plane)
{
    vec3d local = plane.frame->toLocal(pos - plane.p);
    assert(std::abs(local.y) < 1e-6);
    return vec2d(local.x , local.z);
}

vec3d PointCloudRenderer::planeLocationTo3D(vec2d pos , Plane& plane)
{
    vec3d local(pos.x , 0 , pos.y);
    return plane.p + plane.frame->toWorld(local);
}

void PointCloudRenderer::cycleColorGenByRandom(std::vector<Cycle>& cycles , 
	std::vector<Colormap::color>& colors)
{
	/*
	CycleSet& cycleSet = m_cycleSetBreaked;
	int cycleSize = cycleSet.size();

	//get connection info
	int arcSize = m_curveNet.arcs.size();
	std::vector<std::vector<int> > cycleInArcs(arcSize);
	for(int i=0;i<cycleSize;i++){
		for(int j=0;j<cycleSet[i].size();j++){
			cycleInArcs[cycleSet[i][j].arcID].push_back(i);
		}
	}
	
	std::vector<std::vector<bool> > cycleConnected(cycleSize,std::vector<bool>(cycleSize,false));
	for(int i=0;i<arcSize;i++){
		for(int j=0;j<cycleInArcs[i].size()-1;j++){
			for(int k=j+1;k<cycleInArcs[i].size();k++){
				cycleConnected[cycleInArcs[i][j]][cycleInArcs[i][k]] = true;
				cycleConnected[cycleInArcs[i][k]][cycleInArcs[i][j]] = true;				
			}
		}
	}
	
	std::vector<int> colorMap(cycleSize);
	std::vector<int> colorInd;
	int newColor=0;
	colorInd.push_back(newColor);
	colorMap[0]=colorInd[0];
	for(int i=1;i<cycleSize;i++){
		std::vector<int> tcolors = colorInd;
		for(int j=0;j<i;j++){
			if(cycleConnected[i][j]){
				tcolors[colorMap[j]]=-1;
			}
		}
		if(*std::max_element(tcolors.begin(),tcolors.end())<0 || i<4){
			newColor++;
			colorInd.push_back(newColor);
			colorMap[i] = newColor;
		}
		else{
			int randInd=-1;
			while(randInd<0){
				randInd= tcolors[rand()%tcolors.size()];
			}
			colorMap[i] = tcolors[randInd];
		}
	}

	std::vector<Colormap::color> colors;
	Colormap::colormapBSC(colorInd.size(),colors);

	m_colorsCycleBreak.clear();
	for(int i=0;i<colorMap.size();i++){
		m_colorsCycleBreak.push_back(colors[colorMap[i]]);
	}
	*/
}

void PointCloudRenderer::cycleColorGenByRanking(std::vector<double>& cycleScores , 
	std::vector<Colormap::color>& colors)
{

}

void PointCloudRenderer::clearPaths()
{
	clearTemp();
    // curveNet->clear();
    dispCurveNet->clear();
}

void PointCloudRenderer::clearTemp()
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
    unsavedCycles.clear();
    unsavedCyclePoints.clear();
    unsavedCycleCenters.clear();
    unsavedMeshes.clear();
    unsavedNormals.clear();
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

