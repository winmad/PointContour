#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include "SketchGLCanvas.h"
#include <wx/dcclient.h>
#include <wx/Statusbr.h>
#include <wx/filename.h>
#include <wx/string.h>
#include "pointCloudUtils.h"
#include "pointCloudRenderer.h"
#include "LocalFrame.h"
#include "axisWidget.h"

BEGIN_EVENT_TABLE( SketchGLCanvas, wxGLCanvas )
	EVT_PAINT( SketchGLCanvas::OnPaint )
    EVT_PAINT( SketchGLCanvas::OnPaint )
	EVT_MOUSE_EVENTS( SketchGLCanvas::OnMouse )
	EVT_IDLE( SketchGLCanvas::OnIdle )
    EVT_SIZE( SketchGLCanvas::OnSize )
	EVT_KEY_DOWN( SketchGLCanvas::OnKeyDown )
END_EVENT_TABLE()

SketchGLCanvas::SketchGLCanvas(	wxWindow *parent, wxWindowID id,
		 const wxPoint& pos,
		 const wxSize& size, long style,
		 const wxString& name, int *attribList)
		 : wxGLCanvas( parent, id, pos, size, style | wxFULL_REPAINT_ON_RESIZE , name )
{
    m_initialized = false;
    startDraw=false;
    lastKeyBoard=-1;
    isDrag=false;
    isEditSpline=false;
    isChangeView=true;
    m_rotationTimes=1;
	m_rotationCount=0;

    //lastTime = glutGet(GLUT_ELAPSED_TIME);
	frames = 0;

    Initialize();
}

SketchGLCanvas::~SketchGLCanvas ()
{
}

void SketchGLCanvas::Initialize()
{
    SetCurrent();

    GetClientSize( &m_width, &m_height );
    //printf("w=%d, h=%d\n" , m_width , m_height);
    m_initialized = true;
    chosenBsp = chosenCtrlNode = -1;

    m_Eye[0] = 0.0f; m_Eye[1] = 0.0f; m_Eye[2] = -2.0f; //Actual code
    m_PerspectiveAngleDegrees = 45.0f;
    m_NearPlane = 0.01f;
    m_FarPlaneOffset = 100.0f;

    glViewport(0, 0, (GLint) m_width, (GLint) m_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(m_PerspectiveAngleDegrees,
        (GLfloat)m_width/(GLfloat)m_height,
        m_NearPlane,
        m_NearPlane + m_FarPlaneOffset);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //m_RotationMatrix.setIdentity(); //NO_MATRIX
    memset(m_RotationMatrix, 0.0, sizeof(double)*16);
    for (unsigned int i=0; i< 4; ++i){
        m_RotationMatrix[i+(4*i)] = 1.0;
    }

    calcCameraFrame();

    glShadeModel(GL_SMOOTH);							// Enable Smooth Shading

    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    glClearDepth(1.0f);									// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
    //glDepthFunc(GL_LESS);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);			// Really Nice Point Smoothing
    glEnable(GL_LINE_SMOOTH); //anti-aliased lines
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);  // Antialias the lines


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // **** rest of function is code from initGL function of Repousse standalone ****

    GLfloat light_position0[] = { -2.0f, -1.0f, 5.0f, 0.0f };
    GLfloat diffuse_light0[] = {0.7f,0.5f,0.5f,0.5f};
    GLfloat ambient_light0[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat specular_light0[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat As[4] = {0.4f, 0.4f, 0.4f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, As );

    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light0);


    GLfloat light_position1[] = { 2.0f, 1.0f, 0.5f, 0.0f };
    GLfloat diffuse_light1[] = {0.4f,0.2f,0.2f,1.0f};
    GLfloat specular_light1[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat ambient_light1[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_light1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular_light1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_light1);


    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    // enable color tracking
    bool isTwoSided = true;

    if (isTwoSided){
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    } else {
        glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    }
    glEnable(GL_COLOR_MATERIAL);


    // ****** Material properties ******
    // GLfloat mat_specular[] = { 0.1f, 0.1f, 0.1f, 0.5f};
    GLfloat mat_specular[] = { 0.2f, 0.2f, 0.15f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);

    GLfloat mat_shininess[] = { 2 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    //GLfloat mat_diffuseB[] = { 0.6f, 0.6f, 0.8f, 1.0f };
    // GLfloat mat_diffuseB[] = { 0.8f, 0.75f, 0.35f, 1.0f };
    GLfloat mat_diffuseB[] = { 0.8f , 0.55f , 0.f , 1.f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuseB);

    GLfloat mat_ambient_front[] = { 0.15f, 0.12f, .02f, 1.0f };
    //GLfloat mat_ambient_front[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT , mat_ambient_front);

    GLfloat mat_ambient_back[] = { 0.15f, 0.12f, .02f, 1.0f };//{ 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_BACK, GL_AMBIENT , mat_ambient_back);

    /*
      GLfloat mat_emit[] = { 0.1f, 0.1f, .1f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION , mat_emit);
    */


    if (isTwoSided){
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    } else {
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    }


    //glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glEnable(GL_LIGHTING);

    m_shotNum=0;

    m_isRotate = false;
    m_frameRate = 50;
    m_timeCurr = clock();
    m_timePrev = m_timeCurr-100;
    m_rotationCount=0;
    m_frames=0;

}

void SketchGLCanvas::rotateModel(){
	if (m_isRotate){
		m_rotationCount+=1;
		m_rotationCount%=362;
		float rot = 1;

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		if ( m_rotationCount < 361) {
			glRotated(rot, 0,1.0,0);// Y axis
		}
		else{
			if(m_rotationTimes==1)
				m_isRotate=false;
			else{
				m_rotationTimes--;m_rotationCount--;
			}
			m_rotationCount=0;
		}

		glMultMatrixd(m_RotationMatrix); //accummulate the previous transformations
		glGetDoublev(GL_MODELVIEW_MATRIX, m_RotationMatrix); //update
		glPopMatrix();
	}
}

void SketchGLCanvas::Render()
{
    SetCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	//------------------------------
	//	view transformation
	//-----------------------------
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (m_Eye[0], m_Eye[1], m_Eye[2]);	// I uncommented this

	//m_RotationMatrix.glMultMatrix();// NO_MATRIX
	glMultMatrixd(m_RotationMatrix);

	//glPushMatrix();
	//glColor3d(0.5, 0.5, 1.0);
	// Fatemeh: I commented this "glTranslate" out, since we "uncommented
	// the other glTranslate a couple lines above
	//glTranslatef(0,0,-0.15f);

	//matrix4 deviceRotation;
	//deviceRotation.setIdentity();
	////get the rotation matrix from Sixense

	m_FarPlaneOffset = 1000.0f;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(m_PerspectiveAngleDegrees,
		(GLfloat)m_width/(GLfloat)m_height,
		m_NearPlane,
		m_NearPlane + m_FarPlaneOffset);

	glMatrixMode(GL_MODELVIEW);

	m_pcUtils->pcRenderer->render();

	/*
	m_FarPlaneOffset = 200.0f;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-2, 2, -2, 2, m_NearPlane, m_NearPlane + m_FarPlaneOffset);

	gluPerspective(m_PerspectiveAngleDegrees,
		(GLfloat)m_width/(GLfloat)m_height,
		m_NearPlane,
		m_NearPlane + m_FarPlaneOffset);

	glMatrixMode(GL_MODELVIEW);
	//m_cycleUtils->renderCurves();
	//m_meshUtils->renderCurves();

	//glEnable(GL_LIGHTING);
	*/
	SwapBuffers();
}

void SketchGLCanvas::OnPaint ( wxPaintEvent &WXUNUSED( event ) )
{
	// must always be here
	wxPaintDC dc( this );
	if( ! m_initialized ){
		Initialize();
	}
	Render();
}

std::vector<vec3d> SketchGLCanvas::computeRay(int mouseX,int mouseY){
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

void SketchGLCanvas::pickMeshVertex(int mouseX, int mouseY,bool isStore)
{
	std::vector<vec3d> ray;
	ray=computeRay(mouseX,mouseY);
}

void SketchGLCanvas::calcCameraFrame()
{
    std::vector<vec3d> rays = computeRay(m_width / 2 , m_height / 2);
    std::vector<vec3d> rays_up = computeRay(m_width / 2 , m_height / 2 - 20);
    vec3d dir = rays.back() - rays.front();
    dir.normalize();
    vec3d up = rays_up.front() - rays.front();
    up.normalize();
    cameraFrame.n = dir;
    cameraFrame.t = up;
    cameraFrame.s = dir.cross(up);
    cameraFrame.s.normalize();

    cameraPos = rays.front();
}

void SketchGLCanvas::renderSelectionBuffer()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_LIGHTING);
	//glDisable(GL_TEXTURE_2D);
	glDisable(GL_COLOR_MATERIAL);
	glShadeModel(GL_FLAT);							// Enable Smooth Shading
	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_LINE_SMOOTH); //anti-aliased lines
	glDisable(GL_BLEND);

	m_pcUtils->pcRenderer->updateSelectionBuffer();

	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	//glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH); //anti-aliased lines
	glEnable(GL_BLEND);
}

void SketchGLCanvas::OnMouse ( wxMouseEvent &event )
{
	int x = event.GetX();
	int y = event.GetY();
    //printf("mouse pos = (%d,%d)\n" , x , y);

	SetFocus();
	if (event.Dragging() && !event.ControlDown() && !event.ShiftDown()&& !event.AltDown())
    {
		if (event.LeftIsDown())
		{
			//convert the mouse clicked locations to lie between [-1,1] for both X and Y
            /*
			double halfWidth = m_width/2.0;
			double halfHeight = m_height/2.0;
			double xNormalized = (x-halfWidth)/halfWidth;
			double yNormalized = (halfHeight-y)/halfHeight;
			double oldXNormalized = (m_lastx-halfWidth)/halfWidth;
			double oldYNormalized = (halfHeight-m_lasty)/halfHeight;
            */
			// rotates screen
			float rot[3]={0};
			rot[1] -= (m_lasty - y) * 0.5;
			rot[0] -= (m_lastx - x) * 0.5;
			//------------------------------------------------------------------------
			// If rotation angle is greater of 360 or lesser than -360,
			// resets it back to zero.
			//------------------------------------------------------------------------
			for (unsigned int i=0;i<3;i++)
				if (rot[i] > 360 || rot[i] < -360)
					rot[i] = 0;

			//transfer this rotation into the rotation matrix for the scene
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			//glRotated(-rot [0], 0,1.0,0);
			//glRotated(-rot [1], 1.0,0,0);
			glRotated(rot [0], 0,1.0,0);
			glRotated(rot [1], 1.0,0,0);
			glMultMatrixd(m_RotationMatrix); //accummulate the previous transformations
			glGetDoublev(GL_MODELVIEW_MATRIX, m_RotationMatrix); //update
			glPopMatrix();
		}
		else if (event.RightIsDown()) {
			// z axis gives the idea of zooming in and out
			if(event.Dragging())
				m_Eye[2] -= (m_lasty - y) * 0.05; // here I multiply by a 0.05 factor to slow down the zoom
			else{
				if(event.GetWheelRotation()>0)
					m_Eye[2] -= 0.05; // here I multiply by a 0.05 factor to slow down the zoom
				else
					m_Eye[2] += 0.05; // here I multiply by a 0.05 factor to slow down the zoom

			}
		}
		else if (event.MiddleIsDown()) {
            // change eye position
			m_Eye[0] -= (m_lastx - x) * 0.01f;
			m_Eye[1] += (m_lasty - y) * 0.01f;
		}
		Render();
		isChangeView=true;
        calcCameraFrame();
        // printf("eye: (%.6f,%.6f,%.6f)\n" , m_Eye[0] , m_Eye[1] , m_Eye[2]);
        // std::vector<vec3d> rays = computeRay(m_width / 2 , m_height / 2);
        // vec3d dir = rays.back() - rays.front();
        // dir.normalize();
        // m_pcUtils->pcRenderer->sketchPlane.init(rays.front() + dir * 0.5 , -dir , 1);
        pcRenderer->sketchPlane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);
        pcRenderer->sketchPlane.buildFrame();
        m_pcUtils->pcRenderer->freeSketchLines.clear();
	}

	if (event.ShiftDown()) // cycle operation
	{
		if (event.ControlDown())
			m_pcUtils->pcRenderer->isCtrlPress = true;
		m_pcUtils->pcRenderer->pickedCurve = -1;
		m_pcUtils->pcRenderer->pickedSavedCycle = -1;
		lastKeyBoard = 3;
		m_pcUtils->pcRenderer->isShiftPress = true;

		m_pcUtils->pcRenderer->pickCycle(x , y , 0);
		if (event.LeftIsDown())
		{
			if (!event.ControlDown())
			{
                // save a candidate cycle
				m_pcUtils->pcRenderer->backup();
				m_pcUtils->pcRenderer->pickCycle(x , y , 1);
			}
			else
			{
                // add a candidate cycle to cycle group
				m_pcUtils->pcRenderer->pickCycle(x , y , 3);
			}
			m_pcUtils->pcRenderer->pickedCycle = -1;
		}
		if (event.RightIsDown())
		{
			if (event.ControlDown())
			{
                // remove a candidate cycle from cycle group
				m_pcUtils->pcRenderer->pickCycle(x , y , 4);
			}
			m_pcUtils->pcRenderer->pickedCycle = -1;
		}
		if (event.MiddleIsDown())
		{
            // confirm a cycle group
			m_pcUtils->pcRenderer->backup();
			m_pcUtils->pcRenderer->cycleGroupUpdate();
		}
		Render();
	}
    else if (event.ControlDown()) // curve operation
    {
        if (event.AltDown())
            m_pcUtils->pcRenderer->isAltPress = true;
        m_pcUtils->pcRenderer->pickedCurve = -1;
        m_pcUtils->pcRenderer->pickedCycle = -1;
        m_pcUtils->pcRenderer->pickedSavedCycle = -1;
        m_pcUtils->pcRenderer->pickedAutoCurve = -1;
        lastKeyBoard=1;
		m_pcUtils->pcRenderer->isCtrlPress = true;

		if(isChangeView){
			renderSelectionBuffer();
			isChangeView=false;
		}
        if (event.AltDown())
        {
            m_pcUtils->pcRenderer->pathVertex.clear();
            m_pcUtils->pcRenderer->bsp.clear();
			setNull(m_pcUtils->pcRenderer->lastDispPoint);
            setNull(m_pcUtils->pcRenderer->pickedDispPoint);
            m_pcUtils->pcRenderer->pickAutoCurve(x , y , 0);
        }
        else if (m_pcUtils->pcRenderer->drawMode == 0 ||
            m_pcUtils->pcRenderer->drawMode == 1)
        {
            m_pcUtils->pcRenderer->pickPoint(x , y , 0);
        }
        else if (m_pcUtils->pcRenderer->drawMode == 3)
        {
            m_pcUtils->pcRenderer->pathVertex.clear();
            m_pcUtils->pcRenderer->bsp.clear();
            setNull(m_pcUtils->pcRenderer->lastDispPoint);
            setNull(m_pcUtils->pcRenderer->pickedDispPoint);
        }
        
		if (event.LeftIsDown())
		{
            if (event.AltDown())
            {
                m_pcUtils->pcRenderer->backup();
                m_pcUtils->pcRenderer->pickAutoCurve(x , y , 1);
            }
            else if (m_pcUtils->pcRenderer->drawMode == 0 ||
                m_pcUtils->pcRenderer->drawMode == 1)
            {
                m_pcUtils->pcRenderer->backup();
                m_pcUtils->pcRenderer->pickPoint(x , y , 1);
            }
            else if (m_pcUtils->pcRenderer->drawMode == 3 && !(m_selectx==x&&m_selecty==y))
            {
                if(startDraw){
                    double slope = double(y-m_selecty);
                    if(x==m_selectx) slope=1e20;
                    else slope/=double(x-m_selectx);
                    double absSlope= slope>=0?slope:-slope;
                    int interX,interY;
                    interX=m_selectx; interY=m_selecty;
                    do{
                        if(absSlope<=1){
                            if(x>=m_selectx)
							interX++;
                            else
							interX--;
                            interY = y - int(slope * double(x-interX)+.5);
                        }
                        else{
                            if(y>=m_selecty)
							interY++;
                            else
							interY--;
                            interX = x - int(double(y-interY)/slope+.5);
                        }
                        sequence.push_back(std::pair<unsigned,unsigned>(interX,interY));
                    } while (!(interX==x&&interY==y));
                }
                else{
                    // printf("start draw\n");
                    startDraw=true;
                    sequence.clear();
                    sequence.push_back(std::pair<unsigned,unsigned>(x,y));
                }
                m_pcUtils->pcRenderer->freeSketchOnPointCloud(sequence);

                /*
                printf("--------------\n");
                for (int i = 0; i < m_pcUtils->pcRenderer->sketchLine.size(); i++)
                {
                    vec3d& p = m_pcUtils->pcRenderer->sketchLine[i];
                    printf("%.6f %.6f %.6f\n" , p.x , p.y , p.z);
                }
                */
                m_selectx = x;
                m_selecty = y;
            }
		}
        else if (event.LeftUp())
        {
            // printf("left is up\n");
            startDraw=false;
            sequence.clear();
            /*
            printf("===========\n");
            for (int i = 0; i < m_pcUtils->pcRenderer->sketchLine.size(); i++)
            {
                vec3d& p = m_pcUtils->pcRenderer->sketchLine[i];
                printf("%.6f %.6f %.6f\n" , p.x , p.y , p.z);
            }
            */
            m_pcUtils->pcRenderer->freeSketchLines.push_back(m_pcUtils->pcRenderer->sketchLine);
            m_pcUtils->pcRenderer->sketchLine.clear();
        }
        if (event.MiddleIsDown())
        {
            if (m_pcUtils->pcRenderer->drawMode == 3)
            {
                if (!pcRenderer->crossPlane.isValid())
                {
                    m_pcUtils->pcRenderer->autoGenByICP();
                }
                else
                {
                    pcRenderer->autoGenByPclNurbsFitting();
                }
                m_pcUtils->pcRenderer->initFreeSketchMode();
                m_pcUtils->pcRenderer->sketchLine.clear();
                m_pcUtils->pcRenderer->freeSketchLines.clear();
            }
        }
		if (event.RightIsDown())
		{
            if (m_pcUtils->pcRenderer->drawMode == 0 ||
                m_pcUtils->pcRenderer->drawMode == 1)
            {
                // cancel path extension
                m_pcUtils->pcRenderer->pathVertex.clear();
                m_pcUtils->pcRenderer->bsp.clear();
                setNull(m_pcUtils->pcRenderer->lastDispPoint);
            }
            else if (m_pcUtils->pcRenderer->drawMode == 3)
            {
                m_pcUtils->pcRenderer->initFreeSketchMode();
                m_pcUtils->pcRenderer->sketchLine.clear();
                m_pcUtils->pcRenderer->freeSketchLines.clear();
            }
		}

        Render();
	}
    else if (event.AltDown()) // edit operation
    {
        m_pcUtils->pcRenderer->pickedCycle = -1;
        lastKeyBoard = 2;
        m_pcUtils->pcRenderer->isAltPress = true;
        pcRenderer->pathVertex.clear();
        pcRenderer->bsp.clear();
        setNull(pcRenderer->lastDispPoint);
        setNull(pcRenderer->pickedDispPoint);

        if(isChangeView){
			renderSelectionBuffer();
			isChangeView=false;
		}

		bool curvePicked = m_pcUtils->pcRenderer->pickCurve(x , y , 0);
        bool ctrlNodePicked = m_pcUtils->pcRenderer->pickCtrlNode(x , y , m_lastx , m_lasty , 0);
        if (!pcRenderer->isShowCtrlNodes) ctrlNodePicked = false;

        bool crossPlanePicked = pcRenderer->crossPlane.isValid();
        if (!curvePicked)
        {
            m_pcUtils->pcRenderer->pickSavedCycle(x , y , 0);
            m_pcUtils->pcRenderer->pickPoint(x , y , 0);
        }

        // printf("curve picked = %d, ctrlNode picked = %d, crossPlane picked = %d\n" ,
            // curvePicked , ctrlNodePicked , crossPlanePicked);
        
        if (pcRenderer->copyMode == 1)
        {
            int nodeIndex = -1;
            if (!isDrag)
            {
                pcRenderer->chosenAxis = pcRenderer->curveSelectionByRay(x , y , nodeIndex ,
                    pcRenderer->axisWidget.axesPoints);
            }
            // printf("chosen axis = %d\n" , pcRenderer->chosenAxis);
            if (event.Dragging() && pcRenderer->chosenAxis != -1)
            {
                if (!isDrag)
                {
                    isDrag = true;
                    chosenAxis = pcRenderer->chosenAxis;

                    printf("start translation, fixed axis = %d\n" , chosenAxis);
                }

                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin ,
                    pcRenderer->axisWidget.axes[chosenAxis] , 1);
                double delta;
                Plane plane;
                plane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);

                std::vector<vec3d> ray = computeRay(x , y);
                vec3d dir = ray.back() - ray.front();
                dir.normalize();
                // vec3d pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                vec3d pos = plane.intersect(ray.front() , dir);
                ray = computeRay(m_lastx , m_lasty);
                dir = ray.back() - ray.front();
                dir.normalize();
                vec3d last_pos = plane.intersect(ray.front() , dir);

                delta = pcRenderer->axisPlane.n.dot(pos - last_pos) * 10;
                // delta = pcRenderer->axisWidget.axes[chosenAxis].dot(pos - pcRenderer->axisWidget.origin);
                pcRenderer->autoGenByTranslation(delta);
            }
            else if (event.MiddleIsDown())
            {
                pcRenderer->pickAllAutoCurves(0);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }
            else if (event.RightIsDown())
            {
                pcRenderer->pickAllAutoCurves(1);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }

            if (!event.LeftIsDown() && isDrag)
            {
                chosenAxis = -1;
                isDrag = false;

                printf("finish translation!\n");
            }
        }
        else if (pcRenderer->copyMode == 2)
        {
            int nodeIndex = -1;
            if (!isDrag)
            {
                pcRenderer->chosenAxis = pcRenderer->curveSelectionByRay(x , y , nodeIndex ,
                    pcRenderer->axisWidget.axesPoints);
            }
            // printf("chosen axis = %d\n" , pcRenderer->chosenAxis);
            if (event.Dragging() && pcRenderer->chosenAxis != -1)
            {
                if (!isDrag)
                {
                    isDrag = true;
                    chosenAxis = pcRenderer->chosenAxis;

                    printf("start scaling, fixed axis = %d\n" , chosenAxis);
                }

                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin ,
                    pcRenderer->axisWidget.axes[chosenAxis] , 1);
                double delta;
                Plane plane;
                plane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);

                std::vector<vec3d> ray = computeRay(x , y);
                vec3d dir = ray.back() - ray.front();
                dir.normalize();
                // vec3d pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                vec3d pos = plane.intersect(ray.front() , dir);
                ray = computeRay(m_lastx , m_lasty);
                dir = ray.back() - ray.front();
                dir.normalize();
                vec3d last_pos = plane.intersect(ray.front() , dir);

                delta = 1 - pcRenderer->axisPlane.n.dot(pos - last_pos) * 20;
                // printf("scaling delta = %.6f\n" , delta);
                // delta = pcRenderer->axisWidget.axes[chosenAxis].dot(pos - pcRenderer->axisWidget.origin);
                pcRenderer->autoGenByScaling(delta);
            }
            else if (event.MiddleIsDown())
            {
                pcRenderer->pickAllAutoCurves(0);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }
            else if (event.RightIsDown())
            {
                pcRenderer->pickAllAutoCurves(1);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }

            if (!event.LeftIsDown() && isDrag)
            {
                chosenAxis = -1;
                isDrag = false;

                printf("finish scaling!\n");
            }
        }
        else if (pcRenderer->copyMode == 3)
        {
            int nodeIndex = -1;
            if (!isDrag)
            {
                pcRenderer->chosenAxis = pcRenderer->curveSelectionByRay(x , y , nodeIndex ,
                    pcRenderer->axisWidget.globePoints);
            }
            // printf("chosen axis = %d\n" , pcRenderer->chosenAxis);
            if (event.Dragging() && pcRenderer->chosenAxis != -1)
            {
                if (!isDrag)
                {
                    isDrag = true;
                    chosenAxis = pcRenderer->chosenAxis;

                    printf("start rotation, fixed axis = %d\n" , chosenAxis);
                }

                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin ,
                    pcRenderer->axisWidget.axes[chosenAxis] , 1);
                double delta;
                Plane plane;
                plane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);

                std::vector<vec3d> ray = computeRay(x , y);
                vec3d dir = ray.back() - ray.front();
                dir.normalize();
                vec3d pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                // vec3d pos = plane.intersect(ray.front() , dir);
                ray = computeRay(m_lastx , m_lasty);
                dir = ray.back() - ray.front();
                dir.normalize();
                vec3d last_pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                // vec3d last_pos = plane.intersect(ray.front() , dir);

                vec3d x = pos - pcRenderer->axisWidget.origin;
                vec3d y = last_pos - pcRenderer->axisWidget.origin;
                dir = x.cross(y);
                if (dir.dot(pcRenderer->axisPlane.n) < 0)
                {
                    delta = 0.05;
                }
                else
                {
                    delta = -0.05;
                }
                /*
                dir = pcRenderer->axisWidget.origin - cameraPos;
                dir.normalize();
                vec3d origin_proj = plane.intersect(cameraPos , dir);
                */
                //delta = 1 - pcRenderer->axisPlane.n.dot(pos - last_pos) * 20;
                // printf("rotation delta = %.6f\n" , delta);
                // delta = pcRenderer->axisWidget.axes[chosenAxis].dot(pos - pcRenderer->axisWidget.origin);
                pcRenderer->autoGenByRotation(delta);
            }
            else if (event.MiddleIsDown())
            {
                pcRenderer->pickAllAutoCurves(0);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }
            else if (event.RightIsDown())
            {
                pcRenderer->pickAllAutoCurves(1);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }

            if (!event.LeftIsDown() && isDrag)
            {
                chosenAxis = -1;
                isDrag = false;

                printf("finish rotation!\n");
            }
        }
        else if (pcRenderer->copyMode == 4)
        {
            m_pcUtils->pcRenderer->pickedBsp = chosenBsp;
            m_pcUtils->pcRenderer->pickedCtrlNode = chosenCtrlNode;

            int nodeIndex = -1;
            if (!isDrag)
            {
                pcRenderer->chosenAxis = pcRenderer->curveSelectionByRay(x , y ,
                    nodeIndex , pcRenderer->axisWidget.axesPoints);
            }
            // printf("chosen axis = %d\n" , pcRenderer->chosenAxis);
            if (event.Dragging() && pcRenderer->chosenAxis != -1)
            {
                if (!isDrag)
                {
                    isDrag = true;
                    chosenAxis = pcRenderer->chosenAxis;

                    printf("start point translation, fixed axis = %d\n" , chosenAxis);
                }

                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin ,
                    pcRenderer->axisWidget.axes[chosenAxis] , 1);
                double delta;
                Plane plane;
                plane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);

                std::vector<vec3d> ray = computeRay(x , y);
                vec3d dir = ray.back() - ray.front();
                dir.normalize();
                // vec3d pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                vec3d pos = plane.intersect(ray.front() , dir);
                ray = computeRay(m_lastx , m_lasty);
                dir = ray.back() - ray.front();
                dir.normalize();
                vec3d last_pos = plane.intersect(ray.front() , dir);

                delta = pcRenderer->axisPlane.n.dot(pos - last_pos) * 8;
                // delta = pcRenderer->axisWidget.axes[chosenAxis].dot(pos - pcRenderer->axisWidget.origin);
                pcRenderer->autoGenByTranslation(delta);
                vec3d newPos = pcRenderer->axisPlane.p + pcRenderer->axisPlane.n * delta;
                pcRenderer->axisWidget.origin += pcRenderer->axisPlane.n * delta;
                pcRenderer->axisWidget.resamplePoints();
                pcRenderer->dispCurveNet->updatePath(pcRenderer->pickedBsp ,
                    pcRenderer->pickedCtrlNode , newPos , false);
            }

            if (!event.LeftIsDown() && isDrag)
            {
                chosenAxis = -1;
                isDrag = false;

                printf("finish point translation!\n");
                // setNull(m_pcUtils->pcRenderer->dragStartPoint);
                // setNull(m_pcUtils->pcRenderer->dragPlane.p);
                // pcRenderer->dragPlane.setNull();
                // isEditSpline = false;
                pcRenderer->axisWidget.origin = pcRenderer->dispCurveNet->bsplines[chosenBsp].ctrlNodes[chosenCtrlNode];
                pcRenderer->axisWidget.resamplePoints();
            }

            if (event.RightIsDown())
            {
                if (isEditSpline && chosenBsp != -1 && m_pcUtils->pcRenderer->isAutoOpt)
                {
                    m_pcUtils->pcRenderer->dispCurveNet->updateConstraints(chosenBsp);
                    m_pcUtils->pcRenderer->optUpdate(false);
                }
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                chosenBsp = chosenCtrlNode = -1;
                isEditSpline = false;
            }
        }
        else if (pcRenderer->copyMode == 5)
        {
            int nodeIndex = -1;
            if (!isDrag)
            {
                pcRenderer->chosenAxis = pcRenderer->curveSelectionByRay(x , y , nodeIndex ,
                    pcRenderer->axisWidget.axesPoints);
            }
            // printf("chosen axis = %d\n" , pcRenderer->chosenAxis);
            if (event.Dragging() && pcRenderer->chosenAxis != -1)
            {
                if (!isDrag)
                {
                    isDrag = true;
                    chosenAxis = pcRenderer->chosenAxis;

                    printf("start moving axis widget, fixed axis = %d\n" , chosenAxis);
                }

                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin ,
                    pcRenderer->axisWidget.axes[chosenAxis] , 1);
                double delta;
                Plane plane;
                plane.init(cameraPos + cameraFrame.n * 0.5 , -cameraFrame.n , 1);

                std::vector<vec3d> ray = computeRay(x , y);
                vec3d dir = ray.back() - ray.front();
                dir.normalize();
                // vec3d pos = pcRenderer->axisPlane.intersect(ray.front() , dir);
                vec3d pos = plane.intersect(ray.front() , dir);
                ray = computeRay(m_lastx , m_lasty);
                dir = ray.back() - ray.front();
                dir.normalize();
                vec3d last_pos = plane.intersect(ray.front() , dir);

                delta = pcRenderer->axisPlane.n.dot(pos - last_pos) * 8;
                // delta = pcRenderer->axisWidget.axes[chosenAxis].dot(pos - pcRenderer->axisWidget.origin);
                pcRenderer->axisWidget.origin += pcRenderer->axisPlane.n * delta;
                pcRenderer->axisWidget.resamplePoints();
            }

            if (event.RightIsDown() && pcRenderer->pickedBsp != -1 &&
                pcRenderer->pickedCtrlNode != -1 &&
                (pcRenderer->pickedCtrlNode == 0 ||
                    pcRenderer->pickedCtrlNode == pcRenderer->dispCurveNet->bsplines[pcRenderer->pickedBsp].ctrlNodes.size() - 1))
            {
                printf("%d %d\n" , pcRenderer->pickedBsp , pcRenderer->pickedCtrlNode);
                pcRenderer->axisWidget.origin = pcRenderer->dispCurveNet->bsplines[pcRenderer->pickedBsp].ctrlNodes[pcRenderer->pickedCtrlNode];
                pcRenderer->axisWidget.resamplePoints();
            }

            if (!event.LeftIsDown() && isDrag)
            {
                chosenAxis = -1;
                isDrag = false;
            }
        }
        else if (pcRenderer->copyMode == 6)
        {
            if (event.LeftIsDown())
            {
                vec3d newPos = pcRenderer->pickedDispPoint;
                setNull(pcRenderer->pickedDispPoint);
                vec3d dir = newPos - pcRenderer->axisWidget.origin;
                double delta = dir.length();
                dir /= delta;
                pcRenderer->axisPlane.init(pcRenderer->axisWidget.origin , dir , 1);
                pcRenderer->autoGenByTranslation(delta);
            }
            else if (event.RightIsDown())
            {
                pcRenderer->pickAllAutoCurves(1);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }
            else if (event.MiddleIsDown())
            {
                pcRenderer->pickAllAutoCurves(0);
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
            }
        }
        else if (pcRenderer->copyMode == 7)
        {
            if (event.LeftIsDown())
            {
                if (event.Dragging())
                {
                    if (!isDrag)
                    {
                        isDrag = true;
                        pcRenderer->lt_x = x;
                        pcRenderer->lt_y = y;
                        // printf("start drawing rectangle, lt = (%d,%d)\n" ,
                            // pcRenderer->lt_x , pcRenderer->lt_y);
                    }
                    else
                    {
                        pcRenderer->rb_x = x;
                        pcRenderer->rb_y = y;
                        // printf("dragging, rb = (%d,%d)\n" ,
                            // pcRenderer->rb_x , pcRenderer->rb_y);
                        pcRenderer->pickCurve(x , y , 5);
                    }
                }
                else
                {
                    if (!isDrag) pcRenderer->pickCurve(x , y , 4);
                }
            }
            else if (event.MiddleIsDown())
            {
                std::fill(pcRenderer->dispCurveNet->isNodesFixed.begin() , pcRenderer->dispCurveNet->isNodesFixed.end() , false);
                std::fill(pcRenderer->dispCurveNet->isCurvesFixed.begin() , pcRenderer->dispCurveNet->isCurvesFixed.end() , false);
            }

            if (!event.LeftIsDown() && isDrag)
            {
                // printf("not dragging...\n");
                isDrag = false;
                pcRenderer->lt_x = pcRenderer->lt_y = -1;
                for (int i = 0; i < pcRenderer->dispCurveNet->numPolyLines; i++)
                {
                    if (pcRenderer->isCurvesChosen[i])
                    {
                        pcRenderer->dispCurveNet->isCurvesFixed[i] = !pcRenderer->dispCurveNet->isCurvesFixed[i];
                        pcRenderer->isCurvesChosen[i] = false;
                    }
                }
            }
        }
        else if (crossPlanePicked) // cross plane operation, ignore all other editing operations
        {
            if (event.Dragging())
            {
                // rotates screen
                float rot[3]={0};
                rot[1] -= (m_lasty - y) * 0.5;
                rot[0] -= (m_lastx - x) * 0.5;
                //------------------------------------------------------------------------
                // If rotation angle is greater of 360 or lesser than -360,
                // resets it back to zero.
                //------------------------------------------------------------------------
                for (unsigned int i=0;i<3;i++)
                    if (rot[i] > 360 || rot[i] < -360)
                        rot[i] = 0;

                Plane& crossPlane = pcRenderer->crossPlane;
                Eigen::Matrix3d rotateMat;
                rotateMat = Eigen::AngleAxisd(rot[0] * PI / 180.0 ,
                    Eigen::Vector3d(cameraFrame.t.x , cameraFrame.t.y , cameraFrame.t.z)) *
                    Eigen::AngleAxisd(rot[1] * PI / 180.0 ,
                    Eigen::Vector3d(cameraFrame.s.x , cameraFrame.s.y , cameraFrame.s.z));
                Eigen::Vector3d norm(crossPlane.n.x , crossPlane.n.y , crossPlane.n.z);
                norm = rotateMat * norm;
                crossPlane.n = vec3d(norm(0) , norm(1) , norm(2));
                crossPlane.normalize();

                pcRenderer->calcPointsNearCrossPlane();
            }
            else if (event.RightIsDown())
            {
                pcRenderer->crossPlane.setNull();
            }
        }
        else if (pcRenderer->copyMode == 0)
        {
            if (event.LeftIsDown()) // update
            {
                if (ctrlNodePicked && !crossPlanePicked)
                {
                    if (!isEditSpline)
                    {
                        isEditSpline = true;
                        printf("edit spline ready\n");
                        chosenBsp = pcRenderer->pickedBsp;
                        chosenCtrlNode = pcRenderer->pickedCtrlNode;
                        pcRenderer->axisWidget.origin = pcRenderer->dispCurveNet->bsplines[chosenBsp].ctrlNodes[chosenCtrlNode];
                        pcRenderer->axisWidget.resamplePoints();
                        pcRenderer->isShowAxisWidget = true;
                        pcRenderer->chosenAxis = -1;
                        isDrag = false;
                        pcRenderer->copyMode = 4;
                        m_pcUtils->pcRenderer->backup();
                    }
                }
                else if (curvePicked)
                {
                    pcRenderer->pickCurve(x , y , 3);
                }
                else if (!crossPlanePicked && !isEditSpline)
                {
                    Plane& plane = pcRenderer->crossPlane;
                    std::vector<vec3d> rays = computeRay(x , y);
                    plane.p = m_pcUtils->pcRenderer->pickedDispPoint;
                    plane.n = rays.back() - rays.front();
                    plane.n.normalize();
                    plane.d = -plane.p.dot(plane.n);
                    setNull(pcRenderer->pickedDispPoint);

                    pcRenderer->calcPointsNearCrossPlane();
                }
                // m_pcUtils->pcRenderer->pickCtrlNode(x , y , m_lastx , m_lasty , 1);
            }
            else if (event.RightIsDown()) // delete
            {
                m_pcUtils->pcRenderer->backup();
                if (curvePicked)
                {
                    // delete curve
                    m_pcUtils->pcRenderer->pickCurve(x , y , 2);
                }
                else
                {
                    // delete cycle and surface
                    m_pcUtils->pcRenderer->pickSavedCycle(x , y , 2);
                }
            }
        }
		Render();
    }

    // Release shift/ctrl/alt button
	if (!event.ShiftDown())
	{
		m_pcUtils->pcRenderer->isShiftPress = false;
		if (!event.ControlDown())
		{
			for (int i = 0; i < m_pcUtils->pcRenderer->inGroup.size(); i++)
				m_pcUtils->pcRenderer->inGroup[i] = false;
			m_pcUtils->pcRenderer->group.clear();
		}
	}
	if (!event.ControlDown())
	{
		m_pcUtils->pcRenderer->isCtrlPress = false;
        if (m_pcUtils->pcRenderer->drawMode == 3)
        {
            startDraw = false;
            sequence.clear();
        }
		if (!event.ShiftDown())
		{
			for (int i = 0; i < m_pcUtils->pcRenderer->inGroup.size(); i++)
				m_pcUtils->pcRenderer->inGroup[i] = false;
			m_pcUtils->pcRenderer->group.clear();
		}
	}
    if (!event.AltDown())
    {
        m_pcUtils->pcRenderer->isAltPress = false;
        m_pcUtils->pcRenderer->pickedBsp = -1;
        m_pcUtils->pcRenderer->pickedCtrlNode = -1;
    }

	m_lastx = x;
	m_lasty = y;
}

double SketchGLCanvas::fpsCounter()
{
	/*
	int currentTime = glutGet(GLUT_ELAPSED_TIME);
	double elapsedTime = currentTime - lastTime;
	double fps;
	if (elapsedTime > 1000)
	{
		fps = (double)frames * 1000.f / elapsedTime;
		frames = 0;
		lastTime = currentTime;
	}
	frames++;;
	return fps;
	*/
	return 0;
}

void SketchGLCanvas::OnIdle ( wxIdleEvent &event )
{
//	Refresh();
/*
	if(m_meshUtils->m_filename.size()!=0 && !vts.empty()){
		pickMeshVertex(vts.back().first,vts.back().second,false); vts.clear();
		Render();
	}
*/
	if(m_isRotate){
		rotateModel();
		Refresh(false);
		//		wxWakeUpIdle();
		//	Sleep(10);
		//		SetTimer(1,40,NULL);

	}
	/*
	double fps = fpsCounter();
	if (fps > 1)
	{
		char tmp[64];
		sprintf(tmp, "fps: %.2f", fps);
		wxString str(tmp);
		m_pcUtils->statusBar->SetStatusText(str);
	}
	*/
}

void SketchGLCanvas::OnSize ( wxSizeEvent &event )
{
	if( ! m_initialized ){
		Initialize();
		m_initialized = true;
	}

	GetClientSize(&m_width, &m_height);

	glViewport(0, 0, (GLint) m_width, (GLint) m_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(m_PerspectiveAngleDegrees,
				   (GLfloat)m_width/(GLfloat)m_height,
				   m_NearPlane,
				   m_NearPlane + m_FarPlaneOffset);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glMultMatrixd(m_RotationMatrix);

	wxGLCanvas::OnSize( event );
}

void SketchGLCanvas::OnKeyDown(wxKeyEvent &event)
{
    wxChar uc = event.GetUnicodeKey();
	if (uc != WXK_NONE && uc >= 32)
	{
		if (uc == 127) // backspace
		{
			m_pcUtils->pcRenderer->clearPaths();
		}
        else if (uc == 'A') // moving axis widget
        {
            if (pcRenderer->copyMode != 5)
            {
                pcRenderer->copyMode = 5;
                pcRenderer->isShowAxisWidget = true;
                wxString str("moving axis widget\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'B') // show/hide debug points
        {
            m_pcUtils->pcRenderer->isShowFeaturePoints = !m_pcUtils->pcRenderer->isShowFeaturePoints;
            m_pcUtils->pcRenderer->isShowDebugPoints = !m_pcUtils->pcRenderer->isShowDebugPoints;
        }
        else if (uc == 'C') // cycle discovery
        {
            m_pcUtils->pcRenderer->cycleDisc();
            m_pcUtils->pcRenderer->surfacingUnsavedCycles();
            m_pcUtils->pcRenderer->evalUnsavedCycles();
        }
        else if (uc == 'D') // cancel chosen curves
        {
            m_pcUtils->pcRenderer->clearAutoGen();
            for (int i = 0; i < pcRenderer->isCurvesChosen.size(); i++)
            {
                pcRenderer->isCurvesChosen[i] = false;
            }
            pcRenderer->copyMode = 0;
            pcRenderer->isShowAxisWidget = false;
            wxString str("");
            m_pcUtils->statusBar->SetStatusText(str);
        }
        else if (uc == 'E') // easy translation mode
        {
            if (pcRenderer->copyMode != 6)
            {
                if (pcRenderer->copyMode == 0) pcRenderer->initTranslationMode(true);
                else if (pcRenderer->copyMode == 5) pcRenderer->initTranslationMode(false);

                pcRenderer->copyMode = 6;
                pcRenderer->isShowAxisWidget = true;
                wxString str("Easy translation mode\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'F') // fix part of curves
        {
            if (pcRenderer->copyMode != 7)
            {
                pcRenderer->copyMode = 7;
                pcRenderer->isShowFixCurves = true;
                wxString str("Choosing fixed curves & nodes\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowFixCurves = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'G') // generate by symmetry
        {
            m_pcUtils->pcRenderer->autoGenBySymmetry();
        }
        else if (uc == 'H') // show/hide coplanes
        {
            m_pcUtils->pcRenderer->isShowCoplanes = !m_pcUtils->pcRenderer->isShowCoplanes;
        }
        else if (uc == 'I')
        {
        }
        else if (uc == 'J')
        {
            m_pcUtils->pcRenderer->incBspCurveIndex();
        }
        else if (uc == 'K')
        {
            m_pcUtils->pcRenderer->decBspCurveIndex();
        }
        else if (uc == 'L')
        {
        }
        else if (uc == 'M')
        {
        }
        else if (uc == 'N')
        {
        }
        else if (uc == 'O') 
        {
        }
        else if (uc == 'P') // print debug info
        {
            //m_pcUtils->curveNet->conSet->collinearSet.printLog();
            //m_pcUtils->curveNet->conSet->orthoSet.printLog();
			//m_pcUtils->curveNet->debugLog();
            writeLog("CurveNet polylines\n");
            m_pcUtils->curveNet->outputPolyLines();
            /*
            writeLog("\nCrossing plane points\n");
            writeLog("%lu\n" , pcRenderer->crossPoints2d.size());
            for (int i = 0; i < pcRenderer->crossPoints2d.size(); i++)
            {
                vec2d& p = pcRenderer->crossPoints2d[i];
                writeLog("%.6f %.6f\n" , p.x , p.y);
            }
            */
            LocalFrame frame;
            frame.buildFromNormal(pcRenderer->crossPlane.n);
            writeLog("\nFree sketch points\n");
            writeLog("%lu\n" , m_pcUtils->pcRenderer->chosenPoints.size());
            for (int i = 0; i < m_pcUtils->pcRenderer->chosenPoints.size(); i++)
            {
                vec3d& p = m_pcUtils->pcRenderer->chosenPoints[i];
                vec3d posLocal = frame.toLocal(p - pcRenderer->crossPlane.p);
                //writeLog("%.6f %.6f %.6f\n" , p.x , p.y , p.z);
                writeLog("%.6f %.6f 0.0\n" , posLocal.x , posLocal.z);
            }
            /*
			for (int i = 0; i < m_pcUtils->curveNet->coplanes.size(); i++)
			{
				Plane& plane = m_pcUtils->curveNet->coplanes[i];
				printf("plane %d = (%.6f,%.6f,%.6f), n = (%.6f,%.6f,%.6f)\n" , i ,
					plane.p.x , plane.p.y , plane.p.z ,
					plane.n.x , plane.n.y , plane.n.z);
			}
            */
        }
        else if (uc == 'Q')
        {
            std::vector<int> curveIndices;
            m_pcUtils->opt.init(pcRenderer->dispCurveNet , m_pcUtils);
            if (pcRenderer->markMode == 1)
            {
                for (int i = 0; i < pcRenderer->dispCurveNet->numPolyLines; i++)
                {
                    if (pcRenderer->isCurvesChosen[i] &&
                        pcRenderer->dispCurveNet->curveType[i] == 1)
                    {
                        curveIndices.push_back(i);
                    }
                }
                for (int i = 0; i < curveIndices.size(); i++)
                {
                    printf("%d " , curveIndices[i]);
                }
                printf("\n");
                for (int i = 1; i < curveIndices.size(); i++)
                {
                    m_pcUtils->opt.addParallelConstraint(curveIndices[0] , curveIndices[i] ,
                        314);
                }
            }
            m_pcUtils->opt.run(pcRenderer->dispCurveNet);
        }
        else if (uc == 'R') // rotation mode
        {
            if (pcRenderer->copyMode != 3)
            {
                if (pcRenderer->copyMode == 0) pcRenderer->initTranslationMode(true);
                else if (pcRenderer->copyMode == 5) pcRenderer->initTranslationMode(false);

                pcRenderer->copyMode = 3;
                pcRenderer->isShowAxisWidget = true;
                wxString str("copy by rotation\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'S') // scaling mode
        {
            if (pcRenderer->copyMode != 2)
            {
                if (pcRenderer->copyMode == 0) pcRenderer->initTranslationMode(true);
                else if (pcRenderer->copyMode == 5) pcRenderer->initTranslationMode(false);

                pcRenderer->copyMode = 2;
                pcRenderer->isShowAxisWidget = true;
                wxString str("copy by scaling\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'T') // translation mode
        {
            if (pcRenderer->copyMode != 1)
            {
                if (pcRenderer->copyMode == 0) pcRenderer->initTranslationMode(true);
                else if (pcRenderer->copyMode == 5) pcRenderer->initTranslationMode(false);

                pcRenderer->copyMode = 1;
                pcRenderer->isShowAxisWidget = true;
                wxString str("copy by translation\n");
                m_pcUtils->statusBar->SetStatusText(str);
            }
            else
            {
                pcRenderer->copyMode = 0;
                pcRenderer->isShowAxisWidget = false;
                wxString str("");
                m_pcUtils->statusBar->SetStatusText(str);
            }
        }
        else if (uc == 'U') // refresh constraints
        {
            m_pcUtils->pcRenderer->dispCurveNet->refreshAllConstraints();
        }
        else if (uc == 'V') // optimization init
        {
            m_pcUtils->opt.init(m_pcUtils->pcRenderer->dispCurveNet , m_pcUtils);
        }
        else if (uc == 'W')
        {
        }
        else if (uc == 'X')
        {
        }
        else if (uc == 'Y')
        {
        }
        else if (uc == 'Z')
        {
        }
        else if (uc == ' ') // optimization
        {
            m_pcUtils->pcRenderer->optUpdate(true);
        }
        else if (uc == '1') // parallel
        {
            printf("parallel\n");
            pcRenderer->markMode = 1;
        }
        else if (uc == '2') // coplanar
        {
            printf("coplanar\n");
            pcRenderer->markMode = 2;
        }
        else if (uc == '3') // orthogonal
        {
            printf("orthogonal\n");
            pcRenderer->markMode = 3;
        }
        else if (uc == '4') // tangent
        {
            printf("tangent\n");
            pcRenderer->markMode = 4;
        }
	}
	else
	{
        switch (event.GetKeyCode())
        {
            case WXK_ESCAPE:
                exit(0);
            case WXK_UP:
                m_pcUtils->pcRenderer->incBspCurveIndex();
                break;
            case WXK_DOWN:
                m_pcUtils->pcRenderer->decBspCurveIndex();
                break;
        }
    }
}
