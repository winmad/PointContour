#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include "SketchGLCanvas.h"
#include <wx/dcclient.h>
#include <wx/Statusbr.h>
#include <wx/filename.h>
#include "pointCloudUtils.h"

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
    GLfloat mat_specular[] = { 0.1f, 0.1f, 0.1f, 0.5f};
    //GLfloat mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);

    GLfloat mat_shininess[] = { 2 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    //GLfloat mat_diffuseB[] = { 0.6f, 0.6f, 0.8f, 1.0f };
    GLfloat mat_diffuseB[] = { 0.8f, 0.75f, 0.35f, 1.0f };
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
	if ((event.Dragging()||event.GetWheelRotation()!=0) && !event.ControlDown() && !event.ShiftDown()&& !event.AltDown())
    {
		if (event.LeftIsDown())
		{
			//convert the mouse clicked locations to lie between [-1,1] for both X and Y
			double halfWidth = m_width/2.0;
			double halfHeight = m_height/2.0;
			double xNormalized = (x-halfWidth)/halfWidth;
			double yNormalized = (halfHeight-y)/halfHeight;
			double oldXNormalized = (m_lastx-halfWidth)/halfWidth;
			double oldYNormalized = (halfHeight-m_lasty)/halfHeight;

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
        m_pcUtils->pcRenderer->pickedCurve = -1;
        m_pcUtils->pcRenderer->pickedCycle = -1;
        m_pcUtils->pcRenderer->pickedSavedCycle = -1;
        lastKeyBoard=1;
		m_pcUtils->pcRenderer->isCtrlPress = true;

		if(isChangeView){
			renderSelectionBuffer();
			isChangeView=false;
		}

		m_pcUtils->pcRenderer->pickPoint(x , y , 0);

		if (event.LeftIsDown())
		{
            // extend path by adding a new point
            m_pcUtils->pcRenderer->backup();
			m_pcUtils->pcRenderer->pickPoint(x , y , 1);
		}
		if (event.RightIsDown())
		{
            // cancel path extension
			m_pcUtils->pcRenderer->pathVertex.clear();
            m_pcUtils->pcRenderer->bsp.clear();
			setNull(m_pcUtils->pcRenderer->lastDispPoint);
		}

		Render();

	}
    else if (event.AltDown()) // edit operation
    {
        m_pcUtils->pcRenderer->pickedCycle = -1;
        lastKeyBoard = 2;
        m_pcUtils->pcRenderer->isAltPress = true;

        m_pcUtils->pcRenderer->pathVertex.clear();
        m_pcUtils->pcRenderer->bsp.clear();
        setNull(m_pcUtils->pcRenderer->lastDispPoint);
        setNull(m_pcUtils->pcRenderer->dragPlane.n);
        
		bool curvePicked = m_pcUtils->pcRenderer->pickCurve(x , y , 0);
        bool ctrlNodePicked = m_pcUtils->pcRenderer->pickCtrlNode(x , y , m_lastx , m_lasty , 0);
        if (!curvePicked)
        {
            m_pcUtils->pcRenderer->pickSavedCycle(x , y , 0);
        }
        if (event.LeftIsDown()) // update
        {
            if (ctrlNodePicked)
            {
                if (!isEditSpline)
                {
                    isEditSpline = true;
                    chosenBsp = m_pcUtils->pcRenderer->pickedBsp;
                    chosenCtrlNode = m_pcUtils->pcRenderer->pickedCtrlNode;
                    m_pcUtils->pcRenderer->dragStartPoint = m_pcUtils->pcRenderer->dispCurveNet->bsplines[chosenBsp].ctrlNodes[chosenCtrlNode];
                    m_pcUtils->pcRenderer->backup();
                }
            }

            if (isEditSpline)
            {
                m_pcUtils->pcRenderer->pickedBsp = chosenBsp;
                m_pcUtils->pcRenderer->pickedCtrlNode = chosenCtrlNode;
            }
            m_pcUtils->pcRenderer->pickCtrlNode(x , y , m_lastx , m_lasty , 1);
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

        if (!event.LeftIsDown())
        {
            if (isEditSpline && chosenBsp != -1 && m_pcUtils->pcRenderer->isAutoOpt)
            {
                m_pcUtils->pcRenderer->dispCurveNet->updateConstraints(chosenBsp);
                m_pcUtils->pcRenderer->optUpdate(false);
            }
            chosenBsp = chosenCtrlNode = -1;
            setNull(m_pcUtils->pcRenderer->dragStartPoint);
            isEditSpline = false;
        }
		Render();
    }

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
    }

	if(!event.ControlDown()) {startDraw=false;}

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
	int keycode = event.m_keyCode;
    // delete
	if (keycode == 127){
        m_pcUtils->pcRenderer->clearPaths();
        /*
		if(lastKeyBoard==0){
			lastKeyBoard=-1;
		}
		else if(lastKeyBoard==1){
			lastKeyBoard=-1;
		}
		else if(lastKeyBoard==2){
			//lastKeyBoard=-1;
		}
        */
		Render();
	}

/*
	if( keycode == 'C'){
		m_cycleUtils->pushLastCurveNetworkOriginal(*(m_cycleUtils->getCurveNetworkOriginal()));
		m_cycleUtils->chopCurves(0.01);
		*(m_meshUtils->getCurveNetwork()) = *(m_cycleUtils->getCurveNetworkOriginal());

		//m_cycleUtils->popLastCurveNetworkOriginal(m_cycleUtils->getCurveNetworkOriginal());
		//	*(m_meshUtils->getCurveNetwork()) = *(m_cycleUtils->getCurveNetworkOriginal());

		}
*/
}
