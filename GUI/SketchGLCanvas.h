#pragma once
#include "wx/wx.h"
#include "wx/app.h"
#include "wx/defs.h"
#include "wx/frame.h"
#include "wx/menu.h"
#include "wx/splitter.h"
#include "wx/glcanvas.h"
#include "wx/msw/glcanvas.h"
#include <vector>
#include "colormap.h"
#include "nvVector.h"

class PointCloudUtils;

class SketchGLCanvas : public wxGLCanvas
{
public:

	SketchGLCanvas(	wxWindow *parent, wxWindowID id,
				const wxPoint& pos,
				const wxSize& size, long style,
				const wxString& name, int *attribList);

	~SketchGLCanvas ();

	void Initialize();
	void screenShot();
	void RenderMesh();
	void Render();

	void OnPaint ( wxPaintEvent &event );
	void OnMouse ( wxMouseEvent &event );
	void OnIdle ( wxIdleEvent &event );
	void OnSize ( wxSizeEvent &event );
	void OnKeyDown(wxKeyEvent &event);

	PointCloudUtils** getPointCloudUtils(){return &m_pcUtils;}
	void setViewPort();
	void saveViewPort();

	void setRotate(){ m_isRotate = !m_isRotate;}

private:


	/************************************************************************/
	/*                         data structure                               */
	/************************************************************************/

	/*
		for data accessing
	*/
	PointCloudUtils* m_pcUtils;
	bool startDraw;

	/*
		for opengl
	*/
    wxGLContext* m_context;
	double fpsCounter();
	int frames;
	int lastTime;

	bool m_initialized;

	wxCoord m_lastx;
	wxCoord m_lasty;
	wxCoord m_selectx;
	wxCoord m_selecty;
	std::vector<std::pair<int,int> > vts;
	int lastKeyBoard;
	bool isDrag;

	int m_width;
	int m_height;

	wxStatusBar *m_statusbar;

	int m_frameno;
//	timer m_fpstimer;
	unsigned int m_rotationCount;
	unsigned int m_rotationTimes;
	bool m_isRotate;
	clock_t m_frameRate;
	clock_t m_timePrev;
	clock_t m_timeCurr;
	int m_frames;

	//matrix4 m_RotationMatrix; // NO_MATRIX
	double m_RotationMatrix[16];

	vec3d m_Eye;

	double m_PerspectiveAngleDegrees;
	double m_NearPlane;
	double m_FarPlaneOffset;

	bool isChangeView;
	unsigned m_shotNum;
	std::vector<vec3d> computeRay(int mouseX,int mouseY);
	//void pickArcandNode(int mouseX, int mouseY,enum OperationType oT);
	void pickMeshVertex(int mouseX, int mouseY, bool isStore);

	void renderSelectionBuffer();
	void storeCurve();

	void rotateModel();
    DECLARE_EVENT_TABLE()

};
