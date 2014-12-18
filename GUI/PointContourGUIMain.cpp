/***************************************************************
 * Name:      PointContourGUIMain.cpp
 * Purpose:   Code for Application Frame
 * Author:     ()
 * Created:   2014-08-21
 * Copyright:  ()
 * License:
 **************************************************************/

#include "SketchGLCanvas.h"
#include "PointContourGUIMain.h"
#include "pointCloudUtils.h"
#include "ConfigManager.h"
#include "fileIOHelper.h"
#include <wx/wx.h>
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/colordlg.h>
#include <wx/filename.h>

//(*InternalHeaders(PointContourGUIFrame)
#include <wx/intl.h>
#include <wx/string.h>
//*)

int getInt(const wxString& str)
{
	long res;
	str.ToLong(&res);
	return res;
}

double getDouble(const wxString& str)
{
	double res;
	str.ToDouble(&res);
	return res;
}

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(PointContourGUIFrame)
const long PointContourGUIFrame::ID_GLCANVAS1 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX1 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX8 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX2 = wxNewId();
const long PointContourGUIFrame::ID_SLIDER1 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX3 = wxNewId();
const long PointContourGUIFrame::ID_SLIDER2 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX4 = wxNewId();
const long PointContourGUIFrame::ID_SLIDER3 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX5 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX6 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT18 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE3 = wxNewId();
const long PointContourGUIFrame::ID_CHECKBOX7 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT19 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE4 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT20 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE5 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT21 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE6 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT7 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL7 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT8 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL8 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT9 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL9 = wxNewId();
const long PointContourGUIFrame::ID_BUTTON2 = wxNewId();
const long PointContourGUIFrame::ID_SCROLLEDWINDOW2 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT1 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL1 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT2 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL2 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT3 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL3 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT4 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL4 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT5 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL5 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT11 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL11 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT6 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL6 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT10 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL10 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE2 = wxNewId();
const long PointContourGUIFrame::ID_BUTTON1 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT12 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL12 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT13 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL13 = wxNewId();
const long PointContourGUIFrame::ID_CHOICE1 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT14 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL14 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT15 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL15 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT16 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL16 = wxNewId();
const long PointContourGUIFrame::ID_STATICTEXT17 = wxNewId();
const long PointContourGUIFrame::ID_TEXTCTRL17 = wxNewId();
const long PointContourGUIFrame::ID_SCROLLEDWINDOW3 = wxNewId();
const long PointContourGUIFrame::ID_NOTEBOOK1 = wxNewId();
const long PointContourGUIFrame::ID_SCROLLEDWINDOW1 = wxNewId();
const long PointContourGUIFrame::ID_SPLITTERWINDOW1 = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM2 = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM4 = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM1 = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM3 = wxNewId();
const long PointContourGUIFrame::idMenuQuit = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM5 = wxNewId();
const long PointContourGUIFrame::ID_MENUITEM6 = wxNewId();
const long PointContourGUIFrame::idMenuAbout = wxNewId();
const long PointContourGUIFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(PointContourGUIFrame,wxFrame)
    //(*EventTable(PointContourGUIFrame)
    //*)
END_EVENT_TABLE()

PointContourGUIFrame::PointContourGUIFrame(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(PointContourGUIFrame)
    wxStaticBoxSizer* StaticBoxSizer2;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer16;
    wxMenuItem* MenuItem2;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxStaticBoxSizer* StaticBoxSizer7;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxStaticBoxSizer* StaticBoxSizer6;
    wxFlexGridSizer* FlexGridSizer15;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer14;
    wxFlexGridSizer* FlexGridSizer13;
    wxFlexGridSizer* FlexGridSizer12;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer17;
    wxMenu* Menu2;
    wxStaticBoxSizer* StaticBoxSizer5;

    Create(parent, wxID_ANY, _("PointContour"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(1200,800));
    Move(wxPoint(50,20));
    SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxPoint(0,0), wxSize(1200,780), wxSP_3D, _T("ID_SPLITTERWINDOW1"));
    SplitterWindow1->SetMinSize(wxSize(300,300));
    SplitterWindow1->SetMinimumPaneSize(300);
    SplitterWindow1->SetSashGravity(1);
    int GLCanvasAttributes_1[] = {
    	WX_GL_RGBA,
    	WX_GL_DOUBLEBUFFER,
    	WX_GL_DEPTH_SIZE,      16,
    	WX_GL_STENCIL_SIZE,    0,
    	0, 0 };
    m_openGLView = new SketchGLCanvas(SplitterWindow1, ID_GLCANVAS1, wxPoint(0,0), wxSize(800,780), wxTRANSPARENT_WINDOW, _T("ID_GLCANVAS1"), GLCanvasAttributes_1);
    m_openGLView->SetFocus();
    m_openGLView->SetBackgroundColour(wxColour(255,255,255));
    ScrolledWindow1 = new wxScrolledWindow(SplitterWindow1, ID_SCROLLEDWINDOW1, wxPoint(900,0), wxSize(300,800), wxTRANSPARENT_WINDOW, _T("ID_SCROLLEDWINDOW1"));
    ScrolledWindow1->SetBackgroundColour(wxColour(255,255,255));
    FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
    Notebook1 = new wxNotebook(ScrolledWindow1, ID_NOTEBOOK1, wxDefaultPosition, wxSize(300,800), 0, _T("ID_NOTEBOOK1"));
    ScrolledWindow2 = new wxScrolledWindow(Notebook1, ID_SCROLLEDWINDOW2, wxPoint(21,53), wxDefaultSize, wxVSCROLL|wxHSCROLL, _T("ID_SCROLLEDWINDOW2"));
    FlexGridSizer9 = new wxFlexGridSizer(0, 1, 0, 0);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxVERTICAL, ScrolledWindow2, _("Visualization"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 1, 0, 0);
    ShowPoint = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX1, _("show point"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    ShowPoint->SetValue(false);
    FlexGridSizer2->Add(ShowPoint, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    HideDrawnPoints = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX8, _("hide drawn points"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX8"));
    HideDrawnPoints->SetValue(true);
    FlexGridSizer2->Add(HideDrawnPoints, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    HidePointCloud = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX2, _("hide point cloud"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    HidePointCloud->SetValue(false);
    FlexGridSizer2->Add(HidePointCloud, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, ScrolledWindow2, _("Disc radius"));
    DiscRadius = new wxSlider(ScrolledWindow2, ID_SLIDER1, 50, 0, 100, wxDefaultPosition, wxSize(150,20), 0, wxDefaultValidator, _T("ID_SLIDER1"));
    StaticBoxSizer3->Add(DiscRadius, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    ShowHessian = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX3, _("show hessian"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    ShowHessian->SetValue(false);
    FlexGridSizer2->Add(ShowHessian, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    HessianLength = new wxSlider(ScrolledWindow2, ID_SLIDER2, 50, 0, 100, wxDefaultPosition, wxSize(150,20), 0, wxDefaultValidator, _T("ID_SLIDER2"));
    FlexGridSizer2->Add(HessianLength, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    ShowMetric = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX4, _("show metric"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    ShowMetric->SetValue(false);
    FlexGridSizer2->Add(ShowMetric, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    MetricLength = new wxSlider(ScrolledWindow2, ID_SLIDER3, 50, 0, 100, wxDefaultPosition, wxSize(150,20), 0, wxDefaultValidator, _T("ID_SLIDER3"));
    FlexGridSizer2->Add(MetricLength, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    UseBSpline = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX5, _("use B-Spline"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    UseBSpline->SetValue(true);
    FlexGridSizer2->Add(UseBSpline, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    ShowCtrlPoints = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX6, _("show B-Spline Ctrl Points"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX6"));
    ShowCtrlPoints->SetValue(false);
    FlexGridSizer2->Add(ShowCtrlPoints, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer14 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText18 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT18, _("constraints"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT18"));
    FlexGridSizer14->Add(StaticText18, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    ConstraintsVisual = new wxChoice(ScrolledWindow2, ID_CHOICE3, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE3"));
    ConstraintsVisual->SetSelection( ConstraintsVisual->Append(_("none")) );
    ConstraintsVisual->Append(_("collinear"));
    ConstraintsVisual->Append(_("parallel"));
    ConstraintsVisual->Append(_("coplanar"));
    ConstraintsVisual->Append(_("orthogonal"));
    ConstraintsVisual->Append(_("tangent"));
    FlexGridSizer14->Add(ConstraintsVisual, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer14, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    AutoOpt = new wxCheckBox(ScrolledWindow2, ID_CHECKBOX7, _("Auto Optimization"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX7"));
    AutoOpt->SetValue(true);
    FlexGridSizer2->Add(AutoOpt, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer15 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText19 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT19, _("cycles"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
    FlexGridSizer15->Add(StaticText19, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PatchesVisual = new wxChoice(ScrolledWindow2, ID_CHOICE4, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE4"));
    PatchesVisual->SetSelection( PatchesVisual->Append(_("all patches")) );
    PatchesVisual->Append(_("stored patches"));
    PatchesVisual->Append(_("temp patches"));
    PatchesVisual->Append(_("none"));
    FlexGridSizer15->Add(PatchesVisual, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer15, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer16 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText20 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT20, _("patch style"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT20"));
    FlexGridSizer16->Add(StaticText20, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PatchesDraw = new wxChoice(ScrolledWindow2, ID_CHOICE5, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE5"));
    PatchesDraw->SetSelection( PatchesDraw->Append(_("Shaded")) );
    PatchesDraw->Append(_("Lines"));
    FlexGridSizer16->Add(PatchesDraw, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer16, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer17 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText21 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT21, _("draw mode"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT21"));
    FlexGridSizer17->Add(StaticText21, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    DrawMode = new wxChoice(ScrolledWindow2, ID_CHOICE6, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE6"));
    DrawMode->SetSelection( DrawMode->Append(_("shortest path")) );
    DrawMode->Append(_("straight line"));
    FlexGridSizer17->Add(DrawMode, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer17, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(FlexGridSizer2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(StaticBoxSizer1, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer5 = new wxStaticBoxSizer(wxVERTICAL, ScrolledWindow2, _("Debug"));
    FlexGridSizer8 = new wxFlexGridSizer(0, 6, 0, 0);
    StaticText7 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT7, _("x"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer8->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PosX = new wxTextCtrl(ScrolledWindow2, ID_TEXTCTRL7, wxEmptyString, wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer8->Add(PosX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText8 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT8, _("y"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer8->Add(StaticText8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PosY = new wxTextCtrl(ScrolledWindow2, ID_TEXTCTRL8, wxEmptyString, wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    FlexGridSizer8->Add(PosY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText9 = new wxStaticText(ScrolledWindow2, ID_STATICTEXT9, _("z"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer8->Add(StaticText9, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PosZ = new wxTextCtrl(ScrolledWindow2, ID_TEXTCTRL9, wxEmptyString, wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL9"));
    FlexGridSizer8->Add(PosZ, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5->Add(FlexGridSizer8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    PrintInfoButton = new wxButton(ScrolledWindow2, ID_BUTTON2, _("Print Info"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    StaticBoxSizer5->Add(PrintInfoButton, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(StaticBoxSizer5, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    ScrolledWindow2->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(ScrolledWindow2);
    FlexGridSizer9->SetSizeHints(ScrolledWindow2);
    ScrolledWindow3 = new wxScrolledWindow(Notebook1, ID_SCROLLEDWINDOW3, wxPoint(165,16), wxDefaultSize, wxVSCROLL|wxHSCROLL, _T("ID_SCROLLEDWINDOW3"));
    FlexGridSizer10 = new wxFlexGridSizer(0, 1, 0, 0);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxVERTICAL, ScrolledWindow3, _("Parameters"));
    FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
    StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, ScrolledWindow3, _("Grid Resolution"));
    FlexGridSizer4 = new wxFlexGridSizer(0, 6, 0, 0);
    StaticText1 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT1, _("x"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer4->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    GridResX = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL1, _("128"), wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer4->Add(GridResX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT2, _("y"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer4->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    GridResY = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL2, _("128"), wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer4->Add(GridResY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT3, _("z"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer4->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    GridResZ = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL3, _("128"), wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer4->Add(GridResZ, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer4->Add(FlexGridSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(StaticBoxSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText4 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT4, _("Extra Boarder Size"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer5->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    ExtNum = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL4, _("15"), wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer5->Add(ExtNum, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText5 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT5, _("Filter Radius"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer7->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(47,30,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FilterRadius = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL5, _("10"), wxDefaultPosition, wxSize(40,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    FlexGridSizer7->Add(FilterRadius, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer7, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6 = new wxFlexGridSizer(0, 4, 0, 0);
    StaticText11 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT11, _("A1"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer6->Add(StaticText11, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Alpha1 = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL11, _("0.1"), wxDefaultPosition, wxSize(50,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
    FlexGridSizer6->Add(Alpha1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT6, _("A2"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer6->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Alpha2 = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL6, _("0.1"), wxDefaultPosition, wxSize(50,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer6->Add(Alpha2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText10 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT10, _("An"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer6->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    AlphaN = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL10, _("0.2"), wxDefaultPosition, wxSize(50,30), 0, wxDefaultValidator, _T("ID_TEXTCTRL10"));
    FlexGridSizer6->Add(AlphaN, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer6, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer12 = new wxFlexGridSizer(0, 3, 0, 0);
    MetricChoice = new wxChoice(ScrolledWindow3, ID_CHOICE2, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE2"));
    MetricChoice->SetSelection( MetricChoice->Append(_("Min curvature")) );
    MetricChoice->Append(_("Max curvature"));
    FlexGridSizer12->Add(MetricChoice, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer12, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    UpdateParametersButton = new wxButton(ScrolledWindow3, ID_BUTTON1, _("Update"), wxDefaultPosition, wxSize(100,40), 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer3->Add(UpdateParametersButton, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer6 = new wxStaticBoxSizer(wxVERTICAL, ScrolledWindow3, _("Local smooth"));
    FlexGridSizer11 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText12 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT12, _("lambda"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
    FlexGridSizer11->Add(StaticText12, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SmoothScale = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL12, _("0.1"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL12"));
    FlexGridSizer11->Add(SmoothScale, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText13 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT13, _("Iters"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer11->Add(StaticText13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SmoothIter = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL13, _("20"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL13"));
    FlexGridSizer11->Add(SmoothIter, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer6->Add(FlexGridSizer11, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(StaticBoxSizer6, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    SmoothingChoice = new wxChoice(ScrolledWindow3, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
    SmoothingChoice->Append(_("None"));
    SmoothingChoice->Append(_("Laplacian"));
    SmoothingChoice->SetSelection( SmoothingChoice->Append(_("Gradient descent")) );
    FlexGridSizer3->Add(SmoothingChoice, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer7 = new wxStaticBoxSizer(wxHORIZONTAL, ScrolledWindow3, _("Optimization"));
    FlexGridSizer13 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText14 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT14, _("#start points"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
    FlexGridSizer13->Add(StaticText14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    NumStartPoints = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL14, _("6"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL14"));
    FlexGridSizer13->Add(NumStartPoints, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText15 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT15, _("max time"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer13->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    MaxTime = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL15, _("0.2"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL15"));
    FlexGridSizer13->Add(MaxTime, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText16 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT16, _("large bound"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
    FlexGridSizer13->Add(StaticText16, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    LargeBound = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL16, _("0.003"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL16"));
    FlexGridSizer13->Add(LargeBound, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText17 = new wxStaticText(ScrolledWindow3, ID_STATICTEXT17, _("small bound"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT17"));
    FlexGridSizer13->Add(StaticText17, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SmallBound = new wxTextCtrl(ScrolledWindow3, ID_TEXTCTRL17, _("0.001"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER, wxDefaultValidator, _T("ID_TEXTCTRL17"));
    FlexGridSizer13->Add(SmallBound, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer7->Add(FlexGridSizer13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(StaticBoxSizer7, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2->Add(FlexGridSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer10->Add(StaticBoxSizer2, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    ScrolledWindow3->SetSizer(FlexGridSizer10);
    FlexGridSizer10->Fit(ScrolledWindow3);
    FlexGridSizer10->SetSizeHints(ScrolledWindow3);
    Notebook1->AddPage(ScrolledWindow2, _("Visualization"), false);
    Notebook1->AddPage(ScrolledWindow3, _("Parameters"), false);
    FlexGridSizer1->Add(Notebook1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    ScrolledWindow1->SetSizer(FlexGridSizer1);
    SetSizer(FlexGridSizer1);
    Layout();
    SplitterWindow1->SplitVertically(m_openGLView, ScrolledWindow1);
    SplitterWindow1->SetSashPosition(850);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenu();
    OpenPointCloud = new wxMenuItem(MenuItem3, ID_MENUITEM2, _("Point Cloud\tCtrl-F"), wxEmptyString, wxITEM_NORMAL);
    MenuItem3->Append(OpenPointCloud);
    OpenCurveNetwork = new wxMenuItem(MenuItem3, ID_MENUITEM4, _("Curve Network\tCtrl-O"), wxEmptyString, wxITEM_NORMAL);
    MenuItem3->Append(OpenCurveNetwork);
    Menu1->Append(ID_MENUITEM1, _("Open"), MenuItem3, wxEmptyString);
    SaveCurveNetwork = new wxMenuItem(Menu1, ID_MENUITEM3, _("Save curve network\tCtrl-S"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(SaveCurveNetwork);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    Undo = new wxMenuItem(Menu3, ID_MENUITEM5, _("Undo\tCtrl-Z"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(Undo);
    Redo = new wxMenuItem(Menu3, ID_MENUITEM6, _("Redo\tCtrl-R"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(Redo);
    MenuBar1->Append(Menu3, _("Edit"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -40 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);

    m_openGLView->Connect(wxEVT_KEY_DOWN,(wxObjectEventFunction)&PointContourGUIFrame::OnOpenGLViewKeyDown,0,this);
    m_openGLView->Connect(wxEVT_MOUSEWHEEL,(wxObjectEventFunction)&PointContourGUIFrame::OnMouseWheel,0,this);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnShowPointClick);
    Connect(ID_CHECKBOX8,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnHideDrawnPointsClick);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnHidePointCloudClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&PointContourGUIFrame::OnDiscRadiusCmdScrollThumbTrack);
    Connect(ID_CHECKBOX3,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnShowHessianClick);
    Connect(ID_SLIDER2,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&PointContourGUIFrame::OnHessianLengthCmdScrollThumbTrack);
    Connect(ID_CHECKBOX4,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnShowMetricClick);
    Connect(ID_SLIDER3,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&PointContourGUIFrame::OnMetricLengthCmdScrollThumbTrack);
    Connect(ID_CHECKBOX5,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnUseBSplineClick);
    Connect(ID_CHECKBOX6,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnShowCtrlPointsClick);
    Connect(ID_CHOICE3,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnConstraintsVisualSelect);
    Connect(ID_CHECKBOX7,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnAutoOptClick);
    Connect(ID_CHOICE4,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnPatchesVisualSelect);
    Connect(ID_CHOICE5,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnPatchesDrawSelect);
    Connect(ID_CHOICE6,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnDrawModeSelect);
    Connect(ID_TEXTCTRL7,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnPosXText);
    Connect(ID_TEXTCTRL8,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnPosYText);
    Connect(ID_TEXTCTRL9,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnPosZText);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnPrintInfoButtonClick);
    Connect(ID_TEXTCTRL1,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnGridResXText);
    Connect(ID_TEXTCTRL2,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnGridResYText);
    Connect(ID_TEXTCTRL3,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnGridResZText);
    Connect(ID_TEXTCTRL4,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnExtNumText);
    Connect(ID_TEXTCTRL5,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnFilterRadiusText);
    Connect(ID_TEXTCTRL11,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnAlpha1Text);
    Connect(ID_TEXTCTRL6,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnAlpha2Text);
    Connect(ID_TEXTCTRL10,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&PointContourGUIFrame::OnAlphaNText);
    Connect(ID_CHOICE2,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnMetricChoiceSelect);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&PointContourGUIFrame::OnUpdateParametersButtonClick);
    Connect(ID_TEXTCTRL12,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnSmoothScaleTextEnter);
    Connect(ID_TEXTCTRL13,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnSmoothIterTextEnter);
    Connect(ID_CHOICE1,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnSmoothingChoiceSelect);
    Connect(ID_TEXTCTRL14,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnNumStartPointsTextEnter);
    Connect(ID_TEXTCTRL15,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnMaxTimeTextEnter);
    Connect(ID_TEXTCTRL16,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnLargeBoundTextEnter);
    Connect(ID_TEXTCTRL17,wxEVT_COMMAND_TEXT_ENTER,(wxObjectEventFunction)&PointContourGUIFrame::OnSmallBoundTextEnter);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnOpenPointCloudSelected);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnOpenCurveNetworkSelected);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnSaveCurveNetworkSelected);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnQuit);
    Connect(ID_MENUITEM5,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnUndoSelected);
    Connect(ID_MENUITEM6,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnRedoSelected);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&PointContourGUIFrame::OnAbout);
    Connect(wxEVT_KEY_DOWN,(wxObjectEventFunction)&PointContourGUIFrame::OnKeyDown);
    Connect(wxEVT_MOUSEWHEEL,(wxObjectEventFunction)&PointContourGUIFrame::OnMouseWheel);
    Connect(wxEVT_SIZE,(wxObjectEventFunction)&PointContourGUIFrame::OnOpenGLViewResize);
    //*)

    m_pcUtils = new PointCloudUtils();
    *(m_openGLView->getPointCloudUtils()) = m_pcUtils;
	m_config = new ConfigManager(m_pcUtils);
	m_config->load("config.xml");
	m_pcUtils->globalInit();
    m_pcUtils->statusBar = StatusBar1;
}

PointContourGUIFrame::~PointContourGUIFrame()
{
    //(*Destroy(PointContourGUIFrame)
    //*)
}

void PointContourGUIFrame::resetAll()
{
    resetFrame();
}

void PointContourGUIFrame::resetFrame()
{
    wxString mystring = wxString::Format(wxT("%i"), 128);
	//wxString mystring = wxString::Format(wxT("%i"), 32);
	GridResX->SetValue(mystring);
	GridResY->SetValue(mystring);
	GridResZ->SetValue(mystring);

	mystring = wxString::Format(wxT("%i"), 15);
	//mystring = wxString::Format(wxT("%i"), 6);
	ExtNum->SetValue(mystring);

	mystring = wxString::Format(wxT("%i"), 10);
	//mystring = wxString::Format(wxT("%i"), 5);
	FilterRadius->SetValue(mystring);

	mystring = wxString::Format(wxT("%.1f"), 0.1);
    Alpha1->SetValue(mystring);

	mystring = wxString::Format(wxT("%.1f"), 0.5);
	Alpha2->SetValue(mystring);

	gridResChanged = extNumChanged = radiusChanged = alphaChanged = false;
}

void PointContourGUIFrame::constructPointCloud(const char* fileName , const char* fileType)
{
	m_pcUtils->m_fileName = fileName;
    char cacheName[256];
    strcpy(cacheName , fileName);
    for (int i = strlen(cacheName) - 1; i >= 0; i--)
    {
        if (cacheName[i] == '.')
        {
            cacheName[i] = '\0';
            break;
        }
    }
    m_pcUtils->name.clear();
    int pos;
    for (int i = strlen(cacheName) - 1; i >= 0; i--)
    {
        if (cacheName[i] == '/' || cacheName[i] == '\\')
        {
            pos = i + 1;
            break;
        }
    }
    for (int i = pos; i < strlen(cacheName); i++)
    {
        m_pcUtils->name.push_back(cacheName[i]);
    }
    strcat(cacheName , ".cache");
    m_pcUtils->m_cacheName = cacheName;
    printf("fileName = %s\n" , static_cast<const char*>(m_pcUtils->m_fileName.c_str()));
    printf("dirName = %s\n" , static_cast<const char*>(m_pcUtils->m_cacheName.c_str()));
    printf("idName = %s\n" , m_pcUtils->name.c_str());
	resetAll();

	FileIOHelper* fileIOHelper = new FileIOHelper();

	StatusBar1->SetStatusText("Load Point Cloud...");

	if (strcmp(fileType , "npts") == 0)
	{
		std::vector<vec3d> points;
		std::vector<vec3d> normals;
		fileIOHelper->readPointCloudDataNpts(fileName,
			points,
			normals);

		m_pcUtils->pcData.resize(points.size());
		for (int i = 0; i < points.size(); i++)
		{
			m_pcUtils->pcData[i].pos = points[i];
			m_pcUtils->pcData[i].n = normals[i];
			m_pcUtils->pcData[i].n.normalize();
		}
	}
	else if (strcmp(fileType , "ply") == 0)
	{
		std::vector<vec3d> points;
		fileIOHelper->readPointCloudDataPly(fileName,
			points);

		m_pcUtils->pcData.resize(points.size());
		for (int i = 0; i < points.size(); i++)
		{
			m_pcUtils->pcData[i].pos = points[i];
			m_pcUtils->pcData[i].n = vec3d(0 , 0 , 0);
		}
	}
    else
		return;

	// preprocess
	m_pcUtils->init();
	m_pcUtils->preprocess(getInt(GridResX->GetValue()) ,
		getInt(GridResY->GetValue()) , getInt(GridResZ->GetValue()) ,
		getInt(ExtNum->GetValue()) , getInt(FilterRadius->GetValue()) ,
		getDouble(Alpha1->GetValue()) , getDouble(Alpha2->GetValue()) ,
        getDouble(AlphaN->GetValue()));

    m_pcUtils->pcRenderer->pathChoice = SmoothingChoice->GetSelection();
    m_pcUtils->pcRenderer->smoothScale = getDouble(SmoothScale->GetValue());
    m_pcUtils->pcRenderer->smoothIter = getInt(SmoothIter->GetValue());

    m_pcUtils->opt.numStartPoints = getInt(NumStartPoints->GetValue());
    m_pcUtils->opt.maxRealTime = getDouble(MaxTime->GetValue());
    m_pcUtils->opt.largeBound = getDouble(LargeBound->GetValue());
    m_pcUtils->opt.smallBound = getDouble(SmallBound->GetValue());

	StatusBar1->SetStatusText("Finish Loading");

	m_openGLView->Render();
}

void PointContourGUIFrame::OnOpenPointCloudSelected(wxCommandEvent& event)
{
	wxString fileName = wxFileSelector(wxT("Open Point Cloud File"), (const wxChar *) NULL,
		(const wxChar *) NULL, wxT("POINT_CLOUD"),
		wxT("npts files (*.npts)|*.npts|ply files (*.ply)|*.ply|all files (*.*)|*.*"),wxFD_OPEN);

	if(fileName.length()==0) return;

	char tFileName[400];
	strcpy(tFileName,fileName);
	char FileType[400];
	for(int i=strlen(tFileName)-1;i>=0;i--){
		if(tFileName[i] == '.'){
			i++;
			unsigned tsize= strlen(tFileName)-i;
			for(unsigned j=0;j<tsize;j++){
				FileType[j] = tFileName[i+j];
			}
			FileType[tsize]='\0';
			break;
		}
	}

	constructPointCloud(fileName , FileType);
}

void PointContourGUIFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void PointContourGUIFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to Point Cloud Contour"));
}

void PointContourGUIFrame::OnShowPointClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isShowPoints = !m_pcUtils->pcRenderer->isShowPoints;
	m_openGLView->Render();
}

void PointContourGUIFrame::OnHidePointCloudClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isShowPointCloud = !m_pcUtils->pcRenderer->isShowPointCloud;
	m_openGLView->Render();
}

void PointContourGUIFrame::OnDiscRadiusCmdScrollThumbTrack(wxScrollEvent& event)
{
    double scale;
	scale = pow(1.1 , DiscRadius->GetValue() - 50.0);
	m_pcUtils->pcRenderer->discRenderRadiusScale = m_pcUtils->pcRenderer->discRenderBaseRadius * scale;
	m_pcUtils->pcRenderer->callListSurfelDisc();
	m_openGLView->Render();
}

void PointContourGUIFrame::OnShowHessianClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isShowHessian = !m_pcUtils->pcRenderer->isShowHessian;
    m_openGLView->Render();
}

void PointContourGUIFrame::OnHessianLengthCmdScrollThumbTrack(wxScrollEvent& event)
{
    double scale;
	scale = pow(1.1 , HessianLength->GetValue() - 50.0);
	m_pcUtils->pcRenderer->hessianRenderLengthScale = m_pcUtils->pcRenderer->hessianRenderBaseLength * scale;
	m_openGLView->Render();
}

void PointContourGUIFrame::OnShowMetricClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isShowMetric = !m_pcUtils->pcRenderer->isShowMetric;
    m_openGLView->Render();
}

void PointContourGUIFrame::OnMetricLengthCmdScrollThumbTrack(wxScrollEvent& event)
{
    double scale;
	scale = pow(1.1 , MetricLength->GetValue() - 50.0);
	m_pcUtils->pcRenderer->metricRenderLengthScale = m_pcUtils->pcRenderer->metricRenderBaseLength * scale;
	m_openGLView->Render();
}

void PointContourGUIFrame::OnGridResXText(wxCommandEvent& event)
{
    gridResChanged = true;
}

void PointContourGUIFrame::OnGridResYText(wxCommandEvent& event)
{
    gridResChanged = true;
}

void PointContourGUIFrame::OnGridResZText(wxCommandEvent& event)
{
    gridResChanged = true;
}

void PointContourGUIFrame::OnExtNumText(wxCommandEvent& event)
{
    extNumChanged = true;
}

void PointContourGUIFrame::OnFilterRadiusText(wxCommandEvent& event)
{
    radiusChanged = true;
}

void PointContourGUIFrame::OnAlpha1Text(wxCommandEvent& event)
{
    alphaChanged = true;
}

void PointContourGUIFrame::OnAlpha2Text(wxCommandEvent& event)
{
    alphaChanged = true;
}

void PointContourGUIFrame::OnAlphaNText(wxCommandEvent& event)
{
    alphaChanged = true;
}

void PointContourGUIFrame::OnUpdateParametersButtonClick(wxCommandEvent& event)
{
    StatusBar1->SetStatusText("Begin Updating Parameters");

	if (gridResChanged || extNumChanged)
	{
		m_pcUtils->preprocess(getInt(GridResX->GetValue()) ,
			getInt(GridResY->GetValue()) , getInt(GridResZ->GetValue()) ,
			getInt(ExtNum->GetValue()) , getInt(FilterRadius->GetValue()) ,
			getDouble(Alpha1->GetValue()) , getDouble(Alpha2->GetValue()) ,
            getDouble(AlphaN->GetValue()));
	}
	else if (radiusChanged)
	{
		int radius = getInt(FilterRadius->GetValue());
		m_pcUtils->gaussianSmooth(m_pcUtils->originF , m_pcUtils->f , radius / 3.0);

		m_pcUtils->calcHessian(m_pcUtils->f);

		m_pcUtils->alpha1 = getDouble(Alpha1->GetValue());
		m_pcUtils->alpha2 = getDouble(Alpha2->GetValue());
        m_pcUtils->alphaN = getDouble(AlphaN->GetValue());
		m_pcUtils->calcMetric(m_pcUtils->f);

		if (m_pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
		{
			m_pcUtils->calcPointTensor();
			m_pcUtils->buildGraphFromPoints();
		}
	}
	else if (alphaChanged)
	{
		m_pcUtils->alpha1 = getDouble(Alpha1->GetValue());
		m_pcUtils->alpha2 = getDouble(Alpha2->GetValue());
        m_pcUtils->alphaN = getDouble(AlphaN->GetValue());
		m_pcUtils->calcMetric(m_pcUtils->f);

		if (m_pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
		{
			m_pcUtils->calcPointTensor();
			m_pcUtils->buildGraphFromPoints();
		}
	}

	gridResChanged = extNumChanged = radiusChanged = alphaChanged = false;
	m_openGLView->Render();

	StatusBar1->SetStatusText("Finish Updating Parameters");
}

void PointContourGUIFrame::OnMetricChoiceSelect(wxCommandEvent& event)
{
    if (event.GetSelection() == 0)
    {
        m_pcUtils->metricType = PointCloudUtils::MIN_CURVATURE;
    }
    else
    {
        m_pcUtils->metricType = PointCloudUtils::MAX_CURVATURE;
    }

    bool needUpdate = false;
    Tensor& ts = m_pcUtils->tensor[1][1][1];
    if ((m_pcUtils->metricType == PointCloudUtils::MIN_CURVATURE &&
            ts.axisLen[1] < ts.axisLen[2]) ||
        (m_pcUtils->metricType == PointCloudUtils::MAX_CURVATURE &&
            ts.axisLen[1] > ts.axisLen[2]))
    {
        needUpdate = true;
    }
    if (needUpdate)
    {
        m_pcUtils->calcMetric(m_pcUtils->f);
        if (m_pcUtils->graphType == PointCloudUtils::POINT_GRAPH)
        {
            m_pcUtils->calcPointTensor();
            m_pcUtils->buildGraphFromPoints();
        }
        m_openGLView->Render();
        StatusBar1->SetStatusText("Finish Updating Parameters");
    }
}

void PointContourGUIFrame::OnPrintInfoButtonClick(wxCommandEvent& event)
{
    vec3d pos;
    if (debugPosChanged)
    {
        pos.x = getDouble(PosX->GetValue());
        pos.y = getDouble(PosY->GetValue());
        pos.z = getDouble(PosZ->GetValue());
        debugPosChanged = false;
    }
    else
    {
        pos = (m_pcUtils->pcRenderer->pickedDispPoint);
    }

    printf("\n------(%.6lf,%.6lf,%.6lf)------\n" , pos.x , pos.y , pos.z);

    Tensor ts;
    ts.hessian = m_pcUtils->lerpHessian(pos);
    m_pcUtils->calcTensorDecomposition(ts);
    m_pcUtils->calcTensorMetric(ts);

    for (int a = 0; a < 3; a++)
    {
        printf("(%.6lf,%.6lf,%.6lf), eigenVal = %.6lf, length = %.6lf\n" ,
            ts.axis[a].x , ts.axis[a].y , ts.axis[a].z ,
            ts.eigenVal[a] , ts.axisLen[a]);
    }
}

void PointContourGUIFrame::OnKeyDown(wxKeyEvent& event)
{
    OnOpenGLViewKeyDown(event);
}

void PointContourGUIFrame::OnLeftDClick(wxMouseEvent& event)
{
    IsFullScreen()?ShowFullScreen(false):ShowFullScreen(true);
	m_openGLView->Render();
}

void PointContourGUIFrame::OnMouseWheel(wxMouseEvent& event)
{
    if (m_pcUtils->pcRenderer->isShiftPress)
    {
        if (event.GetWheelRotation() > 0)
        {
            m_pcUtils->pcRenderer->patchesVisual = (m_pcUtils->pcRenderer->patchesVisual + 1) % 4;
        }
        else if (event.GetWheelRotation() < 0)
        {
            m_pcUtils->pcRenderer->patchesVisual = (m_pcUtils->pcRenderer->patchesVisual + 3) % 4;
        }
        PatchesVisual->SetSelection(m_pcUtils->pcRenderer->patchesVisual);
    }
    else if (m_pcUtils->pcRenderer->isAltPress)
    {
        if (event.GetWheelRotation() != 0)
            m_pcUtils->pcRenderer->dragPlaneNormalIndex = (m_pcUtils->pcRenderer->dragPlaneNormalIndex + 1) % 2;
    }
    else if (m_pcUtils->pcRenderer->isCtrlPress)
    {
        if (event.GetWheelRotation() != 0)
            m_pcUtils->pcRenderer->drawMode = (m_pcUtils->pcRenderer->drawMode + 1) % 2;
        DrawMode->SetSelection(m_pcUtils->pcRenderer->drawMode);
    }
    else if (!m_pcUtils->pcRenderer->isShiftPress && !m_pcUtils->pcRenderer->isCtrlPress &&
        !m_pcUtils->pcRenderer->isAltPress)
    {
        if (event.GetWheelRotation() != 0)
        {
            m_pcUtils->pcRenderer->isShowCtrlNodes = !m_pcUtils->pcRenderer->isShowCtrlNodes;
            ShowCtrlPoints->SetValue(m_pcUtils->pcRenderer->isShowCtrlNodes);
        }
    }
    m_openGLView->Render();
    m_openGLView->OnMouse(event);
}

void PointContourGUIFrame::OnOpenGLViewKeyDown(wxKeyEvent& event)
{
    switch (event.GetKeyCode())
	{
		case WXK_ESCAPE:
			exit(0);
        case WXK_SPACE:
            // m_pcUtils->pcRenderer->optUpdate();

            // m_pcUtils->pcRenderer->dispCurveNet->debugLog();

            m_pcUtils->pcRenderer->cycleDisc();
            m_pcUtils->pcRenderer->surfacingUnsavedCycles();
            m_pcUtils->pcRenderer->evalUnsavedCycles();
            break;
        case WXK_UP:
            m_pcUtils->pcRenderer->incBspCurveIndex();
            break;
        case WXK_DOWN:
            m_pcUtils->pcRenderer->decBspCurveIndex();
            break;
	}
	m_openGLView->OnKeyDown(event);
	m_openGLView->Render();
}

void PointContourGUIFrame::OnOpenGLViewResize(wxSizeEvent& event)
{
    m_openGLView->OnSize(event);
    Refresh();
}

void PointContourGUIFrame::OnPosXText(wxCommandEvent& event)
{
    debugPosChanged = true;
}

void PointContourGUIFrame::OnPosYText(wxCommandEvent& event)
{
    debugPosChanged = true;
}

void PointContourGUIFrame::OnPosZText(wxCommandEvent& event)
{
    debugPosChanged = true;
}

void PointContourGUIFrame::OnSmoothingChoiceSelect(wxCommandEvent& event)
{
    if (m_pcUtils->pcRenderer->pathForComp.size() == 0)
        return;
    m_pcUtils->pcRenderer->pathChoice = event.GetSelection();
    int choice = event.GetSelection();
    m_pcUtils->pcRenderer->pathVertex = m_pcUtils->pcRenderer->pathForComp[choice];
    m_openGLView->Render();
}

void PointContourGUIFrame::OnSmoothScaleTextEnter(wxCommandEvent& event)
{
    if (!m_pcUtils->initialized) return;
    m_pcUtils->pcRenderer->smoothScale = getDouble(SmoothScale->GetValue());
    m_pcUtils->pcRenderer->smoothIter = getInt(SmoothIter->GetValue());
    printf("iter = %d, lambda = %.6f\n" , m_pcUtils->pcRenderer->smoothIter ,
          m_pcUtils->pcRenderer->smoothScale);
    m_pcUtils->pcRenderer->pathForComp[1] = m_pcUtils->pcRenderer->pathForComp[0];
    m_pcUtils->pcRenderer->pathForComp[2] = m_pcUtils->pcRenderer->pathForComp[0];
    for (int i = 0; i < m_pcUtils->pcRenderer->smoothIter; i++)
    {
        m_pcUtils->laplacianSmooth(m_pcUtils->pcRenderer->pathForComp[1]);
        m_pcUtils->gradientDescentSmooth(m_pcUtils->pcRenderer->pathForComp[2]);
    }
    int choice = m_pcUtils->pcRenderer->pathChoice;
    m_pcUtils->pcRenderer->pathVertex = m_pcUtils->pcRenderer->pathForComp[choice];
    m_openGLView->Render();
}

void PointContourGUIFrame::OnSmoothIterTextEnter(wxCommandEvent& event)
{
    if (!m_pcUtils->initialized) return;
    m_pcUtils->pcRenderer->smoothScale = getDouble(SmoothScale->GetValue());
    m_pcUtils->pcRenderer->smoothIter = getInt(SmoothIter->GetValue());
    //printf("iter = %d, lambda = %.6f\n" , m_pcUtils->pcRenderer->smoothIter ,
    //       m_pcUtils->pcRenderer->smoothScale);
    m_pcUtils->pcRenderer->pathForComp[1] = m_pcUtils->pcRenderer->pathForComp[0];
    m_pcUtils->pcRenderer->pathForComp[2] = m_pcUtils->pcRenderer->pathForComp[0];
    for (int i = 0; i < m_pcUtils->pcRenderer->smoothIter; i++)
    {
        m_pcUtils->laplacianSmooth(m_pcUtils->pcRenderer->pathForComp[1]);
        m_pcUtils->gradientDescentSmooth(m_pcUtils->pcRenderer->pathForComp[2]);
    }
    int choice = m_pcUtils->pcRenderer->pathChoice;
    m_pcUtils->pcRenderer->pathVertex = m_pcUtils->pcRenderer->pathForComp[choice];
    m_openGLView->Render();
}

void PointContourGUIFrame::OnUseBSplineClick(wxCommandEvent& event)
{
    if (!m_pcUtils->initialized) return;
    m_pcUtils->pcRenderer->useBSpline = !m_pcUtils->pcRenderer->useBSpline;
    int choice = m_pcUtils->pcRenderer->pathChoice;
    if (m_pcUtils->pcRenderer->useBSpline)
    {
        m_pcUtils->pcRenderer->pathVertex = m_pcUtils->pcRenderer->pathForComp[choice];
        convert2Spline(m_pcUtils->pcRenderer->pathVertex ,
                                  m_pcUtils->pcRenderer->bsp);
    }
    else
    {
        m_pcUtils->pcRenderer->pathVertex = m_pcUtils->pcRenderer->pathForComp[choice];
    }
    m_openGLView->Render();
}

void PointContourGUIFrame::OnShowCtrlPointsClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isShowCtrlNodes = !m_pcUtils->pcRenderer->isShowCtrlNodes;
    m_openGLView->Render();
}

void PointContourGUIFrame::OnConstraintsVisualSelect(wxCommandEvent& event)
{
    if (m_pcUtils->pcRenderer->dispCurveNet->bsplines.size() == 0) return;
    m_pcUtils->pcRenderer->constraintsVisual = event.GetSelection();
    int& bspIndex = m_pcUtils->pcRenderer->bspIndex;
    int& curveIndex = m_pcUtils->pcRenderer->curveIndex;
    if (bspIndex >= m_pcUtils->pcRenderer->dispCurveNet->bsplines.size() ||
        curveIndex >= m_pcUtils->pcRenderer->dispCurveNet->bsplines[bspIndex].ctrlNodes.size())
    {
        bspIndex = 0;
        curveIndex = 0;
    }
    m_openGLView->Render();
}

void PointContourGUIFrame::OnAutoOptClick(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->isAutoOpt = (!m_pcUtils->pcRenderer->isAutoOpt);
    if (m_pcUtils->pcRenderer->isAutoOpt)
    {
        m_pcUtils->pcRenderer->optUpdate(true);
    }
    m_openGLView->Render();
}

void PointContourGUIFrame::OnOpenCurveNetworkSelected(wxCommandEvent& event)
{
    std::string fileName = m_pcUtils->dataCurvePath + m_pcUtils->name + ".curve";
    m_pcUtils->curveNet->loadCurveNet(fileName.c_str());
    for (int i = 0; i < m_pcUtils->curveNet->nodes.size(); i++)
    {
        m_pcUtils->addPointToGraph(m_pcUtils->curveNet->nodes[i]);
    }
    if (m_pcUtils->pcRenderer->isAutoOpt)
    {
        m_pcUtils->curveNet->refreshAllConstraints();
    }
    m_openGLView->Render();
}

void PointContourGUIFrame::OnSaveCurveNetworkSelected(wxCommandEvent& event)
{
    std::string fileName = m_pcUtils->dataCurvePath + m_pcUtils->name + ".curve";
    m_pcUtils->curveNet->saveCurveNet(fileName.c_str());
}

void PointContourGUIFrame::OnNumStartPointsTextEnter(wxCommandEvent& event)
{
    m_pcUtils->opt.numStartPoints = getInt(NumStartPoints->GetValue());
}

void PointContourGUIFrame::OnMaxTimeTextEnter(wxCommandEvent& event)
{
    m_pcUtils->opt.maxRealTime = getDouble(MaxTime->GetValue());
}

void PointContourGUIFrame::OnLargeBoundTextEnter(wxCommandEvent& event)
{
    m_pcUtils->opt.largeBound = getDouble(LargeBound->GetValue());
}

void PointContourGUIFrame::OnSmallBoundTextEnter(wxCommandEvent& event)
{
    m_pcUtils->opt.smallBound = getDouble(SmallBound->GetValue());
}

void PointContourGUIFrame::OnPatchesVisualSelect(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->patchesVisual = event.GetSelection();
    m_openGLView->Render();
}

void PointContourGUIFrame::OnUndoSelected(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->undo();
    m_openGLView->Render();
}

void PointContourGUIFrame::OnRedoSelected(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->undo();
    m_openGLView->Render();
}

void PointContourGUIFrame::OnPatchesDrawSelect(wxCommandEvent& event)
{
	m_pcUtils->pcRenderer->patchesDraw = event.GetSelection();
	m_openGLView->Render();
}

void PointContourGUIFrame::OnHideDrawnPointsClick(wxCommandEvent& event)
{
	m_pcUtils->pcRenderer->isHideDrawnPoints = !m_pcUtils->pcRenderer->isHideDrawnPoints;
	m_openGLView->Render();
}

void PointContourGUIFrame::OnDrawModeSelect(wxCommandEvent& event)
{
    m_pcUtils->pcRenderer->drawMode = event.GetSelection();
}
