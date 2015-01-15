/***************************************************************
 * Name:      PointContourGUIMain.h
 * Purpose:   Defines Application Frame
 * Author:     ()
 * Created:   2014-08-21
 * Copyright:  ()
 * License:
 **************************************************************/

#ifndef POINTCONTOURGUIMAIN_H
#define POINTCONTOURGUIMAIN_H

//(*Headers(PointContourGUIFrame)
#include <wx/scrolwin.h>
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/splitter.h>
#include <wx/glcanvas.h>
#include <wx/slider.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)

#include <vector>

class SketchGLCanvas;
class PointCloudUtils;
class ConfigManager;

class PointContourGUIFrame: public wxFrame
{
    public:

        PointContourGUIFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~PointContourGUIFrame();

    private:

        //(*Handlers(PointContourGUIFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnShowPointClick(wxCommandEvent& event);
        void OnHidePointCloudClick(wxCommandEvent& event);
        void OnDiscRadiusCmdScrollThumbTrack(wxScrollEvent& event);
        void OnShowHessianClick(wxCommandEvent& event);
        void OnHessianLengthCmdScrollThumbTrack(wxScrollEvent& event);
        void OnShowMetricClick(wxCommandEvent& event);
        void OnMetricLengthCmdScrollThumbTrack(wxScrollEvent& event);
        void OnGridResXText(wxCommandEvent& event);
        void OnGridResYText(wxCommandEvent& event);
        void OnGridResZText(wxCommandEvent& event);
        void OnExtNumText(wxCommandEvent& event);
        void OnFilterRadiusText(wxCommandEvent& event);
        void OnAlpha1Text(wxCommandEvent& event);
        void OnAlpha2Text(wxCommandEvent& event);
        void OnUpdateParametersButtonClick(wxCommandEvent& event);
        void OnPrintInfoButtonClick(wxCommandEvent& event);
        void OnOpenPointCloudSelected(wxCommandEvent& event);
        void OnKeyDown(wxKeyEvent& event);
        void OnLeftDClick(wxMouseEvent& event);
        void OnMouseWheel(wxMouseEvent& event);
        void OnOpenGLViewKeyDown(wxKeyEvent& event);
        void OnOpenGLViewResize(wxSizeEvent& event);
        void OnPosXText(wxCommandEvent& event);
        void OnPosYText(wxCommandEvent& event);
        void OnPosZText(wxCommandEvent& event);
        void OnAlphaNText(wxCommandEvent& event);
        void OnSmoothingChoiceSelect(wxCommandEvent& event);
        void OnSmoothScaleTextEnter(wxCommandEvent& event);
        void OnSmoothIterTextEnter(wxCommandEvent& event);
        void OnUseBSplineClick(wxCommandEvent& event);
        void OnShowCtrlPointsClick(wxCommandEvent& event);
        void OnMetricChoiceSelect(wxCommandEvent& event);
        void OnCollinearButtonClick(wxCommandEvent& event);
        void OnConstraintsVisualSelect(wxCommandEvent& event);
        void OnAutoOptClick(wxCommandEvent& event);
        void OnOpenCurveNetworkSelected(wxCommandEvent& event);
        void OnSaveCurveNetworkSelected(wxCommandEvent& event);
        void OnNumStartPointsTextEnter(wxCommandEvent& event);
        void OnMaxTimeTextEnter(wxCommandEvent& event);
        void OnLargeBoundTextEnter(wxCommandEvent& event);
        void OnSmallBoundTextEnter(wxCommandEvent& event);
        void OnPatchesVisualSelect(wxCommandEvent& event);
        void OnUndoSelected(wxCommandEvent& event);
        void OnRedoSelected(wxCommandEvent& event);
        void OnPatchesDrawSelect(wxCommandEvent& event);
        void OnHideDrawnPointsClick(wxCommandEvent& event);
        void OnDrawModeSelect(wxCommandEvent& event);
        void OnSaveMeshSelected(wxCommandEvent& event);
        //*)

        void resetAll();
        void resetFrame();
        void constructPointCloud(const char* fileName , const char* fileType);

        //(*Identifiers(PointContourGUIFrame)
        static const long ID_GLCANVAS1;
        static const long ID_CHECKBOX1;
        static const long ID_CHECKBOX8;
        static const long ID_CHECKBOX2;
        static const long ID_SLIDER1;
        static const long ID_CHECKBOX3;
        static const long ID_SLIDER2;
        static const long ID_CHECKBOX4;
        static const long ID_SLIDER3;
        static const long ID_CHECKBOX5;
        static const long ID_CHECKBOX6;
        static const long ID_STATICTEXT18;
        static const long ID_CHOICE3;
        static const long ID_CHECKBOX7;
        static const long ID_STATICTEXT19;
        static const long ID_CHOICE4;
        static const long ID_STATICTEXT20;
        static const long ID_CHOICE5;
        static const long ID_STATICTEXT21;
        static const long ID_CHOICE6;
        static const long ID_STATICTEXT7;
        static const long ID_TEXTCTRL7;
        static const long ID_STATICTEXT8;
        static const long ID_TEXTCTRL8;
        static const long ID_STATICTEXT9;
        static const long ID_TEXTCTRL9;
        static const long ID_BUTTON2;
        static const long ID_SCROLLEDWINDOW2;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_TEXTCTRL2;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL4;
        static const long ID_STATICTEXT5;
        static const long ID_TEXTCTRL5;
        static const long ID_STATICTEXT11;
        static const long ID_TEXTCTRL11;
        static const long ID_STATICTEXT6;
        static const long ID_TEXTCTRL6;
        static const long ID_STATICTEXT10;
        static const long ID_TEXTCTRL10;
        static const long ID_CHOICE2;
        static const long ID_BUTTON1;
        static const long ID_STATICTEXT12;
        static const long ID_TEXTCTRL12;
        static const long ID_STATICTEXT13;
        static const long ID_TEXTCTRL13;
        static const long ID_CHOICE1;
        static const long ID_STATICTEXT14;
        static const long ID_TEXTCTRL14;
        static const long ID_STATICTEXT15;
        static const long ID_TEXTCTRL15;
        static const long ID_STATICTEXT16;
        static const long ID_TEXTCTRL16;
        static const long ID_STATICTEXT17;
        static const long ID_TEXTCTRL17;
        static const long ID_SCROLLEDWINDOW3;
        static const long ID_NOTEBOOK1;
        static const long ID_SCROLLEDWINDOW1;
        static const long ID_SPLITTERWINDOW1;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM4;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM3;
        static const long ID_MENUITEM8;
        static const long ID_MENUITEM7;
        static const long idMenuQuit;
        static const long ID_MENUITEM5;
        static const long ID_MENUITEM6;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(PointContourGUIFrame)
        wxStaticText* StaticText10;
        wxTextCtrl* SmoothScale;
        wxChoice* PatchesDraw;
        wxStaticText* StaticText9;
        wxTextCtrl* FilterRadius;
        wxCheckBox* ShowMetric;
        wxTextCtrl* MaxTime;
        wxStaticText* StaticText20;
        wxChoice* SmoothingChoice;
        wxCheckBox* ShowHessian;
        wxCheckBox* HidePointCloud;
        wxSlider* MetricLength;
        wxNotebook* Notebook1;
        wxStaticText* StaticText13;
        wxStaticText* StaticText2;
        wxCheckBox* HideDrawnPoints;
        wxStaticText* StaticText14;
        wxScrolledWindow* ScrolledWindow3;
        wxScrolledWindow* ScrolledWindow1;
        wxTextCtrl* Alpha2;
        wxMenuItem* SaveMesh;
        wxMenu* Menu3;
        wxCheckBox* AutoOpt;
        wxStaticText* StaticText6;
        wxMenu* MenuItem3;
        wxCheckBox* ShowPoint;
        wxMenuItem* SaveCurveNetwork;
        wxStaticText* StaticText19;
        wxStaticText* StaticText8;
        wxStaticText* StaticText11;
        wxMenu* MenuItem4;
        wxTextCtrl* ExtNum;
        wxStaticText* StaticText18;
        wxChoice* PatchesVisual;
        wxTextCtrl* SmoothIter;
        wxButton* PrintInfoButton;
        wxTextCtrl* NumStartPoints;
        wxStaticText* StaticText1;
        wxSlider* HessianLength;
        wxStaticText* StaticText3;
        wxTextCtrl* AlphaN;
        wxMenuItem* OpenCurveNetwork;
        wxStaticText* StaticText21;
        wxMenuItem* Redo;
        wxTextCtrl* Alpha1;
        wxTextCtrl* PosX;
        wxTextCtrl* PosZ;
        wxTextCtrl* GridResZ;
        wxStaticText* StaticText5;
        wxStaticText* StaticText7;
        wxButton* UpdateParametersButton;
        wxStatusBar* StatusBar1;
        wxScrolledWindow* ScrolledWindow2;
        wxCheckBox* ShowCtrlPoints;
        wxChoice* DrawMode;
        wxStaticText* StaticText15;
        wxStaticText* StaticText12;
        wxChoice* MetricChoice;
        wxTextCtrl* GridResY;
        wxChoice* ConstraintsVisual;
        wxTextCtrl* LargeBound;
        wxTextCtrl* GridResX;
        wxTextCtrl* SmallBound;
        wxSplitterWindow* SplitterWindow1;
        wxStaticText* StaticText17;
        wxStaticText* StaticText4;
        SketchGLCanvas* m_openGLView;
        wxMenuItem* Undo;
        wxTextCtrl* PosY;
        wxSlider* DiscRadius;
        wxStaticText* StaticText16;
        wxCheckBox* UseBSpline;
        wxMenuItem* OpenPointCloud;
        //*)

        PointCloudUtils *m_pcUtils;
		ConfigManager *m_config;
        bool gridResChanged , extNumChanged , radiusChanged , alphaChanged;
        bool debugPosChanged;

        std::vector<double> changeColor();

        DECLARE_EVENT_TABLE()
};

#endif // POINTCONTOURGUIMAIN_H
