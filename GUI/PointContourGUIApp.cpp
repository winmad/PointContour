/***************************************************************
 * Name:      PointContourGUIApp.cpp
 * Purpose:   Code for Application Class
 * Author:     ()
 * Created:   2014-08-21
 * Copyright:  ()
 * License:
 **************************************************************/

#include "PointContourGUIApp.h"

//(*AppHeaders
#include "PointContourGUIMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(PointContourGUIApp);

bool PointContourGUIApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	PointContourGUIFrame* Frame = new PointContourGUIFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
