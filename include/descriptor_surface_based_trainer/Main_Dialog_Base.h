/**

Copyright (C) 2016, Allgeyer Tobias, Hutmacher Robin, Meißner Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef MAIN_DIALOG_BASE_H_
#define MAIN_DIALOG_BASE_H_

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/dialog.h>



class MainDialogBase : public wxDialog
{
private:

protected:

    wxStaticText* label_caption;
    wxStaticText* label_name;
    wxTextCtrl* edit_name;
    wxStaticText* label_object_model;
    wxChoice* choice_object_model;
    wxStaticText* label_mesh;
    wxChoice* choice_mesh;
    wxStaticText* label_rotation_type;
    wxChoice* choice_rotation_type;
    wxStaticText* label_orientation;
    wxStaticText* label_orientation_x;
    wxTextCtrl* edit_orientation_x;
    wxStaticText* label_orientation_y;
    wxTextCtrl* edit_orientation_y;
    wxStaticText* label_orientation_z;
    wxTextCtrl* edit_orientation_z;
    wxStaticText* label_diameter;
    wxTextCtrl* edit_diameter;
    wxStaticText* label_score_3D;
    wxTextCtrl* edit_score_3D;

    wxButton* button_cancel;
    wxButton* button_next;


    // Virtual event handlers, overide them in your derived class
    virtual void onDialogClose( wxCloseEvent& event ) { event.Skip(); }
    virtual void onKillFocusName( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterName( wxCommandEvent& event ) { event.Skip(); }
    virtual void onObjectModelSelected( wxCommandEvent& event ) { event.Skip(); }
    virtual void onKillFocusOrientationX( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterOrientationX( wxCommandEvent& event ) { event.Skip(); }
    virtual void onKillFocusOrientationY( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterOrientationY( wxCommandEvent& event ) { event.Skip(); }
    virtual void onKillFocusOrientationZ( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterOrientationZ( wxCommandEvent& event ) { event.Skip(); }
    virtual void onKillFocusDiameter( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterDiameter( wxCommandEvent& event ) { event.Skip(); }
    virtual void onKillFocusScore3D( wxFocusEvent& event ) { event.Skip(); }
    virtual void onTextEnterScore3D( wxCommandEvent& event ) { event.Skip(); }
    virtual void onCancelPressed( wxCommandEvent& event ) { event.Skip(); }
    virtual void onNextPressed( wxCommandEvent& event ) { event.Skip(); }


public:

    MainDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Descriptor Surface Based Trainer"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 600,320 ), long style = wxDEFAULT_DIALOG_STYLE );
    ~MainDialogBase();

};

#endif //MAIN_DIALOG_BASE_H_
