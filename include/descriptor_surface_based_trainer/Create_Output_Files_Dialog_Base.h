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


#ifndef CREATE_OUTPUT_FILES_DIALOG_BASE_H_
#define CREATE_OUTPUT_FILES_DIALOG_BASE_H_

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/gauge.h>
#include <wx/button.h>
#include <wx/dialog.h>


class Create_Output_Files_Dialog_Base : public wxDialog 
{

protected:

    wxStaticText* label_message;
    wxGauge* progress_bar;
    wxButton* button_done;

    // Virtual event handlers, overide them in your derived class
    virtual void onDialogClose( wxCloseEvent& event ) { event.Skip(); }
    virtual void onButtonDoneClicked( wxCommandEvent& event ) { event.Skip(); }


public:

    Create_Output_Files_Dialog_Base( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Creating files"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 250,130 ), long style = wxDEFAULT_DIALOG_STYLE );
    ~Create_Output_Files_Dialog_Base();

};

#endif //CREATE_OUTPUT_FILES_DIALOG_BASE_H_
