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


#ifndef DESC_MODELS_DIALOG_BASE_H_
#define DESC_MODELS_DIALOG_BASE_H_

#include <wx/string.h>
#include <wx/listbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include "wxImagePanel.h"


class DescModelsDialogBase : public wxDialog 
{
private:

protected:
    wxImagePanel* image;
    wxStaticText* label_caption;
    wxListBox* list_box_views;
    wxButton* button_edit_model;
    wxButton* button_add_model;
    wxButton* button_delete_model;
    wxStaticText* label_orientation;
    wxStaticText* label_orientation_x;
    wxStaticText* label_orientation_x_value;
    wxStaticText* label_orientation_y;
    wxStaticText* label_orientation_y_value;
    wxStaticText* label_orientation_z;
    wxStaticText* label_orientation_z_value;
    wxStaticText* label_axis_1;
    wxStaticText* label_axis_1_x;
    wxStaticText* label_axis_1_x_value;
    wxStaticText* label_axis_1_y;
    wxStaticText* label_axis_1_y_value;
    wxStaticText* label_axis_1_z;
    wxStaticText* label_axis_1_z_value;
    wxStaticText* label_axis_1_angle;
    wxStaticText* label_axis_1_angle_value;
    wxStaticText* label_axis_2;
    wxStaticText* label_axis_2_x;
    wxStaticText* label_axis_2_x_value;
    wxStaticText* label_axis_2_y;
    wxStaticText* label_axis_2_y_value;
    wxStaticText* label_axis_2_z;
    wxStaticText* label_axis_2_z_value;
    wxStaticText* label_axis_2_angle;
    wxStaticText* label_axis_2_angle_value;
    wxStaticText* label_score_2D;
    wxStaticText* label_score_2D_value;
    wxStaticText* label_use_color;
    wxStaticText* label_use_color_value;
    wxStaticText* label_invertible;
    wxStaticText* label_invertible_value;
    wxStaticText* label_vertical_offset;
    wxStaticText* label_vertical_offset_value;
    wxStaticText* label_horizontal_offset;
    wxStaticText* label_horizontal_offset_value;

    wxPanel* m_panel1;
    wxStaticText* label_depth;
    wxStaticText* label_depth_value;
    wxStaticText* label_number_ferns;
    wxStaticText* label_number_ferns_value;
    wxStaticText* label_patch_size;
    wxStaticText* label_patch_size_value;
    wxStaticText* label_min_scale;
    wxStaticText* label_min_scale_value;
    wxStaticText* label_max_scale;
    wxStaticText* label_max_scale_value;

    wxButton* button_back;
    wxButton* button_cancel;
    wxButton* button_finish;

    // Virtual event handlers, overide them in your derived class
    virtual void OnDialogClose( wxCloseEvent& event ) { event.Skip(); }
    virtual void onListBoxSelected( wxCommandEvent& event ) { event.Skip(); }
    virtual void onAddModelClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onDeleteModelClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onCancelClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onFinishClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onBackClicked(wxCommandEvent& event)  { event.Skip(); }
    virtual void onEditModelClicked(wxCommandEvent& event) { event.Skip(); }

    virtual void onImageClicked(wxMouseEvent& event) {event.Skip(); }
    virtual void onImageClickedRight(wxMouseEvent& event) {event.Skip(); }


public:

    DescModelsDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Descriptor Surfaced Based Trainer  -  Create views"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 800,540 ), long style = wxDEFAULT_DIALOG_STYLE );
    ~DescModelsDialogBase();

};

#endif //DESC_MODELS_DIALOG_BASE_H_
