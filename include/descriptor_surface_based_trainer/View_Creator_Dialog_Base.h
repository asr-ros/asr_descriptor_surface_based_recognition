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


#ifndef VIEW_CREATOR_DIALOG_BASE_H_
#define VIEW_CREATOR_DIALOG_BASE_H_

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/choice.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/statline.h>
#include <wx/checkbox.h>
#include <wx/textctrl.h>
#include <wx/slider.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include "wxImagePanel.h"



class ViewCreatorDialogBase : public wxDialog 
{
private:

protected:

    wxStaticText* label_orientation;
    wxStaticText* label_orientation_x;
    wxTextCtrl* edit_orientation_x;
    wxStaticText* label_orientation_y;
    wxTextCtrl* edit_orientation_y;
    wxStaticText* label_orientation_z;
    wxTextCtrl* edit_orientation_z;
    wxImagePanel* image_model;
    wxImagePanel* image_test;
    wxStaticText* label_image_source;
    wxChoice* choice_image_source;
    wxStaticText* label_image;
    wxChoice* choice_image;
    wxPanel* panel_image;
    wxStaticLine* m_staticline6;
    wxStaticText* label_test_image;
    wxChoice* choice_test_image;
    wxStaticLine* m_staticline4;
    wxStaticText* label_test_image_source;
    wxChoice* choice_test_image_source;
    wxPanel* panel_test_image;
    wxStaticText* label_select_image;
    wxCheckBox* check_fix_current_image;

    wxStaticText* label_upper_left;
    wxStaticText* label_upper_left_row;
    wxTextCtrl* edit_upper_left_row;
    wxSlider* slider_upper_left_row;
    wxStaticText* label_upper_left_column;
    wxTextCtrl* edit_upper_left_column;
    wxSlider* slider_upper_left_column;
    wxStaticText* label_lower_right_point;
    wxStaticText* label_lower_right_row;
    wxTextCtrl* edit_lower_right_row;
    wxSlider* slider_lower_right_row;
    wxStaticText* label_lower_right_column;
    wxTextCtrl* edit_lower_right_column;
    wxSlider* slider_lower_right_column;
    wxStaticLine* m_staticline61;

    wxButton* button_start_test;
    wxButton* button_end_test;

    wxStaticText* label_score;
    wxStaticText* label_score_value;
    wxStaticText* label_frame_number;
    wxStaticText* label_frame_number_value;
    wxStaticText* label_average_score;
    wxStaticText* label_average_score_value;
    wxStaticText* label_model_available;
    wxStaticText* label_model_available_value;
    wxStaticText* label_model_points;
    wxStaticText* label_model_points_value;
    wxStaticText* label_search_points;
    wxStaticText* label_search_points_value;
    wxStaticText* label_matched_points;
    wxStaticText* label_matched_points_value;
    wxStaticText* label_average_matched_points;
    wxStaticText* label_average_matched_points_value;
    wxStaticText* label_time;
    wxStaticText* label_time_value;
    wxStaticText* label_average_time;
    wxStaticText* label_average_time_value;
    wxStaticLine* m_staticline5;
    wxStaticText* label_score_2D;

    wxTextCtrl* edit_score_2D;
    wxStaticText* label_is_invertable;
    wxCheckBox* check_invertable;
    wxStaticText* label_vertical_offset;
    wxTextCtrl* edit_vertical_offset;
    wxStaticText* label_use_color;
    wxCheckBox* check_use_color;
    wxStaticText* label_horizontal_offset;
    wxTextCtrl* edit_horizontal_offset;
    wxStaticText* label_axis_1;
    wxStaticText* label_axis_1_x;
    wxTextCtrl* edit_axis_1_x;
    wxStaticText* label_axis_1_y;
    wxTextCtrl* edit_axis_1_y;
    wxStaticText* label_axis_1_z;
    wxTextCtrl* edit_axis_1_z;
    wxStaticText* label_axis_1_angle;
    wxTextCtrl* edit_axis_1_angle;
    wxStaticText* label_axis_2;
    wxStaticText* label_axis_2_x;
    wxTextCtrl* edit_axis_2_x;
    wxStaticText* label_axis_2_y;
    wxTextCtrl* edit_axis_2_y;
    wxStaticText* label_axis_2_z;
    wxTextCtrl* edit_axis_2_z;
    wxStaticText* label_axis_2_angle;
    wxTextCtrl* edit_axis_2_angle;

    wxStaticText* label_depth;
    wxTextCtrl* edit_depth;
    wxStaticText* label_number_ferns;
    wxTextCtrl* edit_fern_number;
    wxStaticText* label_patch_size;
    wxTextCtrl* edit_patch_size;
    wxStaticText* label_min_scale;
    wxTextCtrl* edit_min_scale;
    wxStaticText* label_max_scale;
    wxTextCtrl* edit_max_scale;

    wxButton* button_cancel;
    wxButton* button_save;

    // Virtual event handlers, overide them in your derived class
    virtual void OnDialogClose( wxCloseEvent& event ) { event.Skip(); }
    virtual void onChoiceImageSource( wxCommandEvent& event ) { event.Skip(); }
    virtual void onChoiceImage( wxCommandEvent& event ) { event.Skip(); }
    virtual void onChoiceTestImageSource(wxCommandEvent& event) { event.Skip(); }
    virtual void onChoiceTestImage( wxCommandEvent& event ) { event.Skip(); }
    virtual void onButtonCancelClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onButtonSaveClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onButtonStartTestClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onButtonEndTestClicked( wxCommandEvent& event ) { event.Skip(); }
    virtual void onCheckUseCurrentImage( wxCommandEvent& event ) { event.Skip(); }

    virtual void onEditTextUpperLeftRow(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextUpperLeftRowEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextUpperLeftColumn(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextUpperLeftColumnEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextLowerRightRow(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextLowerRightRowEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextLowerRightColumn(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextLowerRightColumnEnter(wxCommandEvent& event) { event.Skip(); }

    virtual void onSlideUpperLeftRow(wxScrollEvent& event) { event.Skip(); }
    virtual void onSlideUpperLeftColumn(wxScrollEvent& event) { event.Skip(); }
    virtual void onSlideLowerRightRow(wxScrollEvent& event) { event.Skip(); }
    virtual void onSlideLowerRightColumn(wxScrollEvent& event) { event.Skip(); }

    virtual void onEditTextOrientationX(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextOrientationY(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextOrientationZ(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextScore(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextVerticalOffset(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextHorizontalOffset(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextDepth(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextFernNumber(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextPatchSize(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextMinScale(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextMaxScale(wxFocusEvent& event) { event.Skip(); }

    virtual void onEditTextOrientationXEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextOrientationYEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextOrientationZEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextScoreEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextVerticalOffsetEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextHorizontalOffsetEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextDepthEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextFernNumberEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextPatchSizeEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextMinScaleEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextMaxScaleEnter(wxCommandEvent& event) { event.Skip(); }

    virtual void onEditTextAxis1X(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1Y(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1Z(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1Angle(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2X(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2Y(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2Z(wxFocusEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2Angle(wxFocusEvent& event) { event.Skip(); }

    virtual void onEditTextAxis1XEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1YEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1ZEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis1AngleEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2XEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2YEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2ZEnter(wxCommandEvent& event) { event.Skip(); }
    virtual void onEditTextAxis2AngleEnter(wxCommandEvent& event) { event.Skip(); }

    virtual void onCheckUpsideDown( wxCommandEvent& event ) { event.Skip(); }
    virtual void onCheckUseColor( wxCommandEvent& event ) { event.Skip(); }



public:

    ViewCreatorDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Descriptor Surfaced Based Trainer  -  Create View"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1000,880 ), long style = wxDEFAULT_DIALOG_STYLE );
    ~ViewCreatorDialogBase();

};

#endif //VIEW_CREATOR_DIALOG_BASE_H_
