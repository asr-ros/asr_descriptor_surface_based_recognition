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


#include "descriptor_surface_based_trainer/View_Creator_Dialog_Base.h"


ViewCreatorDialogBase::ViewCreatorDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxHORIZONTAL );

    wxBoxSizer* bSizer42;
    bSizer42 = new wxBoxSizer(wxHORIZONTAL);
	
    label_image_source = new wxStaticText( this, wxID_ANY, wxT("Image source:"), wxDefaultPosition, wxSize(200, -1), 0 );
	label_image_source->Wrap( -1 );
	label_image_source->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
    bSizer42->Add( label_image_source, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    wxString choice_image_sourceChoices[] = { wxT("<No selection>"), wxT("File"), wxT("Camera") };
    int choice_image_sourceNChoices = sizeof( choice_image_sourceChoices ) / sizeof( wxString );
    choice_image_source = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( -1,-1 ), choice_image_sourceNChoices, choice_image_sourceChoices, 0 );
    choice_image_source->SetSelection( 0 );
    bSizer42->Add( choice_image_source, 1, wxALL|wxEXPAND, 5 );
	
    bSizer4->Add(bSizer42, 1, wxEXPAND, 5);

    m_staticline4 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL);
    bSizer4->Add(m_staticline4, 0, wxEXPAND|wxTOP|wxRIGHT|wxLEFT, 5);

    wxBoxSizer* bSizer43;
    bSizer43 = new wxBoxSizer(wxHORIZONTAL);

    label_test_image_source = new wxStaticText(this, wxID_ANY, wxT("Test image source:"), wxDefaultPosition, wxSize(200, -1), 0);
    label_test_image_source->Wrap(-1);
    label_test_image_source->SetFont(wxFont(wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString));

    bSizer43->Add(label_test_image_source, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5);

    wxString choice_test_image_sourceChoices[] = {wxT("<No selection>"), wxT("File"), wxT("Camera") };
    int choice_test_image_sourceNChoices = sizeof(choice_test_image_sourceChoices) / sizeof(wxString);
    choice_test_image_source = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, choice_test_image_sourceNChoices, choice_test_image_sourceChoices, 0);
    choice_test_image_source->SetSelection(0);
    bSizer43->Add(choice_test_image_source, 1, wxALL, 5);

    bSizer4->Add(bSizer43, 1, wxEXPAND, 5);


	bSizer3->Add( bSizer4, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer241;
	bSizer241 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer27;
	bSizer27 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
    label_image = new wxStaticText( this, wxID_ANY, wxT("Image:"), wxDefaultPosition, wxSize(120, -1), 0 );
	label_image->Wrap( -1 );
	label_image->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer6->Add( label_image, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString choice_imageChoices;
	choice_image = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, choice_imageChoices, 0 );
	choice_image->SetSelection( 0 );
	bSizer6->Add( choice_image, 1, wxALL|wxEXPAND, 5 );
	
	bSizer27->Add( bSizer6, 0, wxEXPAND, 5 );
	
	panel_image = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 480,360 ), wxTAB_TRAVERSAL );
	bSizer27->Add( panel_image, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_BOTTOM, 5 );

    image_model = new wxImagePanel(panel_image, 360, 480);
	
	bSizer241->Add( bSizer27, 1, 0, 5 );
	
	m_staticline6 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
    bSizer241->Add( m_staticline6, 0, wxEXPAND|wxBOTTOM|wxRIGHT|wxLEFT, 5 );
	
	wxBoxSizer* bSizer25;
	bSizer25 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer26;
	bSizer26 = new wxBoxSizer( wxHORIZONTAL );
	
    label_test_image = new wxStaticText( this, wxID_ANY, wxT("Test image:"), wxDefaultPosition, wxSize(120, -1), 0 );
	label_test_image->Wrap( -1 );
	label_test_image->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer26->Add( label_test_image, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    wxArrayString choice_test_imageChoices;
    choice_test_image = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, choice_test_imageChoices, 0 );
	choice_test_image->SetSelection( 0 );
	bSizer26->Add( choice_test_image, 1, wxALL, 5 );
	
	bSizer25->Add( bSizer26, 0, wxEXPAND, 5 );
	
	panel_test_image = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 480,360 ), wxTAB_TRAVERSAL );
	bSizer25->Add( panel_test_image, 0, wxEXPAND | wxALL, 5 );

    image_test = new wxImagePanel(panel_test_image, 360, 480);
	
	bSizer241->Add( bSizer25, 1, 0, 5 );
	
	bSizer3->Add( bSizer241, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer271;
	bSizer271 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer28;
	bSizer28 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxHORIZONTAL );
	
	label_select_image = new wxStaticText( this, wxID_ANY, wxT("Image selection:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_select_image->Wrap( -1 );
	label_select_image->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer31->Add( label_select_image, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer31->Add( 0, 0, 1, 0, 5 );
	
	check_fix_current_image = new wxCheckBox( this, wxID_ANY, wxT("Use current image"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer31->Add( check_fix_current_image, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer31->Add( 100, 0, 0, 0, 5 );
	
	bSizer28->Add( bSizer31, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer32;
	bSizer32 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer32->Add( 20, 0, 0, wxEXPAND, 5 );
	
	label_upper_left = new wxStaticText( this, wxID_ANY, wxT("Upper left point:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_upper_left->Wrap( -1 );
	label_upper_left->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 93, 90, false, wxEmptyString ) );
	
	bSizer32->Add( label_upper_left, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer32->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_upper_left_row = new wxStaticText( this, wxID_ANY, wxT("row:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_upper_left_row->Wrap( -1 );
	bSizer32->Add( label_upper_left_row, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_upper_left_row = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER);
	bSizer32->Add( edit_upper_left_row, 0, wxALL, 5 );
	
	slider_upper_left_row = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxSize( 200,-1 ), wxSL_HORIZONTAL );
	bSizer32->Add( slider_upper_left_row, 0, wxALL, 5 );
	
	bSizer28->Add( bSizer32, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer33;
	bSizer33 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer33->Add( 0, 0, 1, 0, 5 );
	
	label_upper_left_column = new wxStaticText( this, wxID_ANY, wxT("column:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_upper_left_column->Wrap( -1 );
	bSizer33->Add( label_upper_left_column, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_upper_left_column = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	bSizer33->Add( edit_upper_left_column, 0, wxALL, 5 );
	
	slider_upper_left_column = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxSize( 200,-1 ), wxSL_HORIZONTAL );
	bSizer33->Add( slider_upper_left_column, 0, wxALL, 5 );
	
	bSizer28->Add( bSizer33, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer34;
	bSizer34 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer34->Add( 20, 0, 0, 0, 5 );
	
	label_lower_right_point = new wxStaticText( this, wxID_ANY, wxT("Lower right point:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_lower_right_point->Wrap( -1 );
	label_lower_right_point->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 93, 90, false, wxEmptyString ) );
	
	bSizer34->Add( label_lower_right_point, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer34->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_lower_right_row = new wxStaticText( this, wxID_ANY, wxT("row:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_lower_right_row->Wrap( -1 );
	bSizer34->Add( label_lower_right_row, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_lower_right_row = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	bSizer34->Add( edit_lower_right_row, 0, wxALL, 5 );
	
	slider_lower_right_row = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxSize( 200,-1 ), wxSL_HORIZONTAL );
	bSizer34->Add( slider_lower_right_row, 0, wxALL, 5 );
	
	bSizer28->Add( bSizer34, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer35;
	bSizer35 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer35->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_lower_right_column = new wxStaticText( this, wxID_ANY, wxT("column:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_lower_right_column->Wrap( -1 );
	bSizer35->Add( label_lower_right_column, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_lower_right_column = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	bSizer35->Add( edit_lower_right_column, 0, wxALL, 5 );
	
	slider_lower_right_column = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxSize( 200,-1 ), wxSL_HORIZONTAL );
	bSizer35->Add( slider_lower_right_column, 0, wxALL, 5 );
	
	bSizer28->Add( bSizer35, 1, wxEXPAND, 5 );
	
	bSizer271->Add( bSizer28, 1, 0, 5 );
	
	m_staticline61 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	bSizer271->Add( m_staticline61, 0, wxEXPAND | wxALL, 5 );
	
    wxBoxSizer* bSizer30;
    bSizer30 = new wxBoxSizer( wxVERTICAL );

    wxBoxSizer* bSizer36;
    bSizer36 = new wxBoxSizer( wxHORIZONTAL );


    bSizer36->Add( 0, 0, 1, wxEXPAND, 5 );

    button_start_test = new wxButton( this, wxID_ANY, wxT("Start test"), wxDefaultPosition, wxDefaultSize, 0 );
    bSizer36->Add( button_start_test, 0, wxALL, 5 );

    button_end_test = new wxButton( this, wxID_ANY, wxT("End test"), wxDefaultPosition, wxDefaultSize, 0 );
    bSizer36->Add( button_end_test, 0, wxALL, 5 );


    bSizer36->Add( 0, 0, 1, wxEXPAND, 5 );

    bSizer30->Add( bSizer36, 1, wxEXPAND, 5 );

    bSizer30->Add( 0, 5, 0, 0, 5 );

    wxBoxSizer* bSizer37;
    bSizer37 = new wxBoxSizer( wxHORIZONTAL );

    label_score = new wxStaticText( this, wxID_ANY, wxT("Score:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_score->Wrap( -1 );
    bSizer37->Add( label_score, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer37->Add( 10, 0, 0, wxEXPAND, 5 );

    label_score_value = new wxStaticText( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxSize( 100,-1 ), 0 );
    label_score_value->Wrap( -1 );
    bSizer37->Add( label_score_value, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer37->Add( 0, 0, 1, wxEXPAND, 5 );

    label_frame_number = new wxStaticText( this, wxID_ANY, wxT("Frame #:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_frame_number->Wrap( -1 );
    bSizer37->Add( label_frame_number, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer37->Add( 30, 0, 0, wxEXPAND, 5 );

    label_frame_number_value = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 50,-1 ), 0 );
    label_frame_number_value->Wrap( -1 );
    bSizer37->Add( label_frame_number_value, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    bSizer37->Add( 10, 0, 0, wxEXPAND, 5 );

    bSizer30->Add( bSizer37, 0, wxEXPAND, 5 );

    wxBoxSizer* bSizer38;
    bSizer38 = new wxBoxSizer( wxHORIZONTAL );

    label_average_score = new wxStaticText( this, wxID_ANY, wxT("Average score:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_average_score->Wrap( -1 );
    bSizer38->Add( label_average_score, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer38->Add( 10, 0, 0, wxEXPAND, 5 );

    label_average_score_value = new wxStaticText( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxSize( 100,-1 ), 0 );
    label_average_score_value->Wrap( -1 );
    bSizer38->Add( label_average_score_value, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    bSizer38->Add( 0, 0, 1, wxEXPAND, 5 );

    label_model_available = new wxStaticText( this, wxID_ANY, wxT("Model avbl.:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_model_available->Wrap( -1 );
    bSizer38->Add( label_model_available, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    bSizer38->Add( 30, 0, 0, 0, 5 );

    label_model_available_value = new wxStaticText( this, wxID_ANY, wxT("No"), wxDefaultPosition, wxSize( 50,-1 ), 0 );
    label_model_available_value->Wrap( -1 );
    bSizer38->Add( label_model_available_value, 0, wxALL, 5 );


    bSizer38->Add( 10, 0, 0, 0, 5 );


    bSizer30->Add( bSizer38, 0, wxEXPAND, 5 );

    wxBoxSizer* bSizer39;
    bSizer39 = new wxBoxSizer( wxHORIZONTAL );

    label_model_points = new wxStaticText( this, wxID_ANY, wxT("Model features:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_model_points->Wrap( -1 );
    bSizer39->Add( label_model_points, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer39->Add( 10, 0, 0, 0, 5 );

    label_model_points_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), 0 );
    label_model_points_value->Wrap( -1 );
    bSizer39->Add( label_model_points_value, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer39->Add( 0, 0, 1, wxEXPAND, 5 );

    label_search_points = new wxStaticText( this, wxID_ANY, wxT("Found features:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_search_points->Wrap( -1 );
    bSizer39->Add( label_search_points, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer39->Add( 30, 0, 0, 0, 5 );

    label_search_points_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
    label_search_points_value->Wrap( -1 );
    bSizer39->Add( label_search_points_value, 0, wxALL, 5 );


    bSizer39->Add( 10, 0, 0, 0, 5 );

    bSizer30->Add( bSizer39, 0, wxEXPAND, 5 );

    wxBoxSizer* bSizer40;
    bSizer40 = new wxBoxSizer( wxHORIZONTAL );

    label_matched_points = new wxStaticText( this, wxID_ANY, wxT("Matched features:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_matched_points->Wrap( -1 );
    bSizer40->Add( label_matched_points, 0, wxALL, 5 );


    bSizer40->Add( 10, 0, 0, 0, 5 );

    label_matched_points_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), 0 );
    label_matched_points_value->Wrap( -1 );
    bSizer40->Add( label_matched_points_value, 0, wxALL, 5 );


    bSizer40->Add( 0, 0, 1, wxEXPAND, 5 );

    label_average_matched_points = new wxStaticText( this, wxID_ANY, wxT("Avg. matched ftrs.:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
    label_average_matched_points->Wrap( -1 );
    bSizer40->Add( label_average_matched_points, 0, wxALL, 5 );


    bSizer40->Add( 30, 0, 0, 0, 5 );

    label_average_matched_points_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
    label_average_matched_points_value->Wrap( -1 );
    bSizer40->Add( label_average_matched_points_value, 0, wxALL, 5 );


    bSizer40->Add( 10, 0, 0, 0, 5 );

    bSizer30->Add( bSizer40, 0, wxEXPAND, 5 );

    wxBoxSizer* bSizer41;
    bSizer41 = new wxBoxSizer(wxHORIZONTAL);

    label_time = new wxStaticText(this, wxID_ANY, wxT("Time:"), wxDefaultPosition, wxSize(120, -1), 0);
    label_time->Wrap(-1);
    bSizer41->Add(label_time, 0, wxALL, 5);

    bSizer41->Add(10, 0, 0, 0, 5);

    label_time_value = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(100, -1), 0);
    label_time_value->Wrap(-1);
    bSizer41->Add(label_time_value, 0, wxALL, 5);

    bSizer41->Add(0, 0, 1, wxEXPAND, 5);

    label_average_time = new wxStaticText(this, wxID_ANY, wxT("Avg. time:"), wxDefaultPosition, wxSize(120, -1), 0);
    label_average_time->Wrap(-1);
    bSizer41->Add(label_average_time, 0, wxALL, 5);

    bSizer41->Add(30, 0, 0, 0, 5);

    label_average_time_value = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(50, -1), 0);
    label_average_time_value->Wrap(-1);
    bSizer41->Add(label_average_time_value, 0, wxALL, 5);

    bSizer41->Add(10, 0, 0, 0, 5);

    bSizer30->Add(bSizer41, 0, wxEXPAND, 5);




    bSizer30->Add( 0, 0, 1, wxEXPAND, 5 );
	
    bSizer271->Add( bSizer30, 1, wxEXPAND, 5 );
	
	bSizer3->Add( bSizer271, 0, wxEXPAND, 5 );
	
	m_staticline5 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer3->Add( m_staticline5, 0, wxEXPAND | wxALL, 5 );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer7->Add( 30, 0, 0, 0, 5 );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	bSizer8->SetMinSize( wxSize( 580,-1 ) ); 
	
	bSizer8->Add( 0, 0, 1, wxEXPAND, 5 );

    wxBoxSizer* bSizer381;
    bSizer381 = new wxBoxSizer( wxHORIZONTAL );

    label_orientation = new wxStaticText( this, wxID_ANY, wxT("Orientation:"), wxDefaultPosition, wxDefaultSize, 0 );
    label_orientation->Wrap( -1 );
    label_orientation->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );

    bSizer381->Add( label_orientation, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );


    bSizer381->Add( 0, 0, 1, wxEXPAND, 5 );

    label_orientation_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
    label_orientation_x->Wrap( -1 );
    bSizer381->Add( label_orientation_x, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    edit_orientation_x = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
    bSizer381->Add( edit_orientation_x, 0, wxALL, 5 );

    label_orientation_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
    label_orientation_y->Wrap( -1 );
    bSizer381->Add( label_orientation_y, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    edit_orientation_y = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
    bSizer381->Add( edit_orientation_y, 0, wxALL, 5 );

    label_orientation_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
    label_orientation_z->Wrap( -1 );
    bSizer381->Add( label_orientation_z, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

    edit_orientation_z = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
    bSizer381->Add( edit_orientation_z, 0, wxALL, 5 );


    bSizer381->Add( 132, 0, 0, 0, 5 );

    bSizer8->Add( bSizer381, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxHORIZONTAL );
	
	label_score_2D = new wxStaticText( this, wxID_ANY, wxT("Score 2D:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_score_2D->Wrap( -1 );
	label_score_2D->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer10->Add( label_score_2D, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer10->Add( 0, 0, 1, wxEXPAND, 5 );
	
    edit_score_2D = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
    bSizer10->Add( edit_score_2D, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer15->SetMinSize( wxSize( 280,-1 ) ); 
	
	bSizer15->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_is_invertable = new wxStaticText( this, wxID_ANY, wxT("Can be upside-down:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_is_invertable->Wrap( -1 );
	label_is_invertable->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer15->Add( label_is_invertable, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	check_invertable = new wxCheckBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT );
	bSizer15->Add( check_invertable, 0, wxALL, 5 );
	
	bSizer10->Add( bSizer15, 0, wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer8->Add( bSizer10, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxHORIZONTAL );
	
	label_vertical_offset = new wxStaticText( this, wxID_ANY, wxT("Vertical offset:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_vertical_offset->Wrap( -1 );
	label_vertical_offset->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer11->Add( label_vertical_offset, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer11->Add( 0, 0, 1, wxEXPAND, 5 );
	
    edit_vertical_offset = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer11->Add( edit_vertical_offset, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer16;
	bSizer16 = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer16->SetMinSize( wxSize( 280,-1 ) ); 
	
	bSizer16->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_use_color = new wxStaticText( this, wxID_ANY, wxT("Use color:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_use_color->Wrap( -1 );
	label_use_color->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer16->Add( label_use_color, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	check_use_color = new wxCheckBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT );
	bSizer16->Add( check_use_color, 0, wxALL, 5 );
	
	bSizer11->Add( bSizer16, 0, wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer8->Add( bSizer11, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	label_horizontal_offset = new wxStaticText( this, wxID_ANY, wxT("Horizontal offset:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_horizontal_offset->Wrap( -1 );
	label_horizontal_offset->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer12->Add( label_horizontal_offset, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer12->Add( 0, 0, 1, wxEXPAND, 5 );
	
    edit_horizontal_offset = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer12->Add( edit_horizontal_offset, 0, wxALL, 5 );
	
	
	bSizer12->Add( 280, 0, 0, 0, 5 );
	
	bSizer8->Add( bSizer12, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxHORIZONTAL );
	
	label_axis_1 = new wxStaticText( this, wxID_ANY, wxT("Axis 1:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1->Wrap( -1 );
	label_axis_1->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer13->Add( label_axis_1, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer13->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_axis_1_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_x->Wrap( -1 );
	bSizer13->Add( label_axis_1_x, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_1_x = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer13->Add( edit_axis_1_x, 0, wxALL, 5 );
	
	label_axis_1_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_y->Wrap( -1 );
	bSizer13->Add( label_axis_1_y, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_1_y = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer13->Add( edit_axis_1_y, 0, wxALL, 5 );
	
	label_axis_1_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_z->Wrap( -1 );
	bSizer13->Add( label_axis_1_z, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_1_z = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer13->Add( edit_axis_1_z, 0, wxALL, 5 );
	
	label_axis_1_angle = new wxStaticText( this, wxID_ANY, wxT("angle:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_angle->Wrap( -1 );
	bSizer13->Add( label_axis_1_angle, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_1_angle = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer13->Add( edit_axis_1_angle, 0, wxALL, 5 );
	
	
	bSizer13->Add( 10, 0, 0, 0, 5 );
	
	bSizer8->Add( bSizer13, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	label_axis_2 = new wxStaticText( this, wxID_ANY, wxT("Axis 2:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2->Wrap( -1 );
	label_axis_2->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer14->Add( label_axis_2, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer14->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_axis_2_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_x->Wrap( -1 );
	bSizer14->Add( label_axis_2_x, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_2_x = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer14->Add( edit_axis_2_x, 0, wxALL, 5 );
	
	label_axis_2_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_y->Wrap( -1 );
	bSizer14->Add( label_axis_2_y, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_2_y = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer14->Add( edit_axis_2_y, 0, wxALL, 5 );
	
	label_axis_2_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_z->Wrap( -1 );
	bSizer14->Add( label_axis_2_z, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_2_z = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer14->Add( edit_axis_2_z, 0, wxALL, 5 );
	
	label_axis_2_angle = new wxStaticText( this, wxID_ANY, wxT("angle:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_angle->Wrap( -1 );
	bSizer14->Add( label_axis_2_angle, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_axis_2_angle = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer14->Add( edit_axis_2_angle, 0, wxALL, 5 );
	
	
	bSizer14->Add( 10, 0, 0, 0, 5 );
	
	bSizer8->Add( bSizer14, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer18;
	bSizer18 = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer8->Add( bSizer18, 0, wxEXPAND, 5 );
	
	
	bSizer8->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer7->Add( bSizer8, 0, wxEXPAND, 5 );
	
	
	bSizer7->Add( 0, 0, 1, 0, 5 );
	
	wxBoxSizer* bSizer17;
	bSizer17 = new wxBoxSizer( wxVERTICAL );
	
	bSizer17->SetMinSize( wxSize( 300,-1 ) ); 
	
	bSizer17->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxHORIZONTAL );
	
	label_depth = new wxStaticText( this, wxID_ANY, wxT("Depth:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_depth->Wrap( -1 );
	label_depth->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer19->Add( label_depth, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_depth = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer19->Add( edit_depth, 0, wxALL, 5 );
	
	
	bSizer19->Add( 30, 0, 0, 0, 5 );
	
	bSizer17->Add( bSizer19, 0, wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer( wxHORIZONTAL );
	
	label_number_ferns = new wxStaticText( this, wxID_ANY, wxT("Fern number:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_number_ferns->Wrap( -1 );
	label_number_ferns->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer20->Add( label_number_ferns, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_fern_number = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer20->Add( edit_fern_number, 0, wxALL, 5 );
	
	
	bSizer20->Add( 30, 0, 0, 0, 5 );
	
	bSizer17->Add( bSizer20, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer21;
	bSizer21 = new wxBoxSizer( wxHORIZONTAL );
	
	label_patch_size = new wxStaticText( this, wxID_ANY, wxT("Patch size:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_patch_size->Wrap( -1 );
	label_patch_size->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer21->Add( label_patch_size, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_patch_size = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer21->Add( edit_patch_size, 0, wxALL, 5 );
	
	
	bSizer21->Add( 30, 0, 0, 0, 5 );
	
	bSizer17->Add( bSizer21, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer22;
	bSizer22 = new wxBoxSizer( wxHORIZONTAL );
	
	label_min_scale = new wxStaticText( this, wxID_ANY, wxT("Min. scale:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_min_scale->Wrap( -1 );
	label_min_scale->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer22->Add( label_min_scale, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_min_scale = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer22->Add( edit_min_scale, 0, wxALL, 5 );
	
	
	bSizer22->Add( 30, 0, 0, 0, 5 );
	
	bSizer17->Add( bSizer22, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxHORIZONTAL );
	
	label_max_scale = new wxStaticText( this, wxID_ANY, wxT("Max. scale:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_max_scale->Wrap( -1 );
	label_max_scale->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer23->Add( label_max_scale, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
    edit_max_scale = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_PROCESS_ENTER );
	bSizer23->Add( edit_max_scale, 0, wxALL, 5 );
	
	
	bSizer23->Add( 30, 0, 0, 0, 5 );
	
	bSizer17->Add( bSizer23, 0, wxEXPAND, 5 );
	
	
	bSizer17->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer7->Add( bSizer17, 0, wxEXPAND, 5 );
	
	
	bSizer7->Add( 30, 0, 0, wxEXPAND, 5 );
	
	bSizer3->Add( bSizer7, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer24->Add( 0, 0, 1, wxEXPAND, 5 );
	
	button_cancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( button_cancel, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	button_save = new wxButton( this, wxID_ANY, wxT("Save"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( button_save, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer3->Add( bSizer24, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 1, wxEXPAND, 5 );
	
	bSizer1->Add( bSizer2, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( ViewCreatorDialogBase::OnDialogClose ) );
	choice_image_source->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceImageSource ), NULL, this );
	choice_image->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceImage ), NULL, this );
    choice_test_image_source->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceTestImageSource ), NULL, this );
	choice_test_image->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceTestImage ), NULL, this );
	button_cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonCancelClicked ), NULL, this );
	button_save->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonSaveClicked ), NULL, this );
    button_start_test->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonStartTestClicked ), NULL, this );
    button_end_test->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonEndTestClicked ), NULL, this );
    check_fix_current_image->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUseCurrentImage ), NULL, this );
    edit_upper_left_row->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftRow ), NULL, this );
    edit_upper_left_row->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftRowEnter ), NULL, this );
    edit_upper_left_column->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftColumn ), NULL, this );
    edit_upper_left_column->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftColumnEnter ), NULL, this );
    edit_lower_right_row->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextLowerRightRow ), NULL, this );
    edit_lower_right_row->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextLowerRightRowEnter ), NULL, this );
    edit_lower_right_column->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextLowerRightColumn ), NULL, this );
    edit_lower_right_column->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextLowerRightColumnEnter ), NULL, this );
    slider_upper_left_row->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideUpperLeftRow), NULL, this);
    slider_upper_left_column->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideUpperLeftColumn), NULL, this);
    slider_lower_right_row->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideLowerRightRow), NULL, this);
    slider_lower_right_column->Connect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideLowerRightColumn), NULL, this);

    edit_orientation_x->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationX ), NULL, this );
    edit_orientation_y->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationY ), NULL, this );
    edit_orientation_z->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationZ ), NULL, this );
    edit_score_2D->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextScore ), NULL, this );
    edit_vertical_offset->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextVerticalOffset ), NULL, this );
    edit_horizontal_offset->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextHorizontalOffset ), NULL, this );
    edit_depth->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextDepth ), NULL, this );
    edit_fern_number->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextFernNumber ), NULL, this );
    edit_patch_size->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextPatchSize ), NULL, this );
    edit_min_scale->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextMinScale ), NULL, this );
    edit_max_scale->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextMaxScale ), NULL, this );

    edit_orientation_x->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationXEnter ), NULL, this );
    edit_orientation_y->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationYEnter ), NULL, this );
    edit_orientation_z->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationZEnter ), NULL, this );
    edit_score_2D->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextScoreEnter ), NULL, this );
    edit_vertical_offset->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextVerticalOffsetEnter ), NULL, this );
    edit_horizontal_offset->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextHorizontalOffsetEnter ), NULL, this );
    edit_depth->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextDepthEnter ), NULL, this );
    edit_fern_number->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextFernNumberEnter ), NULL, this );
    edit_patch_size->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextPatchSizeEnter ), NULL, this );
    edit_min_scale->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextMinScaleEnter ), NULL, this );
    edit_max_scale->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextMaxScaleEnter ), NULL, this );

    edit_axis_1_x->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1X ), NULL, this );
    edit_axis_1_y->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Y ), NULL, this );
    edit_axis_1_z->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Z ), NULL, this );
    edit_axis_1_angle->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Angle ), NULL, this );
    edit_axis_2_x->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2X ), NULL, this );
    edit_axis_2_y->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Y ), NULL, this );
    edit_axis_2_z->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Z ), NULL, this );
    edit_axis_2_angle->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Angle ), NULL, this );

    edit_axis_1_x->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1XEnter ), NULL, this );
    edit_axis_1_y->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1YEnter ), NULL, this );
    edit_axis_1_z->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1ZEnter ), NULL, this );
    edit_axis_1_angle->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1AngleEnter ), NULL, this );
    edit_axis_2_x->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2XEnter ), NULL, this );
    edit_axis_2_y->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2YEnter ), NULL, this );
    edit_axis_2_z->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2ZEnter ), NULL, this );
    edit_axis_2_angle->Connect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2AngleEnter ), NULL, this );



    check_invertable->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUpsideDown ), NULL, this );
    check_use_color->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUseColor ), NULL, this );



}

ViewCreatorDialogBase::~ViewCreatorDialogBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( ViewCreatorDialogBase::OnDialogClose ) );
	choice_image_source->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceImageSource ), NULL, this );
	choice_image->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceImage ), NULL, this );
    choice_test_image_source->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceTestImageSource ), NULL, this );
    choice_test_image->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( ViewCreatorDialogBase::onChoiceTestImage ), NULL, this );
	button_cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonCancelClicked ), NULL, this );
	button_save->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonSaveClicked ), NULL, this );
    button_start_test->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonStartTestClicked ), NULL, this );
    button_end_test->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onButtonEndTestClicked ), NULL, this );
    check_fix_current_image->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUseCurrentImage ), NULL, this );
    edit_upper_left_row->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftRow ), NULL, this );
    edit_upper_left_row->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftRowEnter ), NULL, this );
    edit_upper_left_column->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftColumn ), NULL, this );
    edit_upper_left_column->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextUpperLeftColumnEnter ), NULL, this );
    edit_lower_right_row->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextLowerRightRow ), NULL, this );
    edit_lower_right_row->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextLowerRightRowEnter ), NULL, this );
    edit_lower_right_column->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextLowerRightColumn ), NULL, this );
    edit_lower_right_column->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextLowerRightColumnEnter ), NULL, this );
    slider_upper_left_row->Disconnect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideUpperLeftRow), NULL, this);
    slider_upper_left_column->Disconnect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideUpperLeftColumn), NULL, this);
    slider_lower_right_row->Disconnect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideLowerRightRow), NULL, this);
    slider_lower_right_column->Disconnect(wxEVT_SCROLL_CHANGED, wxScrollEventHandler(ViewCreatorDialogBase::onSlideLowerRightColumn), NULL, this);


    edit_orientation_x->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationX ), NULL, this );
    edit_orientation_y->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationY ), NULL, this );
    edit_orientation_z->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextOrientationZ ), NULL, this );
    edit_score_2D->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextScore ), NULL, this );
    edit_vertical_offset->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextVerticalOffset ), NULL, this );
    edit_horizontal_offset->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextHorizontalOffset ), NULL, this );
    edit_depth->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextDepth ), NULL, this );
    edit_fern_number->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextFernNumber ), NULL, this );
    edit_patch_size->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextPatchSize ), NULL, this );
    edit_min_scale->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextMinScale ), NULL, this );
    edit_max_scale->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextMaxScale ), NULL, this );

    edit_orientation_x->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationXEnter ), NULL, this );
    edit_orientation_y->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationYEnter ), NULL, this );
    edit_orientation_z->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextOrientationZEnter ), NULL, this );
    edit_score_2D->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextScoreEnter ), NULL, this );
    edit_vertical_offset->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextVerticalOffsetEnter ), NULL, this );
    edit_horizontal_offset->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextHorizontalOffsetEnter ), NULL, this );
    edit_depth->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextDepthEnter ), NULL, this );
    edit_fern_number->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextFernNumberEnter ), NULL, this );
    edit_patch_size->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextPatchSizeEnter ), NULL, this );
    edit_min_scale->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextMinScaleEnter ), NULL, this );
    edit_max_scale->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextMaxScaleEnter ), NULL, this );

    edit_axis_1_x->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1X ), NULL, this );
    edit_axis_1_y->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Y ), NULL, this );
    edit_axis_1_z->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Z ), NULL, this );
    edit_axis_1_angle->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis1Angle ), NULL, this );
    edit_axis_2_x->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2X ), NULL, this );
    edit_axis_2_y->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Y ), NULL, this );
    edit_axis_2_z->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Z ), NULL, this );
    edit_axis_2_angle->Disconnect(wxEVT_KILL_FOCUS, wxFocusEventHandler( ViewCreatorDialogBase::onEditTextAxis2Angle ), NULL, this );

    edit_axis_1_x->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1XEnter ), NULL, this );
    edit_axis_1_y->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1YEnter ), NULL, this );
    edit_axis_1_z->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1ZEnter ), NULL, this );
    edit_axis_1_angle->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis1AngleEnter ), NULL, this );
    edit_axis_2_x->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2XEnter ), NULL, this );
    edit_axis_2_y->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2YEnter ), NULL, this );
    edit_axis_2_z->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2ZEnter ), NULL, this );
    edit_axis_2_angle->Disconnect(wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( ViewCreatorDialogBase::onEditTextAxis2AngleEnter ), NULL, this );

    check_invertable->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUpsideDown ), NULL, this );
    check_use_color->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ViewCreatorDialogBase::onCheckUseColor ), NULL, this );
}
