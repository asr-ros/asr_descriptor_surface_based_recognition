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


#include "descriptor_surface_based_trainer/Desc_Models_Dialog_Base.h"


DescModelsDialogBase::DescModelsDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );

    bSizer1->Add(0, 10, 0, 0, 5);

    wxBoxSizer* bSizer201;
    bSizer201 = new wxBoxSizer(wxHORIZONTAL);

    bSizer201->Add( 20, 0, 0, 0, 5);

    label_caption = new wxStaticText(this, wxID_ANY, wxT("2D recognition parameters"), wxDefaultPosition, wxDefaultSize, 0);
    label_caption->Wrap(-1);
    label_caption->SetFont(wxFont(13, 70, 90, 92, false, wxEmptyString));

    bSizer201->Add(label_caption, 1, wxALL, 5);

    bSizer1->Add(bSizer201, 0, wxEXPAND, 5);

    bSizer1->Add(0, 5, 0, 0, 5);
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );
	
	list_box_views = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxSize( 200,-1 ), 0, NULL, 0 ); 
	bSizer2->Add( list_box_views, 0, wxEXPAND|wxALL, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer4->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxHORIZONTAL );
	
	button_add_model = new wxButton( this, wxID_ANY, wxT("+"), wxDefaultPosition, wxSize( 30,-1 ), 0 );
	bSizer5->Add( button_add_model, 0, wxALL, 5 );
	
	button_delete_model = new wxButton( this, wxID_ANY, wxT("-"), wxDefaultPosition, wxSize( 30,-1 ), 0 );
	bSizer5->Add( button_delete_model, 0, wxALL, 5 );

	bSizer5->Add( 0, 0, 1, wxEXPAND, 5 );

    button_edit_model = new wxButton( this, wxID_ANY, wxT("Edit"), wxDefaultPosition, wxDefaultSize, 0 );
    bSizer5->Add( button_edit_model, 0, wxALL, 5 );
	
	bSizer4->Add( bSizer5, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	label_orientation = new wxStaticText( this, wxID_ANY, wxT("Orientation:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_orientation->Wrap( -1 );
	label_orientation->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer6->Add( label_orientation, 0, wxALL, 5 );
	
	
	bSizer6->Add( 30, 0, 0, 0, 5 );
	
	label_orientation_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_x->Wrap( -1 );
	bSizer6->Add( label_orientation_x, 0, wxALL, 5 );
	
	label_orientation_x_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_orientation_x_value->Wrap( -1 );
	bSizer6->Add( label_orientation_x_value, 0, wxALL, 5 );
	
	label_orientation_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_y->Wrap( -1 );
	bSizer6->Add( label_orientation_y, 0, wxALL, 5 );
	
	label_orientation_y_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_orientation_y_value->Wrap( -1 );
	bSizer6->Add( label_orientation_y_value, 0, wxALL, 5 );
	
	label_orientation_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_z->Wrap( -1 );
	bSizer6->Add( label_orientation_z, 0, wxALL, 5 );
	
	label_orientation_z_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_orientation_z_value->Wrap( -1 );
	bSizer6->Add( label_orientation_z_value, 0, wxALL, 5 );
	
	bSizer4->Add( bSizer6, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer61;
	bSizer61 = new wxBoxSizer( wxHORIZONTAL );
	
	label_axis_1 = new wxStaticText( this, wxID_ANY, wxT("Axis 1:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_axis_1->Wrap( -1 );
	label_axis_1->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer61->Add( label_axis_1, 0, wxALL, 5 );
	
	
	bSizer61->Add( 30, 0, 0, 0, 5 );
	
	label_axis_1_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_x->Wrap( -1 );
	bSizer61->Add( label_axis_1_x, 0, wxALL, 5 );
	
	label_axis_1_x_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_1_x_value->Wrap( -1 );
	bSizer61->Add( label_axis_1_x_value, 0, wxALL, 5 );
	
	label_axis_1_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_y->Wrap( -1 );
	bSizer61->Add( label_axis_1_y, 0, wxALL, 5 );
	
	label_axis_1_y_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_1_y_value->Wrap( -1 );
	bSizer61->Add( label_axis_1_y_value, 0, wxALL, 5 );
	
	label_axis_1_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_1_z->Wrap( -1 );
	bSizer61->Add( label_axis_1_z, 0, wxALL, 5 );
	
	label_axis_1_z_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_1_z_value->Wrap( -1 );
	bSizer61->Add( label_axis_1_z_value, 0, wxALL, 5 );

    label_axis_1_angle = new wxStaticText(this, wxID_ANY, wxT("angle:"), wxDefaultPosition, wxDefaultSize, 0);
    label_axis_1_angle->Wrap(-1);
    bSizer61->Add(label_axis_1_angle, 0, wxALL, 5);

    label_axis_1_angle_value = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(70, -1), 0);
    label_axis_1_angle_value->Wrap(-1);
    bSizer61->Add(label_axis_1_angle_value, 0, wxALL, 5);


	
	bSizer4->Add( bSizer61, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer611;
	bSizer611 = new wxBoxSizer( wxHORIZONTAL );
	
	label_axis_2 = new wxStaticText( this, wxID_ANY, wxT("Axis 2:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_axis_2->Wrap( -1 );
	label_axis_2->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer611->Add( label_axis_2, 0, wxALL, 5 );
	
	
	bSizer611->Add( 30, 0, 0, 0, 5 );
	
	label_axis_2_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_x->Wrap( -1 );
	bSizer611->Add( label_axis_2_x, 0, wxALL, 5 );
	
	label_axis_2_x_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_2_x_value->Wrap( -1 );
	bSizer611->Add( label_axis_2_x_value, 0, wxALL, 5 );
	
	label_axis_2_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_y->Wrap( -1 );
	bSizer611->Add( label_axis_2_y, 0, wxALL, 5 );
	
	label_axis_2_y_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_2_y_value->Wrap( -1 );
	bSizer611->Add( label_axis_2_y_value, 0, wxALL, 5 );
	
	label_axis_2_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_axis_2_z->Wrap( -1 );
	bSizer611->Add( label_axis_2_z, 0, wxALL, 5 );
	
	label_axis_2_z_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), 0 );
	label_axis_2_z_value->Wrap( -1 );
	bSizer611->Add( label_axis_2_z_value, 0, wxALL, 5 );

    label_axis_2_angle = new wxStaticText(this, wxID_ANY, wxT("angle:"), wxDefaultPosition, wxDefaultSize, 0);
    label_axis_2_angle->Wrap(-1);
    bSizer611->Add(label_axis_2_angle, 0, wxALL, 5);

    label_axis_2_angle_value = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(70, -1), 0);
    label_axis_2_angle_value->Wrap(-1);
    bSizer611->Add(label_axis_2_angle_value, 0, wxALL, 5);
	
	bSizer4->Add( bSizer611, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer( wxHORIZONTAL );
	
	label_score_2D = new wxStaticText( this, wxID_ANY, wxT("Score:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_score_2D->Wrap( -1 );
	label_score_2D->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer15->Add( label_score_2D, 0, wxALL, 5 );
	
	
	bSizer15->Add( 30, 0, 0, 0, 5 );
	
	label_score_2D_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_score_2D_value->Wrap( -1 );
	bSizer15->Add( label_score_2D_value, 0, wxALL, 5 );
	
	
	bSizer15->Add( 100, 0, 1, wxEXPAND, 5 );
	
	bSizer4->Add( bSizer15, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer16;
	bSizer16 = new wxBoxSizer( wxHORIZONTAL );
	
	label_use_color = new wxStaticText( this, wxID_ANY, wxT("Color:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_use_color->Wrap( -1 );
	label_use_color->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer16->Add( label_use_color, 0, wxALL, 5 );
	
	
	bSizer16->Add( 30, 0, 0, 0, 5 );
	
	label_use_color_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_use_color_value->Wrap( -1 );
	bSizer16->Add( label_use_color_value, 0, wxALL, 5 );
	
	
	bSizer16->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_invertible = new wxStaticText( this, wxID_ANY, wxT("Can be upside-down:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_invertible->Wrap( -1 );
	label_invertible->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer16->Add( label_invertible, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer16->Add( 30, 0, 0, 0, 5 );
	
	label_invertible_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_invertible_value->Wrap( -1 );
	bSizer16->Add( label_invertible_value, 0, wxALL, 5 );
	
	
	bSizer16->Add( 50, 0, 0, 0, 5 );
	
	bSizer4->Add( bSizer16, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer18;
	bSizer18 = new wxBoxSizer( wxHORIZONTAL );
	
	label_vertical_offset = new wxStaticText( this, wxID_ANY, wxT("Vertical offset:"), wxDefaultPosition, wxSize( 130,-1 ), 0 );
	label_vertical_offset->Wrap( -1 );
	label_vertical_offset->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer18->Add( label_vertical_offset, 0, wxALL, 5 );
	
	
	bSizer18->Add( 30, 0, 0, 0, 5 );
	
	label_vertical_offset_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_vertical_offset_value->Wrap( -1 );
	bSizer18->Add( label_vertical_offset_value, 0, wxALL, 5 );
	
	
	bSizer18->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_horizontal_offset = new wxStaticText( this, wxID_ANY, wxT("Horizontal offset:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_horizontal_offset->Wrap( -1 );
	label_horizontal_offset->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer18->Add( label_horizontal_offset, 0, wxALL, 5 );
	
	
	bSizer18->Add( 30, 0, 0, 0, 5 );
	
	label_horizontal_offset_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_horizontal_offset_value->Wrap( -1 );
	bSizer18->Add( label_horizontal_offset_value, 0, wxALL, 5 );
	
	
	bSizer18->Add( 50, 0, 0, 0, 5 );
	
	bSizer4->Add( bSizer18, 0, wxEXPAND, 5 );
	
	
	bSizer4->Add( 0, 20, 0, 0, 5 );
	
	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer13->Add( 5, 0, 0, 0, 5 );
	
	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer20->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_panel1 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 300,225 ), wxTAB_TRAVERSAL );
	
	bSizer20->Add( m_panel1, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

    image = new wxImagePanel(m_panel1, 225, 300);
	
	
	bSizer20->Add( 0, 0, 1, wxEXPAND, 5 );
	
    bSizer13->Add( bSizer20, 0, wxEXPAND, 5 );
	
	
	bSizer13->Add( 20, 0, 0, 0, 5 );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer14->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer151;
	bSizer151 = new wxBoxSizer( wxHORIZONTAL );
	
	label_depth = new wxStaticText( this, wxID_ANY, wxT("Depth:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
	label_depth->Wrap( -1 );
	label_depth->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer151->Add( label_depth, 0, wxALL, 5 );
	
	
	bSizer151->Add( 30, 0, 0, 0, 5 );
	
    label_depth_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
    label_depth_value->Wrap( -1 );
    bSizer151->Add( label_depth_value, 0, wxALL, 5 );
	
	bSizer14->Add( bSizer151, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer161;
	bSizer161 = new wxBoxSizer( wxHORIZONTAL );
	
	label_number_ferns = new wxStaticText( this, wxID_ANY, wxT("Fern number:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
	label_number_ferns->Wrap( -1 );
	label_number_ferns->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer161->Add( label_number_ferns, 0, wxALL, 5 );
	
	
	bSizer161->Add( 30, 0, 0, 0, 5 );
	
	label_number_ferns_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_number_ferns_value->Wrap( -1 );
	bSizer161->Add( label_number_ferns_value, 0, wxALL, 5 );
	
	bSizer14->Add( bSizer161, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer17;
	bSizer17 = new wxBoxSizer( wxHORIZONTAL );
	
	label_patch_size = new wxStaticText( this, wxID_ANY, wxT("Patch size:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
	label_patch_size->Wrap( -1 );
	label_patch_size->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer17->Add( label_patch_size, 0, wxALL, 5 );
	
	
	bSizer17->Add( 30, 0, 0, 0, 5 );
	
	label_patch_size_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_patch_size_value->Wrap( -1 );
	bSizer17->Add( label_patch_size_value, 0, wxALL, 5 );
	
	bSizer14->Add( bSizer17, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer181;
	bSizer181 = new wxBoxSizer( wxHORIZONTAL );
	
	label_min_scale = new wxStaticText( this, wxID_ANY, wxT("Min. scale:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
	label_min_scale->Wrap( -1 );
	label_min_scale->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer181->Add( label_min_scale, 0, wxALL, 5 );
	
	
	bSizer181->Add( 30, 0, 0, 0, 5 );
	
	label_min_scale_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_min_scale_value->Wrap( -1 );
	bSizer181->Add( label_min_scale_value, 0, wxALL, 5 );
	
	bSizer14->Add( bSizer181, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxHORIZONTAL );
	
	label_max_scale = new wxStaticText( this, wxID_ANY, wxT("Max. scale:"), wxDefaultPosition, wxSize( 120,-1 ), 0 );
	label_max_scale->Wrap( -1 );
	label_max_scale->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer19->Add( label_max_scale, 0, wxALL, 5 );
	
	
	bSizer19->Add( 30, 0, 0, 0, 5 );
	
	label_max_scale_value = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 50,-1 ), 0 );
	label_max_scale_value->Wrap( -1 );
	bSizer19->Add( label_max_scale_value, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer14->Add( bSizer19, 0, wxEXPAND, 5 );
	
	
	bSizer14->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer13->Add( bSizer14, 1, wxEXPAND, 5 );
	
	
	bSizer13->Add( 50, 0, 0, 0, 5 );
	
	bSizer4->Add( bSizer13, 0, wxEXPAND, 5 );
	
	
	bSizer4->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer4, 1, wxEXPAND, 5 );
	
	bSizer1->Add( bSizer2, 1, wxEXPAND, 5 );
	
    bSizer1->Add(0, 5, 0, 0, 5);

	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer3->Add( 0, 0, 1, wxEXPAND, 5 );

    button_back = new wxButton(this, wxID_ANY, wxT("< Back"), wxDefaultPosition, wxDefaultSize, 0);
    bSizer3->Add(button_back, 0, wxALL, 5);
	
    button_cancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
    bSizer3->Add( button_cancel, 0, wxALL, 5 );
	
	button_finish = new wxButton( this, wxID_ANY, wxT("Finish"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer3->Add( button_finish, 0, wxALL, 5 );
	
	bSizer1->Add( bSizer3, 0, wxEXPAND, 5 );

    bSizer1->Add(0, 10, 0, 0, 5);
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( DescModelsDialogBase::OnDialogClose ) );
	list_box_views->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( DescModelsDialogBase::onListBoxSelected ), NULL, this );
	button_add_model->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onAddModelClicked ), NULL, this );
	button_delete_model->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onDeleteModelClicked ), NULL, this );
    button_cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onCancelClicked ), NULL, this );
	button_finish->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onFinishClicked ), NULL, this );
    button_back->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onBackClicked ), NULL, this );
    button_edit_model->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onEditModelClicked ), NULL, this );

    image->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(DescModelsDialogBase::onImageClicked), NULL, this);
    image->Connect(wxEVT_RIGHT_UP, wxMouseEventHandler(DescModelsDialogBase::onImageClickedRight), NULL, this);
}

DescModelsDialogBase::~DescModelsDialogBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( DescModelsDialogBase::OnDialogClose ) );
	list_box_views->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( DescModelsDialogBase::onListBoxSelected ), NULL, this );
	button_add_model->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onAddModelClicked ), NULL, this );
	button_delete_model->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onDeleteModelClicked ), NULL, this );
    button_cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onCancelClicked ), NULL, this );
	button_finish->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onFinishClicked ), NULL, this );
    button_back->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onBackClicked ), NULL, this );
    button_edit_model->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DescModelsDialogBase::onEditModelClicked ), NULL, this );

    image->Disconnect(wxEVT_LEFT_UP, wxMouseEventHandler(DescModelsDialogBase::onImageClicked), NULL, this);
    image->Disconnect(wxEVT_RIGHT_UP, wxMouseEventHandler(DescModelsDialogBase::onImageClickedRight), NULL, this);
}
