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


#include "descriptor_surface_based_trainer/Main_Dialog_Base.h"


MainDialogBase::MainDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer2->Add( 0, 10, 0, 0, 5 );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer10->Add( 20, 0, 0, 0, 5 );
	
    label_caption = new wxStaticText( this, wxID_ANY, wxT("3D recognition parameters"), wxDefaultPosition, wxDefaultSize, 0 );
	label_caption->Wrap( -1 );
	label_caption->SetFont( wxFont( 13, 70, 90, 92, false, wxEmptyString ) );
	
	bSizer10->Add( label_caption, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer2->Add( bSizer10, 0, wxEXPAND, 5 );
	
	
	bSizer2->Add( 0, 10, 0, 0, 5 );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	label_name = new wxStaticText( this, wxID_ANY, wxT("Name"), wxDefaultPosition, wxDefaultSize, 0 );
	label_name->Wrap( -1 );
	bSizer3->Add( label_name, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer3->Add( 0, 0, 1, wxEXPAND, 5 );
	
	edit_name = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 300,-1 ), wxTE_PROCESS_ENTER );
	bSizer3->Add( edit_name, 0, wxALL|wxEXPAND|wxALIGN_CENTER_VERTICAL, 5 );
	
	
    bSizer3->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxHORIZONTAL );
	
	label_object_model = new wxStaticText( this, wxID_ANY, wxT("Object-Model"), wxDefaultPosition, wxDefaultSize, 0 );
	label_object_model->Wrap( -1 );
	bSizer4->Add( label_object_model, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer4->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxArrayString choice_object_modelChoices;
	choice_object_model = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 300,-1 ), choice_object_modelChoices, 0 );
	choice_object_model->SetSelection( 0 );
	bSizer4->Add( choice_object_model, 0, wxEXPAND|wxALIGN_CENTER_VERTICAL|wxALL, 5 );
	
	
    bSizer4->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer4, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer41;
	bSizer41 = new wxBoxSizer( wxHORIZONTAL );
	
    label_mesh = new wxStaticText( this, wxID_ANY, wxT("Textured object-model (Visualisation)"), wxDefaultPosition, wxDefaultSize, 0 );
	label_mesh->Wrap( -1 );
	bSizer41->Add( label_mesh, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer41->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxArrayString choice_meshChoices;
	choice_mesh = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 300,-1 ), choice_meshChoices, 0 );
	choice_mesh->SetSelection( 0 );
	bSizer41->Add( choice_mesh, 0, wxEXPAND|wxALIGN_CENTER_VERTICAL|wxALL, 5 );
	
	
    bSizer41->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer41, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer411;
	bSizer411 = new wxBoxSizer( wxHORIZONTAL );
	
	label_rotation_type = new wxStaticText( this, wxID_ANY, wxT("Rotation-Model-Type"), wxDefaultPosition, wxDefaultSize, 0 );
	label_rotation_type->Wrap( -1 );
	bSizer411->Add( label_rotation_type, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer411->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxString choice_rotation_typeChoices[] = { wxT("No Rotation"), wxT("Cylinder"), wxT("Sphere") };
	int choice_rotation_typeNChoices = sizeof( choice_rotation_typeChoices ) / sizeof( wxString );
	choice_rotation_type = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 300,-1 ), choice_rotation_typeNChoices, choice_rotation_typeChoices, 0 );
	choice_rotation_type->SetSelection( 0 );
	bSizer411->Add( choice_rotation_type, 0, wxEXPAND|wxALIGN_CENTER_VERTICAL|wxALL, 5 );
	
	
    bSizer411->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer411, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	label_orientation = new wxStaticText( this, wxID_ANY, wxT("Model-Orientation"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation->Wrap( -1 );
	bSizer12->Add( label_orientation, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer12->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_orientation_x = new wxStaticText( this, wxID_ANY, wxT("x:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_x->Wrap( -1 );
	bSizer12->Add( label_orientation_x, 0, wxALIGN_CENTER_VERTICAL|wxTOP|wxBOTTOM, 5 );
	
	edit_orientation_x = new wxTextCtrl( this, wxID_ANY, wxT("1.0"), wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer12->Add( edit_orientation_x, 0, wxALL, 5 );
	
	label_orientation_y = new wxStaticText( this, wxID_ANY, wxT("y:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_y->Wrap( -1 );
	bSizer12->Add( label_orientation_y, 0, wxALIGN_CENTER_VERTICAL|wxTOP|wxBOTTOM|wxLEFT, 5 );
	
	edit_orientation_y = new wxTextCtrl( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer12->Add( edit_orientation_y, 0, wxALL, 5 );
	
	label_orientation_z = new wxStaticText( this, wxID_ANY, wxT("z:"), wxDefaultPosition, wxDefaultSize, 0 );
	label_orientation_z->Wrap( -1 );
	bSizer12->Add( label_orientation_z, 0, wxALIGN_CENTER_VERTICAL|wxTOP|wxBOTTOM, 5 );
	
	edit_orientation_z = new wxTextCtrl( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxSize( 70,-1 ), wxTE_PROCESS_ENTER );
	bSizer12->Add( edit_orientation_z, 0, wxALL, 5 );
	
	
    bSizer12->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer12, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxHORIZONTAL );
	
	label_diameter = new wxStaticText( this, wxID_ANY, wxT("Diameter"), wxDefaultPosition, wxDefaultSize, 0 );
	label_diameter->Wrap( -1 );
	bSizer13->Add( label_diameter, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer13->Add( 0, 0, 1, wxEXPAND, 5 );
	
	edit_diameter = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_PROCESS_ENTER );
	bSizer13->Add( edit_diameter, 0, wxALL|wxEXPAND, 5 );
	
	
    bSizer13->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer13, 0, wxEXPAND|wxLEFT, 5 );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	label_score_3D = new wxStaticText( this, wxID_ANY, wxT("Score 3D"), wxDefaultPosition, wxDefaultSize, 0 );
	label_score_3D->Wrap( -1 );
	bSizer14->Add( label_score_3D, 0, wxALL|wxALIGN_BOTTOM, 5 );
	
	
	bSizer14->Add( 0, 0, 1, wxEXPAND, 5 );
	
	edit_score_3D = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_PROCESS_ENTER );
	bSizer14->Add( edit_score_3D, 0, wxALL|wxEXPAND, 5 );
	
	
    bSizer14->Add( 20, 0, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer14, 0, wxLEFT|wxEXPAND, 5 );
	
	
	bSizer2->Add( 0, 5, 0, 0, 5 );
	
	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer15->Add( 0, 0, 1, wxEXPAND, 5 );
	
	button_cancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer15->Add( button_cancel, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	button_next = new wxButton( this, wxID_ANY, wxT("Next >"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer15->Add( button_next, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer2->Add( bSizer15, 1, wxEXPAND, 5 );
	
	
	bSizer2->Add( 0, 10, 0, 0, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
    this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainDialogBase::onDialogClose ) );
    edit_name->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusName ), NULL, this );
    edit_name->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterName ), NULL, this );
    choice_object_model->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialogBase::onObjectModelSelected ), NULL, this );
    edit_orientation_x->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationX ), NULL, this );
    edit_orientation_x->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationX ), NULL, this );
    edit_orientation_y->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationY ), NULL, this );
    edit_orientation_y->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationY ), NULL, this );
    edit_orientation_z->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationZ ), NULL, this );
    edit_orientation_z->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationZ ), NULL, this );
    edit_diameter->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusDiameter ), NULL, this );
    edit_diameter->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterDiameter ), NULL, this );
    edit_score_3D->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusScore3D ), NULL, this );
    edit_score_3D->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterScore3D ), NULL, this );
    button_cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialogBase::onCancelPressed ), NULL, this );
    button_next->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialogBase::onNextPressed ), NULL, this );
}

MainDialogBase::~MainDialogBase()
{
	// Disconnect Events
    this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainDialogBase::onDialogClose ) );
    edit_name->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusName ), NULL, this );
    edit_name->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterName ), NULL, this );
    choice_object_model->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialogBase::onObjectModelSelected ), NULL, this );
    edit_orientation_x->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationX ), NULL, this );
    edit_orientation_x->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationX ), NULL, this );
    edit_orientation_y->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationY ), NULL, this );
    edit_orientation_y->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationY ), NULL, this );
    edit_orientation_z->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusOrientationZ ), NULL, this );
    edit_orientation_z->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterOrientationZ ), NULL, this );
    edit_diameter->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusDiameter ), NULL, this );
    edit_diameter->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterDiameter ), NULL, this );
    edit_score_3D->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( MainDialogBase::onKillFocusScore3D ), NULL, this );
    edit_score_3D->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialogBase::onTextEnterScore3D ), NULL, this );
    button_cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialogBase::onCancelPressed ), NULL, this );
    button_next->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialogBase::onNextPressed ), NULL, this );
}
