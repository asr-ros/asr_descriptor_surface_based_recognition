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


#include "descriptor_surface_based_trainer/Create_Test_Model_Dialog_Base.h"


Create_Test_Model_Dialog_Base::Create_Test_Model_Dialog_Base( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer1->Add( 10, 0, 0, 0, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer4->Add( 0, 10, 0, 0, 5 );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	
	bSizer2->Add( 0, 0, 1, wxEXPAND, 5 );
	
	label_message = new wxStaticText( this, wxID_ANY, wxT("Creating Descriptor model. Please wait..."), wxDefaultPosition, wxSize( -1,50 ), 0 );
	label_message->Wrap( -1 );
	bSizer2->Add( label_message, 0, wxALL|wxEXPAND, 5 );
	
	
	bSizer2->Add( 0, 0, 1, wxEXPAND, 5 );
	
	bSizer4->Add( bSizer2, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer3->Add( 0, 0, 1, wxEXPAND, 5 );
	
	button_abort = new wxButton( this, wxID_ANY, wxT("Abort"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer3->Add( button_abort, 0, wxALL, 5 );
	
	bSizer4->Add( bSizer3, 0, wxEXPAND, 5 );
	
	bSizer1->Add( bSizer4, 1, wxEXPAND, 5 );
	
	
	bSizer1->Add( 10, 0, 0, 0, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
    this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Create_Test_Model_Dialog_Base::onCloseClicked ) );
    button_abort->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Create_Test_Model_Dialog_Base::onButtonAbortClicked ), NULL, this );
}

Create_Test_Model_Dialog_Base::~Create_Test_Model_Dialog_Base()
{
	// Disconnect Events
    this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Create_Test_Model_Dialog_Base::onCloseClicked ) );
    button_abort->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Create_Test_Model_Dialog_Base::onButtonAbortClicked ), NULL, this );
}
