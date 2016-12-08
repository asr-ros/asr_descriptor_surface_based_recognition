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


#include "descriptor_surface_based_trainer/Descriptor_Surface_Based_Trainer.h"

IMPLEMENT_APP(Descriptor_Surface_Based_Trainer)


Descriptor_Surface_Based_Trainer::Descriptor_Surface_Based_Trainer()
{
}

Descriptor_Surface_Based_Trainer::~Descriptor_Surface_Based_Trainer()
{
}

bool Descriptor_Surface_Based_Trainer::OnInit()
{
    local_argv =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i ) {
        local_argv[ i ] = strdup( wxString( argv[ i ] ).char_str() );
    }

    ros::init(argc, local_argv, "Descriptor_Surface_Based_Trainer");

    wxInitAllImageHandlers();

    Main_Dialog *mainDialog = new Main_Dialog( (wxWindow*)NULL);
    mainDialog ->Show();
    SetTopWindow( mainDialog );


    return true;
}

int Descriptor_Surface_Based_Trainer::OnExit() {
    for ( int i = 0; i < argc; ++i ) {
        free( local_argv[ i ] );
    }
    delete [] local_argv;

    return 0;
}



