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


#ifndef DESCRIPTOR_SURFACE_BASED_TRAINER_H_
#define DESCRIPTOR_SURFACE_BASED_TRAINER_H_

#include <wx/wx.h>
#include "ros/ros.h"
#include "Main_Dialog.h"

/**
*   The main class of this trainer
*/
class Descriptor_Surface_Based_Trainer : public wxApp {

public:

    /** The arguments given when the program is started */
    char** local_argv;

    /**
    *   \brief The constructor of this class
    */
    Descriptor_Surface_Based_Trainer();

    /**
    *   \brief The destructor of this class
    */
    virtual ~Descriptor_Surface_Based_Trainer();

    /**
    *   \brief Called when the application is initialized
    *
    *   \return     true to continue processing, false to exit the application immediately
    */
    virtual bool OnInit();

    /**
    *   \brief Called when the application is about to exit
    *
    *   \return     value is currently ignored (see wxwidgets documentation)
    */
    virtual int OnExit();
};

/** Macro used by wxwidgets */
DECLARE_APP(Descriptor_Surface_Based_Trainer)

#endif /* DESCRIPTOR_SURFACE_BASED_TRAINER_H_ */

