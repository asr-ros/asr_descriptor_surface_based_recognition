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


#ifndef MAIN_DIALOG_H_
#define MAIN_DIALOG_H_

#include "Main_Dialog_Base.h"
#include <wx/wx.h>

/**
*   This class represents the dialog which is shown after the start of the application
*/
class Main_Dialog : public MainDialogBase
{

protected:

    /**
    *   Called when the dialog is being closed
    */
    void onDialogClose(wxCloseEvent &event);

    /**
    *   Called when an object-model is selected from the spinner
    */
    void onObjectModelSelected(wxCommandEvent &event);

    /**
    *   Called when the cancel-button is being pressed
    */
    void onCancelPressed( wxCommandEvent& event );

    /**
    *   Called when the next-button is being pressed
    */
    void onNextPressed( wxCommandEvent& event );

    /**
    *   The following functions are called when the focus of the edit-text-fields is being killed
    */
    void onKillFocusName(wxFocusEvent &event);
    void onKillFocusOrientationX(wxFocusEvent &event);
    void onKillFocusOrientationY(wxFocusEvent &event);
    void onKillFocusOrientationZ(wxFocusEvent &event);
    void onKillFocusDiameter(wxFocusEvent &event);
    void onKillFocusScore3D(wxFocusEvent &event);

    /**
    *   The following functions are called when enter is being pressed while a edit-text-field is focused
    */
    void onTextEnterName(wxCommandEvent &event);
    void onTextEnterOrientationX(wxCommandEvent &event);
    void onTextEnterOrientationY(wxCommandEvent &event);
    void onTextEnterOrientationZ(wxCommandEvent &event);
    void onTextEnterDiameter(wxCommandEvent &event);
    void onTextEnterScore3D(wxCommandEvent &event);



public:

    /**
    *   \brief The constructor of this class
    *
    *   \param parent       The parent window of this dialog
    */
    Main_Dialog(wxWindow* parent);

    /**
    *   \brief The extended constructor of this class used when back is pressed on DescModelsDialog
    *
    *   \param parent           The parent window of this dialog
    *   \param name             The name of the object which is being trained
    *   \param object_model     The selected object-model
    *   \param mesh             The selected mesh used for visualisation
    *   \param rotation_type    The rotation type of the model
    *   \param orientation_x    The x-component of the model's orientation
    *   \param orientation_y    The y-component of the model's orientation
    *   \param orientation_z    The z-component of the model's orientation
    *   \param diameter         The maximum diameter of the given object_model
    *   \param score_3D         The score used for 3D-recognition
    */
    Main_Dialog(wxWindow* parent, std::string name, std::string object_model, std::string mesh, std::string rotation_type, std::string orientation_x, std::string orientation_y, std::string orientation_z, std::string diameter, std::string score_3D);
};


#endif //MAIN_DIALOG_H_


