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


#ifndef CREATE_OUTPUT_FILES_DIALOG_H_
#define CREATE_OUTPUT_FILES_DIALOG_H_

#include "Create_Output_Files_Dialog_Base.h"
#include "Desc_Models_Dialog.h"


/**
*   This class represents the dialog which is shown when the output files of the trained data are created
*
*/
class CreateOutputFilesDialog : public Create_Output_Files_Dialog_Base
{
private:

    /** The list containing the parameters of each trained 2D-view of the object */
    std::vector<ViewParamsWrapper> views;

    /** The parameters of the trained object (mostly for 3D-recognition) */
    std::vector<std::string> object_params;

    /** Indicates whether the output files were created successfully */
    bool success;

protected:

    /** Called when this dialog is being closed */
    void onDialogClose( wxCloseEvent& event );

    /** Called when the button of this dialog is being pushed */
    void onButtonDoneClicked( wxCommandEvent& event );

public:

    /**
    *  \brief Constructor of this class
    */
    CreateOutputFilesDialog(DescModelsDialog* parent_dialog);

    /**
    *  \brief Creates the output files with the set parameters
    */
    void createOutputFiles();

};


#endif //CREATE_OUTPUT_FILES_DIALOG_H_


