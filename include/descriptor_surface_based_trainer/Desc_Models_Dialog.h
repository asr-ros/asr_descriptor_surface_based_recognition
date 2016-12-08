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


#ifndef DESC_MODELS_DIALOG_H_
#define DESC_MODELS_DIALOG_H_

#include "Desc_Models_Dialog_Base.h"
#include "View_Params_Wrapper.h"
#include <wx/wx.h>
#include <vector>


/**
*  This class represents the dialog which shows the created views used for 2D-recognition
*/
class DescModelsDialog : public DescModelsDialogBase
{

private:

    /** The name of the trained object */
    std::string name;

    /** The pointcloud the surface-model used for 3D-recognition is created from */
    std::string object_model;

    /** The mesh used for visualisation of the recognized object */
    std::string mesh;

    /** The rotation type of this object (see Utils.h) */
    std::string rotation_type;

    /** The x-component of this object's orientation */
    std::string orientation_x;

    /** The y-component of this object's orientation */
    std::string orientation_y;

    /** The z-component of this object's orientation */
    std::string orientation_z;

    /** The diameter maximum diameter of the given object_model */
    std::string diameter;

    /** The threshold used for 3D-recognition to check whether a found object instance is valid */
    std::string score_3D;

    /** The list containing all currently created views of the object */
    std::vector<ViewParamsWrapper> views;

    /**
    * \brief Resets all the shown values to default
    */
    void resetGuiValues();


protected:

    /**
    *   Called when the dialog is being closed
    */
    void OnDialogClose( wxCloseEvent& event );

    /**
    *   Called when the add-model-button was pushed
    */
    void onAddModelClicked( wxCommandEvent& event );

    /**
    *   Called when the delete-model-button was pushed
    */
    void onDeleteModelClicked( wxCommandEvent& event );

    /**
    *   Called when the cancel-button was pushed
    */
    void onCancelClicked( wxCommandEvent& event );

    /**
    *   Called when the finish-button was pushed
    */
    void onFinishClicked( wxCommandEvent& event );

    /**
    *   Called when an item in the listbox is being selected
    */
    void onListBoxSelected(wxCommandEvent &event);

    /**
    *   Called when the back-button was pushed
    */
    void onBackClicked(wxCommandEvent &event);


    /**
    *   Called when the edit-model-button was pushed
    */
    void onEditModelClicked(wxCommandEvent &event);


    /**
    *   Called when the image-panel is being clicked with the left mouse button
    */
    void onImageClicked(wxMouseEvent &event);

    /**
    *   Called when the image-panel is being clicked with the right mouse button
    */
    void onImageClickedRight(wxMouseEvent &event);

public:

    /**
    * \brief The constructor of this class
    *
    * \param parent             The parent window of this dialog
    * \param name               --
    * \param image                |
    * \param depth                |
    * \param number_ferns         |-- see class members
    * \param patch_size           |
    * \param min_scale            |
    * \param max_scale            |
    * \param use_color          --
    */
    DescModelsDialog(wxWindow* parent, std::string name, std::string object_model, std::string mesh, std::string rotation_type, std::string orientation_x, std::string orientation_y, std::string orientation_z, std::string diameter, std::string score_3D);

    /**
    * \brief Returns the list of object parameters
    */
    std::vector<std::string>getObjectParameters();

    /**
    * \brief Returns the list of created 2D-views
    */
    std::vector<ViewParamsWrapper> getViews();
};


#endif //DESC_MODELS_DIALOG_H_



