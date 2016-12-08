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


#ifndef ABORT_DIALOG_H_
#define ABORT_DIALOG_H_

#include "Create_Test_Model_Dialog_Base.h"
#include <HalconCpp.h>

/**
*  This class represents the dialog which is shown when a test-descriptor-model is created with the given parameters used for testing
*/
class CreateTestModelDialog : public Create_Test_Model_Dialog_Base
{

private:

    /** Indicates whether the creation of the test model was successful */
    bool success;

    /**
    * \brief Creates the test-model with the given parameters
    *
    * \param desc_model         A pointer to the descriptor-model which will be created by this function
    * \param image              The view of the object the model is created with
    * \param descParamNames     The names of the descriptor-model-parameters
    * \param descParamValues    The values of the parameters given in descParamNames
    * \param use_color          Indicates whether a colored image should be used for the creation of the model
    */
    void createDescriptorModel(HalconCpp::HDescriptorModel *desc_model, HalconCpp::HImage image, HalconCpp::HTuple descParamNames, HalconCpp::HTuple descParamValues, bool use_color);


protected:

    /**
    *   Called when the abort button is pushed
    */
    void onButtonAbortClicked(wxCommandEvent &event);

    /**
    *   Called when the dialog is being closed
    */
    void onCloseClicked(wxCloseEvent &event);

public:

    /**
    * \brief The constructor of this class
    *
    * \param parent             The parent window of this dialog
    * \param desc_model         A pointer to the descriptor-model which will be created by class
    * \param image              The view of the object the model is created with
    * \param depth              The depth of the the classification ferns used by the descriptor-based recognition
    * \param number_ferns       The number of ferns used by the descriptor-based recognition
    * \param patch_size         The patch size used by the descriptor-based recognition
    * \param min_scale          The minimal scale of the view the model is created from
    * \param max_scale          The maximal scale of the view the model is created from
    * \param use_color          Indicates whether a colored image should be used for the creation of the model
    */
    CreateTestModelDialog(wxWindow* parent, HalconCpp::HDescriptorModel *desc_model, HalconCpp::HImage image, int depth, int number_ferns, int patch_size, double min_scale, double max_scale, bool use_color);
};


#endif //ABORT_DIALOG_H_

