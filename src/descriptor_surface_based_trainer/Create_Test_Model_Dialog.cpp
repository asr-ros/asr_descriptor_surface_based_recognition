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


#include "descriptor_surface_based_trainer/Create_Test_Model_Dialog.h"
#include <wx/wx.h>
#include <boost/thread.hpp>



void CreateTestModelDialog::createDescriptorModel(HalconCpp::HDescriptorModel *desc_model, HalconCpp::HImage image, HalconCpp::HTuple descParamNames, HalconCpp::HTuple descParamValues, bool use_color) {
    try {
        if (!use_color) {
            image = image.Rgb1ToGray();
        }
        *desc_model = HalconCpp::HDescriptorModel(image, "harris", HalconCpp::HTuple(), HalconCpp::HTuple(), descParamNames, descParamValues, 42);
    } catch(HalconCpp::HException exc) {
        button_abort->Enable(true);
        return;
    }
    success = true;
    button_abort->Enable(true);
}



CreateTestModelDialog::CreateTestModelDialog(wxWindow* parent, HalconCpp::HDescriptorModel *desc_model, HalconCpp::HImage image, int depth, int number_ferns, int patch_size, double min_scale, double max_scale, bool use_color)
    : Create_Test_Model_Dialog_Base(parent), success(false)
{
    label_message->SetLabel(wxT("Creating new Descriptor model..."));

    button_abort->SetLabel(wxT("Done"));
    button_abort->Enable(false);

    HalconCpp::HTuple descparamname("depth");
    descparamname.Append("number_ferns");
    descparamname.Append("patch_size");
    descparamname.Append("max_scale");
    descparamname.Append("min_scale");
    HalconCpp::HTuple descparamvalue(depth);
    descparamvalue.Append(number_ferns);
    descparamvalue.Append(patch_size);
    descparamvalue.Append(max_scale);
    descparamvalue.Append(min_scale);

    boost::thread desc_mdl_thread(boost::bind(&CreateTestModelDialog::createDescriptorModel, this, desc_model, image, descparamname, descparamvalue, use_color));

}



void CreateTestModelDialog::onButtonAbortClicked(wxCommandEvent &event)
{

    if (success) {
        EndModal(wxID_OK);

    } else {
        EndModal(wxID_CANCEL);
    }
    Destroy();

}

void CreateTestModelDialog::onCloseClicked(wxCloseEvent &event)
{

}
