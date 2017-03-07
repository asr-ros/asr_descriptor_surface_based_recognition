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


#include "descriptor_surface_based_trainer/Main_Dialog.h"
#include "descriptor_surface_based_trainer/Desc_Models_Dialog.h"
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/regex.hpp>

#include <HalconCpp.h>
#include "descriptor_surface_based_trainer/Utils.h"

Main_Dialog::Main_Dialog(wxWindow *parent) : MainDialogBase(parent)
{
    std::vector<boost::filesystem::path> files;
    boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");

    get_all_files_with_ext(input_path, ".obj", files);
    for (int i = 0; i < files.size(); i++) {
        choice_object_model->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
    }   
    choice_object_model->SetSelection(0);
    if (choice_object_model->GetCount() > 0) {

        std::string object_model_name =  std::string(choice_object_model->GetStringSelection().mb_str());
        std::string object_model_path = input_path.string() + object_model_name;
        HalconCpp::HObjectModel3D model;
        model.ReadObjectModel3d(object_model_path.c_str(), "mm", HalconCpp::HTuple(), HalconCpp::HTuple());
        wxString diameter_string;
        diameter_string << model.MaxDiameterObjectModel3d();
        std::replace(diameter_string.begin(), diameter_string.end(), ',', '.');
        edit_diameter->SetValue(diameter_string);
    }

    files.clear();
    get_all_files_with_ext(input_path, ".dae", files);
    for (int i = 0; i < files.size(); i++) {
        choice_mesh->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
    }
    choice_mesh->SetSelection(0);

    Centre();

}

Main_Dialog::Main_Dialog(wxWindow *parent, std::string name, std::string object_model, std::string mesh, std::string rotation_type, std::string orientation_x, std::string orientation_y, std::string orientation_z, std::string diameter, std::string score_3D)
    : MainDialogBase(parent)
{
    edit_name->SetValue(wxString(name.c_str(), wxConvUTF8));
    edit_orientation_x->SetValue(wxString(orientation_x.c_str(), wxConvUTF8));
    edit_orientation_y->SetValue(wxString(orientation_y.c_str(), wxConvUTF8));
    edit_orientation_z->SetValue(wxString(orientation_z.c_str(), wxConvUTF8));
    edit_diameter->SetValue(wxString(diameter.c_str(), wxConvUTF8));
    edit_score_3D->SetValue(wxString(score_3D.c_str(), wxConvUTF8));

    std::vector<boost::filesystem::path> files;
    boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");

    get_all_files_with_ext(input_path, ".obj", files);
    int selection = 0;
    for (int i = 0; i < files.size(); i++) {
        choice_object_model->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
        if (files[i].string() == object_model) {
            selection = i;
        }
    }
    choice_object_model->SetSelection(selection);


    files.clear();
    selection = 0;
    get_all_files_with_ext(input_path, ".dae", files);
    for (int i = 0; i < files.size(); i++) {
        choice_mesh->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
        if (files[i].string() == mesh) {
            selection = i;
        }
    }
    choice_mesh->SetSelection(selection);

    if (rotation_type == ROTATIONTYPE_NO_ROTATION) { choice_rotation_type->SetSelection(0); }
    else if (rotation_type == ROTATIONTYPE_CYLINDER) { choice_rotation_type->SetSelection(1); }
    else if (rotation_type == ROTATIONTYPE_SPHERE) { choice_rotation_type->SetSelection(2); }

    Centre();
}

void Main_Dialog::onDialogClose(wxCloseEvent &event) {
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure to quit?"), wxT("Quit"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          Destroy();
      }
}

void Main_Dialog::onObjectModelSelected(wxCommandEvent &event) {
    boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
    std::string object_model_name =  std::string(choice_object_model->GetStringSelection().mb_str());
    std::string object_model_path = input_path.string() + object_model_name;
    HalconCpp::HObjectModel3D model;
    model.ReadObjectModel3d(object_model_path.c_str(), "mm", HalconCpp::HTuple(), HalconCpp::HTuple());
    wxString diameter_string;
    diameter_string << model.MaxDiameterObjectModel3d();;
    edit_diameter->SetValue(diameter_string);
}

void Main_Dialog::onCancelPressed( wxCommandEvent& event )
{  
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure to quit?"), wxT("Quit"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          Destroy();
      }
}



void Main_Dialog::onNextPressed( wxCommandEvent& event )
{

    std::string name = trim(std::string(edit_name->GetValue().mb_str()));
    std::string orientation_x = trim(std::string(edit_orientation_x->GetValue().mb_str()));
    std::string orientation_y = trim(std::string(edit_orientation_y->GetValue().mb_str()));
    std::string orientation_z = trim(std::string(edit_orientation_z->GetValue().mb_str()));
    std::string diameter = trim(std::string(edit_diameter->GetValue().mb_str()));
    std::string score_3D = trim(std::string(edit_score_3D->GetValue().mb_str()));

    if (check_string_redex(name, boost::regex("[a-zA-Z]+"))) {
        if (check_string_redex(orientation_x, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
            if (check_string_redex(orientation_y, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
                if (check_string_redex(orientation_z, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
                    if (check_string_redex(diameter, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
                        if (check_string_redex(score_3D, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
                            //edit values are valid, check choice-values:
                            if (choice_object_model->GetCount() > 0) {
                                if (choice_mesh->GetCount() > 0) {
                                    //all input values are valid
                                    std::string object_model = std::string(choice_object_model->GetStringSelection().mb_str());
                                    std::string mesh = std::string(choice_mesh->GetStringSelection().mb_str());
                                    std::string rotation_type = std::string(choice_rotation_type->GetStringSelection().mb_str());
                                    DescModelsDialog *descModelsDialog = new DescModelsDialog( (wxWindow*)NULL, name, object_model, mesh, rotation_type, orientation_x, orientation_y, orientation_z, diameter, score_3D);

                                    descModelsDialog ->Show();
                                    Destroy();


                                } else {
                                    wxMessageDialog *dial = new wxMessageDialog(this,
                                       wxT("There was no valid mesh found in the input folder. Please provide a mesh-file (*.dae)"), wxT("Warning"), wxOK | wxICON_WARNING);
                                    dial->ShowModal();
                                }
                            } else {
                                wxMessageDialog *dial = new wxMessageDialog(this,
                                   wxT("There was no valid object-model found in the input folder. Please provide an object model (*.obj)"), wxT("Warning"), wxOK | wxICON_WARNING);
                                dial->ShowModal();
                            }
                        } else {
                            wxMessageDialog *dial = new wxMessageDialog(this,
                               wxT("The provided score is not a valid double!"), wxT("Warning"), wxOK | wxICON_WARNING);
                            dial->ShowModal();
                        }
                    } else {
                        wxMessageDialog *dial = new wxMessageDialog(this,
                           wxT("The provided diameter is not a valid double!"), wxT("Warning"), wxOK | wxICON_WARNING);
                        dial->ShowModal();
                    }
                } else {
                    wxMessageDialog *dial = new wxMessageDialog(this,
                       wxT("The provided z-value of the orientation is not a valid double!"), wxT("Warning"), wxOK | wxICON_WARNING);
                    dial->ShowModal();
                }
            } else {
                wxMessageDialog *dial = new wxMessageDialog(this,
                   wxT("The provided y-value of the orientation is not a valid double!"), wxT("Warning"), wxOK | wxICON_WARNING);
                dial->ShowModal();
            }
        } else {
            wxMessageDialog *dial = new wxMessageDialog(this,
               wxT("The provided x-value of the orientation is not a valid double!"), wxT("Warning"), wxOK | wxICON_WARNING);
            dial->ShowModal();
        }
    } else {
        wxMessageDialog *dial = new wxMessageDialog(this,
           wxT("The provided name is not correct, please enter a name containing only letters!"), wxT("Warning"), wxOK | wxICON_WARNING);
        dial->ShowModal();
    }

}


void Main_Dialog::onKillFocusName(wxFocusEvent &event)
{
    std::string name = trim(std::string(edit_name->GetValue().mb_str()));
    if (!(check_string_redex(name, boost::regex("^[a-zA-Z]+$")))) {
        edit_name->SetValue(wxT(""));
    }
}

void Main_Dialog::onKillFocusOrientationX(wxFocusEvent &event)
{
    std::string orientation_x = trim(std::string(edit_orientation_x->GetValue().mb_str()));
    if (!(check_string_redex(orientation_x, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$")))) {
        edit_orientation_x->SetValue(wxT("1.0"));
    }
}

void Main_Dialog::onKillFocusOrientationY(wxFocusEvent &event)
{
    std::string orientation_y = trim(std::string(edit_orientation_y->GetValue().mb_str()));
    if (!(check_string_redex(orientation_y, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$")))) {
        edit_orientation_y->SetValue(wxT("0.0"));
    }
}

void Main_Dialog::onKillFocusOrientationZ(wxFocusEvent &event)
{
    std::string orientation_z = trim(std::string(edit_orientation_z->GetValue().mb_str()));
    if (!(check_string_redex(orientation_z, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$")))) {
        edit_orientation_z->SetValue(wxT("0.0"));
    }
}

void Main_Dialog::onKillFocusDiameter(wxFocusEvent &event)
{
    std::string diameter = trim(std::string(edit_diameter->GetValue().mb_str()));
    if (check_string_redex(diameter, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double dia = boost::lexical_cast<double>(trim(std::string(edit_diameter->GetValue().mb_str())));
        if (dia <= 0) {
            edit_diameter->SetValue(wxT("0.2"));
        }
    } else {
        edit_diameter->SetValue(wxT("0.2"));
    }
}

void Main_Dialog::onKillFocusScore3D(wxFocusEvent &event)
{
    std::string score_3D = trim(std::string(edit_score_3D->GetValue().mb_str()));
    if (check_string_redex(score_3D, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double score = boost::lexical_cast<double>(trim(std::string(edit_score_3D->GetValue().mb_str())));
        if ((score <= 0) || (score >= 1)) {
            edit_score_3D->SetValue(wxT("0.15"));
        }
    } else {
        edit_score_3D->SetValue(wxT("0.15"));
    }
}

void Main_Dialog::onTextEnterName(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusName(evt);
}

void Main_Dialog::onTextEnterOrientationX(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusOrientationX(evt);
}

void Main_Dialog::onTextEnterOrientationY(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusOrientationY(evt);
}

void Main_Dialog::onTextEnterOrientationZ(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusOrientationZ(evt);
}

void Main_Dialog::onTextEnterDiameter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusDiameter(evt);
}

void Main_Dialog::onTextEnterScore3D(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onKillFocusScore3D(evt);
}



