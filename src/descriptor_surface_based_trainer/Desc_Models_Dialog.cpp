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


#include "descriptor_surface_based_trainer/Desc_Models_Dialog.h"
#include "descriptor_surface_based_trainer/View_Creator_Dialog.h"
#include "descriptor_surface_based_trainer/Create_Output_Files_Dialog.h"
#include "descriptor_surface_based_trainer/Utils.h"
#include "descriptor_surface_based_trainer/Main_Dialog.h"



void DescModelsDialog::resetGuiValues()
{
    label_orientation_x_value->SetLabel(wxT(""));
    label_orientation_y_value->SetLabel(wxT(""));
    label_orientation_z_value->SetLabel(wxT(""));

    label_axis_1_x_value->SetLabel(wxT(""));
    label_axis_1_y_value->SetLabel(wxT(""));
    label_axis_1_z_value->SetLabel(wxT(""));
    label_axis_1_angle_value->SetLabel(wxT(""));
    label_axis_2_x_value->SetLabel(wxT(""));
    label_axis_2_y_value->SetLabel(wxT(""));
    label_axis_2_z_value->SetLabel(wxT(""));
    label_axis_2_angle_value->SetLabel(wxT(""));

    label_score_2D_value->SetLabel(wxT(""));
    label_use_color_value->SetLabel(wxT(""));
    label_invertible_value->SetLabel(wxT(""));

    label_vertical_offset_value->SetLabel(wxT(""));
    label_horizontal_offset_value->SetLabel(wxT(""));
    label_depth_value->SetLabel(wxT(""));
    label_number_ferns_value->SetLabel(wxT(""));
    label_patch_size_value->SetLabel(wxT(""));
    label_min_scale_value->SetLabel(wxT(""));
    label_max_scale_value->SetLabel(wxT(""));

    button_edit_model->Enable(false);

    image->setImage(new wxBitmap(wxImage(300, 225)));
    image->paintNow();
}

std::vector<std::string> DescModelsDialog::getObjectParameters()
{
    std::vector<std::string> params;
    params.push_back(name);
    params.push_back(object_model);
    params.push_back(mesh);
    params.push_back(rotation_type);
    params.push_back(orientation_x);
    params.push_back(orientation_y);
    params.push_back(orientation_z);
    params.push_back(diameter);
    params.push_back(score_3D);
    return params;
}

std::vector<ViewParamsWrapper> DescModelsDialog::getViews()
{
    return views;
}



DescModelsDialog::DescModelsDialog(wxWindow* parent, std::string name, std::string object_model, std::string mesh, std::string rotation_type, std::string orientation_x, std::string orientation_y, std::string orientation_z, std::string diameter, std::string score_3D)
    : DescModelsDialogBase(parent),
      name(name),
      object_model(object_model),
      mesh(mesh),
      rotation_type(rotation_type),
      orientation_x(orientation_x),
      orientation_y(orientation_y),
      orientation_z(orientation_z),
      diameter(diameter),
      score_3D(score_3D)
{
    button_edit_model->Enable(false);
    image->setImage(new wxBitmap(wxImage(300, 225)));
    image->paintNow();
}

void DescModelsDialog::OnDialogClose( wxCloseEvent& event )
{
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure to quit?"), wxT("Quit"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          Destroy();
      }
}

void DescModelsDialog::onAddModelClicked( wxCommandEvent& event )
{
    ViewParamsWrapper params(rotation_type);
    ViewCreatorDialog *descModelsDialog = new ViewCreatorDialog( (wxWindow*)NULL, &params);
    if (descModelsDialog ->ShowModal() == wxID_SAVE) {
        if (params.getIsValid()) {
            list_box_views->Clear();
            views.push_back(params);
            for (int i = 0; i < views.size(); i++) {
                std::string name = this->name + "_view_" + boost::lexical_cast<std::string>(i + 1);
                list_box_views->Insert(wxString(name.c_str(), wxConvUTF8), i);
            }
            resetGuiValues();
            list_box_views->SetSelection(views.size() - 1, true);
            wxCommandEvent evt;
            onListBoxSelected(evt);

        }
    }



}

void DescModelsDialog::onDeleteModelClicked( wxCommandEvent& event )
{
    if (list_box_views->GetSelection() >= 0) {
        views.erase(views.begin() + list_box_views->GetSelection());
        list_box_views->Clear();
        for (int i = 0; i < views.size(); i++) {
            std::string name = this->name + "_view_" + boost::lexical_cast<std::string>(i + 1);
            list_box_views->Insert(wxString(name.c_str(), wxConvUTF8), i);
        }
        resetGuiValues();
    }
}

void DescModelsDialog::onEditModelClicked(wxCommandEvent &event)
{
    if (list_box_views->GetCount() > 0) {
        views.at(list_box_views->GetSelection()).clearBoundingBox();
        ViewCreatorDialog *descModelsDialog = new ViewCreatorDialog( (wxWindow*)NULL, &(views.at(list_box_views->GetSelection())));
        descModelsDialog ->ShowModal();
        wxCommandEvent evt;
        onListBoxSelected(evt);
    }
}

void DescModelsDialog::onCancelClicked( wxCommandEvent& event )
{
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure to quit?"), wxT("Quit"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          Destroy();
      }
}



void DescModelsDialog::onFinishClicked( wxCommandEvent& event )
{
    if (list_box_views->GetCount() > 0) {
        bool boxes_available = true;
        for (int i = 0; i < views.size(); i++) {
            if (!(views.at(i).getHasBoundingBox())) {
                boxes_available = false;
                break;
            }
        }
        if (boxes_available) {
            wxMessageDialog *dial = new wxMessageDialog(this,
                  wxT("Do you want to create the object with the selected values? This can take several minutes depending on your parameters!"), wxT("Finish"),
                  wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
              if (dial->ShowModal() == wxID_YES) {
                  CreateOutputFilesDialog *createOutputFilesDialog = new CreateOutputFilesDialog(this);
                  if (createOutputFilesDialog->ShowModal() == wxID_OK) {
                      wxMessageDialog *dial = new wxMessageDialog(this,
                            wxT("All files successfully created!"), wxT("Success"), wxOK);
                      dial->ShowModal();
                      Destroy();
                  } else {
                      wxMessageDialog *dial = new wxMessageDialog(this,
                            wxT("Not all files could be created, please check the logfile in the output folder for more information. End the program anyway?"), wxT("Warning"),
                            wxYES_NO | wxNO_DEFAULT | wxICON_ERROR);
                        if (dial->ShowModal() == wxID_YES) {
                            Destroy();
                        }
                  }
              }
        } else {
            wxMessageDialog *dial = new wxMessageDialog(this,
                  wxT("Please create a bounding box for all views (Select them by clicking on the image panel; clockwise, starting with the top left corner)"), wxT("Warning"), wxOK | wxICON_WARNING);
               dial->ShowModal();
        }
    } else {
        wxMessageDialog *dial = new wxMessageDialog(this,
              wxT("Please create at least one view!"), wxT("Warning"), wxOK | wxICON_WARNING);
           dial->ShowModal();
    }

}

void DescModelsDialog::onBackClicked( wxCommandEvent& event )
{
    if (list_box_views->GetCount() > 0) {
        wxMessageDialog *dial = new wxMessageDialog(this,
              wxT("Your created views will be lost if you go back, continue anyway?"), wxT("Warning"),
              wxYES_NO | wxNO_DEFAULT | wxICON_WARNING);
          if (dial->ShowModal() != wxID_YES) {
            return;
          }
    }
    Main_Dialog *mainDialog = new Main_Dialog( (wxWindow*)NULL, name, object_model, mesh, rotation_type, orientation_x, orientation_y, orientation_z, diameter, score_3D);
    mainDialog ->Show();
    Destroy();


}



void DescModelsDialog::onListBoxSelected(wxCommandEvent &event)
{
    if (list_box_views->GetSelection() >= 0) {
        double orientation_x = views.at(list_box_views->GetSelection()).getOrientation()[0];
        double orientation_y = views.at(list_box_views->GetSelection()).getOrientation()[1];
        double orientation_z = views.at(list_box_views->GetSelection()).getOrientation()[2];
        if (views.at(list_box_views->GetSelection()).getRotationType() == ROTATIONTYPE_SPHERE) {
            double axis_2_x = views.at(list_box_views->GetSelection()).getAxis2()[0];
            double axis_2_y = views.at(list_box_views->GetSelection()).getAxis2()[1];
            double axis_2_z = views.at(list_box_views->GetSelection()).getAxis2()[2];
            double axis_2_angle = views.at(list_box_views->GetSelection()).getAxis2Angle();
            label_axis_2_x_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_2_x)));
            label_axis_2_y_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_2_y)));
            label_axis_2_z_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_2_z)));
            label_axis_2_angle_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_2_angle)));
        } else {
            label_axis_2_x_value->SetLabel(wxT("-"));
            label_axis_2_y_value->SetLabel(wxT("-"));
            label_axis_2_z_value->SetLabel(wxT("-"));
            label_axis_2_angle_value->SetLabel(wxT("-"));
        }
        if (views.at(list_box_views->GetSelection()).getRotationType() != ROTATIONTYPE_NO_ROTATION) {
            double axis_1_x = views.at(list_box_views->GetSelection()).getAxis1()[0];
            double axis_1_y = views.at(list_box_views->GetSelection()).getAxis1()[1];
            double axis_1_z = views.at(list_box_views->GetSelection()).getAxis1()[2];
            double axis_1_angle = views.at(list_box_views->GetSelection()).getAxis1Angle();
            label_axis_1_x_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_1_x)));
            label_axis_1_y_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_1_y)));
            label_axis_1_z_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_1_z)));
            label_axis_1_angle_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), axis_1_angle)));
        } else {
            label_axis_1_x_value->SetLabel(wxT("-"));
            label_axis_1_y_value->SetLabel(wxT("-"));
            label_axis_1_z_value->SetLabel(wxT("-"));
            label_axis_1_angle_value->SetLabel(wxT("-"));
        }
        double score_2D = views.at(list_box_views->GetSelection()).getScore2D();
        bool use_color = views.at(list_box_views->GetSelection()).getUseColor();
        bool is_invertible = views.at(list_box_views->GetSelection()).getIsInvertible();
        int vertical_offset = views.at(list_box_views->GetSelection()).getVerticalOffset();
        int horizontal_offset = views.at(list_box_views->GetSelection()).getHorizontalOffset();
        int depth = views.at(list_box_views->GetSelection()).getDepth();
        int number_ferns = views.at(list_box_views->GetSelection()).getNumberFerns();
        int patch_size = views.at(list_box_views->GetSelection()).getPatchSize();
        double min_scale = views.at(list_box_views->GetSelection()).getMinScale();
        double max_scale = views.at(list_box_views->GetSelection()).getMaxScale();

        label_orientation_x_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), orientation_x)));
        label_orientation_y_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), orientation_y)));
        label_orientation_z_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), orientation_z)));
        label_score_2D_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), score_2D)));
        if (use_color) {
            label_use_color_value->SetLabel(wxT("Yes"));
        } else {
            label_use_color_value->SetLabel(wxT("No"));
        }
        if (is_invertible) {
            label_invertible_value->SetLabel(wxT("Yes"));
        } else {
            label_invertible_value->SetLabel(wxT("No"));
        }

        label_vertical_offset_value->SetLabel(wxString::Format(wxT("%i"), vertical_offset));
        label_horizontal_offset_value->SetLabel(wxString::Format(wxT("%i"), horizontal_offset));
        label_depth_value->SetLabel(wxString::Format(wxT("%i"), depth));
        label_number_ferns_value->SetLabel(wxString::Format(wxT("%i"), number_ferns));
        label_patch_size_value->SetLabel(wxString::Format(wxT("%i"), patch_size));
        label_min_scale_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), min_scale)));
        label_max_scale_value->SetLabel(trimDoubleString(wxString::Format(wxT("%f"), max_scale)));

        button_edit_model->Enable(true);
        image->setImage(createBitmap(drawBoundingBox(views.at(list_box_views->GetSelection()).getImage(), views.at(list_box_views->GetSelection()).getBoxCorners()), 300, 225));
        image->paintNow();

    }

}

void DescModelsDialog::onImageClicked(wxMouseEvent &event)
{
    int pos_x = event.GetX();
    int pos_y = event.GetY();

    if ((pos_x > 0 && pos_x < 300) && (pos_y > 0 && pos_y < 225) && (list_box_views->GetSelection() >= 0)) {
        if (views.at(list_box_views->GetSelection()).getBoxCorners().size() == 4) {
            views.at(list_box_views->GetSelection()).clearBoundingBox();
        } else {
            double pos_x_factor = pos_x / 300.0f;
            double pos_y_factor = pos_y / 225.0f;
            pos_x =  (double)views.at(list_box_views->GetSelection()).getImage().Width() * pos_x_factor;
            pos_y =  (double)views.at(list_box_views->GetSelection()).getImage().Height() * pos_y_factor;
            views.at(list_box_views->GetSelection()).addBoxCorner(Eigen::Vector2i(pos_x, pos_y));
            if (views.at(list_box_views->GetSelection()).getBoxCorners().size() == 4) {
                views.at(list_box_views->GetSelection()).setHasBoundingBox();
            } else {
                views.at(list_box_views->GetSelection()).setHasBoundingBox(false);
            }
        }

        image->setImage(createBitmap(drawBoundingBox(views.at(list_box_views->GetSelection()).getImage(), views.at(list_box_views->GetSelection()).getBoxCorners()), 300, 225));
        image->paintNow();
    }
}

void DescModelsDialog::onImageClickedRight(wxMouseEvent &event)
{
    int pos_x = event.GetX();
    int pos_y = event.GetY();

    if ((pos_x > 0 && pos_x < 300) && (pos_y > 0 && pos_y < 225) && (list_box_views->GetSelection() >= 0)) {
        views.at(list_box_views->GetSelection()).clearBoundingBox();
        image->setImage(createBitmap(drawBoundingBox(views.at(list_box_views->GetSelection()).getImage(), views.at(list_box_views->GetSelection()).getBoxCorners()), 300, 225));
        image->paintNow();
    }
}
