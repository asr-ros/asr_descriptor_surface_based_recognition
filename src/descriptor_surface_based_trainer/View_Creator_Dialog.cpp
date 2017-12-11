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


#include "descriptor_surface_based_trainer/View_Creator_Dialog.h"
#include "descriptor_surface_based_trainer/Create_Test_Model_Dialog.h"
#include <wx/sizer.h>
#include <asr_halcon_bridge/halcon_image.h>
#include <boost/regex.hpp>
#include <ros/package.h>
#include "descriptor_surface_based_trainer/Utils.h"
#include <boost/thread.hpp>



void ViewCreatorDialog::enableGuiElements(bool enable)
{
    edit_upper_left_row->Enable(enable);
    edit_upper_left_column->Enable(enable);
    edit_lower_right_row->Enable(enable);
    edit_lower_right_column->Enable(enable);
    slider_upper_left_row->Enable(enable);
    slider_upper_left_column->Enable(enable);
    slider_lower_right_row->Enable(enable);
    slider_lower_right_column->Enable(enable);
    edit_orientation_x->Enable(enable);
    edit_orientation_y->Enable(enable);
    edit_orientation_z->Enable(enable);
    edit_score_2D->Enable(enable);
    edit_vertical_offset->Enable(enable);
    edit_horizontal_offset->Enable(enable);
    edit_axis_1_x->Enable(enable);
    edit_axis_1_y->Enable(enable);
    edit_axis_1_z->Enable(enable);
    edit_axis_1_angle->Enable(enable);
    edit_axis_2_x->Enable(enable);
    edit_axis_2_y->Enable(enable);
    edit_axis_2_z->Enable(enable);
    edit_axis_2_angle->Enable(enable);
    edit_depth->Enable(enable);
    edit_fern_number->Enable(enable);
    edit_patch_size->Enable(enable);
    edit_min_scale->Enable(enable);
    edit_max_scale->Enable(enable);
    check_invertable->Enable(enable);
    check_use_color->Enable(enable);
}

void ViewCreatorDialog::enableGuiElementsTest(bool enable)
{

    enableGuiElements(enable);
    choice_image_source->Enable(enable);
    choice_image->Enable(enable);
    choice_test_image->Enable(enable);
    choice_test_image_source->Enable(enable);
    if ((choice_image_source->GetSelection() == 2) && (choice_image->GetSelection() > 0)) {
        check_fix_current_image->Enable(enable);
    }
    button_save->Enable(enable);
    button_cancel->Enable(enable);

}


void ViewCreatorDialog::updateImageOnCrop() {
    if ((check_fix_current_image->IsChecked()) && (fixed_image_available)) {

        int row1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_row->GetValue().mb_str())));
        int column1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_column->GetValue().mb_str())));
        int row2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_row->GetValue().mb_str())));
        int column2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_column->GetValue().mb_str())));

        HalconCpp::HRegion region;
        region.GenRectangle1(row1, column1, row2, column2);
        HalconCpp::HImage croppedImage = fixedImage.ReduceDomain(region);
        croppedImage = croppedImage.ChangeFormat(croppedImage.Width(), croppedImage.Height());

        image_model->setImage(createBitmap(croppedImage, 480, 360));
        image_model->paintNow();

    }
}

void ViewCreatorDialog::paint_test_image(HalconCpp::HImage img)
{
    img = img.ZoomImageSize(480, 360, "bilinear");
    image_test->setImage(createBitmap(img, 480, 360));
    image_test->paintNow();

}



ViewCreatorDialog::ViewCreatorDialog(wxWindow *parent, ViewParamsWrapper *params)
    : ViewCreatorDialogBase(parent), params(params), fixed_image_available(false),
      desc_model_available(false), test_running(false), frame_counter(0),
      score_sum(0), feature_sum(0), time_sum(0), update_timer(0)
{

    if (params->getRotationType() == ROTATIONTYPE_NO_ROTATION || params->getRotationType() == ROTATIONTYPE_CYLINDER) {
       label_axis_2->Show(false);
        label_axis_2_x->Show(false);
        edit_axis_2_x->Show(false);
        label_axis_2_y->Show(false);
        edit_axis_2_y->Show(false);
        label_axis_2_z->Show(false);
        edit_axis_2_z->Show(false);
        label_axis_2_angle->Show(false);
        edit_axis_2_angle->Show(false);

        if (params->getRotationType() == ROTATIONTYPE_NO_ROTATION) {
            label_axis_1->Show(false);
            label_axis_1_x->Show(false);
            edit_axis_1_x->Show(false);
            label_axis_1_y->Show(false);
            edit_axis_1_y->Show(false);
            label_axis_1_z->Show(false);
            edit_axis_1_z->Show(false);
            label_axis_1_angle->Show(false);
            edit_axis_1_angle->Show(false);
        }
    }


    update_timer = new wxTimer(this);
    update_timer->Start(200);
    Connect(update_timer->GetId(), wxEVT_TIMER, wxTimerEventHandler(ViewCreatorDialog::onUpdate), NULL, this);

    image_test->setImage(new wxBitmap(wxImage(480, 360)));
    image_test->paintNow();



    if (params->getIsValid()) {
        choice_image_source->AppendString(wxT("edit image"));
        choice_image_source->SetSelection(3);
        wxCommandEvent evt;
        onChoiceImageSource(evt);

    } else {
        image_model->setImage(new wxBitmap(wxImage(480, 360)));
        image_model->paintNow();

        choice_image->Clear();
        choice_image->Insert(wxString("<No selection>",  wxConvUTF8), 0);
        choice_image->SetSelection(0);

        choice_test_image->Clear();
        choice_test_image->Insert(wxString("<No selection>",  wxConvUTF8), 0);
        choice_test_image->SetSelection(0);

        edit_upper_left_row->SetValue(wxT("0"));
        edit_upper_left_column->SetValue(wxT("0"));
        edit_lower_right_row->SetValue(wxT("359"));
        edit_lower_right_column->SetValue(wxT("479"));
        slider_lower_right_row->SetMax(359);
        slider_lower_right_row->SetValue(359);
        slider_lower_right_column->SetMax(479);
        slider_lower_right_column->SetValue(479);
        edit_score_2D->SetValue(wxT("0.2"));
        edit_vertical_offset->SetValue(wxT("0"));
        edit_horizontal_offset->SetValue(wxT("0"));
        edit_axis_1_angle->SetValue(wxT("1.0"));
        edit_axis_2_angle->SetValue(wxT("1.0"));
        edit_depth->SetValue(wxT("8"));
        edit_fern_number->SetValue(wxT("50"));
        edit_patch_size->SetValue(wxT("17"));
        edit_min_scale->SetValue(wxT("0.6"));
        edit_max_scale->SetValue(wxT("1.3"));


        enableGuiElements(false);

        check_fix_current_image->Enable(false);

    }

    label_model_points_value->SetLabel(wxT("0"));
    label_search_points_value->SetLabel(wxT("0"));
    label_matched_points_value->SetLabel(wxT("0"));
    label_average_matched_points_value->SetLabel(wxT("0"));
    label_time_value->SetLabel(wxT("0.0"));
    label_average_time_value->SetLabel(wxT("0.0"));
    button_end_test->Enable(false);


}

void ViewCreatorDialog::onModelCameraImage(const sensor_msgs::Image::ConstPtr& msg) {
    if (!check_fix_current_image->IsChecked()) {
        image_model->setImage(createBitmap(msg, 480, 360));
        image_model->paintNow();
    } else if (!fixed_image_available){
        if ((msg->encoding == "bgr8") || (msg->encoding == "rgb8") || (msg->encoding == "mono8")) {
            image_model->setImage(createBitmap(msg, 480, 360));
            image_model->paintNow();
            fixedImage = *halcon_bridge::toHalconCopy(*msg)->image;
            fixed_image_available = true;

            slider_upper_left_column->SetValue(0);
            slider_upper_left_column->SetMax((int)fixedImage.Width() - 1);
            edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), 0));

            slider_upper_left_row->SetMax((int)fixedImage.Height() - 1);
            slider_upper_left_row->SetValue(0);
            edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), 0));

            slider_lower_right_column->SetMax((int)fixedImage.Width() - 1);
            slider_lower_right_column->SetValue((int)fixedImage.Width() - 1);
            edit_lower_right_column->SetValue(wxString::Format(wxT("%i"),(int)fixedImage.Width() - 1));

            slider_lower_right_row->SetMax((int)fixedImage.Height() - 1);
            slider_lower_right_row->SetValue((int)fixedImage.Height() - 1);
            edit_lower_right_row->SetValue(wxString::Format(wxT("%i"),(int)fixedImage.Height() - 1));

            enableGuiElements(true);
        }
    }
}



void ViewCreatorDialog::onTestCameraImage(const sensor_msgs::Image::ConstPtr& msg, bool use_file) {
    if (test_running ) {
        HalconCpp::HImage img;
        std::string enc = "rgb8";
        if (use_file) {
            boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
            std::string filename_str = input_path.string() + std::string(choice_test_image->GetStringSelection().mb_str());
            img = HalconCpp::HImage(filename_str.c_str());
        } else {
            enc = msg->encoding;
            img = *halcon_bridge::toHalconCopy(msg)->image;
        }
        frame_counter++;
        HalconCpp::HTuple sc, search_all_row, search_all_column, search_row, search_column, model_row, model_column;
        HalconCpp::HTuple color(255);
        if (!(check_use_color->IsChecked())) {
            img = img.Rgb1ToGray();
            enc = "mono8";
        } else {
            color.Append(255);
            color.Append(0);
        }

        try {
            ros::Time start_time = ros::Time::now();
            double set_score = boost::lexical_cast<double>(trim(std::string(edit_score_2D->GetValue().mb_str())));
            if ((set_score <= 0) || (set_score >= 1)) {
                set_score = 0.0;
            }
            img.FindUncalibDescriptorModel(desc_model, HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), set_score, 1, "inlier_ratio", &sc);
            ros::Duration search_duration = ros::Time::now() - start_time;
            time_sum += search_duration.toSec();
            label_time_value->SetLabel(wxString::Format(wxT("%f"), search_duration.toSec()));
            label_average_time_value->SetLabel(wxString::Format(wxT("%f"), time_sum / frame_counter));
            desc_model.GetDescriptorModelPoints("search", "all", &search_all_row, &search_all_column);

            label_search_points_value->SetLabel(wxString::Format(wxT("%i"), search_all_row.Length()));
            desc_model.GetDescriptorModelPoints("search", 0, &search_row, &search_column);
            label_matched_points_value->SetLabel(wxString::Format(wxT("%i"), search_row.Length()));
            if (search_row.Length() > 0) {
                HalconCpp::HRegion points;
                points.GenRegionPoints(search_row, search_column);
                HalconCpp::HString type = "fill";
                img = points.PaintRegion(img, color, type);

                int vertical_offset = boost::lexical_cast<int>(trim(std::string(edit_vertical_offset->GetValue().mb_str())));
                int horizontal_offset = boost::lexical_cast<int>(trim(std::string(edit_horizontal_offset->GetValue().mb_str())));
                HalconCpp::HTuple matrix_tuple = desc_model.GetDescriptorModelResults(0, "homography");
                HalconCpp::HHomMat2D matrix;
                matrix.SetFromTuple(matrix_tuple);

                double trans_x, trans_y, trans_w;
                trans_x = matrix.ProjectiveTransPoint2d(vertical_offset, horizontal_offset, 1,  &trans_y, &trans_w);

                int tex_point_row = (int) (trans_x / trans_w);
                int tex_point_column = (int) (trans_y / trans_w);

                HalconCpp::HXLDCont tex_point;
                tex_point.GenCircleContourXld(tex_point_row, tex_point_column, 10, 0, 6.28318, "positive", 1);
                img = tex_point.PaintXld(img, color);


            }
            feature_sum += search_row.Length();
            label_average_matched_points_value->SetLabel(wxString::Format(wxT("%i"), (int)(feature_sum / frame_counter)));



        }catch (HalconCpp::HException exc) {
            label_matched_points_value->SetLabel(wxT("0"));
        }
        double score = 0;
        if (sc.Length() > 0) {
            score = sc[0];
        }

        score_sum += score;
        label_score_value->SetLabel(wxString::Format(wxT("%f"), score));
        label_frame_number_value->SetLabel(wxString::Format(wxT("%i"), frame_counter));
        label_average_score_value->SetLabel(wxString::Format(wxT("%f"), score_sum / frame_counter));


        desc_model.GetDescriptorModelPoints("model", "all", &model_row, &model_column);
        label_model_points_value->SetLabel(wxString::Format(wxT("%i"), model_row.Length()));


        paint_thread = boost::thread(boost::bind(&ViewCreatorDialog::paint_test_image, this, img));

        paint_thread.timed_join(boost::posix_time::millisec(100));




    } else {
        paint_thread.join();
        image_test->setImage(createBitmap(msg, 480, 360));
        image_test->paintNow();
    }

}


void ViewCreatorDialog::onUpdate(wxTimerEvent &evt)
{
    ros::spinOnce();
    if (!ros::ok())
    {
        update_timer->Stop();
        EndModal(wxID_CANCEL);
        Destroy();
    }
    if ((choice_test_image_source->GetSelection() == 1) && (choice_test_image->GetSelection() > 0) && (test_running) && (desc_model_available)) {
        if (update_counter == 0) {
            paint_thread.join();
            sensor_msgs::ImageConstPtr msg;
            onTestCameraImage(msg, true);
        }
    }

    update_counter = ((update_counter + 1) % 3);
}



void ViewCreatorDialog::OnDialogClose(wxCloseEvent &event)
{
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure you want to cancel?"), wxT("Cancel"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          update_timer->Stop();
          image_model_sub.shutdown();
          image_test_sub.shutdown();
          EndModal(wxID_CANCEL);
          Destroy();
      }
}

void ViewCreatorDialog::onChoiceImageSource( wxCommandEvent& event )
{
    image_model_sub.shutdown();
    choice_image->Clear();
    if (choice_image_source->GetSelection() != 3) {
        wxBitmap* empty = new wxBitmap(wxImage(480, 360));
        image_model->setImage(empty);
        image_model->paintNow();
        choice_image->Insert(wxString("<No selection>",  wxConvUTF8), 0);
        choice_image->SetSelection(0);

        check_fix_current_image->Enable(false);
        check_fix_current_image->SetValue(false);
        enableGuiElements(false);
        fixed_image_available = false;
    }

    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));

    switch (choice_image_source->GetSelection()) {
        case 1:
            {
                std::vector<boost::filesystem::path> files;
                boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
                get_all_files_with_ext(input_path, ".png", files);
                get_all_files_with_ext(input_path, ".jpg", files);
                std::sort(files.begin(), files.end());
                for (unsigned int i = 0; i < files.size(); i++) {
                    choice_image->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
                }

                break;
            }
        case 2:
            {
                std::vector<std::string> image_topics;
                ros::master::V_TopicInfo master_topics;
                ros::master::getTopics(master_topics);

                for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
                    const ros::master::TopicInfo& info = *it;
                    if (info.datatype == "sensor_msgs/Image") {
                        image_topics.push_back(info.name);
                    }
                }
                std::sort(image_topics.begin(), image_topics.end());
                for (unsigned int i = 0; i < image_topics.size(); i++) {
                    choice_image->Insert(wxString(image_topics.at(i).c_str(), wxConvUTF8),i + 1);
                }
                break;
            }
        case 3:
            {
                choice_image->AppendString(wxT("editable image"));
                choice_image->SetSelection(0);
                wxCommandEvent evt;
                onChoiceImage(evt);
            }
    }
}

void ViewCreatorDialog::onChoiceImage( wxCommandEvent& event )
{
    image_model_sub.shutdown();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));

    //case: edit image
    if (choice_image_source->GetSelection() == 3) {
        enableGuiElements(true);
        check_fix_current_image->SetValue(true);
        check_fix_current_image->Enable(false);
        fixed_image_available = true;


        fixedImage = params->getOriginalImage();

        slider_upper_left_column->SetMax((int)fixedImage.Width() - 1);
        slider_upper_left_column->SetValue(params->getColumn1());
        edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), params->getColumn1()));

        slider_upper_left_row->SetMax((int)fixedImage.Height() - 1);
        slider_upper_left_row->SetValue(params->getRow1());
        edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), params->getRow1()));

        slider_lower_right_column->SetMax((int)fixedImage.Width() - 1);
        slider_lower_right_column->SetValue(params->getColumn2());
        edit_lower_right_column->SetValue(wxString::Format(wxT("%i"),params->getColumn2()));

        slider_lower_right_row->SetMax((int)fixedImage.Height() - 1);
        slider_lower_right_row->SetValue(params->getRow2());
        edit_lower_right_row->SetValue(wxString::Format(wxT("%i"),params->getRow2()));

        updateImageOnCrop();

        edit_orientation_x->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getOrientation()[0])));
        edit_orientation_y->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getOrientation()[1])));
        edit_orientation_z->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getOrientation()[2])));

        if (params->getRotationType() != ROTATIONTYPE_NO_ROTATION) {
            edit_axis_1_x->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis1()[0])));
            edit_axis_1_y->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis1()[1])));
            edit_axis_1_z->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis1()[2])));
            edit_axis_1_angle->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis1Angle())));
            if (params->getRotationType() == ROTATIONTYPE_SPHERE) {
                edit_axis_2_x->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis2()[0])));
                edit_axis_2_y->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis2()[1])));
                edit_axis_2_z->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis2()[2])));
                edit_axis_2_angle->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getAxis2Angle())));
            }
        }

        edit_score_2D->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getScore2D())));
        edit_vertical_offset->SetValue(wxString::Format(wxT("%i"), params->getVerticalOffset()));
        edit_horizontal_offset->SetValue(wxString::Format(wxT("%i"), params->getHorizontalOffset()));

        edit_depth->SetValue(wxString::Format(wxT("%i"), params->getDepth()));
        edit_fern_number->SetValue(wxString::Format(wxT("%i"), params->getNumberFerns()));
        edit_patch_size->SetValue(wxString::Format(wxT("%i"), params->getPatchSize()));
        edit_min_scale->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getMinScale())));
        edit_max_scale->SetValue(trimDoubleString(wxString::Format(wxT("%f"), params->getMaxScale())));

        if (params->getIsInvertible()) {
            check_invertable->SetValue(true);
        }
        if (params->getUseColor()) {
            check_use_color->SetValue(true);
        }

    //case: no editing
    } else {
        enableGuiElements(false);
        check_fix_current_image->SetValue(false);
        fixed_image_available = false;
        if (choice_image->GetSelection() > 0) {
            if (choice_image_source->GetSelection() == 2) {
                check_fix_current_image->Enable(true);
                std::string str = std::string(choice_image->GetStringSelection().mb_str());
                image_model_sub = nh.subscribe<sensor_msgs::Image>(str, 1, &ViewCreatorDialog::onModelCameraImage, this);
            } else if (choice_image_source->GetSelection() == 1) {
                boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
                wxString filename = wxString(input_path.string().c_str(), wxConvUTF8) + choice_image->GetStringSelection();
                wxImage image(filename);
                image = image.Scale(480, 360);
                image_model->setImage(new wxBitmap(image));
                image_model->paintNow();

                check_fix_current_image->SetValue(true);

                std::string filename_str = input_path.string() + std::string(choice_image->GetStringSelection().mb_str());
                fixedImage = HalconCpp::HImage(filename_str.c_str());

                fixed_image_available = true;

                slider_upper_left_column->SetValue(0);
                slider_upper_left_column->SetMax((int)fixedImage.Width() - 1);
                edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), 0));

                slider_upper_left_row->SetMax((int)fixedImage.Height() - 1);
                slider_upper_left_row->SetValue(0);
                edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), 0));

                slider_lower_right_column->SetMax((int)fixedImage.Width() - 1);
                slider_lower_right_column->SetValue((int)fixedImage.Width() - 1);
                edit_lower_right_column->SetValue(wxString::Format(wxT("%i"),(int)fixedImage.Width() - 1));

                slider_lower_right_row->SetMax((int)fixedImage.Height() - 1);
                slider_lower_right_row->SetValue((int)fixedImage.Height() - 1);
                edit_lower_right_row->SetValue(wxString::Format(wxT("%i"),(int)fixedImage.Height() - 1));

                enableGuiElements(true);
            }
        } else {
            check_fix_current_image->Enable(false);
            wxBitmap* empty = new wxBitmap(wxImage(480, 360));
            image_model->setImage(empty);
            image_model->paintNow();
        }
    }

}

void ViewCreatorDialog::onChoiceTestImageSource(wxCommandEvent &event)
{
    image_test_sub.shutdown();
    choice_test_image->Clear();
    image_test->setImage(new wxBitmap(wxImage(480, 360)));
    image_test->paintNow();
    choice_test_image->Insert(wxString("<No selection>",  wxConvUTF8), 0);
    choice_test_image->SetSelection(0);

    switch (choice_test_image_source->GetSelection()) {
        case 1:
            {
                std::vector<boost::filesystem::path> files;
                boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
                get_all_files_with_ext(input_path, ".png", files);
                get_all_files_with_ext(input_path, ".jpg", files);
                std::sort(files.begin(), files.end());
                for (unsigned int i = 0; i < files.size(); i++) {
                    choice_test_image->AppendString(wxString(files[i].string().c_str(), wxConvUTF8));
                }

                break;
            }
        case 2:
            {
                std::vector<std::string> image_topics;
                ros::master::V_TopicInfo master_topics;
                ros::master::getTopics(master_topics);

                for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
                    const ros::master::TopicInfo& info = *it;
                    if (info.datatype == "sensor_msgs/Image") {
                        image_topics.push_back(info.name);
                    }
                }
                std::sort(image_topics.begin(), image_topics.end());
                for (unsigned int i = 0; i < image_topics.size(); i++) {
                    choice_test_image->Insert(wxString(image_topics.at(i).c_str(), wxConvUTF8),i + 1);
                }
                break;
            }
    }
}

void ViewCreatorDialog::onChoiceTestImage( wxCommandEvent& event )
{
    image_test_sub.shutdown();
    if (choice_test_image->GetSelection() > 0) {
        if (choice_test_image_source->GetSelection() == 1) {
            boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
            wxString filename = wxString(input_path.string().c_str(), wxConvUTF8) + choice_test_image->GetStringSelection();
            wxImage image(filename);
            image = image.Scale(480, 360);
            image_test->setImage(new wxBitmap(image));
            image_test->paintNow();
        } else if (choice_test_image_source->GetSelection() == 2) {
            std::string str = std::string(choice_test_image->GetStringSelection().mb_str());
            image_test_sub = nh.subscribe<sensor_msgs::Image>(str, 1, boost::bind(&ViewCreatorDialog::onTestCameraImage, this, _1, false));
        }
    } else {
        image_test->setImage(new wxBitmap(wxImage(480, 360)));
        image_test->paintNow();
    }

}



void ViewCreatorDialog::onButtonStartTestClicked(wxCommandEvent &event)
{
    if (choice_test_image->GetSelection() > 0) {
        if (check_fix_current_image->IsChecked() && fixed_image_available) {
            if (desc_model_available) {
                test_running = true;
                label_model_available_value->SetLabel(wxT("Yes"));
                label_score_value->SetLabel(wxT("0.0"));
                score_sum = 0;
                frame_counter = 0;
                feature_sum = 0;
                time_sum = 0;
                label_frame_number_value->SetLabel(wxT("0"));
                label_average_score_value->SetLabel(wxT("0.0"));
                label_model_points_value->SetLabel(wxT("0"));
                label_search_points_value->SetLabel(wxT("0"));
                label_matched_points_value->SetLabel(wxT("0"));
                label_average_matched_points_value->SetLabel(wxT("0"));
                label_time_value->SetLabel(wxT("0.0"));
                label_average_time_value->SetLabel(wxT("0.0"));
                button_start_test->Enable(false);
                button_end_test->Enable(true);
                enableGuiElementsTest(false);

            } else {
                wxMessageDialog *dial = new wxMessageDialog(this,
                      wxT("No descriptor model with current parameters available. Do you want to create one? This can take several minutes!"), wxT("Warning"),
                      wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
                  if (dial->ShowModal() == wxID_YES) {
                      int row1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_row->GetValue().mb_str())));
                      int column1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_column->GetValue().mb_str())));
                      int row2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_row->GetValue().mb_str())));
                      int column2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_column->GetValue().mb_str())));


                      int depth = boost::lexical_cast<int>(trim(std::string(edit_depth->GetValue().mb_str())));
                      int number_ferns = boost::lexical_cast<int>(trim(std::string(edit_fern_number->GetValue().mb_str())));
                      int patch_size = boost::lexical_cast<int>(trim(std::string(edit_patch_size->GetValue().mb_str())));
                      double min_scale = boost::lexical_cast<double>(trim(std::string(edit_min_scale->GetValue().mb_str())));
                      double max_scale = boost::lexical_cast<double>(trim(std::string(edit_max_scale->GetValue().mb_str())));
                      bool use_color = check_use_color->IsChecked();

                      CreateTestModelDialog *createTestModeldialog = new CreateTestModelDialog(this, &desc_model, fixedImage.CropRectangle1(row1, column1, row2, column2), depth, number_ferns, patch_size, min_scale, max_scale, use_color);
                      if (createTestModeldialog ->ShowModal() == wxID_OK) {
                          desc_model_available = true;
                          test_running = true;
                          label_model_available_value->SetLabel(wxT("Yes"));
                          label_score_value->SetLabel(wxT("0.0"));
                          score_sum = 0;
                          feature_sum = 0;
                          time_sum = 0;
                          frame_counter = 0;
                          label_frame_number_value->SetLabel(wxT("0"));
                          label_average_score_value->SetLabel(wxT("0.0"));
                          label_model_points_value->SetLabel(wxT("0"));
                          label_search_points_value->SetLabel(wxT("0"));
                          label_matched_points_value->SetLabel(wxT("0"));
                          label_average_matched_points_value->SetLabel(wxT("0"));
                          button_start_test->Enable(false);
                          button_end_test->Enable(true);
                          enableGuiElementsTest(false);

                      } else {
                          wxMessageDialog *dial = new wxMessageDialog(this,
                                wxT("Error creating model"), wxT("Error"), wxOK | wxICON_ERROR);
                             dial->ShowModal();
                      }
                  }
            }
        } else {
            wxMessageDialog *dial = new wxMessageDialog(this,
                  wxT("No image selected!"), wxT("Warning"), wxOK | wxICON_WARNING);
               dial->ShowModal();
        }
    } else {
        wxMessageDialog *dial = new wxMessageDialog(this,
              wxT("No test image topic selected!"), wxT("Warning"), wxOK | wxICON_WARNING);
           dial->ShowModal();
    }
}

void ViewCreatorDialog::onButtonEndTestClicked(wxCommandEvent &event)
{
    test_running = false;
    button_start_test->Enable(true);
    button_end_test->Enable(false);
    enableGuiElementsTest(true);
}

void ViewCreatorDialog::onButtonCancelClicked( wxCommandEvent& event )
{
    wxMessageDialog *dial = new wxMessageDialog(this,
          wxT("Are you sure you want to cancel?"), wxT("Cancel"),
          wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);
      if (dial->ShowModal() == wxID_YES) {
          update_timer->Stop();
          image_model_sub.shutdown();
          image_test_sub.shutdown();
          EndModal(wxID_CANCEL);
          Destroy();
      }

}

void ViewCreatorDialog::onButtonSaveClicked( wxCommandEvent& event )
{
    if (check_fix_current_image->IsChecked() && fixed_image_available) {

        std::string score_2D_string = trim(std::string(edit_score_2D->GetValue().mb_str()));
        if (check_string_redex(score_2D_string, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {

            std::string vertical_offset_string = trim(std::string(edit_vertical_offset->GetValue().mb_str()));
            if (check_string_redex(vertical_offset_string, boost::regex("^[-+]?[0-9]+$"))) {

                std::string horizontal_offset_string = trim(std::string(edit_horizontal_offset->GetValue().mb_str()));
                if (check_string_redex(horizontal_offset_string, boost::regex("^[-+]?[0-9]+$"))) {

                    std::string depth_string = trim(std::string(edit_depth->GetValue().mb_str()));
                    if (check_string_redex(depth_string, boost::regex("^[0-9]+$"))) {

                        std::string number_ferns_string = trim(std::string(edit_fern_number->GetValue().mb_str()));
                        if (check_string_redex(number_ferns_string, boost::regex("^[0-9]+$"))) {

                            std::string patch_size_string = trim(std::string(edit_patch_size->GetValue().mb_str()));
                            if (check_string_redex(patch_size_string, boost::regex("^[0-9]+$"))) {

                                std::string min_scale_string = trim(std::string(edit_min_scale->GetValue().mb_str()));
                                if (check_string_redex(min_scale_string, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {

                                    std::string max_scale_string = trim(std::string(edit_max_scale->GetValue().mb_str()));
                                    if (check_string_redex(max_scale_string, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {

                                        std::string orientation_x_string = trim(std::string(edit_orientation_x->GetValue().mb_str()));
                                        if (check_string_redex(orientation_x_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                            std::string orientation_y_string = trim(std::string(edit_orientation_y->GetValue().mb_str()));
                                            if (check_string_redex(orientation_y_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                std::string orientation_z_string = trim(std::string(edit_orientation_z->GetValue().mb_str()));
                                                if (check_string_redex(orientation_z_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                    bool axes_valid = true;
                                                    if (params->getRotationType() != ROTATIONTYPE_NO_ROTATION) {

                                                        std::string axis_1_x_string = trim(std::string(edit_axis_1_x->GetValue().mb_str()));
                                                        if (check_string_redex(axis_1_x_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                            std::string axis_1_y_string = trim(std::string(edit_axis_1_y->GetValue().mb_str()));
                                                            if (check_string_redex(axis_1_y_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                                std::string axis_1_z_string = trim(std::string(edit_axis_1_z->GetValue().mb_str()));
                                                                if (check_string_redex(axis_1_z_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
                                                                    axes_valid = true;

                                                                    double axis_1_x = boost::lexical_cast<double>(trim(std::string(edit_axis_1_x->GetValue().mb_str())));
                                                                    double axis_1_y = boost::lexical_cast<double>(trim(std::string(edit_axis_1_y->GetValue().mb_str())));
                                                                    double axis_1_z = boost::lexical_cast<double>(trim(std::string(edit_axis_1_z->GetValue().mb_str())));
                                                                    double axis_1_angle = boost::lexical_cast<double>(trim(std::string(edit_axis_1_angle->GetValue().mb_str())));
                                                                    Eigen::Vector3d axis_1(axis_1_x, axis_1_y, axis_1_z);
                                                                    params->setAxis1(axis_1);
                                                                    params->setAxis1Angle(axis_1_angle);

                                                                    if (params->getRotationType() == ROTATIONTYPE_SPHERE) {

                                                                        std::string axis_2_x_string = trim(std::string(edit_axis_2_x->GetValue().mb_str()));
                                                                        if (check_string_redex(axis_2_x_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                                            std::string axis_2_y_string = trim(std::string(edit_axis_2_y->GetValue().mb_str()));
                                                                            if (check_string_redex(axis_2_y_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {

                                                                                std::string axis_2_z_string = trim(std::string(edit_axis_2_z->GetValue().mb_str()));
                                                                                if (check_string_redex(axis_2_z_string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
                                                                                    axes_valid = true;

                                                                                    double axis_2_x = boost::lexical_cast<double>(trim(std::string(edit_axis_2_x->GetValue().mb_str())));
                                                                                    double axis_2_y = boost::lexical_cast<double>(trim(std::string(edit_axis_2_y->GetValue().mb_str())));
                                                                                    double axis_2_z = boost::lexical_cast<double>(trim(std::string(edit_axis_2_z->GetValue().mb_str())));
                                                                                    double axis_2_angle = boost::lexical_cast<double>(trim(std::string(edit_axis_2_angle->GetValue().mb_str())));
                                                                                    Eigen::Vector3d axis_2(axis_2_x, axis_2_y, axis_2_z);
                                                                                    params->setAxis2(axis_2);
                                                                                    params->setAxis2Angle(axis_2_angle);


                                                                                } else {
                                                                                    wxMessageDialog *dial = new wxMessageDialog(this,
                                                                                           wxT("Axis 2 (z) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                                                       dial->ShowModal();
                                                                                    axes_valid = false;
                                                                                }
                                                                            } else {
                                                                                wxMessageDialog *dial = new wxMessageDialog(this,
                                                                                       wxT("Axis 2 (y) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                                                   dial->ShowModal();
                                                                                axes_valid = false;
                                                                            }
                                                                        } else {
                                                                            wxMessageDialog *dial = new wxMessageDialog(this,
                                                                                   wxT("Axis 2 (x) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                                               dial->ShowModal();
                                                                            axes_valid = false;
                                                                        }
                                                                    }


                                                                } else {
                                                                    wxMessageDialog *dial = new wxMessageDialog(this,
                                                                           wxT("Axis 1 (z) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                                       dial->ShowModal();
                                                                    axes_valid = false;
                                                                }
                                                            } else {
                                                                wxMessageDialog *dial = new wxMessageDialog(this,
                                                                       wxT("Axis 1 (y) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                                   dial->ShowModal();
                                                                axes_valid = false;
                                                            }
                                                        } else {
                                                            wxMessageDialog *dial = new wxMessageDialog(this,
                                                                   wxT("Axis 1 (x) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                               dial->ShowModal();
                                                            axes_valid = false;
                                                        }
                                                    }
                                                    if (axes_valid) {
                                                        //all strings are valid!
                                                        int row1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_row->GetValue().mb_str())));
                                                        int column1 = boost::lexical_cast<int>(trim(std::string(edit_upper_left_column->GetValue().mb_str())));
                                                        int row2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_row->GetValue().mb_str())));
                                                        int column2 = boost::lexical_cast<int>(trim(std::string(edit_lower_right_column->GetValue().mb_str())));
                                                        params->setImage(fixedImage.CropRectangle1(row1, column1, row2, column2));
                                                        params->setOriginalImage(fixedImage);
                                                        params->setImageBounds(row1, column1, row2, column2);
                                                        params->setIsInvertible(check_invertable->GetValue());
                                                        params->setUserColor(check_use_color->GetValue());
                                                        double orientation_x = boost::lexical_cast<double>(trim(std::string(edit_orientation_x->GetValue().mb_str())));
                                                        double orientation_y = boost::lexical_cast<double>(trim(std::string(edit_orientation_y->GetValue().mb_str())));
                                                        double orientation_z = boost::lexical_cast<double>(trim(std::string(edit_orientation_z->GetValue().mb_str())));
                                                        Eigen::Vector3d orientation(orientation_x, orientation_y, orientation_z);
                                                        params->setOrientation(orientation);
                                                        double score_2D = boost::lexical_cast<double>(trim(std::string(edit_score_2D->GetValue().mb_str())));
                                                        params->setScore2D(score_2D);
                                                        int vertical_offset = boost::lexical_cast<int>(trim(std::string(edit_vertical_offset->GetValue().mb_str())));
                                                        params->setVerticalOffset(vertical_offset);
                                                        int horizontal_offset = boost::lexical_cast<int>(trim(std::string(edit_horizontal_offset->GetValue().mb_str())));
                                                        params->setHorizontalOffset(horizontal_offset);
                                                        int depth = boost::lexical_cast<int>(trim(std::string(edit_depth->GetValue().mb_str())));
                                                        params->setDepth(depth);
                                                        int number_ferns = boost::lexical_cast<int>(trim(std::string(edit_fern_number->GetValue().mb_str())));
                                                        params->setNumberFerns(number_ferns);
                                                        int patch_size = boost::lexical_cast<int>(trim(std::string(edit_patch_size->GetValue().mb_str())));
                                                        params->setPatchSize(patch_size);
                                                        double min_scale = boost::lexical_cast<double>(trim(std::string(edit_min_scale->GetValue().mb_str())));
                                                        params->setMinScale(min_scale);
                                                        double max_scale = boost::lexical_cast<double>(trim(std::string(edit_max_scale->GetValue().mb_str())));
                                                        params->setMaxScale(max_scale);


                                                        params->setIsValid(true);

                                                        update_timer->Stop();
                                                        image_model_sub.shutdown();
                                                        image_test_sub.shutdown();
                                                        EndModal(wxID_SAVE);
                                                        Destroy();




                                                    }
                                                } else {
                                                    wxMessageDialog *dial = new wxMessageDialog(this,
                                                           wxT("Orientation (z) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                       dial->ShowModal();
                                                }
                                            } else {
                                                wxMessageDialog *dial = new wxMessageDialog(this,
                                                       wxT("Orientation (y) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                                   dial->ShowModal();
                                            }
                                        } else {
                                            wxMessageDialog *dial = new wxMessageDialog(this,
                                                   wxT("Orientation (x) is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                               dial->ShowModal();
                                        }
                                    } else {
                                        wxMessageDialog *dial = new wxMessageDialog(this,
                                              wxT("Max. scale is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                           dial->ShowModal();
                                    }
                                } else {
                                    wxMessageDialog *dial = new wxMessageDialog(this,
                                          wxT("Min. scale is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                       dial->ShowModal();
                                }
                            } else {
                                wxMessageDialog *dial = new wxMessageDialog(this,
                                      wxT("Patch size is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                                   dial->ShowModal();
                            }
                        } else {
                            wxMessageDialog *dial = new wxMessageDialog(this,
                                  wxT("Fern number is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                               dial->ShowModal();
                        }
                    } else {
                        wxMessageDialog *dial = new wxMessageDialog(this,
                              wxT("Depth is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                           dial->ShowModal();
                    }
                } else {
                    wxMessageDialog *dial = new wxMessageDialog(this,
                          wxT("Horizontal offset is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                       dial->ShowModal();
                }
            } else {
                wxMessageDialog *dial = new wxMessageDialog(this,
                      wxT("Vertical offset is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
                   dial->ShowModal();
            }
        } else {
            wxMessageDialog *dial = new wxMessageDialog(this,
                  wxT("Score is not valid!"), wxT("Error"), wxOK | wxICON_ERROR);
               dial->ShowModal();
        }
    } else {
        wxMessageDialog *dial = new wxMessageDialog(this,
              wxT("No image selected!"), wxT("Error"), wxOK | wxICON_ERROR);
           dial->ShowModal();
    }
}

void ViewCreatorDialog::onCheckUseCurrentImage(wxCommandEvent &event)
{
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
    if (check_fix_current_image->IsChecked()) {

    } else {

        fixed_image_available = false;
        enableGuiElements(false);
    }
}

void ViewCreatorDialog::onEditTextUpperLeftRow(wxFocusEvent &event)
{

    std::string upperLeftRowString = trim(std::string(edit_upper_left_row->GetValue().mb_str()));
    std::string lowerRightRowString = trim(std::string(edit_lower_right_row->GetValue().mb_str()));

    if (check_string_redex(upperLeftRowString, boost::regex("^[0-9]+$"))) {
        if (check_string_redex(lowerRightRowString, boost::regex("^[0-9]+$"))) {
            int upperLeftRow = boost::lexical_cast<int>(upperLeftRowString);
            int lowerRightRow = boost::lexical_cast<int>(lowerRightRowString);
            if ((upperLeftRow >= 0) && (upperLeftRow <= slider_lower_right_row->GetMax() - 2)) {
                slider_upper_left_row->SetValue(upperLeftRow);
                if (lowerRightRow <= upperLeftRow) {
                    edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), upperLeftRow + 1));
                    slider_lower_right_row->SetValue(upperLeftRow + 1);
                }
            } else {
                edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), 0));
                slider_upper_left_row->SetValue(0);
            }
        }
    } else {
        edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), 0));
        slider_upper_left_row->SetValue(0);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onEditTextUpperLeftColumn(wxFocusEvent &event)
{
    std::string upperLeftColumnString = trim(std::string(edit_upper_left_column->GetValue().mb_str()));
    std::string lowerRightColumnString = trim(std::string(edit_lower_right_column->GetValue().mb_str()));

    if (check_string_redex(upperLeftColumnString, boost::regex("^[0-9]+$"))) {
        if (check_string_redex(lowerRightColumnString, boost::regex("^[0-9]+$"))) {
            int upperLeftColumn = boost::lexical_cast<int>(upperLeftColumnString);
            int lowerRightColumn = boost::lexical_cast<int>(lowerRightColumnString);
            if ((upperLeftColumn >= 0) && (upperLeftColumn <= slider_lower_right_column->GetMax() - 2)) {
                slider_upper_left_column->SetValue(upperLeftColumn);
                if (lowerRightColumn <= upperLeftColumn) {
                    edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), upperLeftColumn + 1));
                    slider_lower_right_column->SetValue(upperLeftColumn + 1);
                }
            } else {
                edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), 0));
                slider_upper_left_column->SetValue(0);
            }
        }
    } else {
        edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), 0));
        slider_upper_left_column->SetValue(0);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}



void ViewCreatorDialog::onEditTextLowerRightRow(wxFocusEvent &event)
{
    std::string lowerRightRowString = trim(std::string(edit_lower_right_row->GetValue().mb_str()));
    std::string upperLeftRowString = trim(std::string(edit_upper_left_row->GetValue().mb_str()));

    if (check_string_redex(lowerRightRowString, boost::regex("^[0-9]+$"))) {
        if (check_string_redex(upperLeftRowString, boost::regex("^[0-9]+$"))) {
            int lowerRightRow = boost::lexical_cast<int>(lowerRightRowString);
            int upperLeftRow = boost::lexical_cast<int>(upperLeftRowString);
            if ((lowerRightRow <= slider_lower_right_row->GetMax()) && (lowerRightRow >= 2)) {
                slider_lower_right_row->SetValue(lowerRightRow);
                if (upperLeftRow >= lowerRightRow) {
                    edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), lowerRightRow - 1));
                    slider_upper_left_row->SetValue(lowerRightRow - 1);
                }
            } else {
                edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), slider_lower_right_row->GetMax()));
                slider_lower_right_row->SetValue(slider_lower_right_row->GetMax());
            }
        }
    } else {
        edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), slider_lower_right_row->GetMax()));
        slider_lower_right_row->SetValue(slider_lower_right_row->GetMax());
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}


void ViewCreatorDialog::onEditTextLowerRightColumn(wxFocusEvent &event)
{
    std::string lowerRightColumnString = trim(std::string(edit_lower_right_column->GetValue().mb_str()));
    std::string upperLeftColumnString = trim(std::string(edit_upper_left_column->GetValue().mb_str()));

    if (check_string_redex(lowerRightColumnString, boost::regex("^[0-9]+$"))) {
        if (check_string_redex(upperLeftColumnString, boost::regex("^[0-9]+$"))) {
            int lowerRightColumn = boost::lexical_cast<int>(lowerRightColumnString);
            int upperLeftColumn = boost::lexical_cast<int>(upperLeftColumnString);
            if ((lowerRightColumn <= slider_lower_right_column->GetMax()) && (lowerRightColumn >= 2)) {
                slider_lower_right_column->SetValue(lowerRightColumn);
                if (upperLeftColumn >= lowerRightColumn) {
                    edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), lowerRightColumn - 1));
                    slider_upper_left_column->SetValue(lowerRightColumn - 1);
                }
            } else {
                edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), slider_lower_right_column->GetMax()));
                slider_lower_right_column->SetValue(slider_lower_right_column->GetMax());
            }
        }
    } else {
        edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), slider_lower_right_column->GetMax()));
        slider_lower_right_column->SetValue(slider_lower_right_column->GetMax());
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onSlideUpperLeftRow(wxScrollEvent &event)
{
    int upperLeftRowValue = slider_upper_left_row->GetValue();
    if (upperLeftRowValue <= slider_upper_left_row->GetMax() - 2) {
        edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), upperLeftRowValue));
        slider_upper_left_row->SetValue(upperLeftRowValue);
    } else {
        edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), slider_upper_left_row->GetMax() - 2));
        slider_upper_left_row->SetValue(slider_upper_left_row->GetMax() - 2);
    }

    upperLeftRowValue = boost::lexical_cast<int>(trim(std::string(edit_upper_left_row->GetValue().mb_str())));
    if (slider_lower_right_row->GetValue() <= upperLeftRowValue) {
        edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), upperLeftRowValue + 1));
        slider_lower_right_row->SetValue(upperLeftRowValue + 1);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onSlideUpperLeftColumn(wxScrollEvent &event)
{
    int upperLeftColumnValue = slider_upper_left_column->GetValue();
    if (upperLeftColumnValue <= slider_upper_left_column->GetMax() - 2) {
        edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), upperLeftColumnValue));
        slider_upper_left_column->SetValue(upperLeftColumnValue);
    } else {
        edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), slider_upper_left_column->GetMax() - 2));
        slider_upper_left_column->SetValue(slider_upper_left_column->GetMax() - 2);
    }

    upperLeftColumnValue = boost::lexical_cast<int>(trim(std::string(edit_upper_left_column->GetValue().mb_str())));
    if (slider_lower_right_column->GetValue() <= upperLeftColumnValue) {
        edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), upperLeftColumnValue + 1));
        slider_lower_right_column->SetValue(upperLeftColumnValue + 1);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onSlideLowerRightRow(wxScrollEvent &event)
{
    int lowerRightRowValue = slider_lower_right_row->GetValue();
    if (lowerRightRowValue >= 2) {
        edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), lowerRightRowValue));
        slider_lower_right_row->SetValue(lowerRightRowValue);
    } else {
        edit_lower_right_row->SetValue(wxString::Format(wxT("%i"), 2));
        slider_lower_right_row->SetValue(2);
    }

    lowerRightRowValue = boost::lexical_cast<int>(trim(std::string(edit_lower_right_row->GetValue().mb_str())));
    if (slider_upper_left_row->GetValue() >= lowerRightRowValue) {
        edit_upper_left_row->SetValue(wxString::Format(wxT("%i"), lowerRightRowValue - 1));
        slider_upper_left_row->SetValue(lowerRightRowValue - 1);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onSlideLowerRightColumn(wxScrollEvent &event)
{
    int lowerRightColumnValue = slider_lower_right_column->GetValue();
    if (lowerRightColumnValue >= 2) {
        edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), lowerRightColumnValue));
        slider_lower_right_column->SetValue(lowerRightColumnValue);
    } else {
        edit_lower_right_column->SetValue(wxString::Format(wxT("%i"), 2));
        slider_lower_right_column->SetValue(2);
    }

    lowerRightColumnValue = boost::lexical_cast<int>(trim(std::string(edit_lower_right_column->GetValue().mb_str())));
    if (slider_upper_left_column->GetValue() >= lowerRightColumnValue) {
        edit_upper_left_column->SetValue(wxString::Format(wxT("%i"), lowerRightColumnValue - 1));
        slider_upper_left_column->SetValue(lowerRightColumnValue - 1);
    }

    updateImageOnCrop();
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onEditTextUpperLeftRowEnter(wxCommandEvent &event) {
    wxFocusEvent evt;
    onEditTextUpperLeftRow(evt);
}

void ViewCreatorDialog::onEditTextUpperLeftColumnEnter(wxCommandEvent &event) {
    wxFocusEvent evt;
    onEditTextUpperLeftColumn(evt);
}

void ViewCreatorDialog::onEditTextLowerRightRowEnter(wxCommandEvent &event) {
    wxFocusEvent evt;
    onEditTextLowerRightRow(evt);
}

void ViewCreatorDialog::onEditTextLowerRightColumnEnter(wxCommandEvent &event) {
    wxFocusEvent evt;
    onEditTextLowerRightColumn(evt);
}

void ViewCreatorDialog::onEditTextOrientationX(wxFocusEvent &event)
{
    std::string orientation_x = trim(std::string(edit_orientation_x->GetValue().mb_str()));
    if (!check_string_redex(orientation_x, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_orientation_x->SetValue(wxT("1.0"));
    }
}

void ViewCreatorDialog::onEditTextOrientationY(wxFocusEvent &event)
{
    std::string orientation_y = trim(std::string(edit_orientation_y->GetValue().mb_str()));
    if (!check_string_redex(orientation_y, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_orientation_y->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextOrientationZ(wxFocusEvent &event)
{
    std::string orientation_z = trim(std::string(edit_orientation_z->GetValue().mb_str()));
    if (!check_string_redex(orientation_z, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_orientation_z->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextScore(wxFocusEvent &event)
{
    std::string score = trim(std::string(edit_score_2D->GetValue().mb_str()));
    if (!check_string_redex(score, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        edit_score_2D->SetValue(wxT("0.2"));
    }
}

void ViewCreatorDialog::onEditTextVerticalOffset(wxFocusEvent &event)
{
    std::string vertical_offset = trim(std::string(edit_vertical_offset->GetValue().mb_str()));
    if (!check_string_redex(vertical_offset, boost::regex("^[-+]?[0-9]+$$"))) {
        edit_vertical_offset->SetValue(wxT("0"));
    }
}

void ViewCreatorDialog::onEditTextHorizontalOffset(wxFocusEvent &event)
{
    std::string horizontal_offset = trim(std::string(edit_horizontal_offset->GetValue().mb_str()));
    if (!check_string_redex(horizontal_offset, boost::regex("^[-+]?[0-9]+$"))) {
        edit_horizontal_offset->SetValue(wxT("0"));
    }
}

void ViewCreatorDialog::onEditTextDepth(wxFocusEvent &event)
{
    std::string depth_string = trim(std::string(edit_depth->GetValue().mb_str()));
    if (check_string_redex(depth_string, boost::regex("^[0-9]+$"))) {
        int depth = boost::lexical_cast<int>(trim(std::string(edit_depth->GetValue().mb_str())));
        if ((depth <= 0) || (depth > 15)) {
            edit_depth->SetValue(wxT("8"));
        }
    } else {
        edit_depth->SetValue(wxT("8"));
    }

    if (desc_model_available) {
        HalconCpp::HTuple detName, detValue, descName, descParam;
        desc_model.GetDescriptorModelParams(&detName, &detValue, &descName, &descParam);
        int curr_depth = boost::lexical_cast<int>(trim(std::string(edit_depth->GetValue().mb_str())));
        if (curr_depth != (int)descParam[0]) {
            desc_model_available = false;
            label_model_available_value->SetLabel(wxT("No"));
        }
    }
}

void ViewCreatorDialog::onEditTextFernNumber(wxFocusEvent &event)
{
    std::string fern_number_string = trim(std::string(edit_fern_number->GetValue().mb_str()));
    if (check_string_redex(fern_number_string, boost::regex("^[0-9]+$"))) {
        int fern_number = boost::lexical_cast<int>(trim(std::string(edit_fern_number->GetValue().mb_str())));
        if ((fern_number <= 0) || (fern_number > 500)) {
            edit_fern_number->SetValue(wxT("50"));
        }
    } else {
        edit_fern_number->SetValue(wxT("50"));
    }

    if (desc_model_available) {
        HalconCpp::HTuple detName, detValue, descName, descParam;
        desc_model.GetDescriptorModelParams(&detName, &detValue, &descName, &descParam);
        int curr_ferns = boost::lexical_cast<int>(trim(std::string(edit_fern_number->GetValue().mb_str())));
        if (curr_ferns != (int)descParam[1]) {
            desc_model_available = false;
            label_model_available_value->SetLabel(wxT("No"));
        }
    }
}

void ViewCreatorDialog::onEditTextPatchSize(wxFocusEvent &event)
{
    std::string patch_size_string = trim(std::string(edit_patch_size->GetValue().mb_str()));
    if (check_string_redex(patch_size_string, boost::regex("^[0-9]+$"))) {
        int patch_size = boost::lexical_cast<int>(trim(std::string(edit_patch_size->GetValue().mb_str())));
        if ((patch_size <= 10) || (patch_size > 40) || (patch_size % 2 == 0)) {
            edit_patch_size->SetValue(wxT("17"));
        }
    } else {
        edit_patch_size->SetValue(wxT("17"));
    }

    if (desc_model_available) {
        HalconCpp::HTuple detName, detValue, descName, descParam;
        desc_model.GetDescriptorModelParams(&detName, &detValue, &descName, &descParam);
        int curr_patches = boost::lexical_cast<int>(trim(std::string(edit_patch_size->GetValue().mb_str())));
        if (curr_patches != (int)descParam[2]) {
            desc_model_available = false;
            label_model_available_value->SetLabel(wxT("No"));
        }
    }
}

void ViewCreatorDialog::onEditTextMinScale(wxFocusEvent &event)
{
    std::string min_scale_string = trim(std::string(edit_min_scale->GetValue().mb_str()));
    if (check_string_redex(min_scale_string, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double min_scale = boost::lexical_cast<double>(trim(std::string(edit_min_scale->GetValue().mb_str())));
        if ((min_scale < 0.2) || (min_scale > 1.2)) {
            edit_min_scale->SetValue(wxT("0.6"));
        }
    } else {
        edit_min_scale->SetValue(wxT("0.6"));
    }

    if (desc_model_available) {
        HalconCpp::HTuple detName, detValue, descName, descParam;
        desc_model.GetDescriptorModelParams(&detName, &detValue, &descName, &descParam);
        double curr_min_scale = boost::lexical_cast<double>(trim(std::string(edit_min_scale->GetValue().mb_str())));
        if (curr_min_scale != (double)descParam[8]) {
            desc_model_available = false;
            label_model_available_value->SetLabel(wxT("No"));
        }
    }
}

void ViewCreatorDialog::onEditTextMaxScale(wxFocusEvent &event)
{
    std::string max_scale_string = trim(std::string(edit_max_scale->GetValue().mb_str()));
    if (check_string_redex(max_scale_string, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double max_scale = boost::lexical_cast<double>(trim(std::string(edit_max_scale->GetValue().mb_str())));
        if ((max_scale < 0.8) || (max_scale > 1.8)) {
            edit_max_scale->SetValue(wxT("1.1"));
        }
    } else {
        edit_max_scale->SetValue(wxT("1.1"));
    }

    if (desc_model_available) {
        HalconCpp::HTuple detName, detValue, descName, descParam;
        desc_model.GetDescriptorModelParams(&detName, &detValue, &descName, &descParam);
        double curr_max_scale = boost::lexical_cast<double>(trim(std::string(edit_max_scale->GetValue().mb_str())));
        if (curr_max_scale != (double)descParam[9]) {
            desc_model_available = false;
            label_model_available_value->SetLabel(wxT("No"));
        }
    }
}

void ViewCreatorDialog::onCheckUpsideDown(wxCommandEvent &event)
{
    //Do nothing
}

void ViewCreatorDialog::onCheckUseColor(wxCommandEvent &event)
{
    desc_model_available = false;
    label_model_available_value->SetLabel(wxT("No"));
}

void ViewCreatorDialog::onEditTextAxis1X(wxFocusEvent &event)
{
    std::string axis_1_x = trim(std::string(edit_axis_1_x->GetValue().mb_str()));
    if (!check_string_redex(axis_1_x, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_1_x->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis1Y(wxFocusEvent &event)
{
    std::string axis_1_y = trim(std::string(edit_axis_1_y->GetValue().mb_str()));
    if (!check_string_redex(axis_1_y, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_1_y->SetValue(wxT("1.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis1Z(wxFocusEvent &event)
{
    std::string axis_1_z = trim(std::string(edit_axis_1_z->GetValue().mb_str()));
    if (!check_string_redex(axis_1_z, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_1_z->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis1Angle(wxFocusEvent &event)
{
    std::string axis_1_angle = trim(std::string(edit_axis_1_angle->GetValue().mb_str()));
    if (check_string_redex(axis_1_angle, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double angle = boost::lexical_cast<double>(trim(std::string(edit_axis_1_angle->GetValue().mb_str())));
        if ((angle <= 0) || (angle >= 360)) {
            edit_axis_1_angle->SetValue(wxT("1.0"));
        }
    } else {
        edit_axis_1_angle->SetValue(wxT("1.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis2X(wxFocusEvent &event)
{
    std::string axis_2_x = trim(std::string(edit_axis_2_x->GetValue().mb_str()));
    if (!check_string_redex(axis_2_x, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_2_x->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis2Y(wxFocusEvent &event)
{
    std::string axis_2_y = trim(std::string(edit_axis_2_y->GetValue().mb_str()));
    if (!check_string_redex(axis_2_y, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_2_y->SetValue(wxT("0.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis2Z(wxFocusEvent &event)
{
    std::string axis_2_z = trim(std::string(edit_axis_2_z->GetValue().mb_str()));
    if (!check_string_redex(axis_2_z, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)?$"))) {
        edit_axis_2_z->SetValue(wxT("-1.0"));
    }
}

void ViewCreatorDialog::onEditTextAxis2Angle(wxFocusEvent &event)
{
    std::string axis_2_angle = trim(std::string(edit_axis_2_angle->GetValue().mb_str()));
    if (check_string_redex(axis_2_angle, boost::regex("^[0-9]+(\\.[0-9]+)?$"))) {
        double angle = boost::lexical_cast<double>(trim(std::string(edit_axis_2_angle->GetValue().mb_str())));
        if ((angle <= 0) || (angle >= 360)) {
            edit_axis_2_angle->SetValue(wxT("1.0"));
        }
    } else {
        edit_axis_2_angle->SetValue(wxT("1.0"));
    }
}



void ViewCreatorDialog::onEditTextOrientationXEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextOrientationX(evt);
}

void ViewCreatorDialog::onEditTextOrientationYEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextOrientationY(evt);
}

void ViewCreatorDialog::onEditTextOrientationZEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextOrientationZ(evt);
}

void ViewCreatorDialog::onEditTextScoreEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextScore(evt);
}

void ViewCreatorDialog::onEditTextVerticalOffsetEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextVerticalOffset(evt);
}

void ViewCreatorDialog::onEditTextHorizontalOffsetEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextHorizontalOffset(evt);
}

void ViewCreatorDialog::onEditTextDepthEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextDepth(evt);
}

void ViewCreatorDialog::onEditTextFernNumberEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextFernNumber(evt);
}

void ViewCreatorDialog::onEditTextPatchSizeEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextPatchSize(evt);
}

void ViewCreatorDialog::onEditTextMinScaleEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextMinScale(evt);
}

void ViewCreatorDialog::onEditTextMaxScaleEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextMaxScale(evt);
}

void ViewCreatorDialog::onEditTextAxis1XEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis1X(evt);
}

void ViewCreatorDialog::onEditTextAxis1YEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis1Y(evt);
}

void ViewCreatorDialog::onEditTextAxis1ZEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis1Z(evt);
}

void ViewCreatorDialog::onEditTextAxis1AngleEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis1Angle(evt);
}

void ViewCreatorDialog::onEditTextAxis2XEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis2X(evt);
}

void ViewCreatorDialog::onEditTextAxis2YEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis2Y(evt);
}

void ViewCreatorDialog::onEditTextAxis2ZEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis2Z(evt);
}

void ViewCreatorDialog::onEditTextAxis2AngleEnter(wxCommandEvent &event)
{
    wxFocusEvent evt;
    onEditTextAxis2Angle(evt);
}
