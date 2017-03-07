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


#include "descriptor_surface_based_trainer/Create_Output_Files_Dialog.h"
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <fstream>
#include <ros/package.h>
#include "descriptor_surface_based_trainer/Utils.h"


void CreateOutputFilesDialog::createOutputFiles()
{

    boost::filesystem::path output_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + OUTPUT_FOLDER + "/" + object_params.at(0) + "/");
    boost::filesystem::path input_path(ros::package::getPath("asr_descriptor_surface_based_recognition") + INPUT_FOLDER + "/");
    boost::filesystem::create_directory(output_path);

    std::ofstream logFile;
    std::string log_filename = output_path.string() + "log.txt";
    logFile.open (log_filename.c_str());
    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Start creation.\n";


    rapidxml::xml_document<> doc;
    rapidxml::xml_node<>* decl = doc.allocate_node(rapidxml::node_declaration);
    decl->append_attribute(doc.allocate_attribute("version", "1.0"));
    decl->append_attribute(doc.allocate_attribute("encoding", "utf-8"));
    doc.append_node(decl);

    rapidxml::xml_node<>* root = doc.allocate_node(rapidxml::node_element, "object");
    doc.append_node(root);

    rapidxml::xml_node<>* node_name = doc.allocate_node(rapidxml::node_element, "name");
    node_name->value(object_params.at(0).c_str());
    root->append_node(node_name);

    rapidxml::xml_node<>* node_surface_mdl = doc.allocate_node(rapidxml::node_element, "surface_model");
    std::string surface_mdl_name = object_params.at(0) + "_surfaceMdl.sfm";
    node_surface_mdl->value(surface_mdl_name.c_str());
    root->append_node(node_surface_mdl);

    rapidxml::xml_node<>* node_mesh = doc.allocate_node(rapidxml::node_element, "mesh");
    node_mesh->value(object_params.at(2).c_str());
    root->append_node(node_mesh);

    rapidxml::xml_node<>* node_rotation_type = doc.allocate_node(rapidxml::node_element, "rotation_model_type");
    std::string rot_type = "0";
    if (object_params.at(3) == ROTATIONTYPE_CYLINDER) { rot_type = "1";}
    if (object_params.at(3) == ROTATIONTYPE_SPHERE) { rot_type = "2";}
    node_rotation_type->value(rot_type.c_str());
    root->append_node(node_rotation_type);

    rapidxml::xml_node<>* node_orientation = doc.allocate_node(rapidxml::node_element, "model_orientation");
    std::string orientation = object_params.at(4) + ", " + object_params.at(5) + ", " + object_params.at(6);
    node_orientation->value(orientation.c_str());
    root->append_node(node_orientation);

    rapidxml::xml_node<>* node_diameter = doc.allocate_node(rapidxml::node_element, "diameter");
    node_diameter->value(object_params.at(7).c_str());
    root->append_node(node_diameter);

    rapidxml::xml_node<>* node_score3D = doc.allocate_node(rapidxml::node_element, "score3D");
    node_score3D->value(object_params.at(8).c_str());
    root->append_node(node_score3D);

    rapidxml::xml_node<>* node_desc_models = doc.allocate_node(rapidxml::node_element, "descriptor_models");

    for (int i = 0; i < views.size(); i++) {

        rapidxml::xml_node<>* node_desc_model = doc.allocate_node(rapidxml::node_element, "descriptor_model");

        rapidxml::xml_node<>* node_desc_model_name = doc.allocate_node(rapidxml::node_element, "name");
        std::string desc_model_name = object_params.at(0) + "_view_" + boost::lexical_cast<std::string>(i + 1) + ".dsm";

        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Creating descriptor model (" << desc_model_name << ") with params: depth = " << views.at(i).getDepth() << ", fern-number = " << views.at(i).getNumberFerns() << ", patch-size = " << views.at(i).getPatchSize() << ", min-scale = " << views.at(i).getMinScale() << ", max-scale = " << views.at(i).getMaxScale() << "\n";

        node_desc_model_name->value(doc.allocate_string(desc_model_name.c_str()));
        node_desc_model->append_node(node_desc_model_name);

        std::string view_image_name = object_params.at(0) + "_view_" + boost::lexical_cast<std::string>(i + 1) + ".png";
        try {
            //create desc model
            HalconCpp::HImage img = views.at(i).getImage();
            if (!(views.at(i).getUseColor())) {
                img = img.Rgb1ToGray();
            }
            img.WriteImage("png", 0, (output_path.string() + view_image_name).c_str());
            HalconCpp::HTuple descparamname("depth");
            descparamname.Append("number_ferns");
            descparamname.Append("patch_size");
            descparamname.Append("max_scale");
            descparamname.Append("min_scale");
            HalconCpp::HTuple descparamvalue(views.at(i).getDepth());
            descparamvalue.Append(views.at(i).getNumberFerns());
            descparamvalue.Append(views.at(i).getPatchSize());
            descparamvalue.Append(views.at(i).getMaxScale());
            descparamvalue.Append(views.at(i).getMinScale());
            img.CreateUncalibDescriptorModel("harris_binomial", HalconCpp::HTuple(), HalconCpp::HTuple(), descparamname, descparamvalue, 42).WriteDescriptorModel((output_path.string() + desc_model_name).c_str());
            logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Descriptor model creation successful (" << desc_model_name << ").\n";
        } catch(HalconCpp::HException exc) {
            logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Could not create descriptor model (" << desc_model_name << "), Error msg: " << exc.ErrorText() << ".\n";
            success = false;
            continue;
        }



        rapidxml::xml_node<>* node_desc_model_orientation = doc.allocate_node(rapidxml::node_element, "view_orientation");
        std::string view_or_0 = boost::lexical_cast<std::string>(views.at(i).getOrientation()[0]);
        std::string view_or_1 = boost::lexical_cast<std::string>(views.at(i).getOrientation()[1]);
        std::string view_or_2 = boost::lexical_cast<std::string>(views.at(i).getOrientation()[2]);
        std::replace(view_or_0.begin(), view_or_0.end(), ',', '.');
        std::replace(view_or_1.begin(), view_or_1.end(), ',', '.');
        std::replace(view_or_2.begin(), view_or_2.end(), ',', '.');
        std::string desc_model_orientation = view_or_0 + ", " + view_or_1 + ", " + view_or_2;

        node_desc_model_orientation->value(doc.allocate_string(desc_model_orientation.c_str()));
        node_desc_model->append_node(node_desc_model_orientation);

        rapidxml::xml_node<>* node_desc_model_score2D = doc.allocate_node(rapidxml::node_element, "score2D");
        std::string desc_model_score2D = boost::lexical_cast<std::string>(views.at(i).getScore2D());
        std::replace(desc_model_score2D.begin(), desc_model_score2D.end(), ',', '.');
        node_desc_model_score2D->value(doc.allocate_string(desc_model_score2D.c_str()));
        node_desc_model->append_node(node_desc_model_score2D);

        rapidxml::xml_node<>* node_desc_model_use_color = doc.allocate_node(rapidxml::node_element, "use_color");
        std::string desc_model_use_color = boost::lexical_cast<std::string>(views.at(i).getUseColor());
        node_desc_model_use_color->value(doc.allocate_string(desc_model_use_color.c_str()));
        node_desc_model->append_node(node_desc_model_use_color);

        rapidxml::xml_node<>* node_desc_model_vertical_offset = doc.allocate_node(rapidxml::node_element, "vertical_texture_offset");
        std::string desc_model_vertical_offset = boost::lexical_cast<std::string>(views.at(i).getVerticalOffset());
        node_desc_model_vertical_offset->value(doc.allocate_string(desc_model_vertical_offset.c_str()));
        node_desc_model->append_node(node_desc_model_vertical_offset);

        rapidxml::xml_node<>* node_desc_model_horizontal_offset = doc.allocate_node(rapidxml::node_element, "horizontal_texture_offset");
        std::string desc_model_horizontal_offset = boost::lexical_cast<std::string>(views.at(i).getHorizontalOffset());
        node_desc_model_horizontal_offset->value(doc.allocate_string(desc_model_horizontal_offset.c_str()));
        node_desc_model->append_node(node_desc_model_horizontal_offset);

        rapidxml::xml_node<>* node_desc_model_invertible = doc.allocate_node(rapidxml::node_element, "invertible");
        std::string desc_model_invertible = boost::lexical_cast<std::string>(views.at(i).getIsInvertible());
        node_desc_model_invertible->value(doc.allocate_string(desc_model_invertible.c_str()));
        node_desc_model->append_node(node_desc_model_invertible);

        rapidxml::xml_node<>* node_desc_model_bounding_box = doc.allocate_node(rapidxml::node_element, "bounding_box");
        for (int j = 0; j < views.at(i).getBoxCorners().size(); j++) {
            rapidxml::xml_node<>* node_desc_model_bounding_box_corner = doc.allocate_node(rapidxml::node_element, "corner_point");
            std::string point_vector = boost::lexical_cast<std::string>((int)(views.at(i).getBoxCorners().at(j)[0] - 0.5 * (int)views.at(i).getImage().Width())) + ", " + boost::lexical_cast<std::string>((int)(views.at(i).getBoxCorners().at(j)[1] - 0.5 * (int)views.at(i).getImage().Height()));
            node_desc_model_bounding_box_corner->value(doc.allocate_string(point_vector.c_str()));
            node_desc_model_bounding_box->append_node(node_desc_model_bounding_box_corner);
        }

        node_desc_model->append_node(node_desc_model_bounding_box);


        if (views.at(i).getRotationType() != ROTATIONTYPE_NO_ROTATION) {
            rapidxml::xml_node<>* node_desc_model_axes = doc.allocate_node(rapidxml::node_element, "rotation_axes");

            rapidxml::xml_node<>* node_desc_model_axis_1 = doc.allocate_node(rapidxml::node_element, "axis_1");
            std::string axis_1_x =boost::lexical_cast<std::string>(views.at(i).getAxis1()[0]);
            std::string axis_1_y =boost::lexical_cast<std::string>(views.at(i).getAxis1()[1]);
            std::string axis_1_z =boost::lexical_cast<std::string>(views.at(i).getAxis1()[2]);
            std::replace(axis_1_x.begin(), axis_1_x.end(), ',', '.');
            std::replace(axis_1_y.begin(), axis_1_y.end(), ',', '.');
            std::replace(axis_1_z.begin(), axis_1_z.end(), ',', '.');

            std::string desc_model_axis_1 = axis_1_x + ", " + axis_1_y + ", " + axis_1_z;
            node_desc_model_axis_1->value(doc.allocate_string(desc_model_axis_1.c_str()));
            std::string desc_model_axis_1_angle = boost::lexical_cast<std::string>(views.at(i).getAxis1Angle());
            std::replace(desc_model_axis_1_angle.begin(), desc_model_axis_1_angle.end(), ',', '.');

            rapidxml::xml_attribute<> *axis_1_angle_attr = doc.allocate_attribute("angle", doc.allocate_string(desc_model_axis_1_angle.c_str()));
            node_desc_model_axis_1->append_attribute(axis_1_angle_attr);
            node_desc_model_axes->append_node(node_desc_model_axis_1);

            if (views.at(i).getRotationType() == ROTATIONTYPE_SPHERE) {
                rapidxml::xml_node<>* node_desc_model_axis_2 = doc.allocate_node(rapidxml::node_element, "axis_2");
                std::string axis_2_x =boost::lexical_cast<std::string>(views.at(i).getAxis2()[0]);
                std::string axis_2_y =boost::lexical_cast<std::string>(views.at(i).getAxis2()[1]);
                std::string axis_2_z =boost::lexical_cast<std::string>(views.at(i).getAxis2()[2]);
                std::replace(axis_2_x.begin(), axis_2_x.end(), ',', '.');
                std::replace(axis_2_y.begin(), axis_2_y.end(), ',', '.');
                std::replace(axis_2_z.begin(), axis_2_z.end(), ',', '.');
                std::string desc_model_axis_2 = axis_2_x + ", " + axis_2_y + ", " + axis_2_z;
                node_desc_model_axis_2->value(doc.allocate_string(desc_model_axis_2.c_str()));
                std::string desc_model_axis_2_angle = boost::lexical_cast<std::string>(views.at(i).getAxis2Angle());
                std::replace(desc_model_axis_2_angle.begin(), desc_model_axis_2_angle.end(), ',', '.');
                rapidxml::xml_attribute<> *axis_2_angle_attr = doc.allocate_attribute("angle", doc.allocate_string(desc_model_axis_2_angle.c_str()));
                node_desc_model_axis_2->append_attribute(axis_2_angle_attr);
                node_desc_model_axes->append_node(node_desc_model_axis_2);
            }

            node_desc_model->append_node(node_desc_model_axes);
        }
        node_desc_models->append_node(node_desc_model);

        progress_bar->SetValue(progress_bar->GetValue() + 1);
    }
    root->append_node(node_desc_models);

    std::string filename = object_params.at(0) + ".xml";
    filename = output_path.string() + filename;
    std::ofstream file_stored(filename.c_str());
    file_stored << doc;
    file_stored.close();
    doc.clear();

    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "XML-file successfully created.\n";

    progress_bar->SetValue(progress_bar->GetValue() + 1);

    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Creating surface model.\n";
    try {
        std::string object_model_path = input_path.string() + object_params.at(1);
        HalconCpp::HObjectModel3D model;
        model.ReadObjectModel3d(object_model_path.c_str(), "mm", HalconCpp::HTuple(), HalconCpp::HTuple());
        HalconCpp::HSurfaceModel surface_mdl = model.CreateSurfaceModel(0.035, HalconCpp::HTuple(), HalconCpp::HTuple());
        surface_mdl.WriteSurfaceModel((output_path.string() + object_params.at(0) + "_surfaceMdl.sfm").c_str());
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Surface model successfully created.\n";
    } catch (HalconCpp::HException exc) {
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Surface model could not be created, error msg: " << exc.ErrorText() << ".\n";
        success = false;
    }
    progress_bar->SetValue(progress_bar->GetValue() + 1);

    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Copying mesh file.\n";
    std::string mesh_path = input_path.string() + object_params.at(2);
    std::string mesh_destination_path = output_path.string() + object_params.at(2);
    try {
        boost::filesystem::copy_file(mesh_path, mesh_destination_path, boost::filesystem::copy_option::overwrite_if_exists);
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Mesh file successfully copied.\n";
    } catch (boost::filesystem::filesystem_error err) {
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Could not copy mesh file.\n";
        success = false;
    }
    progress_bar->SetValue(progress_bar->GetValue() + 1);

    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Copying texture-file of mesh.\n";
    try {
        rapidxml::file<> xmlFile(mesh_path.c_str());
        rapidxml::xml_document<> mesh_doc;
        mesh_doc.parse<0>(xmlFile.data());
        if ((mesh_doc.first_node()) && (mesh_doc.first_node()->first_node("library_images")) && (mesh_doc.first_node()->first_node("library_images")->first_node("image")) && mesh_doc.first_node()->first_node("library_images")->first_node("image")->first_node("init_from")) {
            std::string texture_name = mesh_doc.first_node()->first_node("library_images")->first_node("image")->first_node("init_from")->value();
            std::string texture_path = input_path.string() + texture_name;
            std::string texture_destination_path = output_path.string() + texture_name;
            boost::filesystem::copy_file(texture_path, texture_destination_path, boost::filesystem::copy_option::overwrite_if_exists);
            logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Texture file successfully copied.\n";
        } else {
            logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Could not copy mesh file.\n";
            success = false;
        }
    } catch(boost::filesystem::filesystem_error err) {
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Could not copy mesh file.\n";
        success = false;
    } catch (std::runtime_error err) {
        logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Error: Could not copy mesh file.\n";
        success = false;
    }
    progress_bar->SetValue(progress_bar->GetValue() + 1);



    if (progress_bar->GetValue() < progress_bar->GetRange()) {
        progress_bar->SetValue(progress_bar->GetRange());
    }
    logFile << "[" << boost::lexical_cast<std::string>(ros::Time::now().toSec()) << "] " << "Output files creation done.\n";
    logFile.close();
    button_done->Enable();

}

CreateOutputFilesDialog::CreateOutputFilesDialog(DescModelsDialog* parent_dialog)
    : Create_Output_Files_Dialog_Base(parent_dialog),
      views(parent_dialog->getViews()),
      object_params(parent_dialog->getObjectParameters()),
      success(true)
{
    button_done->Enable(false);
    progress_bar->SetRange(4 + views.size());

    boost::thread create_files_thread(boost::bind(&CreateOutputFilesDialog::createOutputFiles, this));
}

void CreateOutputFilesDialog::onButtonDoneClicked(wxCommandEvent &event)
{
    if (success) {
        EndModal(wxID_OK);
    } else {
        EndModal(wxID_CANCEL);
    }
    Destroy();
}

void CreateOutputFilesDialog::onDialogClose(wxCloseEvent &event)
{

}
