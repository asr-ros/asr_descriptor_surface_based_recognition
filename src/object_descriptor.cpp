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


#include <rviz/mesh_loader.h>

#include <ros/console.h>

#include <boost/lexical_cast.hpp>
#include <ros_uri/ros_uri.hpp>

#include "object_descriptor.h"
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include "util.h"
#include "rotation_axis.h"


namespace descriptor_surface_based_recognition {

ObjectDescriptor::ObjectDescriptor(std::string folder_path, std::string xml_path, int instance_count, bool use_pose_val) : valid_object_(false), instance_count_(instance_count), use_pose_val_(use_pose_val)
{
    std::string package_path = folder_path;
    folder_path = ros_uri::absolute_path(folder_path);
    try {
        rapidxml::file<> xmlFile(xml_path.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());


        rapidxml::xml_node<> *first_node = doc.first_node();
        if (first_node) {
            rapidxml::xml_node<> *node_name = first_node->first_node("name");
            rapidxml::xml_node<> *node_surface_model = first_node->first_node("surface_model");
            rapidxml::xml_node<> *node_mesh = first_node->first_node("mesh");
            rapidxml::xml_node<> *node_rotation_type = first_node->first_node("rotation_model_type");
            rapidxml::xml_node<> *node_orientation = first_node->first_node("model_orientation");
            rapidxml::xml_node<> *node_diameter = first_node->first_node("diameter");
            rapidxml::xml_node<> *node_score3D = first_node->first_node("score3D");
            rapidxml::xml_node<> *node_desc_models = first_node->first_node("descriptor_models");


            if (node_name && node_surface_model && node_mesh && node_rotation_type && node_orientation && node_diameter && node_score3D && node_desc_models) {

                name_ = node_name->value();
                surface_model_ = HalconCpp::HSurfaceModel((folder_path + node_surface_model->value()).c_str());

                mesh_name_ = package_path + node_mesh->value();
                rotation_model_type_ = boost::lexical_cast<int>(node_rotation_type->value());
                orientation_ = parseStringVector(node_orientation->value(), " ,");
                diameter_ = boost::lexical_cast<double>(node_diameter->value());
                score_3D_ = boost::lexical_cast<double>(node_score3D->value());
                std::vector<ObjectViewDescriptor> object_descs;
                for (rapidxml::xml_node<> *node_desc_model = node_desc_models->first_node("descriptor_model"); node_desc_model; node_desc_model = node_desc_model->next_sibling("descriptor_model")) {
                    rapidxml::xml_node<> *node_desc_model_name = node_desc_model->first_node("name");
                    rapidxml::xml_node<> *node_desc_model_orientation = node_desc_model->first_node("view_orientation");
                    rapidxml::xml_node<> *node_desc_model_score2D = node_desc_model->first_node("score2D");
                    rapidxml::xml_node<> *node_desc_model_use_color = node_desc_model->first_node("use_color");
                    rapidxml::xml_node<> *node_desc_model_vertical_offset = node_desc_model->first_node("vertical_texture_offset");
                    rapidxml::xml_node<> *node_desc_model_horizontal_offset = node_desc_model->first_node("horizontal_texture_offset");
                    rapidxml::xml_node<> *node_desc_model_invertible = node_desc_model->first_node("invertible");
                    rapidxml::xml_node<> *node_desc_model_bounding_box = node_desc_model->first_node("bounding_box");
                    if (node_desc_model_name && node_desc_model_orientation && node_desc_model_score2D && node_desc_model_use_color && node_desc_model_use_color && node_desc_model_vertical_offset && node_desc_model_horizontal_offset && node_desc_model_invertible && node_desc_model_bounding_box) {
                        std::vector<RotationAxis> axes;
                        std::vector<Eigen::Vector2i> box_corners;
                        if (rotation_model_type_ != ROTATION_MODEL_NONE) {
                            rapidxml::xml_node<> *node_desc_model_axes = node_desc_model->first_node("rotation_axes");
                            if (node_desc_model_axes) {
                                rapidxml::xml_node<> *axis_1 = node_desc_model_axes->first_node("axis_1");
                                if (axis_1 && axis_1->first_attribute("angle")) {
                                    double angle_1 = boost::lexical_cast<double>(axis_1->first_attribute("angle")->value());
                                    RotationAxis rot_axis_1(angle_1, parseStringVector(axis_1->value(), " ,"));
                                    axes.push_back(rot_axis_1);
                                } else {
                                    //axis_1 invalid
                                    return;
                                }
                                if (rotation_model_type_ == ROTATION_MODEL_SPHERE) {
                                    rapidxml::xml_node<> *axis_2 = node_desc_model_axes->first_node("axis_2");
                                    if (axis_2 && axis_2->first_attribute("angle")) {
                                        double angle_2 = boost::lexical_cast<double>(axis_2->first_attribute("angle")->value());
                                        RotationAxis rot_axis_2(angle_2, parseStringVector(axis_2->value(), " ,"));
                                        axes.push_back(rot_axis_2);
                                    } else {
                                        //axis_2 invalid
                                        return;
                                    }
                                }
                            } else {
                                //no axes node
                                return;
                            }
                        }
                        if ((int)(axes.size()) != rotation_model_type_) {
                            //axes creation error
                            return;
                        }

                        for (rapidxml::xml_node<> *node_corner_point = node_desc_model_bounding_box->first_node("corner_point"); node_corner_point; node_corner_point = node_corner_point->next_sibling("corner_point")) {
                            box_corners.push_back(parseStringVector2i(node_corner_point->value(), " ,"));
                        }
                        if (box_corners.size() != 4) {
                            //invalid bounding box
                            return;
                        }

                        HalconCpp::HDescriptorModel desc_mdl((folder_path + node_desc_model_name->value()).c_str());
                        Eigen::Vector3d desc_orientation = parseStringVector(node_desc_model_orientation->value(), " ,");
                        double desc_score2D = boost::lexical_cast<double>(node_desc_model_score2D->value());
                        bool desc_use_color = boost::lexical_cast<bool>(node_desc_model_use_color->value());
                        int desc_vertical_offset = boost::lexical_cast<int>(node_desc_model_vertical_offset->value());
                        int desc_horizontal_offset = boost::lexical_cast<int>(node_desc_model_horizontal_offset->value());
                        bool desc_invertible = boost::lexical_cast<bool>(node_desc_model_invertible->value());
                        object_descs.push_back(ObjectViewDescriptor(desc_mdl, desc_orientation, desc_score2D, desc_use_color, desc_vertical_offset, desc_horizontal_offset, desc_invertible, axes, box_corners));
                    }
                }
                model_view_descriptors_ = object_descs;
                valid_object_ = true;
            } else {
                ROS_ERROR_STREAM("Could not create object: " << folder_path << " (node missing)");
            }
        }
    } catch(std::runtime_error err) {
        ROS_ERROR_STREAM("Could not create object: " << folder_path << " (XML-parser error)");
    } catch(HalconCpp::HException exc) {
        ROS_ERROR_STREAM(exc.ErrorText());
        ROS_ERROR_STREAM("Could not create object: " << folder_path << " (Halcon error)");
    } catch(boost::bad_lexical_cast) {
        ROS_ERROR_STREAM("Could not create object: " << folder_path << " (Boost cast error)");
    } catch (rapidxml::parse_error err) {
        ROS_ERROR_STREAM("Could not create object: " << folder_path << " (XML-parser error)");
    }
}

bool ObjectDescriptor::isValid() const { return valid_object_; }

std::string ObjectDescriptor::getName() const { return name_; }

HalconCpp::HSurfaceModel ObjectDescriptor::getSurfaceModel() const { return surface_model_; }

std::string ObjectDescriptor::getMesh() const { return mesh_name_; }

int ObjectDescriptor::getRotationModelType() const { return rotation_model_type_; }

Eigen::Vector3d ObjectDescriptor::getOrientation() const { return orientation_; }

double ObjectDescriptor::getDiameter() const { return diameter_; }

double ObjectDescriptor::getScore3D() const { return score_3D_; }

std::vector<ObjectViewDescriptor> ObjectDescriptor::getModelViewDescriptors() const { return model_view_descriptors_; }

int ObjectDescriptor::getInstanceCount() const { return instance_count_; }

bool ObjectDescriptor::getUsePoseVal() const { return use_pose_val_; }


void ObjectDescriptor::setCount(int count) {
    if (count > 0) {
        this->instance_count_ = count;
    }
}

void ObjectDescriptor::setUsePoseVal(bool usePoseVal) {this->use_pose_val_ = usePoseVal; }

}
