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
#include <rviz/ogre_helpers/render_system.h>

#include <OGRE/Ogre.h>
#include <OGRE/OgreException.h>
#include <OGRE/OgreRoot.h>

#include <OgreMesh.h>
#include <OGRE/OgreLight.h>

#include <ros/package.h>
#include <ros/console.h>
#include <ros_uri/ros_uri.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <halcon_image.h>

#include <iostream>
#include <fstream>

#include "descriptor_surface_based_recognition.h"
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>





namespace descriptor_surface_based_recognition {

std::string PACKAGE_PATH;

DescriptorSurfaceBasedRecognition::DescriptorSurfaceBasedRecognition() : nh_(NODE_NAME), frame_counter_(0)
{

    ROS_DEBUG_STREAM("Initialize DescriptorSurfaceBasedRecognition");

    double far_plane = 100;
    double near_plane = 0.01;
    double cx = 648.95153;
    double cy = 468.29311;
    double fx = 1689.204742;
    double fy = 1689.204742;
    double image_width = 1292.0;
    double image_height = 964.0;
    int render_width = 800;
    int render_height = 600;
    nh_.getParam("pose_val_far_plane", far_plane);
    nh_.getParam("pose_val_near_plane", near_plane);
    nh_.getParam("pose_val_cx", cx);
    nh_.getParam("pose_val_cy", cy);
    nh_.getParam("pose_val_fx", fx);
    nh_.getParam("pose_val_fy", fy);
    nh_.getParam("pose_val_image_width", image_width);
    nh_.getParam("pose_val_image_height", image_height);
    nh_.getParam("pose_val_render_image_width", render_width);
    nh_.getParam("pose_val_render_image_height", render_height);

    nh_.getParam("image_color_topic", image_color_topic_);
    nh_.getParam("image_mono_topic", image_mono_topic_);
    nh_.getParam("point_cloud_topic", point_cloud_topic_);
    nh_.getParam("output_objects_topic", output_objects_topic_);
    nh_.getParam("output_marker_topic", output_marker_topic_);
    nh_.getParam("output_marker_bounding_box_topic", output_marker_bounding_box_topic_);
    nh_.getParam("output_cloud_topic", output_cloud_topic_);
    nh_.getParam("output_image_topic", output_image_topic_);

    PACKAGE_PATH = ros::package::getPath("descriptor_surface_based_recognition");

    // Set up dynamic reconfigure
    reconfigure_server_.setCallback(boost::bind(&DescriptorSurfaceBasedRecognition::configCallback, this, _1, _2));

    if (config_.evaluation) {
        //clear output file
        std::string path = PACKAGE_PATH + "/" + OUTPUT_EVALUATION_DIR;
        boost::filesystem::path dir(path.c_str());
        boost::filesystem::create_directory(dir);
        outstream_times_.open((path + "/" + OUTPUT_EVALUATION_FILE_TIME).c_str(), std::ofstream::out | std::ofstream::trunc);
        outstream_poses_.open((path + "/" + OUTPUT_EVALUATION_FILE_POSES).c_str(), std::ofstream::out | std::ofstream::trunc);
    }

    object_db_service_client_ = nh_.serviceClient<object_database::ObjectMetaData>(OBJECT_DB_SERVICE_OBJECT_TYPE);
    object_db_meshes_service_client_ = nh_.serviceClient<object_database::RecognizerListMeshes>(OBJECT_DB_SERVICE_OBJECT_MESHES);

    ROS_DEBUG_STREAM("Database clients initialized");

    if (config_.usePoseValidation) {
        pose_val_ = PoseValidation(image_width, image_height, far_plane, near_plane, cx, cy, fx, fy, render_width, render_height);
        initializeMeshes();
    }

    ROS_DEBUG_STREAM("Pose validation initialized");

    thread_pool_ = boost::threadpool::pool(4);
    msgs_marker_array_ = boost::make_shared<visualization_msgs::MarkerArray>();
    msgs_box_marker_array_ = boost::make_shared<visualization_msgs::MarkerArray>();

    //ros services
    get_recognizer_service_ = nh_.advertiseService(GET_RECOGNIZER_SERVICE_NAME, &DescriptorSurfaceBasedRecognition::processGetRecognizerRequest, this);
    release_recognizer_service_ = nh_.advertiseService(RELEASE_RECOGNIZER_SERVICE_NAME, &DescriptorSurfaceBasedRecognition::processReleaseRecognizerRequest, this);
    get_object_list_service_ = nh_.advertiseService(GET_OBJECT_LIST_SERVICE_NAME, &DescriptorSurfaceBasedRecognition::processGetObjectListRequest, this);
    clear_all_recognizers_service_ = nh_.advertiseService(CLEAR_ALL_RECOGNIZERS_SERVICE_NAME, &DescriptorSurfaceBasedRecognition::processClearAllRecognizersRequest, this);

    //ros subscriptions
    image_sub_.subscribe(nh_, image_color_topic_, 1);
    image_mono_sub_.subscribe(nh_, image_mono_topic_, 1);
    pc_with_guppy_sub_.subscribe(nh_, point_cloud_topic_, 1);

    //ros publishers
    marker_pub_ = nh_.advertise<visualization_msgs::Marker> (output_marker_topic_, 1);
    boxes_pub_ = nh_.advertise<visualization_msgs::MarkerArray> (output_marker_bounding_box_topic_, 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (output_cloud_topic_, 1);
    objects_pub_ = nh_.advertise<asr_msgs::AsrObject> (output_objects_topic_, 1);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (output_image_topic_, 1);

    sync_policy_.reset(new ApproximateSync(ApproximatePolicy(20), image_sub_, image_mono_sub_, pc_with_guppy_sub_) );
    sync_policy_->registerCallback(boost::bind(&DescriptorSurfaceBasedRecognition::rosCallback, this, _1, _2, _3));

    ROS_DEBUG_STREAM("ROS services and publishers initialized");

    ROS_INFO_STREAM("TO RECOGNIZE A NEW OBJECT CALL THE FOLLOWING SERVICE: " << std::endl <<
                    "                                       /descriptor_surface_based_recognition/get_recognizer <object_type_name> <instance_number> <use_pose_validation>" << std::endl <<
                    "                                TO END RECOGNITION FOR A SPECIFIC OBJECT CALL: " << std::endl <<
                    "                                       /descriptor_surface_based_recognition/release_recognizer <object_type_name>" << std::endl <<
                    "                                TO END RECOGNITION FOR ALL OBJECTS OF A TOPIC CALL: " << std::endl <<
                    "                                       /descriptor_surface_based_recognition/clear_all_recognizers" << std::endl <<
                    "                                TO GET A LIST OF ALL OBJECTS IT IS CURRENTLY SEARCHED FOR CALL: " << std::endl <<
                    "                                       /descriptor_surface_based_recognition/get_object_list");
}

bool DescriptorSurfaceBasedRecognition::processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res) {
    std::string name = req.object_name;
    int count = req.count;
    if ((count < 1) || (count > 10)) {
        count = 1;
    }
    bool use_pose_val = req.use_pose_val;

    startObjectRecognition(name, count, use_pose_val) ? res.success = true : res.success = false;
    res.object_name = name;

    return true;
}

bool DescriptorSurfaceBasedRecognition::processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res) {
    std::string name = req.object_name;
    stopObjectRecognition(name);
    return true;
}

bool DescriptorSurfaceBasedRecognition::processGetObjectListRequest(GetObjectList::Request &req, GetObjectList::Response &res) {
    std::vector<std::string> object_names;
    for (std::vector<ObjectDescriptor>::iterator iter = objects_.begin(); iter != objects_.end(); ++iter) {
        object_names.push_back(iter->getName());
    }
    res.objects = object_names;
    return true;
}

bool DescriptorSurfaceBasedRecognition::processClearAllRecognizersRequest(ClearAllRecognizers::Request &req, ClearAllRecognizers::Response &res) {
    objects_.clear();
    meshes_.clear();
    return true;
}

void DescriptorSurfaceBasedRecognition::initializeMeshes() {

    object_database::RecognizerListMeshes objectMeshes;
    objectMeshes.request.recognizer = OBJECT_DATABASE_CATEGORY;
    object_db_meshes_service_client_.call(objectMeshes.request, objectMeshes.response);

    std::vector<std::string> mesh_paths = objectMeshes.response.meshes;
    for (std::vector<std::string>::iterator iter = mesh_paths.begin(); iter != mesh_paths.end(); ++iter) {
        rviz::loadMeshFromResource(*iter);
    }
}

bool DescriptorSurfaceBasedRecognition::startObjectRecognition(std::string name, int count, bool use_pose_val) {
    object_database::ObjectMetaData objectType;
    objectType.request.object_type = name;
    objectType.request.recognizer = OBJECT_DATABASE_CATEGORY;
    object_db_service_client_.call(objectType.request, objectType.response);
    if (objectType.response.is_valid) {
        std::string object_name = name;
        std::string xml_path = ros_uri::absolute_path(objectType.response.object_folder) + "/" + name + ".xml";

        try {
            rapidxml::file<> xmlFile(xml_path.c_str());
            rapidxml::xml_document<> doc;
            doc.parse<0>(xmlFile.data());

            rapidxml::xml_node<> *first_node = doc.first_node();
            if (first_node) {
                rapidxml::xml_node<> *node_name = first_node->first_node("name");
                if (node_name) {
                    object_name = node_name->value();
                }

            }
        } catch(std::runtime_error err) {
            ROS_DEBUG_STREAM("Can't parse meta-xml-file (add_object -> name). error: " << err.what());
        } catch (rapidxml::parse_error err) {
            ROS_DEBUG_STREAM("Can't parse meta-xml-file (add_object -> name). error: " << err.what());
        }

        bool object_in_list = false;
        for (unsigned int i = 0; i < objects_.size(); i++) {
            if (objects_.at(i).getName() == object_name) {
                if (objects_.at(i).getInstanceCount() != count) {
                    objects_.at(i).setCount(count);
                }
                if (objects_.at(i).getUsePoseVal() != use_pose_val) {
                    objects_.at(i).setUsePoseVal(use_pose_val);
                }
                object_in_list = true;
                break;
            }
        }
        if (!object_in_list) {
            ObjectDescriptor object(objectType.response.object_folder + "/", xml_path, count, use_pose_val);
            if (object.isValid()) {
                objects_.push_back(object);
                if (pose_val_.isInitialized()) {
                    meshes_.push_back(rviz::loadMeshFromResource(object.getMesh()));
                }
            }

        }
        return true;
    }
    return false;
}


void DescriptorSurfaceBasedRecognition::stopObjectRecognition(std::string name) {
    for (unsigned int i = 0; i < objects_.size(); i++) {
        if (objects_.at(i).getName() == name) {
            objects_.erase(objects_.begin() + i);
            if (pose_val_.isInitialized()) {
                meshes_.erase(meshes_.begin() + i);
            }
            break;
        }
    }
}



void DescriptorSurfaceBasedRecognition::rosCallback(const sensor_msgs::ImageConstPtr &input_image_guppy, const sensor_msgs::ImageConstPtr& input_image_guppy_mono, const sensor_msgs::PointCloud2ConstPtr &input_point_cloud_with_guppy)
{
    frame_counter_++;

    if (objects_.size() > 0) {
        ROS_INFO_STREAM("Enter callback for new frame");
        ros::Time time_frame_enter = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_with_guppy_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_point_cloud_with_guppy, *point_cloud_with_guppy_ptr);

        HalconCpp::HImage scene_image = *halcon_bridge::toHalconCopy(input_image_guppy)->image;
        HalconCpp::HImage scene_image_mono = *halcon_bridge::toHalconCopy(input_image_guppy_mono)->image;

        ROS_INFO_STREAM("Search objects");
        std::string object_list = "[";
        for (unsigned int i = 0; i < objects_.size(); i++) {
            if (i > 0) {
                object_list += ", ";
            }
            object_list += objects_.at(i).getName();
        }
        object_list += "]";
        ROS_DEBUG_STREAM("Searching for: " << object_list);

        //prepare search
        std::vector<PoseRecognition*> pose_recs;
        for (unsigned int i = 0; i < objects_.size(); i++) {
            HalconCpp::HImage scene_image_curr = scene_image;
            HalconCpp::HImage scene_image_mono_curr = scene_image_mono;
            for (unsigned int j = 0; j < last_frame_positions_.size(); j++) {
                if (last_frame_positions_.at(j).getObjectIndex() == (int)(i)) {
                    if ((int)(last_frame_positions_.at(j).getPositions().size()) == objects_.at(i).getInstanceCount()) {
                        HalconCpp::HRegion region;
                        for (unsigned int k = 0; k < last_frame_positions_.at(j).getPositions().size(); k++) {
                            if (k == 0) {
                                region.GenRectangle1(last_frame_positions_.at(j).getPositions().at(k)[0] - last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[1] - last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[0] + last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[1] + last_frame_positions_.at(j).getSearchRadii().at(k));
                            } else {
                                HalconCpp::HRegion region_curr;
                                region_curr.GenRectangle1(last_frame_positions_.at(j).getPositions().at(k)[0] - last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[1] - last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[0] + last_frame_positions_.at(j).getSearchRadii().at(k), last_frame_positions_.at(j).getPositions().at(k)[1] + last_frame_positions_.at(j).getSearchRadii().at(k));
                                region = region.Union2(region_curr);
                            }
                        }
                        scene_image_curr = scene_image_curr.ReduceDomain(region);
                        scene_image_mono_curr = scene_image_mono_curr.ReduceDomain(region);
                    }
                    break;
                }
            }
            pose_recs.push_back(new PoseRecognition(scene_image_curr, scene_image_mono_curr, point_cloud_with_guppy_ptr, &(objects_.at(i)), config_.medianPointsOffset, config_.samplingDistance, config_.keypointFraction, config_.evaluation, objects_.at(i).getInstanceCount()));
        }



        //search
        for (unsigned int i = 0; i < pose_recs.size(); i++) {
            thread_pool_.schedule(boost::bind(&DescriptorSurfaceBasedRecognition::threadTask, this, pose_recs.at(i)));
        }
        thread_pool_.wait();
        thread_pool_.clear();
        ros::Duration search_duration = ros::Time::now() - time_frame_enter;

        //validate found poses
        ros::Duration validation_duration;
        if (pose_val_.isInitialized()) {
            ros::Time validation_time_start = ros::Time::now();
            for (unsigned int i = 0; i < pose_recs.size(); i++) {
                if (objects_.at(i).getUsePoseVal()) {
                    for (unsigned int j = 0; j < pose_recs.at(i)->getResults().size(); j++) {
                        if (pose_recs.at(i)->getResults().at(j)->checkModelFound()) {
                            HalconCpp::HTuple matrix_tuple = pose_val_.validateObject(&objects_.at(i), pose_recs.at(i)->getResults().at(j), meshes_.at(i));
                            if (matrix_tuple.Length() > 0) {
                                HalconCpp::HHomMat2D matrix;
                                matrix.SetFromTuple(matrix_tuple);
                                if (!(pose_val_.compareHomographyMatrices(pose_recs.at(i)->getResults().at(j)->getTransMatrix2D(), matrix, config_.poseValidationDistanceError))) {
                                    pose_recs.at(i)->getResults().at(j)->setModelFound(false);
                                    pose_recs.at(i)->getResults().at(j)->setPoseValid(false);
                                    ROS_DEBUG_STREAM("Found homography of object " << objects_.at(i).getName() << " (instance " << j << ") in the validation image does not match the homography found in the scene");
                                }
                            } else {
                                ROS_DEBUG_STREAM("Object " << objects_.at(i).getName() << " (instance " << j << ") could not be found in the validation image");
                                if (config_.aggressivePoseValidation) {
                                    ROS_DEBUG_STREAM("Object " << objects_.at(i).getName() << " (instance " << j << ") is set as invalid due to the aggressive validation strategy");
                                    pose_recs.at(i)->getResults().at(j)->setModelFound(false);
                                    pose_recs.at(i)->getResults().at(j)->setPoseValid(false);
                                }
                            }
                        }
                    }
                }
            }

            validation_duration = ros::Time::now() - validation_time_start;
        }


        //visualisation
        ros::Time visualisation_time_start = ros::Time::now();

        //publish markers
        if (config_.evaluation) {
            outstream_poses_ << frame_counter_ << ";";
        }
        clearMarkers();
        last_frame_positions_.clear();
        for (unsigned int i = 0; i < pose_recs.size(); i++) {
            std::string name =  pose_recs.at(i)->getObjectDescriptor()->getName();
            std::string mesh_path = pose_recs.at(i)->getObjectDescriptor()->getMesh();
            Object2DPositions positions(i);
            if (config_.useVisualisationColor) {
                overlaySceneWith2DResults(pose_recs.at(i), &scene_image);
            } else {
                overlaySceneWith2DResults(pose_recs.at(i), &scene_image_mono);
            }
            for (unsigned int j = 0; j < pose_recs.at(i)->getResults().size(); j++) {
                if (pose_recs.at(i)->getResults().at(j)->checkModelFound()) {
                    asr_msgs::AsrObjectPtr object = createAsrMessage(pose_recs.at(i), j, input_point_cloud_with_guppy->header);
                    geometry_msgs::PoseWithCovariance obj_pose = object->sampledPoses.front();
                    if (config_.evaluation) {
                        HalconCpp::HQuaternion quat;
                        HalconCpp::HTuple quat_tuple = HalconCpp::HTuple::TupleGenConst(4, 0);
                        quat_tuple[0] = obj_pose.pose.orientation.w;
                        quat_tuple[1] = obj_pose.pose.orientation.x;
                        quat_tuple[2] = obj_pose.pose.orientation.y;
                        quat_tuple[3] = obj_pose.pose.orientation.z;
                        quat.SetFromTuple(quat_tuple);

                        double axis_x_x, axis_x_y, axis_x_z;
                        axis_x_x = quat.QuatRotatePoint3d(1, 0, 0, &axis_x_y, &axis_x_z);
                        double axis_y_x, axis_y_y, axis_y_z;
                        axis_y_x = quat.QuatRotatePoint3d(0, 1, 0, &axis_y_y, &axis_y_z);
                        double axis_z_x, axis_z_y, axis_z_z;
                        axis_z_x = quat.QuatRotatePoint3d(0, 0, 1, &axis_z_y, &axis_z_z);

                        outstream_poses_ << object->type << "_" << object->identifier << ";" << obj_pose.pose.position.x << ";" << obj_pose.pose.position.y << ";" << obj_pose.pose.position.z << ";" << obj_pose.pose.orientation.w << ";" << obj_pose.pose.orientation.x << ";" << obj_pose.pose.orientation.y << ";" << obj_pose.pose.orientation.z << ";" << pose_recs.at(i)->getResults().at(j)->getScore2D() << ";" << pose_recs.at(i)->getResults().at(j)->getScore3D() << ";" << axis_x_x << ";" << axis_x_y << ";" << axis_x_z << ";" << axis_y_x << ";" << axis_y_y << ";" << axis_y_z << ";" << axis_z_x << ";" << axis_z_y << ";" << axis_z_z << "#";
                    }
                    ROS_INFO_STREAM("------- " << name << " found: " << obj_pose.pose.position.x << " " << obj_pose.pose.position.y << " " << obj_pose.pose.position.z << " " << obj_pose.pose.orientation.w << " " << obj_pose.pose.orientation.x << " " << obj_pose.pose.orientation.y << " " << obj_pose.pose.orientation.z << "; score 2D: " << pose_recs.at(i)->getResults().at(j)->getScore2D() << ", score 3D: " << pose_recs.at(i)->getResults().at(j)->getScore3D());
                    createMarker(object, mesh_path);
                    positions.addPosition(pose_recs.at(i)->getResults().at(j)->getTexPoint(), pose_recs.at(i)->getResults().at(j)->getSearchRadius());
                    objects_pub_.publish(object);
                } else {
                    if (pose_recs.at(i)->getResults().at(j)->getPoseValid()) {
                        ROS_INFO_STREAM("------- " << name << " not found but texture found with score: " << pose_recs.at(i)->getResults().at(j)->getScore2D());
                    } else {
                        ROS_INFO_STREAM("------- " << name << " found but pose is invalid");
                    }
                }
            }
            last_frame_positions_.push_back(positions);
        }

        //publish result cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud = createVisualizationCloud(pose_recs, input_point_cloud_with_guppy->header);
        result_cloud->header = pcl_conversions::toPCL(input_point_cloud_with_guppy->header);
        sensor_msgs::PointCloud2 reduced_cloud_sensor_msg;
        pcl::toROSMsg(*result_cloud, reduced_cloud_sensor_msg);
        cloud_pub_.publish(reduced_cloud_sensor_msg);

        //publish reduced clouds' bounding boxes (created in create_result_point_cloud)
        boxes_pub_.publish(msgs_box_marker_array_);

        //publish 2D results
        halcon_bridge::HalconImagePtr ptr = boost::make_shared<halcon_bridge::HalconImage>();
        ptr->image = new HalconCpp::HImage();
        if (config_.useVisualisationColor) {
            ptr->header = input_image_guppy->header;
            ptr->encoding = input_image_guppy->encoding;
            *ptr->image = scene_image;
        } else {
            ptr->header = input_image_guppy_mono->header;
            ptr->encoding = input_image_guppy_mono->encoding;
            *ptr->image = scene_image_mono;
        }
        sensor_msgs::ImagePtr result = ptr->toImageMsg();
        image_pub_.publish(result);

        //show durations
        ROS_INFO_STREAM("Time for search: " << search_duration << " seconds");
        if (pose_val_.isInitialized()) {
            ROS_INFO_STREAM("Time for pose validation: " << validation_duration << " seconds");
        }
        ROS_INFO_STREAM("Time for visualisation: " << ros::Time::now() - visualisation_time_start << " seconds");
        ROS_INFO_STREAM("Time for frame: " << ros::Time::now() - time_frame_enter << " seconds");

        //evaluation
        if (config_.evaluation) {
            outstream_times_ << frame_counter_ << "#" << search_duration << ";";
            if (pose_val_.isInitialized()) {
                outstream_times_ << validation_duration << ";";
            } else {
                outstream_times_ << "-;";
            }
            outstream_times_ << ros::Time::now() - visualisation_time_start << ";" << ros::Time::now() - time_frame_enter << ";" << std::endl;
            outstream_poses_ << std::endl;
        }


        std::cout << std::endl; //end line

        for (unsigned int i = 0; i < objects_.size(); i++) {
            delete pose_recs.at(i);
        }
    }
}


void DescriptorSurfaceBasedRecognition::configCallback(DescriptorSurfaceBasedRecognitionConfig &config, uint32_t level)
{
    this->config_ = config;
}


void  DescriptorSurfaceBasedRecognition::threadTask(PoseRecognition *pose_rec)
{
    pose_rec->findPoses();
}


asr_msgs::AsrObjectPtr DescriptorSurfaceBasedRecognition::createAsrMessage(PoseRecognition *pose_rec, int results_index, std_msgs::Header header)
{
    HalconCpp::HPose pose;
    pose.SetFromTuple(pose_rec->getResults().at(results_index)->getPose());
    HalconCpp::HQuaternion quaternion;
    quaternion.PoseToQuat(pose);
    quaternion = quaternion.QuatCompose(pose_rec->getResults().at(results_index)->getAdjustedRotation());

    asr_msgs::AsrObjectPtr object;
    object.reset(new asr_msgs::AsrObject());

    object->header = header;
    object->providedBy = "descriptor_surface_based_recognition";

    geometry_msgs::Pose current_pose;

    current_pose.position.x = (double)pose.ConvertToTuple()[0];
    current_pose.position.y = (double)pose.ConvertToTuple()[1];
    current_pose.position.z = (double)pose.ConvertToTuple()[2];

    current_pose.orientation.w = (double)quaternion.ConvertToTuple().ToDArr()[0];
    current_pose.orientation.x = (double)quaternion.ConvertToTuple().ToDArr()[1];
    current_pose.orientation.y = (double)quaternion.ConvertToTuple().ToDArr()[2];
    current_pose.orientation.z = (double)quaternion.ConvertToTuple().ToDArr()[3];

    geometry_msgs::PoseWithCovariance current_pose_with_c;
    current_pose_with_c.pose = current_pose;
    for(unsigned int i = 0; i < current_pose_with_c.covariance.size(); i++)
        current_pose_with_c.covariance.at(i) = 0.0f;

    object->sampledPoses.push_back(current_pose_with_c);

    HalconCpp::HTuple bounding_corners = pose_rec->getObjectDescriptor()->getSurfaceModel().GetSurfaceModelParam("bounding_box1");
    boost::array< ::geometry_msgs::Point_<std::allocator<void> > , 8> bounding_box;
    for (unsigned int z = 0; z < 2; z++) {
        for (unsigned int y = 0; y < 2; y++) {
            for (unsigned int x = 0; x < 2; x++) {
                bounding_box[4 * z + 2 * y + x].x = (double)bounding_corners[x * 3];
                bounding_box[4 * z + 2 * y + x].y = (double)bounding_corners[y * 3 + 1];
                bounding_box[4 * z + 2 * y + x].z = (double)bounding_corners[z * 3 + 2];
            }
        }
    }
    object->boundingBox = bounding_box;

    object->colorName = "textured";
    object->type = pose_rec->getObjectDescriptor()->getName();

    object->identifier = boost::lexical_cast<std::string>(results_index);
    object->meshResourcePath = pose_rec->getObjectDescriptor()->getMesh();

    // sizeConfidence: score of 3D-recognition (== number of scene points that lie on the surface of the found object / number of model points)
    // typeConfidence: score of 2D-recognition (== number of found features / number of all features)
    object->sizeConfidence = pose_rec->getResults().at(results_index)->getScore3D();
    object->typeConfidence = pose_rec->getResults().at(results_index)->getScore2D();
    return object;
}


void DescriptorSurfaceBasedRecognition::createMarker(asr_msgs::AsrObjectPtr &object, std::string &mesh_path)
{
    visualization_msgs::Marker marker;
    marker.header = object->header;
    marker.ns = "Recognition results";
    marker.id = msgs_marker_array_->markers.size() + 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = object->sampledPoses.front().pose.position;

    marker.pose.orientation = object->sampledPoses.front().pose.orientation;

    marker.color.a = 0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.mesh_use_embedded_materials = true;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_path;
    marker.lifetime = ros::Duration(5);

    msgs_marker_array_->markers.push_back(marker);
    marker_pub_.publish(marker);
}


void DescriptorSurfaceBasedRecognition::clearMarkers()
{
    for (unsigned int i = 0; i < msgs_marker_array_->markers.size(); i++) {
        msgs_marker_array_->markers[i].action = visualization_msgs::Marker::DELETE;
    }
    for (unsigned int i = 0; i < msgs_box_marker_array_->markers.size(); i++) {
        msgs_box_marker_array_->markers[i].action = visualization_msgs::Marker::DELETE;
    }
    msgs_marker_array_->markers.clear();

    boxes_pub_.publish(msgs_box_marker_array_);
    msgs_box_marker_array_->markers.clear();
}


void DescriptorSurfaceBasedRecognition::createBoxMarker(RecognitionResult *result, std_msgs::Header header, rgb color, bool drawCompleteBoxes)
{
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*(result->getSearchCloud()), min, max);

    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "Result Bounding Boxes";
    marker.id = msgs_box_marker_array_->markers.size() + 1;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.01;

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    geometry_msgs::Point p;

    //upper-front line
    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    marker.points.push_back(p);
    p.x = max.x;
    marker.points.push_back(p);

    //right-front line
    marker.points.push_back(p);
    p.y = max.y;
    marker.points.push_back(p);

    //lower-front line
    marker.points.push_back(p);
    p.x = min.x;
    marker.points.push_back(p);

    //left-front line
    marker.points.push_back(p);
    p.y = min.y;
    marker.points.push_back(p);


    if (drawCompleteBoxes) {
        //upper-back line
        p.x = min.x;
        p.y = min.y;
        p.z = max.z;
        marker.points.push_back(p);
        p.x = max.x;
        marker.points.push_back(p);

        //right-back line
        marker.points.push_back(p);
        p.y = max.y;
        marker.points.push_back(p);

        //lower-back line
        marker.points.push_back(p);
        p.x = min.x;
        marker.points.push_back(p);

        //left-back line
        marker.points.push_back(p);
        p.y = min.y;
        marker.points.push_back(p);


        //top-left-conn line
        p.x = min.x;
        p.y = min.y;
        p.z = min.z;
        marker.points.push_back(p);
        p.z = max.z;
        marker.points.push_back(p);

        //top-right-conn line
        p.x = max.x;
        p.y = min.y;
        p.z = min.z;
        marker.points.push_back(p);
        p.z = max.z;
        marker.points.push_back(p);

        //bottom-right-conn line
        p.x = max.x;
        p.y = max.y;
        p.z = min.z;
        marker.points.push_back(p);
        p.z = max.z;
        marker.points.push_back(p);

        //bottom-left-conn line
        p.x = min.x;
        p.y = max.y;
        p.z = min.z;
        marker.points.push_back(p);
        p.z = max.z;
        marker.points.push_back(p);
    }

    msgs_box_marker_array_->markers.push_back(marker);
}


void DescriptorSurfaceBasedRecognition::overlaySceneWith2DResults(PoseRecognition *pose_rec, HalconCpp::HImage *scene_image)
{
    const int BOX_BORDER_SIZE = config_.boundingBoxBorderSize;
    const int ORIENTATION_TRIANGLE_SIZE = config_.orientationTriangleSize;
    const int POINTS_RADIUS = config_.featurePointsRadius;

    HalconCpp::HTuple color_points, color_box;
    if (config_.useVisualisationColor) {
        color_points.Append(config_.visualisationColorPointsRed);
        color_points.Append(config_.visualisationColorPointsGreen);
        color_points.Append(config_.visualisationColorPointsBlue);
        color_box.Append(config_.visualisationColorBoxRed);
        color_box.Append(config_.visualisationColorBoxGreen);
        color_box.Append(config_.visualisationColorBoxBlue);
    } else {
        color_points.Append(255);
        color_box.Append(255);
    }

    for (unsigned int i = 0; i < pose_rec->getResults().size(); i++) {

        //paint feature points
        HalconCpp::HTuple row, column;
        HalconCpp::HRegion search_points;
        for (unsigned int k = 0; k <= i; k++) {
            try {
                pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getDescModel().GetDescriptorModelPoints("search", k, &row, &column);
                search_points.GenCircle(row, column, HalconCpp::HTuple::TupleGenConst(row.Length(), POINTS_RADIUS));
                *scene_image = search_points.PaintRegion(*scene_image, color_points, HalconCpp::HTuple("fill"));
            }catch(HalconCpp::HException exc) {
            }
        }

        //paint bounding box (create inner and outer box and paint difference to get a visible border)
        pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getDescModel().GetDescriptorModelPoints("model", "all", &row, &column);
        HalconCpp::HRegion bounding_box;
        bounding_box.GenRegionPoints(row, column);
        Hlong row1, column1, row2, column2;
        bounding_box.SmallestRectangle1(&row1, &column1, &row2, &column2);

        HalconCpp::HTuple rows, columns;
        double temp;
        double upperLeftInnerX, upperRightInnerX, lowerRightInnerX, lowerLeftInnerX;
        double upperLeftInnerY, upperRightInnerY, lowerRightInnerY, lowerLeftInnerY;
        upperLeftInnerX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(0)[1], pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(0)[0], 1.0, &upperLeftInnerY, &temp);
        upperLeftInnerX = upperLeftInnerX / temp;
        upperLeftInnerY = upperLeftInnerY / temp;
        rows.Append((int)upperLeftInnerX);
        columns.Append((int)upperLeftInnerY);

        upperRightInnerX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(1)[1], pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(1)[0], 1.0, &upperRightInnerY, &temp);
        rows.Append((int)(upperRightInnerX / temp));
        columns.Append((int)(upperRightInnerY / temp));

        lowerRightInnerX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(2)[1], pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(2)[0], 1.0, &lowerRightInnerY, &temp);
        rows.Append((int)(lowerRightInnerX / temp));
        columns.Append((int)(lowerRightInnerY / temp));

        lowerLeftInnerX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(3)[1], pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(3)[0], 1.0, &lowerLeftInnerY, &temp);
        rows.Append((int)(lowerLeftInnerX / temp));
        columns.Append((int)(lowerLeftInnerY / temp));

        rows.Append((int)upperLeftInnerX);
        columns.Append((int)upperLeftInnerY);

        HalconCpp::HRegion inner_box;
        inner_box.GenRegionPolygonFilled(rows, columns);

        HalconCpp::HTuple rows_outer, columns_outer;
        double temp_outer;
        double upperLeftOuterX, upperRightOuterX, lowerRightOuterX, lowerLeftouterX;
        double upperLeftOuterY, upperRightOuterY, lowerRightOuterY, lowerLeftOuterY;
        upperLeftOuterX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(0)[1] - BOX_BORDER_SIZE, pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(0)[0] - BOX_BORDER_SIZE, 1.0, &upperLeftOuterY, &temp_outer);
        upperLeftOuterX = upperLeftOuterX / temp_outer;
        upperLeftOuterY = upperLeftOuterY / temp_outer;
        rows_outer.Append((int)upperLeftOuterX);
        columns_outer.Append((int)upperLeftOuterY);

        upperRightOuterX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(1)[1] - BOX_BORDER_SIZE, pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(1)[0] + BOX_BORDER_SIZE, 1.0, &upperRightOuterY, &temp_outer);
        rows_outer.Append((int)(upperRightOuterX / temp_outer));
        columns_outer.Append((int)(upperRightOuterY / temp_outer));

        lowerRightOuterX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(2)[1] + BOX_BORDER_SIZE, pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(2)[0] + BOX_BORDER_SIZE, 1.0, &lowerRightOuterY, &temp_outer);
        rows_outer.Append((int)(lowerRightOuterX / temp_outer));
        columns_outer.Append((int)(lowerRightOuterY / temp_outer));

        lowerLeftouterX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(3)[1] + BOX_BORDER_SIZE, pose_rec->getObjectDescriptor()->getModelViewDescriptors().at(pose_rec->getResults().at(i)->getViewIndex()).getBoxCorners().at(3)[0] - BOX_BORDER_SIZE, 1.0, &lowerLeftOuterY, &temp_outer);
        rows_outer.Append((int)(lowerLeftouterX / temp_outer));
        columns_outer.Append((int)(lowerLeftOuterY / temp_outer));

        rows_outer.Append((int)upperLeftOuterX);
        columns_outer.Append((int)upperLeftOuterY);

        HalconCpp::HRegion outer_box;
        outer_box.GenRegionPolygonFilled(rows_outer, columns_outer);

        inner_box = outer_box.Difference(inner_box);

        *scene_image = inner_box.PaintRegion(*scene_image, color_box, HalconCpp::HTuple("fill"));


        //get search radius for next frame
        Hlong r, c, r2, c2;
        inner_box.SmallestRectangle1(&r, &c, &r2, &c2);
        int max_expansion = r2 - r;
        if (c2 - c > max_expansion) {
            max_expansion = c2 - c;
        }
        pose_rec->getResults().at(i)->setSearchRadius((int)(max_expansion * 0.5));


        //paint orientation triangle
        rows.Clear();
        columns.Clear();
        double triLeftX, triRightX, triUpX;
        double triLeftY, triRightY, triUpY;
        triLeftX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(ORIENTATION_TRIANGLE_SIZE, -1 * ORIENTATION_TRIANGLE_SIZE, 1.0, &triLeftY, &temp);
        triLeftX = triLeftX / temp;
        triLeftY = triLeftY / temp;
        rows.Append((int)triLeftX);
        columns.Append((int)triLeftY);

        triRightX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(ORIENTATION_TRIANGLE_SIZE, ORIENTATION_TRIANGLE_SIZE, 1.0, &triRightY, &temp);
        rows.Append((int)(triRightX / temp));
        columns.Append((int)(triRightY / temp));

        triUpX = pose_rec->getResults().at(i)->getTransMatrix2D().ProjectiveTransPoint2d(-2.5 * ORIENTATION_TRIANGLE_SIZE, 0, 1.0, &triUpY, &temp);
        rows.Append((int)(triUpX / temp));
        columns.Append((int)(triUpY / temp));

        rows.Append((int)triLeftX);
        columns.Append((int)triLeftY);

        HalconCpp::HRegion orientation_triangle;
        orientation_triangle.GenRegionPolygonFilled(rows, columns);
        *scene_image = orientation_triangle.PaintRegion(*scene_image, color_box, HalconCpp::HTuple("fill"));
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr DescriptorSurfaceBasedRecognition::createVisualizationCloud(std::vector<PoseRecognition*> &pose_recs, std_msgs::Header header)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int num_colors = 0;
    for (unsigned int i = 0; i < pose_recs.size(); i++) {
        for(unsigned int j = 0; j < pose_recs.at(i)->getResults().size(); j++) {
            if (pose_recs.at(i)->getResults().at(j)->checkModelFound()) {
                num_colors++;
            }
        }
    }
    std::vector<rgb> colors;
    if (num_colors > 0) {
        for(int i = 0; i < 360; i += 360 / num_colors) {
            hsv c;
            c.h = i;
            c.s = 0.9;
            c.v = 0.5;
            colors.push_back(hsv2rgb(c));
        }
    } else {
        hsv c;
        c.h = 0;
        c.s = 0.9;
        c.v = 0.5;
        colors.push_back(hsv2rgb(c));
    }
    int counter = 0;
    for (unsigned int i = 0; i < pose_recs.size(); i++) {
        for(unsigned int j = 0; j < pose_recs.at(i)->getResults().size(); j++) {
            if (pose_recs.at(i)->getResults().at(j)->checkModelFound()) {

                //draw cloud boxes
                if (config_.drawCloudBoxes) {
                    createBoxMarker(pose_recs.at(i)->getResults().at(j), header, colors.at(counter), config_.drawCompleteCloudBoxes);
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*(pose_recs.at(i)->getResults().at(j)->getSearchCloud()), *tmp);

                for(unsigned int j = 0; j < tmp->width * tmp->height; j++) {
                    tmp->points[j].r = colors.at(counter).r * 255;
                    tmp->points[j].g = colors.at(counter).g * 255;
                    tmp->points[j].b = colors.at(counter).b * 255;
                }
                *result_cloud += *tmp;

                counter = (counter + 1) % colors.size();
            }
        }
    }

    if (counter == 0) {
        //Dummy point, to show "empty" cloud in rviz
        pcl::PointXYZRGB point;
        point.x = 1000;
        point.y = 1000;
        point.z = 1000;
        result_cloud->push_back(point);
    }
    return result_cloud;
}

}

int
main (int argc, char** argv)
{

    ros::init (argc, argv, "descriptor_surface_based_recognition");

    descriptor_surface_based_recognition::DescriptorSurfaceBasedRecognition rec;

    ros::spin();

    return 0;
}
