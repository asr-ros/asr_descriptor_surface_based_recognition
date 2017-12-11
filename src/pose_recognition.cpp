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


#include "descriptor_surface_based_recognition.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <asr_halcon_bridge/halcon_pointcloud.h>

#include <pcl/common/common.h>
#include <ros/package.h>

#include "pose_recognition.h"
#include "util.h"


namespace descriptor_surface_based_recognition {

PoseRecognition::PoseRecognition(HalconCpp::HImage &scene_image, HalconCpp::HImage &scene_image_mono, pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_with_guppy, ObjectDescriptor *obj_desc, int median_points_offset, double samplingDistance, double keypointFraction, bool eval, int max_instances)
    : scene_image_(scene_image), scene_image_mono_(scene_image_mono),
      point_cloud_with_guppy_(point_cloud_with_guppy), obj_desc_(obj_desc), max_instances_(max_instances), median_points_offset_(median_points_offset),
      sampling_distance_(samplingDistance), keypoint_fraction_(keypointFraction), eval_(eval) { }


PoseRecognition::~PoseRecognition()
{
    for (unsigned int i = 0; i < results_.size(); i++) {
        delete results_.at(i);
    }
}

void PoseRecognition::findPoses() {
    std::ofstream object_ofstream;
    std::string path = PACKAGE_PATH + "/" + OUTPUT_EVALUATION_DIR + "/" + obj_desc_->getName() + "_times.txt";
    if (eval_) {
        object_ofstream.open (path.c_str(), std::ofstream::out | std::ofstream::app);
    }
    ros::Time find_poses_start = ros::Time::now();

    //find object in 2D
    findTexture();

    ros::Time textures_found = ros::Time::now();
    if (eval_) {
        object_ofstream << textures_found - find_poses_start;
    }
    for (unsigned int i = 0; i < results_.size(); i++) {
        if (eval_) {
                object_ofstream << "#";
        }
        ros::Time loop_entry = ros::Time::now();

        //reduce cloud
        if (reducePointCloud(i)) {
            ros::Time cloud_reduced = ros::Time::now();
            if (eval_) {
                object_ofstream << cloud_reduced - loop_entry << ";";
            }

            //find object
            if (findModelInCloud(i)) {
                ros::Time model_found = ros::Time::now();
                if (eval_) {
                    object_ofstream << model_found - cloud_reduced << ";";
                }

                if (sqrt(pow(results_.at(i)->getPose()[0] - results_.at(i)->getTexPoint3D()[0], 2) + pow(results_.at(i)->getPose()[1] - results_.at(i)->getTexPoint3D()[1], 2) + pow(results_.at(i)->getPose()[2] - results_.at(i)->getTexPoint3D()[2], 2)) < (obj_desc_->getDiameter() * 0.5)) {
                    results_.at(i)->setModelFound();
                    //find rotation
                    adjustRotation(i);
                    if (eval_) {
                        object_ofstream << ros::Time::now() - model_found;
                    }

                }
            }
        }
    }
    if (eval_) {
        object_ofstream << std::endl;
        object_ofstream.close();
    }
}


void PoseRecognition::findTexture()
{
    for (unsigned int i = 0; i < obj_desc_->getModelViewDescriptors().size(); i++) {
        HalconCpp::HTuple score;
        if (obj_desc_->getModelViewDescriptors().at(i).getUseColor()) {
            scene_image_.FindUncalibDescriptorModel(obj_desc_->getModelViewDescriptors().at(i).getDescModel(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), obj_desc_->getModelViewDescriptors().at(i).getScore2D(), max_instances_, "inlier_ratio", &score);
        } else {
            scene_image_mono_.FindUncalibDescriptorModel(obj_desc_->getModelViewDescriptors().at(i).getDescModel(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), obj_desc_->getModelViewDescriptors().at(i).getScore2D(), max_instances_, "inlier_ratio", &score);
        }
        if (score.Length() > 0) {
            for (int j = 0; j < score.Length(); j++) {
                HalconCpp::HTuple matrix_tuple = obj_desc_->getModelViewDescriptors().at(i).getDescModel().GetDescriptorModelResults(j, "homography");
                HalconCpp::HHomMat2D matrix;
                matrix.SetFromTuple(matrix_tuple);

                double trans_x, trans_y, trans_w;
                trans_x = matrix.ProjectiveTransPoint2d(obj_desc_->getModelViewDescriptors().at(i).getVerticalTexOffset(), obj_desc_->getModelViewDescriptors().at(i).getHorizontalTexOffset(), 1,  &trans_y, &trans_w);

                int row = (int) (trans_x / trans_w);
                int column = (int) (trans_y / trans_w);

                results_.push_back(new RecognitionResult(i, (double)score[j], matrix, Eigen::Vector2i(row, column)));
                ROS_DEBUG_STREAM("Texture of " << obj_desc_->getName() << " (instance " << j << ") found at (" << row << ", " << column << ")");
                if ((int)(results_.size()) >= max_instances_) {
                    return;
                }
            }
        }
    }
}

bool PoseRecognition::reducePointCloud(int result_index)
{
    double radius = obj_desc_->getDiameter() / 2;
    int image_height = scene_image_.Height();
    int image_width = scene_image_.Width();

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*point_cloud_with_guppy_, min, max);

    int row = results_.at(result_index)->getTexPoint()[0];
    int column = results_.at(result_index)->getTexPoint()[1];

    std::vector<pcl::PointXYZ> points;
    int offsets[10] = {0, 0, median_points_offset_, 0, 0, -1 * median_points_offset_, -1 * median_points_offset_, 0, 0, median_points_offset_ };

    for (int i = 0; i < 5; i++) {
        if (row - median_points_offset_ < 0 || column - median_points_offset_ < 0 || row + median_points_offset_ >= image_height || column + median_points_offset_ >= image_width) {
            return false;
        }
        pcl::PointXYZ current_point = findPoint3D(point_cloud_with_guppy_, (*point_cloud_with_guppy_)(column + offsets[i * 2], row + offsets[i * 2 + 1]), row + offsets[i * 2 + 1], column + offsets[i * 2], image_height, image_width);
        if (pcl::isFinite(current_point)) {
            points.push_back(current_point);
        }
    }

    if (points.size() > 0) {
        pcl::PointXYZ median_point = computeMedian(points);

        if (pcl::isFinite(median_point)) {
            if (median_point.x - radius > min.x) { min.x = median_point.x - radius; }
            if (median_point.x + radius < max.x) { max.x = median_point.x + radius; }
            if (median_point.y - radius > min.y) { min.y = median_point.y - radius; }
            if (median_point.y + radius < max.y) { max.y = median_point.y + radius; }
            if (median_point.z - radius > min.z) { min.z = median_point.z - radius; }
            if (median_point.z + radius < max.z) { max.z = median_point.z + radius; }

        } else {
            return false;
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (point_cloud_with_guppy_);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (min.x, max.x);
        pass.filter (*reduced_cloud);

        pass.setInputCloud (reduced_cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (min.y, max.y);
        pass.filter (*reduced_cloud);

        results_.at(result_index)->setTexPoint3D(median_point);
        results_.at(result_index)->setSearchCloud(reduced_cloud);

        return true;
    }
    return false;

}

bool PoseRecognition::findModelInCloud(int result_index)
{

    if ((results_.at(result_index)->getSearchCloud()->width * results_.at(result_index)->getSearchCloud()->height > 0)) {

        std::vector< int > index;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = results_.at(result_index)->getSearchCloud();
        pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

        results_.at(result_index)->setSearchCloud(cloud);
        sensor_msgs::PointCloud2 p_n_cloud;
        pcl::toROSMsg(*cloud, p_n_cloud);
        halcon_bridge::HalconPointcloudPtr ptr = halcon_bridge::toHalconCopy(p_n_cloud);
        HalconCpp::HObjectModel3D scene = *ptr->model;

        HalconCpp::HSurfaceMatchingResultArray result;
        HalconCpp::HTuple *score = new HalconCpp::HTuple();
        HalconCpp::HTuple genParamName("num_matches");
        genParamName.Append("pose_ref_use_scene_normals");
        HalconCpp::HTuple genParamValue(1);
        genParamValue.Append("false");
        HalconCpp::HPoseArray poses = scene.FindSurfaceModel(obj_desc_->getSurfaceModel(), sampling_distance_, keypoint_fraction_, obj_desc_->getScore3D(), "false", genParamName, genParamValue, score, &result);

        if ((score->Length() > 0) && ((*score)[0] >= obj_desc_->getScore3D())) {
            bool pose_already_found = false;
            for (int i = 0; i < result_index; i++) {
                if ((results_.at(i)->checkModelFound()) && (sqrt(pow(results_.at(i)->getPose()[0] - poses.ConvertToTuple()[0], 2) + pow(results_.at(i)->getPose()[1] - poses.ConvertToTuple()[1], 2) + pow(results_.at(i)->getPose()[2] - poses.ConvertToTuple()[2], 2)) < (obj_desc_->getDiameter() * 0.5))) {
                    pose_already_found = true;
                    break;
                }
            }
            if (!pose_already_found) {
                ROS_DEBUG_STREAM("Found object " << obj_desc_->getName() << " (instance " << result_index << ") in the point cloud");
                results_.at(result_index)->setPose(poses.ConvertToTuple());
                results_.at(result_index)->setScore3D((*score)[0]);
                return true;
            } else {
                ROS_DEBUG_STREAM("Found pose of object " << obj_desc_->getName() << " (instance " << result_index << ") was found for another instance already");
            }
        }
    } else {
        ROS_DEBUG_STREAM("Reduced point cloud of object " << obj_desc_->getName() << " (instance " << result_index << ") is empty");
    }
    return false;
}

void PoseRecognition::adjustRotation(int result_index)
{
    HalconCpp::HPose pose;
    pose.SetFromTuple(results_.at(result_index)->getPose());

    HalconCpp::HQuaternion rotation;
    rotation.PoseToQuat(pose);
    rotation = rotation.QuatNormalize();

    Eigen::Vector3d location(results_.at(result_index)->getPose()[0], results_.at(result_index)->getPose()[1], results_.at(result_index)->getPose()[2]);

    //case: model is rotated around given axes
    switch(obj_desc_->getRotationModelType()) {
    case ObjectDescriptor::ROTATION_MODEL_SPHERE:
    {
        //not implemented
    }
    case ObjectDescriptor::ROTATION_MODEL_CYLINDER:
    {
        Eigen::Vector3d texFromObjOrigin;
        texFromObjOrigin = results_.at(result_index)->getTexPoint3D() - location;
        texFromObjOrigin.normalize();

        double x, y, z;
        x = rotation.QuatRotatePoint3d(obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[0], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[1], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[2], &y, &z);
        Eigen::Vector3d viewOrientationFromObjOrigin(x, y, z);
        viewOrientationFromObjOrigin.normalize();

        double dot = texFromObjOrigin.dot(viewOrientationFromObjOrigin);
        double maxDot = dot;
        double bestAngle = 0.0;
        double currentAngle = obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAngle();
        while (currentAngle < 360) {
            double radians = (currentAngle) * (PI / 180);
            HalconCpp::HQuaternion current_rot(obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[0], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[1], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[2], radians);

            double x_curr, y_curr, z_curr;
            x_curr = current_rot.QuatRotatePoint3d(obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[0], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[1], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[2], &y_curr, &z_curr);

            Eigen::Vector3d vec_curr_obj(x_curr, y_curr, z_curr);
            vec_curr_obj.normalize();
            x_curr = rotation.QuatRotatePoint3d(vec_curr_obj[0], vec_curr_obj[1], vec_curr_obj[2], &y_curr, &z_curr);

            Eigen::Vector3d vec_curr(x_curr, y_curr, z_curr);
            vec_curr.normalize();

            if (texFromObjOrigin.dot(vec_curr) > maxDot) {
                maxDot = texFromObjOrigin.dot(vec_curr);
                bestAngle = currentAngle;
            }
            currentAngle += obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAngle();
        }
        HalconCpp::HQuaternion quat_result(obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[0], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[1], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getAxes().at(0).getAxis()[2], bestAngle * (PI / 180));
        results_.at(result_index)->setAdjustedRotation(quat_result.QuatCompose(results_.at(result_index)->getAdjustedRotation()));
    }
    }

    //case: model is upside-down
    if (obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getIsInvertible()) {

        double trans_x, trans_y, trans_w;
        trans_x = results_.at(result_index)->getTransMatrix2D().ProjectiveTransPoint2d(0, 1, 1,  &trans_y, &trans_w);
        Eigen::Vector2d upperPoint(trans_x / trans_w, trans_y / trans_w);
        trans_x = results_.at(result_index)->getTransMatrix2D().ProjectiveTransPoint2d(0, 0, 1,  &trans_y, &trans_w);
        Eigen::Vector2d centerPoint(trans_x / trans_w, trans_y / trans_w);
        upperPoint = upperPoint - centerPoint;
        upperPoint.normalize();

        double x_upper, y_upper, z_upper;
        x_upper = rotation.QuatRotatePoint3d(0, -1, 0, &y_upper, &z_upper);
        Eigen::Vector2d upperPoint3D(x_upper, -1 * y_upper);
        upperPoint3D.normalize();

        if (upperPoint.dot(upperPoint3D) < 0) {
            double axis_rot_x, axis_rot_y, axis_rot_z;
            axis_rot_x = results_.at(result_index)->getAdjustedRotation().QuatRotatePoint3d(obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[0], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[1], obj_desc_->getModelViewDescriptors().at(results_.at(result_index)->getViewIndex()).getViewOrientation()[2], &axis_rot_y, &axis_rot_z);
            HalconCpp::HQuaternion quat_upside_down(axis_rot_x, axis_rot_y, axis_rot_z, PI);
            results_.at(result_index)->setAdjustedRotation(quat_upside_down.QuatCompose(results_.at(result_index)->getAdjustedRotation()));
        }
    }
}

std::vector<RecognitionResult*> PoseRecognition::getResults() const { return results_; }

ObjectDescriptor* PoseRecognition::getObjectDescriptor() const { return obj_desc_; }

HalconCpp::HRegion PoseRecognition::getInputImageDomain() const { return scene_image_mono_.GetDomain(); }

}
