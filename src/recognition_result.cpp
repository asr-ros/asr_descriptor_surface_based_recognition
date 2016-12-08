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


#include "recognition_result.h"

namespace descriptor_surface_based_recognition {

RecognitionResult::RecognitionResult(int view_index, double score_2D, HalconCpp::HHomMat2D &trans_matrix_2D, const Eigen::Vector2i &tex_point)
    : model_found_(false), view_index(view_index), score_2D_(score_2D), trans_matrix_2D_(trans_matrix_2D), tex_point_(tex_point), adjusted_rotation_(HalconCpp::HQuaternion(0, 0, 0, 0)),
      search_radius_(225), pose_valid_(true) { }


bool RecognitionResult::checkModelFound() { return model_found_; }

void RecognitionResult::setSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &search_cloud) { this->search_cloud_ = search_cloud; }

void RecognitionResult::setPose(HalconCpp::HTuple pose) { this->pose_ = pose; }

void RecognitionResult::setModelFound(bool model_found) { this->model_found_ = model_found; }

void RecognitionResult::setTexPoint3D(pcl::PointXYZ tex_point_3D) { this->tex_point_3D_ = Eigen::Vector3d(tex_point_3D.x, tex_point_3D.y, tex_point_3D.z); }

void RecognitionResult::setAdjustedRotation(HalconCpp::HQuaternion quaternion) { this->adjusted_rotation_ = quaternion; }

void RecognitionResult::setScore3D(double score_3D) { this->score_3D_ = score_3D; }

void RecognitionResult::setSearchRadius(int radius) { this->search_radius_ = radius; }

void RecognitionResult::setPoseValid(bool pose_valid) { this->pose_valid_ = pose_valid; }


HalconCpp::HTuple RecognitionResult::getPose() const { return pose_; }

double RecognitionResult::getScore2D() const { return score_2D_; }

double RecognitionResult::getScore3D() const { return score_3D_; }

int RecognitionResult::getViewIndex() const { return view_index; }

HalconCpp::HHomMat2D RecognitionResult::getTransMatrix2D() const { return trans_matrix_2D_; }

Eigen::Vector2i RecognitionResult::getTexPoint() const { return tex_point_; }

Eigen::Vector3d RecognitionResult::getTexPoint3D() const { return tex_point_3D_; }

pcl::PointCloud<pcl::PointXYZ>::Ptr RecognitionResult::getSearchCloud() const { return search_cloud_; }

HalconCpp::HQuaternion RecognitionResult::getAdjustedRotation() const { return adjusted_rotation_; }

int RecognitionResult::getSearchRadius() const { return search_radius_; }

bool RecognitionResult::getPoseValid() const { return pose_valid_; }

}
