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


#include "descriptor_surface_based_trainer/View_Params_Wrapper.h"

ViewParamsWrapper::ViewParamsWrapper(std::string rotation_type) :
    is_valid(false), rotation_type(rotation_type), has_bounding_box(false)
{

}

void ViewParamsWrapper::setImage(HalconCpp::HImage image) {this->image = image;}
void ViewParamsWrapper::setOriginalImage(HalconCpp::HImage original_image) {this->originalImage = original_image; }
void ViewParamsWrapper::setImageBounds(int row1, int column1, int row2, int column2) { this->row1 = row1; this->column1 = column1; this->row2 = row2; this->column2 = column2;}
void ViewParamsWrapper::setRotationType(std::string rotation_type) { this->rotation_type = rotation_type; }
void ViewParamsWrapper::setAxis1(Eigen::Vector3d axis1) {this->axis1 = axis1;}
void ViewParamsWrapper::setAxis2(Eigen::Vector3d axis2) {this->axis2 = axis2;}
void ViewParamsWrapper::setAxis1Angle(double axis1_angle) {this->axis1_angle = axis1_angle;}
void ViewParamsWrapper::setAxis2Angle(double axis2_angle) {this->axis2_angle = axis2_angle;}
void ViewParamsWrapper::setOrientation(Eigen::Vector3d orientation) {this->orientation = orientation; }
void ViewParamsWrapper::setIsInvertible(bool is_invertible) {this->is_invertible = is_invertible;}
void ViewParamsWrapper::setScore2D(double score_2D) {this->score_2D = score_2D;}
void ViewParamsWrapper::setUserColor(bool use_color) {this->use_color = use_color;}
void ViewParamsWrapper::setVerticalOffset(int vertical_offset) {this->vertical_offset = vertical_offset;}
void ViewParamsWrapper::setHorizontalOffset(int horizontal_offset) {this->horizontal_offset = horizontal_offset;}
void ViewParamsWrapper::setDepth(int depth) {this->depth = depth;}
void ViewParamsWrapper::setNumberFerns(int number_ferns) {this->number_ferns = number_ferns;}
void ViewParamsWrapper::setPatchSize(int patch_size) {this->patch_size = patch_size;}
void ViewParamsWrapper::setMinScale(double min_scale) {this->min_scale = min_scale;}
void ViewParamsWrapper::setMaxScale(double max_scale) {this->max_scale = max_scale;}
void ViewParamsWrapper::setIsValid(bool is_valid) {this->is_valid = is_valid;}

void ViewParamsWrapper::addBoxCorner(Eigen::Vector2i corner)
{
    if (!(box_corners.size() < 4)) {
        box_corners.clear();
    }
    box_corners.push_back(corner);
}

void ViewParamsWrapper::setHasBoundingBox(bool has_bounding_box) {this->has_bounding_box = has_bounding_box;}
void ViewParamsWrapper::clearBoundingBox()
{
    box_corners.clear();
    has_bounding_box = false;
}


HalconCpp::HImage ViewParamsWrapper::getImage() {return image; }
HalconCpp::HImage ViewParamsWrapper::getOriginalImage() {return originalImage; }
int ViewParamsWrapper::getRow1() {return row1; }
int ViewParamsWrapper::getColumn1() {return column1; }
int ViewParamsWrapper::getRow2() {return row2; }
int ViewParamsWrapper::getColumn2() {return column2; }
std::string ViewParamsWrapper::getRotationType() {return rotation_type;}
Eigen::Vector3d ViewParamsWrapper::getAxis1() {return axis1; }
Eigen::Vector3d ViewParamsWrapper::getAxis2() {return axis2; }
Eigen::Vector3d ViewParamsWrapper::getOrientation() {return orientation;}
double ViewParamsWrapper::getAxis1Angle() {return axis1_angle; }
double ViewParamsWrapper::getAxis2Angle() {return axis2_angle; }
bool ViewParamsWrapper::getIsInvertible() {return is_invertible; }
double ViewParamsWrapper::getScore2D() {return score_2D; }
bool ViewParamsWrapper::getUseColor() {return use_color; }
int ViewParamsWrapper::getVerticalOffset() {return vertical_offset; }
int ViewParamsWrapper::getHorizontalOffset() {return horizontal_offset; }
int ViewParamsWrapper::getDepth() {return depth; }
int ViewParamsWrapper::getNumberFerns() {return number_ferns; }
int ViewParamsWrapper::getPatchSize() {return patch_size; }
double ViewParamsWrapper::getMinScale() {return min_scale; }
double ViewParamsWrapper::getMaxScale() {return max_scale; }
bool ViewParamsWrapper::getIsValid() {return is_valid; }

bool ViewParamsWrapper::getHasBoundingBox() { return has_bounding_box; }
std::vector<Eigen::Vector2i> ViewParamsWrapper::getBoxCorners() {return box_corners; }
