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


#include "object_view_descriptor.h"

namespace descriptor_surface_based_recognition {

ObjectViewDescriptor::ObjectViewDescriptor(HalconCpp::HDescriptorModel &descModel, Eigen::Vector3d view_orientation, double score_2D, bool use_color, int vertical_tex_offset, int horizontal_tex_offset, bool is_invertible, std::vector<RotationAxis> &axes, std::vector<Eigen::Vector2i> &box_corners)
    : desc_model_(descModel), view_orientation_(view_orientation), score_2D_(score_2D), use_color_(use_color),
      vertical_tex_offset_(vertical_tex_offset), horizontal_tex_offset_(horizontal_tex_offset), is_invertible_(is_invertible), axes_(axes), box_corners_(box_corners) { }


HalconCpp::HDescriptorModel ObjectViewDescriptor::getDescModel() const{ return desc_model_; }

Eigen::Vector3d ObjectViewDescriptor::getViewOrientation() const { return view_orientation_; }

double ObjectViewDescriptor::getScore2D() const { return score_2D_; }

bool ObjectViewDescriptor::getUseColor() const { return use_color_; }

int ObjectViewDescriptor::getVerticalTexOffset() const { return vertical_tex_offset_; }

int ObjectViewDescriptor::getHorizontalTexOffset() const { return horizontal_tex_offset_; }

bool ObjectViewDescriptor::getIsInvertible() const { return is_invertible_; }

std::vector<RotationAxis> ObjectViewDescriptor::getAxes() const { return axes_; }

std::vector<Eigen::Vector2i> ObjectViewDescriptor::getBoxCorners() const { return box_corners_; }

}
