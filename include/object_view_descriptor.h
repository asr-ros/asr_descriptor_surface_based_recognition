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


#ifndef OBJECT_VIEW_DESCRIPTOR_H
#define OBJECT_VIEW_DESCRIPTOR_H

#include <HalconCpp.h>

#include <Eigen/Dense>
#include <vector>

#include "rotation_axis.h"

namespace descriptor_surface_based_recognition {

/**
*  This class describes a specific view of an object which is used for 2D recognition
*/
class ObjectViewDescriptor {

private:

    /** The descriptor-model of this view used for recognition */
    HalconCpp::HDescriptorModel desc_model_;

    /** This view's orientation */
    Eigen::Vector3d view_orientation_;

    /** The minimum score needed to recognize this view */
    double score_2D_;

    /** If true a colored image is used for recognition, a greyscale one otherwise */
    bool use_color_;

    /** The vertical offset of the point which marks the center of the descriptor-model's feature points */
    int vertical_tex_offset_;

    /** The horizontal offset of the point which marks the center of the descriptor-model's feature points */
    int horizontal_tex_offset_;

    /** If true the object can be rotated by 180° around this view's orientation */
    bool is_invertible_;

    /** The rotation axes of this view (number depends on the object's rotation-type, see ObjectDescriptor) */
    std::vector<RotationAxis> axes_;

    /** The corners of this view's bounding box (relative to the feature points' center => (0,0))*/
    std::vector<Eigen::Vector2i> box_corners_;



public:

    /**
    *  \brief The constructor of this class
    *
    *  \param descModel                    --
    *  \param view_orientation              |
    *  \param score_2D                      |
    *  \param use_color                     |-- see above
    *  \param vertical_tex_offset           |
    *  \param horizontal_tex_offset         |
    *  \param axes                          |
    *  \param box_corners                  --
    */
    ObjectViewDescriptor(HalconCpp::HDescriptorModel &desc_model, Eigen::Vector3d view_orientation, double score_2D, bool use_color, int vertical_tex_offset, int horizontal_tex_offset, bool is_invertible, std::vector<RotationAxis> &axes, std::vector<Eigen::Vector2i> &box_corners);

    /**
    *   Getters for the class members
    */
    HalconCpp::HDescriptorModel getDescModel() const;
    Eigen::Vector3d getViewOrientation() const;
    double getScore2D() const;
    bool getUseColor() const;
    int getVerticalTexOffset() const;
    int getHorizontalTexOffset() const;
    bool getIsInvertible() const;
    std::vector<RotationAxis> getAxes() const;
    std::vector<Eigen::Vector2i> getBoxCorners() const;
};


}

#endif
