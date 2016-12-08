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


#ifndef VIEW_PARAMS_WRAPPER_H_
#define VIEW_PARAMS_WRAPPER_H_

#include <Eigen/Dense>
#include <HalconCpp.h>
#include <vector>

/**
*   This class contains the parameters of a specific view created in ViewCreatorDialog
*/
class ViewParamsWrapper
{

private:

    /** Indicates whether this view is valid or contains errors */
    bool is_valid;

    /** The rotation type of the trained object (see Utils) */
    std::string rotation_type;

    /** The cropped image of this view */
    HalconCpp::HImage image;

    /** The original, non-cropped image of this view */
    HalconCpp::HImage originalImage;

    /** The row of the point indicating the upper left corner of the cropping rectangle */
    int row1;

    /** The column of the point indicating the upper left corner of the cropping rectangle */
    int column1;

    /** The row of the point indicating the lower right corner of the cropping rectangle */
    int row2;

    /** The column of the point indicating the lower right corner of the cropping rectangle */
    int column2;

    /** The rotation axis used if the object has a cylindrical or spherical symmetry */
    Eigen::Vector3d axis1;

    /** The second rotation axis used if the object has a spherical symmetry */
    Eigen::Vector3d axis2;

    /** The orientation describing this view */
    Eigen::Vector3d orientation;

    /** The angle steps around axis1 used during 3D-recognition */
    double axis1_angle;

    /** The angle steps around axis2 used during 3D-recognition */
    double axis2_angle;

    /** Indicates whether the view can be upside down */
    bool is_invertible;

    /** The score used for 2D-recognition of this view */
    double score_2D;

    /** Indicates whether a colored image will be used for 2D-recognition */
    bool use_color;

    /** The vertical offset of the texture center */
    int vertical_offset;

    /** The horizontal offset of the texture center */
    int horizontal_offset;

    /** The depth of the the classification ferns used by the descriptor-based recognition */
    int depth;

    /** The fern-number used by the descriptor-based recognition */
    int number_ferns;

    /** The patch size used by the descriptor-based recognition */
    int patch_size;

    /** The minimal scale of the view the model is created from */
    double min_scale;

    /** The maximal scale of the view the model is created from */
    double max_scale;

    /** Indicates whether a bounding box for the view has been created */
    bool has_bounding_box;

    /** The corner points of the created bounding box of this view */
    std::vector<Eigen::Vector2i> box_corners;


public:

    /**
    *   \brief The constructor of this class
    *
    *   \param rotation_type    The rotation-type of the trained object (see Utils)
    */
    ViewParamsWrapper(std::string rotation_type);

    /**
    *   The setters of this class
    */
    void setImage(HalconCpp::HImage image);
    void setOriginalImage(HalconCpp::HImage original_image);
    void setImageBounds(int row1, int column1, int row2, int column2);
    void setRotationType(std::string rotation_type);
    void setAxis1(Eigen::Vector3d axis1);
    void setAxis2(Eigen::Vector3d axis2);
    void setOrientation(Eigen::Vector3d orientation);
    void setAxis1Angle(double axis1_angle);
    void setAxis2Angle(double axis2_angle);
    void setIsInvertible(bool is_invertible);
    void setScore2D(double score_2D);
    void setUserColor(bool use_color);
    void setVerticalOffset(int vertical_offset);
    void setHorizontalOffset(int horizontal_offset);
    void setDepth(int depth);
    void setNumberFerns(int number_ferns);
    void setPatchSize(int patch_size);
    void setMinScale(double min_scale);
    void setMaxScale(double max_scale);
    void setIsValid(bool is_valid);
    void setHasBoundingBox(bool has_bounding_box = true);

    /**
    *   \brief Adds a corner point to the list of bounding box corners
    *
    *   \param corner   The corner point which will be added
    */
    void addBoxCorner(Eigen::Vector2i corner);

    /**
    *   Clears the list of bounding box corner points
    */
    void clearBoundingBox();


    /**
    *   The getters of this class
    */
    HalconCpp::HImage getImage();
    HalconCpp::HImage getOriginalImage();
    int getRow1();
    int getColumn1();
    int getRow2();
    int getColumn2();
    std::string getRotationType();
    Eigen::Vector3d getAxis1();
    Eigen::Vector3d getAxis2();
    Eigen::Vector3d getOrientation();
    double getAxis1Angle();
    double getAxis2Angle();
    bool getIsInvertible();
    double getScore2D();
    bool getUseColor();
    int getVerticalOffset();
    int getHorizontalOffset();
    int getDepth();
    int getNumberFerns();
    int getPatchSize();
    double getMinScale();
    double getMaxScale();
    bool getIsValid();
    bool getHasBoundingBox();
    std::vector<Eigen::Vector2i> getBoxCorners();

};


#endif //VIEW_PARAMS_WRAPPER_H_



