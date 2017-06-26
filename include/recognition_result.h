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


#ifndef RECOGNITION_RESULT_H
#define RECOGNITION_RESULT_H

#include <HalconCpp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace descriptor_surface_based_recognition {

/**
*  This class represents a recognized instance of an object
*/
class RecognitionResult {

private:
    /** If true the pose of the model was correctly found (3D and 2D), false if the object was found only in 2D  */
    bool model_found_;

    /** The index of the object view found in the 2D-recognition */
    int view_index;

    /** The found 2D-recognition score */
    double score_2D_;

    /** The transformation matrix of the view found in the 2D-recognition */
    HalconCpp::HHomMat2D trans_matrix_2D_;

    /** The transformed (with the matrix above) center of the found view's feature points */
    Eigen::Vector2i tex_point_;

    /** The quaternion used to adjust the rotation of the pose above to get the correct pose */
    HalconCpp::HQuaternion adjusted_rotation_;

    /** The length of the square, reduced search domain used for the 2D-recognition  */
    int search_radius_;

    /** Indicates whether the found pose is valid */
    bool pose_valid_;

    /** The found 3D-recognition score */
    double score_3D_;

    /** The 3D-point in the point cloud corrensponding to the texPoint above */
    Eigen::Vector3d tex_point_3D_;

    /** The reduced point cloud used to recognize the object */
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud_;

    /** The found pose */
    HalconCpp::HTuple pose_;

public:
    /** Macro needed to allow a dynamic construction of this class */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
    *  \brief The constructor of this class
    *
    *  \param view_index            --
    *  \param score2D                |-- see above
    *  \param trans_matrix_2D        |
    *  \param texPoint              --
    */
    RecognitionResult(int view_index, double score_2D, HalconCpp::HHomMat2D &trans_matrix_2D, const Eigen::Vector2i &tex_point);

    /**
    *  \brief Checks whether the model was correctly found
    *
    *  \return True if the model was correctly found in 3D and 2D, false if it was found only in 2D
    */
    bool checkModelFound();

    /**
    *   Setters for the class members
    */
    void setSearchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &search_cloud);
    void setPose(HalconCpp::HTuple pose);
    void setModelFound(bool model_found = true);
    void setTexPoint3D(pcl::PointXYZ tex_point_3D);
    void setAdjustedRotation(HalconCpp::HQuaternion quaternion);
    void setScore3D(double score_3D);
    void setSearchRadius(int radius);
    void setPoseValid(bool pose_valid);

    /**
    *   Getters for the class members
    */
    HalconCpp::HTuple getPose() const;
    int getViewIndex() const;
    double getScore2D() const;
    double getScore3D() const;
    HalconCpp::HHomMat2D getTransMatrix2D() const;
    Eigen::Vector2i getTexPoint() const;
    Eigen::Vector3d getTexPoint3D() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getSearchCloud() const;
    HalconCpp::HQuaternion getAdjustedRotation() const;
    int getSearchRadius() const;
    bool getPoseValid() const;

};


}

#endif
