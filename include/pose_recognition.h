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


#ifndef POSE_RECOGNITION_H
#define POSE_RECOGNITION_H

#include <HalconCpp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "object_descriptor.h"
#include "recognition_result.h"




namespace descriptor_surface_based_recognition {

/**
*   This class is used for the recognition of an object
*/
class PoseRecognition {

private:

    /** The colored image of the scene used for the 2D-recognition */
    HalconCpp::HImage scene_image_;

    /** The greyscale image of the scene used for the 2D-recognition */
    HalconCpp::HImage scene_image_mono_;

    /** The point cloud of the scene used for the 3D-recognition */
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_with_guppy_;

    /** The descriptor of the object this class can recognize */
    ObjectDescriptor *obj_desc_;

    /** The maximum number of instances which can be found */
    int max_instances_;

    /** The pixel distance used to choose points for the lookup of the 3D point which is used for the reduction of the point cloud */
    int median_points_offset_;

    /** Scene sampling distance used for preparation of the scene the object will be recognized with (3D-recognition) */
    double sampling_distance_;

    /** Fraction of sampled scene points used as key points in the 3D-recognition */
    double keypoint_fraction_;

    /** The found object instances */
    std::vector<RecognitionResult*> results_;

    /** Indicates whether evaluation information are put out */
    bool eval_;


    /**
    *  \brief Recognizes the object in the 2D-scene
    */
    void findTexture();

    /**
    *  \brief Prepares the 3D-recognition by reducing the point cloud depending on the 2D-recognition result
    *
    *  \param result_index      The index of the found instance in the list above
    */
    bool reducePointCloud(int result_index);

    /**
    *  \brief Recognizes the object in the reduced 3D-scene
    *
    *  \param result_index      The index of the found instance in the list above
    */
    bool findModelInCloud(int result_index);

    /**
    *  \brief Adjusts the rotation of the found object instance depending on its rotation-type
    *
    *  \param result_index      The index of the found instance in the list above
    */
    void adjustRotation(int result_index);

public:

    /**
    *  \brief The constructor of this class
    *
    *  \param scene_image                   --
    *  \param scene_image_mono               |
    *  \param point_cloud_with_guppy         |
    *  \param obj_desc                       |-- see above
    *  \param median_points_offset           |
    *  \param samplingDistance               |
    *  \param keypointFraction               |
    *  \param max_instances                 --
    */
    PoseRecognition(HalconCpp::HImage &scene_image, HalconCpp::HImage &scene_image_mono, pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_with_guppy, ObjectDescriptor *obj_desc, int median_points_offset, double samplingDistance, double keypointFraction, bool eval, int max_instances = 1);

    /**
    *  \brief The destructor of this class
    */
    virtual ~PoseRecognition();

    /**
    *  \brief Recognizes the present object instances in the scene
    */
    void findPoses();

    /**
    *   Getters for the class members
    */
    std::vector<RecognitionResult*> getResults() const;
    ObjectDescriptor* getObjectDescriptor() const;
    HalconCpp::HRegion getInputImageDomain() const;
};


}

#endif
