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


#ifndef OBJECT_DESCRIPTOR_H
#define OBJECT_DESCRIPTOR_H

#include <string>
#include <HalconCpp.h>
#include "object_view_descriptor.h"


namespace descriptor_surface_based_recognition {

/**
*  This class represents an object which can be recognized
*/
class ObjectDescriptor {

private:

    /** The name of the object */
    std::string name_;

    /** The surface-model used for the recognition of the object in the point cloud */
    HalconCpp::HSurfaceModel surface_model_;

    /** The name of the object model used for visualisation */
    std::string mesh_name_;

    /** The rotation-type of the object */
    int rotation_model_type_;

    /** The orientation of the object */
    Eigen::Vector3d orientation_;

    /** The maximum diameter of the object's bounding box */
    double diameter_;

    /** The minimum score a valid 3D-recognition result needs */
    double score_3D_;

    /** A list of descriptors of the object views used for 2D-recognition */
    std::vector<ObjectViewDescriptor> model_view_descriptors_;

    /** Indicates whether this object is valid and can be used for recognition */
    bool valid_object_;

    /** The maximum number of instances which can be found */
    int instance_count_;

    bool use_pose_val_;

public:

    /**
    *  The rotation-types an object can have
    */
    static const int ROTATION_MODEL_NONE = 0;
    static const int ROTATION_MODEL_CYLINDER = 1;
    static const int ROTATION_MODEL_SPHERE = 2;

    /**
    *  \brief The constructor of this class
    *
    *  \param folder_path       The path to the folder containing the trained objects
    *  \param xml_path          The path to the file containing the meta data for this object
    *  \param instance_count    The maximum number of instances which can be found
    */
    ObjectDescriptor(std::string folder_path, std::string xml_path, int instance_count, bool use_pose_val);

    /**
    *  \brief Checks whether this object can be used for recognition
    *
    *  \return True if this object is valid, false otherwise
    */
    bool isValid() const;

    /**
    *   Getters for the class members
    */
    std::string getName() const;
    HalconCpp::HSurfaceModel getSurfaceModel() const;
    std::string getMesh() const;
    int getRotationModelType() const;
    Eigen::Vector3d getOrientation() const;
    double getDiameter() const;
    double getScore3D() const;
    int getInstanceCount() const;
    std::vector<ObjectViewDescriptor> getModelViewDescriptors() const;
    bool getUsePoseVal() const;

    /**
    *   Setters for the class members
    */
    void setCount(int count);
    void setUsePoseVal(bool usePoseVal);
};

}
#endif
