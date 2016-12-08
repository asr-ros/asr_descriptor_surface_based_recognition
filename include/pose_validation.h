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


#ifndef POSE_VALIDATION_H
#define POSE_VALIDATION_H

#include <rviz/ogre_helpers/render_system.h>

#include <OGRE/Ogre.h>
#include <OGRE/OgreException.h>
#include <OGRE/OgreRoot.h>

#include <HalconCpp.h>

#include "object_descriptor.h"
#include "recognition_result.h"



namespace descriptor_surface_based_recognition {

/**
*   This class is used for the validation of found poses
*/
class PoseValidation {

private:
     /** The width of the rendered image */
    double image_width_;

     /** The height of the rendered image */
    double image_height_;

    /** Indicates whether this object is initialized */
    bool is_initialized_;


    /** Ogre variables used for the rendering of the image */
    Ogre::Root* root_;
    Ogre::SceneManager* scene_manager_;
    rviz::RenderSystem* render_sys_;
    Ogre::Camera* camera_;
    Ogre::SceneNode* camera_node_;
    Ogre::TexturePtr rtt_texture_;
    Ogre::RenderTexture* render_texture_;
    Ogre::RenderWindow *window_;

    /**
    *  \brief Renders the given mesh
    *
    *  \param recognition_result      The object containing the information about the found pose
    *  \param mesh                    The mesh which will be rendered
    *
    * \return The rendered image
    */
    HalconCpp::HImage renderObjectImage(RecognitionResult *recognition_result, Ogre::MeshPtr mesh);


public:

    /**
    * \brief The empty constructor of this class
    */
    PoseValidation();

    /**
    * \brief The constructor of this class
    *
    * \param image_width    The width of the rendered image
    * \param image_height   The height of the rendered image
    */
    PoseValidation(double image_width, double image_height, double far_plane, double near_plane, double cx, double cy, double fx, double fy, int render_width, int render_height);

    /**
    * \brief Returns whether this object has been initialized
    *
    * \param True if initialized, false otherwise
    */
    bool isInitialized() const;

    /**
    * \brief Searches the rendered image for the given object
    *
    * \param object                 The object descriptor of the rendered object
    * \param recognition_result     The information about the found pose
    * \param mesh                   The mesh which will be rendered
    *
    * \return The found projection matrix as a HalconCpp::HTuple (empty if no object was found)
    */
    HalconCpp::HTuple validateObject(ObjectDescriptor *object, RecognitionResult *recognition_result, Ogre::MeshPtr mesh);

    /**
    * \brief Compares the given matrices and returns whether they describe the same projection
    *
    * \param original                       The matrix which was found in the original image
    * \param rendered                       The matrix which was found in the rendered image
    * \param poseValidationDistanceError    The maximum distance two points transformed with the given matrices may have
    *
    * \return True if the matrices describe the same projection, false otherwise
    */
    bool compareHomographyMatrices(HalconCpp::HHomMat2D original, HalconCpp::HHomMat2D rendered, int pose_validation_dist_err);

};


}

#endif
