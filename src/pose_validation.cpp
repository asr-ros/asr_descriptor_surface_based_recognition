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


#include "pose_validation.h"

#include <rviz/mesh_loader.h>

#include <OgreMesh.h>
#include <OGRE/OgreLight.h>

#include <ros/package.h>
#include <cmath>
#include <ros/console.h>

namespace descriptor_surface_based_recognition {

PoseValidation::PoseValidation() : is_initialized_(false)
{

}

PoseValidation::PoseValidation(double image_width, double image_height, double far_plane, double near_plane, double cx, double cy, double fx, double fy, int render_width, int render_height) :
    image_width_(image_width),
    image_height_(image_height),
    is_initialized_(true)
{

    ROS_DEBUG_STREAM("Initialize render system");

    render_sys_ = rviz::RenderSystem::get();
    root_ = render_sys_->root();
    root_->initialise(false);

    ROS_DEBUG_STREAM("Render system initialized");

    scene_manager_ = root_->createSceneManager(Ogre::ST_GENERIC);

    scene_manager_->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

    Ogre::Light* light2 = scene_manager_->createLight("MainLight");
    light2->setType( Ogre::Light::LT_DIRECTIONAL );
    light2->setDirection( Ogre::Vector3( 0 , 0, -1 ) );
    light2->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );


    camera_ = scene_manager_->createCamera("mainCamera");

    Ogre::Matrix4 proj_matrix;
    proj_matrix = Ogre::Matrix4::ZERO;

    proj_matrix[0][0] = 2.0 * fx/image_width;
    proj_matrix[1][1] = 2.0 * fy/image_height;
    proj_matrix[0][2] = 2.0 * (0.5 - cx/image_width);
    proj_matrix[1][2] = 2.0 * (cy/image_height - 0.5);
    proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
    proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);
    proj_matrix[3][2] = -1;
    camera_->setCustomProjectionMatrix(true, proj_matrix);

    camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    camera_node_->attachObject(camera_);


    rtt_texture_ =
            Ogre::TextureManager::getSingleton().createManual(
                "RttTex",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D,
                image_width, image_height,
                0,
                Ogre::PF_R8G8B8,
                Ogre::TU_RENDERTARGET);


    render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();

    render_texture_->addViewport(camera_);
    render_texture_->getViewport(0)->setClearEveryFrame(true);
    render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Blue);
    render_texture_->getViewport(0)->setOverlaysEnabled(false);

    render_texture_->update();

    ROS_DEBUG_STREAM("Create render window");

    window_ = root_->createRenderWindow("Pose validation render window", render_width, render_height, false);
    Ogre::WindowEventUtilities::messagePump();
    window_->addViewport(camera_);


}

bool PoseValidation::isInitialized() const
{
    return is_initialized_;
}

HalconCpp::HImage PoseValidation::renderObjectImage(RecognitionResult *recognition_result, Ogre::MeshPtr mesh)
{

    HalconCpp::HPose pose;
    pose.SetFromTuple(recognition_result->getPose());
    HalconCpp::HQuaternion quaternion;
    quaternion.PoseToQuat(pose);
    quaternion = quaternion.QuatCompose(recognition_result->getAdjustedRotation());

    scene_manager_->destroyAllEntities();

    Ogre::Entity* meshEntity = scene_manager_->createEntity(mesh->getName());
    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(pose.ConvertToTuple()[0], -1 * pose.ConvertToTuple()[1], -1 * pose.ConvertToTuple()[2]));
    node->scale(0.001, 0.001, 0.001);
    node->attachObject(meshEntity);
    Ogre::Quaternion orient(quaternion.ConvertToTuple()[0], quaternion.ConvertToTuple()[1], quaternion.ConvertToTuple()[2], quaternion.ConvertToTuple()[3]);
    orient = Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X) * orient;
    node->setOrientation(orient);

    root_->renderOneFrame();
    render_texture_->update();

    rtt_texture_->getBuffer()->lock(Ogre::HardwareBuffer::HBL_NORMAL);

    const Ogre::PixelBox& pixelBox = rtt_texture_->getBuffer()->getCurrentLock();

    Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
    long* pixeldata = (long*)const_cast<unsigned char*>(&pDest[0]);
    rtt_texture_->getBuffer()->unlock();

    HalconCpp::HImage img;
    img.GenImageInterleaved(pixeldata, "bgrx", image_width_, image_height_, 0, "byte", image_width_, image_height_, 0, 0, -1, 0);

    return img;

}


HalconCpp::HTuple PoseValidation::validateObject(ObjectDescriptor *object, RecognitionResult *recognition_result, Ogre::MeshPtr mesh)
{
    bool use_color = object->getModelViewDescriptors().at(recognition_result->getViewIndex()).getUseColor();
    HalconCpp::HImage rendered_image = renderObjectImage(recognition_result, mesh);
    if (!use_color) {
        rendered_image = rendered_image.Rgb1ToGray();
    }

    HalconCpp::HTuple score;
    rendered_image.FindUncalibDescriptorModel(object->getModelViewDescriptors().at(recognition_result->getViewIndex()).getDescModel(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), HalconCpp::HTuple(), object->getModelViewDescriptors().at(recognition_result->getViewIndex()).getScore2D(), 1, "inlier_ratio", &score);

    if (score.Length() > 0) {
        HalconCpp::HTuple matrix_tuple = object->getModelViewDescriptors().at(recognition_result->getViewIndex()).getDescModel().GetDescriptorModelResults(0, "homography");
        return matrix_tuple;
    }
    return HalconCpp::HTuple();
}


bool PoseValidation::compareHomographyMatrices(HalconCpp::HHomMat2D original, HalconCpp::HHomMat2D rendered, int pose_validation_dist_err)
{
    double x_1, y_1, w_1, x_2, y_2, w_2, x_3, y_3, w_3;
    x_1 = original.ProjectiveTransPoint2d(0, 0, 1, &y_1, &w_1);
    x_2 = original.ProjectiveTransPoint2d(0, 10, 1, &y_2, &w_2);
    x_3 = original.ProjectiveTransPoint2d(10, 0, 1, &y_3, &w_3);

    double x_rend_1, y_rend_1, w_rend_1, x_rend_2, y_rend_2, w_rend_2, x_rend_3, y_rend_3, w_rend_3;
    x_rend_1 = rendered.ProjectiveTransPoint2d(0, 0, 1, &y_rend_1, &w_rend_1);
    x_rend_2 = rendered.ProjectiveTransPoint2d(0, 10, 1, &y_rend_2, &w_rend_2);
    x_rend_3 = rendered.ProjectiveTransPoint2d(10, 0, 1, &y_rend_3, &w_rend_3);

    x_1 = x_1 / w_1;
    y_1 = y_1 / w_1;
    x_2 = x_2 / w_2;
    y_2 = y_2 / w_2;
    x_3 = x_3 / w_3;
    y_3 = y_3 / w_3;

    x_rend_1 = x_rend_1 / w_rend_1;
    y_rend_1 = y_rend_1 / w_rend_1;
    x_rend_2 = x_rend_2 / w_rend_2;
    y_rend_2 = y_rend_2 / w_rend_2;
    x_rend_3 = x_rend_3 / w_rend_3;
    y_rend_3 = y_rend_3 / w_rend_3;

    double dist_1, dist_2, dist_3;
    dist_1 = sqrt(pow(x_1 -  x_rend_1, 2) + pow(y_1 -  y_rend_1, 2));
    dist_2 = sqrt(pow(x_2 -  x_rend_2, 2) + pow(y_2 -  y_rend_2, 2));
    dist_3 = sqrt(pow(x_3 -  x_rend_3, 2) + pow(y_3 -  y_rend_3, 2));

    ROS_DEBUG_STREAM("Pose validation: Max. error = " << pose_validation_dist_err << ", dist1: " << dist_1 << ", dist2: " << dist_2 << ", dist3: " << dist_3);
    return dist_1 < pose_validation_dist_err && dist_2 < pose_validation_dist_err && dist_3 < pose_validation_dist_err;
}

}
