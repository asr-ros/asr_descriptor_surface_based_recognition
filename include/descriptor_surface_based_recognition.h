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


#ifndef DESCRIPTOR_SURFACED_BASED_RECOGNITION_H
#define DESCRIPTOR_SURFACED_BASED_RECOGNITION_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <asr_descriptor_surface_based_recognition/DescriptorSurfaceBasedRecognitionConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Dense>

#include <asr_msgs/AsrObject.h>
#include <asr_object_database/ObjectMetaData.h>
#include <asr_object_database/RecognizerListMeshes.h>

#include <asr_descriptor_surface_based_recognition/GetRecognizer.h>
#include <asr_descriptor_surface_based_recognition/ReleaseRecognizer.h>
#include <asr_descriptor_surface_based_recognition/GetObjectList.h>
#include <asr_descriptor_surface_based_recognition/ClearAllRecognizers.h>

#include "threadpool/threadpool.hpp"
#include "pose_validation.h"
#include "object_descriptor.h"
#include "pose_recognition.h"
#include "object_2D_positions.h"
#include "util.h"


namespace descriptor_surface_based_recognition {

using namespace asr_descriptor_surface_based_recognition;

const static std::string NODE_NAME("asr_descriptor_surface_based_recognition");

const static std::string GET_RECOGNIZER_SERVICE_NAME("get_recognizer");
const static std::string RELEASE_RECOGNIZER_SERVICE_NAME("release_recognizer");
const static std::string GET_OBJECT_LIST_SERVICE_NAME("get_object_list");
const static std::string CLEAR_ALL_RECOGNIZERS_SERVICE_NAME("clear_all_recognizers");
const static std::string OBJECT_DB_SERVICE_OBJECT_TYPE ("/asr_object_database/object_meta_data");
const static std::string OBJECT_DB_SERVICE_OBJECT_MESHES("/asr_object_database/recognizer_list_meshes");

/**
*   The Evaluation files' names
*/
const static std::string OUTPUT_EVALUATION_DIR("eval");
const static std::string OUTPUT_EVALUATION_FILE_TIME("global_times.txt");
const static std::string OUTPUT_EVALUATION_FILE_POSES("global_poses.txt");

/**
*   The name of the category in the object database the objects of this recognizer belong to
*/
const static std::string OBJECT_DATABASE_CATEGORY("descriptor");

/** The path to this package */
extern std::string PACKAGE_PATH;



typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;


/**
*   The central class of the recognition system used for managing the ros subscriptions, configuration changes, loading of the objects, the search tasks and the visualisation of the results
*/
class DescriptorSurfaceBasedRecognition {

private:

    /** The name of the topic the colored input image is published on **/
    std::string image_color_topic_;

    /** The name of the topic the greyscale input image is published on **/
    std::string image_mono_topic_;

    /** The name of the topic the input point cloud is published on **/
    std::string point_cloud_topic_;

    /** The name of the topic the found objects are published on as asr_msgs **/
    std::string output_objects_topic_;

    /** The name of the topic the object visualizations are published on **/
    std::string output_marker_topic_;

    /** The name of the topic the visualized bounding boxes around the found objects are published on **/
    std::string output_marker_bounding_box_topic_;

    /** The name of the topic the reduced point clouds are published on **/
    std::string output_cloud_topic_;

    /** The name of the topic the results of the texture search are published on **/
    std::string output_image_topic_;


    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh_;

    /** Dynamic reconfigure server which keeps track of the callback function */
    dynamic_reconfigure::Server<DescriptorSurfaceBasedRecognitionConfig> reconfigure_server_;

    /** The configuration file containing the dynamic parameters of this system **/
    DescriptorSurfaceBasedRecognitionConfig config_;

    /** Ros subscription filters which will only pass the received messages they subscribed to (=> see input topics above) */
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_mono_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_with_guppy_sub_;

    /** Ros service handlers used for handling requests */
    ros::ServiceServer get_recognizer_service_;
    ros::ServiceServer release_recognizer_service_;
    ros::ServiceServer get_object_list_service_;
    ros::ServiceServer clear_all_recognizers_service_;

    ros::ServiceClient object_db_service_client_;
    ros::ServiceClient object_db_meshes_service_client_;

    /** The policy the subscribers are synced with */
    boost::shared_ptr<ApproximateSync> sync_policy_;

    /** Ros publishers which manage the advertisement of specific topics (=> see output topics above) */
    ros::Publisher objects_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher boxes_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher image_pub_;


    /** Arrays containing the object markers and the reduced clouds' bounding boxes of the current frame */
    visualization_msgs::MarkerArrayPtr msgs_marker_array_;
    visualization_msgs::MarkerArrayPtr msgs_box_marker_array_;

    /** Threadpool used to parallelize the search tasks */
    boost::threadpool::pool thread_pool_;

    /** A list containing the loaded objects which can be recognized */
    std::vector<ObjectDescriptor> objects_;

    /** A list containing the loaded meshes which can be used for pose validation */
    std::vector<Ogre::MeshPtr> meshes_;

    /** A list containing the positions of the found objects of the last frame in the image used for 2D-recognition */
    std::vector<Object2DPositions> last_frame_positions_;

    /** Used for (optional) validation of found objects */
    PoseValidation pose_val_;

    /** Evaluation ofstream (times) */
    std::ofstream outstream_times_;

    /** Evaluation ofstream (poses) */
    std::ofstream outstream_poses_;

    /** A counter for the processed frames (used only during evaluation) */
    int frame_counter_;





    /**
    * \brief  The callback function for the ros subscriptions (images and point cloud; called for every new frame)
    *
    * \param input_image_guppy              The colored image received by the camera the topic IMAGE_COLOR_TOPIC belongs to
    * \param input_image_guppy_mono         The greyscale image received by the camera the topic IMAGE_MONO_TOPIC belongs to
    * \param input_point_cloud_with_guppy   The point cloud received by the device the topic POINT_CLOUD_TOPIC belongs to
    */
    void rosCallback(const sensor_msgs::ImageConstPtr& input_image_guppy, const sensor_msgs::ImageConstPtr& input_image_guppy_mono, const sensor_msgs::PointCloud2ConstPtr& input_point_cloud_with_guppy);

    /**
    * \brief  The callback function which is called when the configuration file has changed
    *
    * \param config         The updated configuration
    * \param level          The level which is the result of ORing together all of level values of the parameters that have changed
    */
    void configCallback(DescriptorSurfaceBasedRecognitionConfig &config_, uint32_t level);

    /**
    * \brief  The task given to each thread in the threadpool (=> Search a specific object)
    *
    * \param pose_rec       The class which contains all relevant data for the search of the object and is used for the search itself
    */
    void threadTask(PoseRecognition *pose_rec);

    /**
    * \brief Creates a Asr-Object-Message containing the results of a found object instance
    *
    * \param pose_rec       A pointer to the recognition class containing the results for a specific object
    * \param results_index  The index of the found instance of the given object class (in pose_rec)
    * \param header         Contains information about the frame the found pose is relative to
    *
    * \return               The Asr-Object-Message containing meta-data of the found object instance and its recognition results (e.g. its pose)
    */
    asr_msgs::AsrObjectPtr createAsrMessage(PoseRecognition *pose_rec, int results_index, std_msgs::Header header);

    /**
    * \brief Adds a marker which indicates the pose of the found object to the array containing all found markers of this frame (msgs_markerArray)
    *
    * \param object         The found object instance containing the pose and frame information
    * \param mesh_path      Path to the (visualisation-) mesh of the given object instance
    */
    void createMarker(asr_msgs::AsrObjectPtr &object, std::string &mesh_path);

    /**
    * \brief Clears the visualized markers of the last frame
    */
    void clearMarkers();

    /**
    * \brief Adds a bounding box marker which indicates the size of the reduced point cloud used to recognize the given object instance to the array of bounding box markers (msgs_box_markerArray)
    *
    * \param result             The result of the object recognition (contains information about one specific found object instance)
    * \param header             Contains information about the frame the found pose is relative to
    * \param color              The (rgb-) color of the bounding box
    * \param drawCompleteBoxes  If false only the side of the box which faces the camera is drawn, the complete box otherwise
    */
    void createBoxMarker(RecognitionResult *result, std_msgs::Header header, rgb color, bool drawCompleteBoxes);

    /**
    * \brief Paints the results of the 2D recognition (bounding box, feature points & orientation triangle) to the given scene image
    *
    * \param pose_rec       A pointer to the recognition class containing the results for a specific object
    * \param scene_image    A pointer to the scene image the results are painted on
    */
    void overlaySceneWith2DResults(PoseRecognition *pose_rec, HalconCpp::HImage *scene_image);

    /**
    * \brief  Creates a single, colored point cloud from the reduced point clouds which were used to recognize the found objects
    *
    * \param pose_recs              The list of the classes used for searching the objects (contain the search results)
    * \param input_point_cloud      The point cloud the recuced clouds are based on
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createVisualizationCloud(std::vector<PoseRecognition*> &pose_recs, std_msgs::Header header);

    /**
     * Processes the request to recognize the given object
     * \param req the request message
     * \param res the correlated response message.
     */
    bool processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res);

    /**
     * Processes the request to release a previously created recognizer.
     * \param req the request message
     * \param res the correlated response message.
     */
    bool processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res);

    /**
     * Processes the request to get a list of all objects it is searched for
     * \param req the request message
     * \param res the correlated response message.
     */
    bool processGetObjectListRequest(GetObjectList::Request &req, GetObjectList::Response &res);

    /**
     * Processes the request to clear all recognizers
     * \param req the request message
     * \param res the correlated response message.
     */
    bool processClearAllRecognizersRequest(ClearAllRecognizers::Request &req, ClearAllRecognizers::Response &res);

    /**
    * \brief Loads all available object meshes so they can be used later
    */
    void initializeMeshes();

    /**
    * \brief Adds an object to the list of searchable objects
    *
    * \param name           The name of the object
    * \param count          The number of instances the system searches for
    * \param use_pose_val   Indicates whether a pose validation is used for this object
    *
    * \return True if the object was added successfully, false otherwise
    */
    bool startObjectRecognition(std::string name, int count, bool use_pose_val);

    /**
    * \brief Removes an object from the list of searchable objects
    *
    * \param name           The name of the object
    */
    void stopObjectRecognition(std::string name);

public:

    /**
    * \brief  The constructor of this class
    */
    DescriptorSurfaceBasedRecognition();

};

}

#endif
