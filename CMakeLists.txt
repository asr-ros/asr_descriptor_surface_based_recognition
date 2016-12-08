cmake_minimum_required(VERSION 2.4.6)
project(asr_descriptor_surface_based_recognition)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  std_msgs
  roscpp
  sensor_msgs
  asr_halcon_bridge
  roslib
  image_transport
  asr_msgs
  pcl_conversions
  pcl_ros
  rviz
  message_generation
  cmake_modules
  ros_uri
  asr_rapidxml
)

find_package(cmake_modules)
find_package(Boost REQUIRED filesystem system thread regex)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

generate_dynamic_reconfigure_options( cfg/DescriptorSurfaceBasedRecognition.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_service_files(
   DIRECTORY
   srv
   FILES
   GetRecognizer.srv
   ReleaseRecognizer.srv
   GetObjectList.srv
   ClearAllRecognizers.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS dynamic_reconfigure std_msgs roscpp ros_uri sensor_msgs asr_halcon_bridge roslib image_transport asr_msgs rviz asr_rapidxml
   DEPENDS Boost Eigen)
   
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

FILE(GLOB_RECURSE LIBFILES "include/*.h")
add_executable(descriptor_surface_based_recognition src/descriptor_surface_based_recognition.cpp
                                                  src/pose_validation.cpp
                                                  src/object_descriptor.cpp
                                                  src/object_view_descriptor.cpp
                                                  src/rotation_axis.cpp
                                                  src/util.cpp
                                                  src/pose_recognition.cpp
                                                  src/recognition_result.cpp
                                                  src/object_2D_positions.cpp ${LIBFILES})
TARGET_LINK_LIBRARIES(descriptor_surface_based_recognition
      ${Boost_LIBRARIES}
      ${catkin_LIBRARIES}
)

add_dependencies(descriptor_surface_based_recognition
        ${PROJECT_NAME}_gencfg
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        asr_msgs_generate_messages_cpp
)

add_dependencies(descriptor_surface_based_recognition ${catkin_EXPORTED_TARGETS})


add_executable(descriptor_surface_based_trainer src/descriptor_surface_based_trainer/Descriptor_Surface_Based_Trainer.cpp
                                                    src/descriptor_surface_based_trainer/Main_Dialog_Base.cpp
                                                    src/descriptor_surface_based_trainer/Main_Dialog.cpp
                                                    src/descriptor_surface_based_trainer/Desc_Models_Dialog_Base.cpp
                                                    src/descriptor_surface_based_trainer/Desc_Models_Dialog.cpp
                                                    src/descriptor_surface_based_trainer/View_Params_Wrapper.cpp
                                                    src/descriptor_surface_based_trainer/View_Creator_Dialog_Base.cpp
                                                    src/descriptor_surface_based_trainer/wxImagePanel.cpp
                                                    src/descriptor_surface_based_trainer/View_Creator_Dialog.cpp
                                                    src/descriptor_surface_based_trainer/Utils.cpp
                                                    src/descriptor_surface_based_trainer/Create_Test_Model_Dialog_Base.cpp
                                                    src/descriptor_surface_based_trainer/Create_Test_Model_Dialog.cpp
                                                    src/descriptor_surface_based_trainer/Create_Output_Files_Dialog_Base.cpp
                                                    src/descriptor_surface_based_trainer/Create_Output_Files_Dialog.cpp ${LIBFILES})

TARGET_LINK_LIBRARIES(descriptor_surface_based_trainer
        ${wxWidgets_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)
