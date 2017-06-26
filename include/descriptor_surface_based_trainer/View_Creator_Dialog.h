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


#ifndef VIEW_CREATOR_DIALOG_H_
#define VIEW_CREATOR_DIALOG_H_

#include "View_Creator_Dialog_Base.h"
#include <wx/wx.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <HalconCpp.h>
#include "View_Params_Wrapper.h"
#include <boost/thread.hpp>

/**
*   This class represents the dialog which is used to create a new view used for 2D-recognition
*/
class ViewCreatorDialog : public ViewCreatorDialogBase
{

private:

    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh;

    /** Ros subscriber which manages the subscription callback on the image-topic */
    ros::Subscriber image_model_sub;

    /** Ros subscriber which manages the subscription callback on the test-image-topic */
    ros::Subscriber image_test_sub;

    /** The fixed image */
    HalconCpp::HImage fixedImage;

    /** A pointer to the parameters used for this view */
    ViewParamsWrapper *params;

    /** Indicates whether an input camera image is fixed and can be used for the creation of the view */
    bool fixed_image_available;

    /** Indicates whether a descriptor-model used for testing is already created */
    bool desc_model_available;

    /** Indicates whether a test is running */
    bool test_running;

    /** The number of frames of the current test */
    int frame_counter;

    /** The sum of the scores of each frame of the current test */
    double score_sum;

    /** The sum of the feature-amounts of each frame of the current test */
    double feature_sum;

    /** The sum of the recognition-times of each frame of the current test */
    double time_sum;

    /** Timer used to update the shown camera images */
    wxTimer* update_timer;

    /** The descriptor-model used for testing */
    HalconCpp::HDescriptorModel desc_model;





    /** The thread used for painting the found features onto the input image during testing */
    boost::thread paint_thread;

    /** The number of updates (see update_timer) */
    int update_counter;

    /**
    *   \brief The callback function for the update-timer
    */
    void onUpdate(wxTimerEvent& evt);

    /**
    *   \brief The callback function for the image-topic-subscriber
    *
    *   \param msg  The input ros-image-message
    */
    void onModelCameraImage(const sensor_msgs::Image::ConstPtr& msg);

    /**
    *   \brief The callback function for the test-image-topic-subscriber
    *
    *   \param msg          The input ros-image-message
    *   \param use_file     Indicates whether the test image is read from a file
    */
    void onTestCameraImage(const sensor_msgs::Image::ConstPtr& msg, bool use_file);

    /**
    *   \brief Enables/Disables the Gui-elements used for setting the parameters
    *
    *   \param enable       Indicates whether the elements should be enabled or disabled
    */
    void enableGuiElements(bool enable = true);

    /**
    *   \brief Enables/Disables all Gui-elements which are not used during testing
    *
    *   \param enable       Indicates whether the elements should be enabled or disabled
    */
    void enableGuiElementsTest(bool enable = true);

    /**
    *   \brief Updates the shown image (left panel) after it was cropped with the sliders
    */
    void updateImageOnCrop();

    /**
    *   \brief Shows the given image on the right image panel
    *
    *   \param  img     The given image
    */
    void paint_test_image(HalconCpp::HImage img);




protected:

    /**
    *   wxwidgets Gui-callback-functions
    */
    void OnDialogClose(wxCloseEvent &event);
    void onChoiceImageSource( wxCommandEvent& event );
    void onChoiceImage( wxCommandEvent& event );
    void onChoiceTestImageSource(wxCommandEvent &event);
    void onChoiceTestImage( wxCommandEvent& event );
    void onButtonCancelClicked( wxCommandEvent& event );
    void onButtonSaveClicked( wxCommandEvent& event );
    void onButtonStartTestClicked(wxCommandEvent &event);
    void onButtonEndTestClicked(wxCommandEvent &event);
    void onCheckUseCurrentImage(wxCommandEvent &event);

    void onEditTextUpperLeftRow(wxFocusEvent &event);
    void onEditTextUpperLeftColumn(wxFocusEvent& event);
    void onEditTextLowerRightRow(wxFocusEvent& event);
    void onEditTextLowerRightColumn(wxFocusEvent& event);

    void onSlideUpperLeftRow(wxScrollEvent &event);
    void onSlideUpperLeftColumn(wxScrollEvent &event);
    void onSlideLowerRightRow(wxScrollEvent &event);
    void onSlideLowerRightColumn(wxScrollEvent &event);

    void onEditTextUpperLeftRowEnter(wxCommandEvent &event);
    void onEditTextUpperLeftColumnEnter(wxCommandEvent &event);
    void onEditTextLowerRightRowEnter(wxCommandEvent &event);
    void onEditTextLowerRightColumnEnter(wxCommandEvent &event);

    void onEditTextOrientationX(wxFocusEvent &event);
    void onEditTextOrientationY(wxFocusEvent &event);
    void onEditTextOrientationZ(wxFocusEvent &event);
    void onEditTextScore(wxFocusEvent &event);
    void onEditTextVerticalOffset(wxFocusEvent &event);
    void onEditTextHorizontalOffset(wxFocusEvent &event);
    void onEditTextDepth(wxFocusEvent &event);
    void onEditTextFernNumber(wxFocusEvent &event);
    void onEditTextPatchSize(wxFocusEvent &event);
    void onEditTextMinScale(wxFocusEvent &event);
    void onEditTextMaxScale(wxFocusEvent &event);

    void onEditTextOrientationXEnter(wxCommandEvent &event);
    void onEditTextOrientationYEnter(wxCommandEvent &event);
    void onEditTextOrientationZEnter(wxCommandEvent &event);
    void onEditTextScoreEnter(wxCommandEvent &event);
    void onEditTextVerticalOffsetEnter(wxCommandEvent &event);
    void onEditTextHorizontalOffsetEnter(wxCommandEvent &event);
    void onEditTextDepthEnter(wxCommandEvent &event);
    void onEditTextFernNumberEnter(wxCommandEvent &event);
    void onEditTextPatchSizeEnter(wxCommandEvent &event);
    void onEditTextMinScaleEnter(wxCommandEvent &event);
    void onEditTextMaxScaleEnter(wxCommandEvent &event);

    void onEditTextAxis1X(wxFocusEvent &event);
    void onEditTextAxis1Y(wxFocusEvent &event);
    void onEditTextAxis1Z(wxFocusEvent &event);
    void onEditTextAxis1Angle(wxFocusEvent &event);
    void onEditTextAxis2X(wxFocusEvent &event);
    void onEditTextAxis2Y(wxFocusEvent &event);
    void onEditTextAxis2Z(wxFocusEvent &event);
    void onEditTextAxis2Angle(wxFocusEvent &event);

    void onEditTextAxis1XEnter(wxCommandEvent &event);
    void onEditTextAxis1YEnter(wxCommandEvent &event);
    void onEditTextAxis1ZEnter(wxCommandEvent &event);
    void onEditTextAxis1AngleEnter(wxCommandEvent &event);
    void onEditTextAxis2XEnter(wxCommandEvent &event);
    void onEditTextAxis2YEnter(wxCommandEvent &event);
    void onEditTextAxis2ZEnter(wxCommandEvent &event);
    void onEditTextAxis2AngleEnter(wxCommandEvent &event);

    void onCheckUpsideDown(wxCommandEvent &event);
    void onCheckUseColor(wxCommandEvent &event);


public:
    /**
    *   \brief The constructor of this class
    *
    *   \param parent      The parent window of this dialog
    *   \param params      The parameters used if this dialog was created after pushing the edit-button on DescModelDialog
    */
    ViewCreatorDialog(wxWindow* parent, ViewParamsWrapper* params);
};


#endif //DESC_MODELS_DIALOG_H_




