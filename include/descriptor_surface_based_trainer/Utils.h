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


#ifndef UTILS_H_
#define UTILS_H_

#include <boost/regex.hpp>
#include <wx/wx.h>
#include <sensor_msgs/Image.h>
#include <HalconCpp.h>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>

/**
*   Global utility functions
*/


/** Rotation type describing an object without axial symmetry */
const std::string ROTATIONTYPE_NO_ROTATION = "No Rotation";

/** Rotation type describing an object cylindrical symmetry */
const std::string ROTATIONTYPE_CYLINDER = "Cylinder";

/** Rotation type describing an object with spherical symmetry */
const std::string ROTATIONTYPE_SPHERE = "Sphere";

/** The input folder containing the data used for training (relative to package) */
const std::string INPUT_FOLDER = "/trainer_data/input";

/** The output folder the created trained object is written to (relative to package) */
const std::string OUTPUT_FOLDER = "/trainer_data/output";


/**
*   \brief Checks a string with the given regex
*
*   \return true if the string matches the given regex, false otherwise
*/
bool check_string_redex(std::string to_check, boost::regex regex);

/**
*   \brief Removes spaces from the beginning and end of the given string
*
*   \return The trimmed string
*/
std::string trim(std::string input);

/**
*   \brief Converts a ros-image-message to a bitmap-file used by wxwidgets
*
*   \param msg      The given ros-image-message
*   \param width    The width of the converted image
*   \param height   The height of the converted image
*
*   \return The converted bitmap
*/
wxBitmap* createBitmap(const sensor_msgs::Image::ConstPtr& msg, int width, int height);

/**
*   \brief Converts a halcon-image to a bitmap-file used by wxwidgets
*
*   \param image    The given halcon-image
*   \param width    The width of the converted image
*   \param height   The height of the converted image
*
*   \return The converted bitmap
*/
wxBitmap* createBitmap(HalconCpp::HImage image, int width, int height);

/**
*   \brief Formats the given string by removing fractional zeros at the end of it
*
*   \param input    The given string
*
*   \return The formatted string
*/
wxString trimDoubleString(wxString input);

/**
*   \brief Gets all files in a directory with a specific extension
*
*   \param root    The given directory
*   \param ext     The extension the files are chosen by
*   \param ret     The list the found files are added to
*/
void get_all_files_with_ext(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);

/**
*   \brief Draws a bounding box on the given image with the also given corner-points
*
*   \param image                The image the bounding box is drawn on
*   \param corner_points        The points describing the corner points of the bounding box in the image
*
*   \return The image with the bounding box
*/
HalconCpp::HImage drawBoundingBox(HalconCpp::HImage image, std::vector<Eigen::Vector2i> corner_points);

#endif //UTILS_H_
