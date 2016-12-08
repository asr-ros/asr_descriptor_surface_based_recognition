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


#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Dense>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace descriptor_surface_based_recognition {



/**
*   Global utility functions
*/


/**
*  \brief Parses the given string and returns the 3D-double-vector described by it
*
*  \param input_string      The input vector given as a string
*  \param delim             The string used to split the input_string
*
*  \return The parsed vector
*/
Eigen::Vector3d parseStringVector(std::string input_string, std::string delim);

/**
*  \brief Parses the given string and returns the 2D-int-vector described by it
*
*  \param input_string      The input vector given as a string
*  \param delim             The string used to split the input_string
*
*  \return The parsed vector
*/
Eigen::Vector2i parseStringVector2i(std::string input_string, std::string delim);

/**
*  \brief Computes the median of the given points
*
*  \param points        The input points used for the calculation of the median
*
*  \return The median vector
*/
Eigen::Vector3d computeMedian(std::vector<Eigen::Vector3d> points);

/**
*  \brief Computes the median of the given points
*
*  \param points        The input points used for the calculation of the median
*
*  \return The median vector
*/
pcl::PointXYZ computeMedian(std::vector<pcl::PointXYZ> points);

/**
*  \brief Finds the corresponding 3D point to the given 2D point
*
*  \param cloud             The point cloud containing the correct 2D-3D-relation
*  \param current_point     The starting point which is returned if it is valid
*  \param row               The row of the 2D point
*  \param column            The column of the 2D point
*  \param image_height      The height of the image
*  \param image_width       The width of the image
*
*  \return The 3D-point which fits to the point in the image
*/
pcl::PointXYZ findPoint3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ current_point, int row, int column, int image_height, int image_width);

/**
*   Describes a rgb-color
*/
typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

/**
*   Describes a hsv-color
*/
typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;


/**
*  \brief Converts the given hsv-color to rgb
*
*  \param in    The given hsv-color
*
*  \return The converted rgb-color
*/
rgb hsv2rgb(hsv in);


}

#endif
