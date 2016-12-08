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


#include <boost/algorithm/string.hpp>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/weighted_median.hpp>

#include "util.h"

namespace descriptor_surface_based_recognition {



using namespace boost::accumulators;

Eigen::Vector3d parseStringVector(std::string input_string, std::string delim) {

    std::vector<std::string> strvec;

    boost::algorithm::trim_if(input_string, boost::algorithm::is_any_of(delim));
    boost::algorithm::split(strvec, input_string, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);

    Eigen::Vector3d vector;
    for( unsigned int i=0; i < strvec.size(); i++) {
        vector[i] = boost::lexical_cast<double>(strvec[i]);
    }

    return vector;
}

Eigen::Vector2i parseStringVector2i(std::string input_string, std::string delim)
{
    std::vector<std::string> strvec;

    boost::algorithm::trim_if(input_string, boost::algorithm::is_any_of(delim));
    boost::algorithm::split(strvec, input_string, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);

    Eigen::Vector2i vector;
    for( unsigned int i=0; i < strvec.size(); i++) {
        vector[i] = boost::lexical_cast<int>(strvec[i]);
    }

    return vector;
}

Eigen::Vector3d computeMedian(std::vector<Eigen::Vector3d> points)
{
    if (points.size() > 0) {
        Eigen::Vector3d median_point = points.at(0);

        for (unsigned int i = 0; i < 3; i++) {
            std::vector<double> nums;
            for (unsigned int j = 0; j < points.size(); j++) {
                nums.push_back(points.at(j)[i]);
            }
            std::sort(nums.begin(), nums.end());
            if (nums.size() % 2 == 0) {
                median_point(i) = (nums[nums.size() / 2 - 1] + nums[nums.size() / 2]) / 2;
            } else {
                median_point(i) = nums[nums.size() / 2];
            }
        }
        return median_point;
    }
    return Eigen::Vector3d(0, 0, 0);
}

pcl::PointXYZ computeMedian(std::vector<pcl::PointXYZ> points)
{
    if (points.size() > 0) {
        std::vector<Eigen::Vector3d> points_eigen;
        for (unsigned i = 0; i < points.size(); i++) {
            if (pcl::isFinite(points.at(i))) {
                points_eigen.push_back(Eigen::Vector3d(points.at(i).x, points.at(i).y, points.at(i).z));
            }
        }
        Eigen::Vector3d median_point = computeMedian(points_eigen);
        return pcl::PointXYZ(median_point[0], median_point[1], median_point[2]);
    }
    return pcl::PointXYZ();
}

pcl::PointXYZ findPoint3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ current_point, int row, int column, int image_height, int image_width) {
    int step_counter = 1;
    bool finite_found = false;
    while ((!pcl::isFinite(current_point)) && (row > 1) && (column > 1) && (row < image_height - 1 - step_counter) && (column < image_width - 1 - step_counter)) {
        for (int i = 1; i <= step_counter; i++) {
            row = row + i;
            current_point = (*cloud)(column, row);
            if (pcl::isFinite(current_point)) {
                finite_found = true;
                break;
            }
        }
        if (finite_found) break;
        for (int i = 1; i <= step_counter; i++) {
            column = column + i;
            current_point = (*cloud)(column, row);
            if (pcl::isFinite(current_point)) {
                finite_found = true;
                break;
            }
        }
        if (finite_found) break;
        step_counter++;
        for (int i = 1; i <= step_counter; i++) {
            row = row - i;
            current_point = (*cloud)(column, row);
            if (pcl::isFinite(current_point)) {
                finite_found = true;
                break;
            }
        }
        if (finite_found) break;
        for (int i = 1; i <= step_counter; i++) {
            column = column - i;
            current_point = (*cloud)(column, row);
            if (pcl::isFinite(current_point)) {
                finite_found = true;
                break;
            }
        }
        if (finite_found) break;
        step_counter++;
    }
    return current_point;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}



}
