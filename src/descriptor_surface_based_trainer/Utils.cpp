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


#include "descriptor_surface_based_trainer/Utils.h"
#include <halcon_image.h>
#include <boost/make_shared.hpp>



bool check_string_redex(std::string to_check, boost::regex regex)
{
    boost::cmatch what;
    return boost::regex_match(to_check.c_str(), what, regex);
}


std::string trim(std::string input) {
    input.erase(0,input.find_first_not_of(" "));
    input.erase(input.find_last_not_of(" ")+1);
    return input;
}

wxBitmap* createBitmap(const sensor_msgs::Image::ConstPtr& msg, int width, int height) {
    wxImage img(msg->width, msg->height);
    int bpp = msg->step / msg->width;
    if (msg->encoding == "bgr8") {
        for (int i = 0; i < img.GetHeight(); i++) {
            for (int j = 0; j < img.GetWidth(); j++) {
                img.SetRGB(j, i,
                        msg->data[i * msg->step + bpp * j + 2],
                        msg->data[i * msg->step + bpp * j + 1],
                        msg->data[i * msg->step + bpp * j]);
            }
        }
    } else if (msg->encoding == "mono8") {
        for (int i = 0; i < img.GetHeight(); i++) {
            for (int j = 0; j < img.GetWidth(); j++) {
                img.SetRGB(j, i,
                        msg->data[i * msg->step + bpp * j],
                        msg->data[i * msg->step + bpp * j],
                        msg->data[i * msg->step + bpp * j]);
            }
        }
    } else if (msg->encoding == "rgb8") {
        unsigned char *data = (unsigned char*)(&msg->data[0]);
        img = wxImage(msg->width, msg->height, data, true);
    }
    img = img.Scale(width, height);
    wxBitmap* image = new wxBitmap(img);
    return image;

}

wxBitmap* createBitmap(HalconCpp::HImage image, int width, int height)
{
    halcon_bridge::HalconImagePtr ptr = boost::make_shared<halcon_bridge::HalconImage>();
    if (image.CountChannels() == 1) {
        ptr->encoding = "mono8";
    } else {
        ptr->encoding = "rgb8";
    }
    ptr->image = new HalconCpp::HImage();
    *ptr->image = image;
    sensor_msgs::ImagePtr imgptr = ptr->toImageMsg();
    return createBitmap(imgptr, width, height);
}


wxString trimDoubleString(wxString input)
{
    std::string string = std::string(input.mb_str());
    if (check_string_redex(string, boost::regex("^[-+]?[0-9]+(\\.[0-9]+)$"))) {
        char cur = *string.rbegin();
        while (cur == '0') {
            string.resize(string.size() - 1);
            cur = *string.rbegin();
        }
        if (cur == '.') {
            string += '0';
        }
        return wxString(string.c_str(), wxConvUTF8);
    }
    return input;
}

void get_all_files_with_ext(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
        ++it;
    }

}

HalconCpp::HImage drawBoundingBox(HalconCpp::HImage image, std::vector<Eigen::Vector2i> corner_points)
{
    int max_expand = image.Width();
    if (image.Width() < image.Height()) {
        max_expand = image.Height();
    }
    int scale_factor = max_expand / 100;
    int BOX_BORDER_SIZE = 8;
    int CORNER_RADIUS = 3;
    if (scale_factor > 1) {
        BOX_BORDER_SIZE = BOX_BORDER_SIZE * (1.f + (scale_factor / 10.f));
        CORNER_RADIUS = CORNER_RADIUS * (1.f + (scale_factor / 10.f));
    }
    HalconCpp::HTuple color(255);
    if (image.CountChannels() > 1) {
        color.Append(0);
        color.Append(0);
    }
    HalconCpp::HTuple rows, columns;
    for (int i = 0; i < corner_points.size(); i++) {
        rows.Append(corner_points.at(i)[1]);
        columns.Append(corner_points.at(i)[0]);
    }
    if (corner_points.size() == 4) {
        HalconCpp::HTuple rows_outer, columns_outer;
        rows_outer.Append(rows[0] - BOX_BORDER_SIZE);
        rows_outer.Append(rows[1] - BOX_BORDER_SIZE);
        rows_outer.Append(rows[2] + BOX_BORDER_SIZE);
        rows_outer.Append(rows[3] + BOX_BORDER_SIZE);
        columns_outer.Append(columns[0] - BOX_BORDER_SIZE);
        columns_outer.Append(columns[1] + BOX_BORDER_SIZE);
        columns_outer.Append(columns[2] + BOX_BORDER_SIZE);
        columns_outer.Append(columns[3] - BOX_BORDER_SIZE);
        HalconCpp::HRegion box_points;
        box_points.GenRegionPolygonFilled(rows, columns);

        HalconCpp::HRegion box_points_outer;
        box_points_outer.GenRegionPolygonFilled(rows_outer, columns_outer);

        box_points = box_points_outer.Difference(box_points);
        image = box_points.PaintRegion(image, color, HalconCpp::HTuple("fill"));
    } else if (corner_points.size() > 0) {
        HalconCpp::HTuple radius;
        radius = HalconCpp::HTuple::TupleGenConst(rows.Length(), CORNER_RADIUS);
        HalconCpp::HRegion box_points;
        box_points.GenCircle(rows, columns, radius);
        image = box_points.PaintRegion(image, color, HalconCpp::HTuple("fill"));
    }

    return image;
}
