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


#include "object_2D_positions.h"

namespace descriptor_surface_based_recognition {

Object2DPositions::Object2DPositions(int object_index) : object_index_(object_index) { }

void Object2DPositions::addPosition(Eigen::Vector2i position, int search_radius)
{
    positions_.push_back(position);
    search_radii_.push_back(search_radius);
}

std::vector<Eigen::Vector2i> Object2DPositions::getPositions() const { return positions_; }

int Object2DPositions::getObjectIndex() const { return object_index_; }

std::vector<int> Object2DPositions::getSearchRadii() const { return search_radii_; }


}
