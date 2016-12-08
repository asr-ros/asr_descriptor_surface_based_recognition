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


#ifndef OBJECT_2D_POSITIONS_H
#define OBJECT_2D_POSITIONS_H

#include <vector>
#include <Eigen/Dense>

namespace descriptor_surface_based_recognition {

/**
*  This class represents the 2D position(s) of an object found in the last frame and is used to reduce the search domain for the current frame
*/
class Object2DPositions {

private:

    /** The index of the object this position(s) belong(s) to (out of the list of all objects) */
    int object_index_;

    /** The 2D-positions (one for every found instance) */
    std::vector<Eigen::Vector2i> positions_;

    /** The length of the square, reduced search domain centered at the above positions  */
    std::vector<int> search_radii_;

public:

    /**
    * \brief The constructor of this class
    *
    * \param object_index   The index of the object this position(s) belong(s) to
    */
    Object2DPositions(int object_index_);

    /**
    * \brief Adds a new object instance position to the list
    *
    * \param position           The object instance's position
    * \param search_radius      The object instance's reduced search domain's length (== maximum extent of the axis aligned bounding box)
    */
    void addPosition(Eigen::Vector2i position, int search_radius);

    /**
    *   Getters for the class members
    */
    std::vector<Eigen::Vector2i> getPositions() const;
    int getObjectIndex() const;
    std::vector<int> getSearchRadii() const;
};

}
#endif
