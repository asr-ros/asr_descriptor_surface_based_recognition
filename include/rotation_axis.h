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


#ifndef ROTATION_AXIS_H
#define ROTATION_AXIS_H

#include <HalconCpp.h>
#include <Eigen/Dense>
#include <vector>

namespace descriptor_surface_based_recognition {

/**
*   This class describes a rotation axis
*/
class RotationAxis {

private:

    /** The angle describing in what steps around the axis can be rotated */
    double angle_;

    /** The rotation axis */
    Eigen::Vector3d axis_;

public:

    /**
    *  \brief The constructor of this class
    *                  --
    *  \param angle     |-- see above
    *  \param axis      |
    *                  --
    */
    RotationAxis(double angle, Eigen::Vector3d axis);

    /**
    *   Getters for the class members
    */
    double getAngle() const;
    Eigen::Vector3d getAxis() const;
};

}

#endif

