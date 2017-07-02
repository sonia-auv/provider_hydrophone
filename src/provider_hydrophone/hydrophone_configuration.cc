/**
 * \file	hydrophone_configuration.cc
 * \author  Marc-Antoine Couture <coumarc9@outlook.com>
 * \date	02/07/2016
 *
 * \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Changes by: S.O.N.I.A.
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "hydrophone_configuration.h"


namespace provider_hydrophone
{

    HydrophoneConfiguration::HydrophoneConfiguration(const ros::NodeHandlePtr &nh) {

    }

    HydrophoneConfiguration::~HydrophoneConfiguration() {}

    void HydrophoneConfiguration::DeserializeConfiguration() {

    }

    void HydrophoneConfiguration::FindParameter(const std::string &str, Tp_ &p) {

        if (nh_->hasParam("/provider_hydrophone" + str))
            nh_->getParam("/provider_hydrophone" + str, p);
        else
            ROS_WARN("Did not find /provider_hydrophone %s. Using default value instead.", str);

    }

}