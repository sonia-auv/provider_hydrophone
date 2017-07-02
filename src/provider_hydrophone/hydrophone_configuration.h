/**
 * \file	hydrophone_configuration.h
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

#ifndef PROVIDER_HYDROPHONE_HYDROPHONE_CONFIGURATION_H
#define PROVIDER_HYDROPHONE_HYDROPHONE_CONFIGURATION_H

namespace provider_hydrophone
{
    class HydrophoneConfiguration {

    public:

        HydrophoneConfiguration(const ros::NodeHandlePtr &nh);
        ~HydrophoneConfiguration();

    private:

        void DeserializeConfiguration();

        template <typename Tp_>
        void FindParameter(const std::string &str, Tp_ &p);

        ros::NodeHandlePtr nh_;

    };

}

#endif //PROVIDER_HYDROPHONE_HYDROPHONE_CONFIGURATION_H
