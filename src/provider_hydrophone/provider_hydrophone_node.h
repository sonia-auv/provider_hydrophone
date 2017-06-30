/**
 * \file	provider_hydrophone_node.h
 * \author	Marc-Antoine Couture <coumarc9@outlook.com>
 * \date	06/25/2017
 *
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

#ifndef PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_
#define PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_

#include <ros/node_handle.h>
#include "drivers/HydroUsbDriver.h"

namespace provider_hydrophone {

class ProviderHydrophoneNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProviderHydrophoneNode(const ros::NodeHandlePtr &nh);

  ~ProviderHydrophoneNode();

  /// Taking care of the spinning of the ROS thread.
  /// Each iteration of the loop, this will take the objects in the object
  /// registery, empty it and publish the objects.
  void Spin();

private:
    ros::NodeHandlePtr nh_;

    HydroUsbDriver driver;

};

}  // namespace provider_hydrophone

#endif  // PROVIDER_HYDROPHONE_PROVIDER_HYDROPHONE_NODE_H_
