/**
 * \file	provider_hydrophone_node.cc
 * \author	Francis Alonzo
 * \date	2021/10/20
 *
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
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

#include "provider_hydrophone_node.h"

#define MAX_BUFFER_SIZE 4096

namespace provider_hydrophone {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
  ProviderHydrophoneNode::ProviderHydrophoneNode(const ros::NodeHandlePtr &nh)
    : nh_(nh),
      configuration_(nh),
      serialConnection_(configuration_.getTtyPort())
  {
    pingPublisher_ = nh_->advertise<sonia_common::PingMsg>("/provider_hydrophone/ping", 100);

    readerThread = std::thread(std::bind(&ProviderHydrophoneNode::readThread, this));
    h1ParseThread = std::thread(std::bind(&ProviderHydrophoneNode::h1RegisterThread, this));

    settingsHydro_ = nh_->advertiseService("/provider_hydrophone/change_settings", &ProviderHydrophoneNode::changeSettings, this);
  }

  //------------------------------------------------------------------------------
  //
  ProviderHydrophoneNode::~ProviderHydrophoneNode() {
      serialConnection_.~Serial();
      readerThread.~thread();
      h1ParseThread.~thread();
  }

  //==============================================================================
  // M E T H O D   S E C T I O N
  //------------------------------------------------------------------------------
  //
  void ProviderHydrophoneNode::Spin() 
  {
    ros::Rate r(10);  // 10 hz

    startAcquireNormalMode();

    while (ros::ok()) 
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  void ProviderHydrophoneNode::readThread()
  {
    ros::Rate r(5);
    char buffer[MAX_BUFFER_SIZE];
    ROS_INFO_STREAM("Read Thread started");

    while(!ros::isShuttingDown())
    {
      if(isAcquiringData())
      {
        do
        {
          serialConnection_.readOnce(buffer, 0);
        } while (buffer[0] != 'H');

        uint16_t i;

        for(i = 1; buffer[i-1] != '\n' && i < MAX_BUFFER_SIZE; ++i)
        {
          serialConnection_.readOnce(buffer, i);
        }
        
        if(i >= MAX_BUFFER_SIZE)
        {
          continue;
        }

        if(!strncmp(&buffer[0], H1_REGISTER, 2))
        {
          std::unique_lock<std::mutex> mlock(h1_mutex);
          h1_string = std::string(buffer);
          h1_cond.notify_one();
        }
      }
      r.sleep();
    }
  }

  void ProviderHydrophoneNode::h1RegisterThread()
  {
    ros::Rate r(2); // 2 Hz
    Ping ping;

    while(!ros::isShuttingDown())
    {
      sonia_common::PingMsg ping_msg;
      std::string x = "";
      std::string y = "";
      std::string frequency = "";
      std::string debug = "";

      std::unique_lock<std::mutex> mlock(h1_mutex);
      h1_cond.wait(mlock);

      try
      {
        if(!h1_string.empty())
        {
          std::stringstream ss(h1_string);

          ping_msg.header.stamp = ros::Time::now();

          std::getline(ss, debug, ',');
          std::getline(ss, debug, ',');

          ping_msg.debug = stoi(debug);

          std::getline(ss, frequency, ',');

          ping_msg.frequency = stoi(frequency);

          std::getline(ss, x, ',');
          std::getline(ss, y, '\n');

          uint32_t x_int = fixedToFloat(stoi(x));
          uint32_t y_int = fixedToFloat(stoi(y));

          ping_msg.heading = ping.calculateHeading(x_int, y_int);
          ping_msg.elevation = ping.calculateElevation(x_int, y_int, stoi(frequency));

          pingPublisher_.publish(ping_msg);
        }
      }
      catch(...)
      {
        ROS_WARN_STREAM("Received bad Packet");
      }
      r.sleep();
    }
  }

  bool ProviderHydrophoneNode::changeSettings(sonia_common::SetHydroSettings::Request &req, sonia_common::SetHydroSettings::Response &res)
  {
    bool result = false;

    if(req.setting < req.SET_GAIN || req.setting > req.SET_SIGNAL_THRESHOLD)
    {
      return false;
    }

    // If is acquiring data, stop
    if (isAcquiringData())
    {
      ROS_INFO_STREAM("We were acquiring data. Acquisition will stop for a moment");
      stopAcquireData();
    }
    
    switch (req.setting)
    {
    case req.SET_GAIN:
      result = setGain(req.setting);
      // Add data to the response
      break;
    
    case req.SET_SNR_THRESHOLD:
      result = setSNRThreshold(req.setting);
      // Add data to response
      break;

    case req.SET_SIGNAL_THRESHOLD:
      result = setSignalThreshold(req.setting);
      // Add data to response
      break;
    
    default:
      break;
    }
    
    // If we were acquiring data before, restart
    if (isAcquiringData())
    {
      ROS_INFO_STREAM("Gain has been setted. Acquisition restart");
      startAcquireNormalMode();
    }

    return result;
  }

  bool ProviderHydrophoneNode::isAcquiringData() 
  {
    return acquiringNormalData_;
  }

  void ProviderHydrophoneNode::startAcquireNormalMode() 
  {
    ROS_DEBUG("Start acquiring data");

    if (isAcquiringData()) return;

    serialConnection_.transmit(SET_NORMAL_MODE_COMMAND);

    // Give time to board to execute command
    ros::Duration(0.5).sleep();

    serialConnection_.flush();

    acquiringNormalData_ = true;
  }

  void ProviderHydrophoneNode::stopAcquireData() {

    ROS_DEBUG("Stop acquiring data");

    if (!isAcquiringData()) return;

    serialConnection_.transmit(EXIT_COMMAND);

    // Give time to board to execute command
    ros::Duration(0.5).sleep();

    serialConnection_.flush();
    
    acquiringNormalData_ = false;
  }

  bool ProviderHydrophoneNode::setGain(uint8_t gain) {

    ROS_INFO_STREAM("Setting a new gain");

    if (gain > 7 || gain < 0)
    {
      return false;
    }

    serialConnection_.transmit(SET_GAIN_COMMAND);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    serialConnection_.transmit(std::to_string(gain) + ENTER_COMMAND_CHAR);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    ROS_INFO_STREAM("Gain has been setted : " << gain);

    return true;
  }

  bool ProviderHydrophoneNode::setSNRThreshold(uint8_t threshold)
  {
    ROS_INFO_STREAM("Setting new SNR Threshold");
    
    if(threshold < 0 || threshold > sizeof(uint8_t))
    {
      return false;
    }

    serialConnection_.transmit(SET_SNR_THRESHOLD);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    serialConnection_.transmit(std::to_string(threshold) + ENTER_COMMAND_CHAR);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    ROS_INFO_STREAM("SNR Threshold has been setted : " << threshold);

    return true;
  }

  bool ProviderHydrophoneNode::setSignalThreshold(uint32_t threshold)
  {
    ROS_INFO_STREAM("Setting new Signal Threshold");

    if(threshold < 0 || threshold > sizeof(threshold))
    {
      return false;
    }

    serialConnection_.transmit(SET_SIGNAL_THRESHOLD);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    serialConnection_.transmit(std::to_string(threshold) + ENTER_COMMAND_CHAR);

    // Give time to board to execute command
    ros::Duration(0.1).sleep();

    ROS_INFO_STREAM("Signal Threshold has been setted : " << threshold);

    return true;
  }

  float_t ProviderHydrophoneNode::fixedToFloat(uint32_t data)
  {
    return ((float_t) data / (float_t)(1 << FIXED_POINT_FRACTIONAL_BITS));
  }

}  // namespace provider_hydrophone
