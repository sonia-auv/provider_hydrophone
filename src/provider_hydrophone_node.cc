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
    stopAcquireData();
    setGain(configuration_.getGain());
    pingPublisher_ = nh_->advertise<sonia_common::PingMsg>("/provider_hydrophone/ping", 100);
    debugPublisher_ = nh->advertise<std_msgs::UInt32MultiArray>("/provider_hydrophone/debug_ping", 100);

    readerThread = std::thread(std::bind(&ProviderHydrophoneNode::readThread, this));
    h1ParseThread = std::thread(std::bind(&ProviderHydrophoneNode::h1RegisterThread, this));
    h6ParseThread = std::thread(std::bind(&ProviderHydrophoneNode::h6RegisterThread, this));

    settingsHydro_ = nh_->advertiseService("/provider_hydrophone/change_settings", &ProviderHydrophoneNode::changeSettings, this);
    modeHydro_ = nh_->advertiseService("/provider_hydrophone/change_mode", &ProviderHydrophoneNode::changeMode, this);
  }

  //------------------------------------------------------------------------------
  //
  ProviderHydrophoneNode::~ProviderHydrophoneNode() {
      serialConnection_.~Serial();
      readerThread.~thread();
      h1ParseThread.~thread();
      h6ParseThread.~thread();
  }

  //==============================================================================
  // M E T H O D   S E C T I O N
  //------------------------------------------------------------------------------
  //
  void ProviderHydrophoneNode::Spin() 
  {
    ros::Rate r(10);  // 10 hz

    startAcquireData(H1_REGISTER);

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
      if(isAcquiring())
      {
        do
        {
          serialConnection_.readOnce(buffer, 0);
        } while (buffer[0] != 'H');

        uint16_t i;
        for(i =1; buffer[i-1] != '\n' && i < MAX_BUFFER_SIZE; ++i)
        {
          serialConnection_.readOnce(buffer, i);
        }
        
        if(i >= MAX_BUFFER_SIZE)
        {
          continue;
        }

        buffer[i] = '\0';

        if(!strncmp(&buffer[0], H1_REGISTER, 2))
        {
          std::unique_lock<std::mutex> mlock(h1_mutex);
          h1_string = std::string(buffer);
          h1_cond.notify_one();
        }
        if(!strncmp(&buffer[0], H6_REGISTER, 2))
        {
          std::unique_lock<std::mutex> mlock(h6_mutex);
          h6_string = std::string(buffer);
          h6_cond.notify_one();
        }
      }
      else r.sleep();
    }
  }

  void ProviderHydrophoneNode::h1RegisterThread()
  {
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
          std::getline(ss, y, '*');

          float_t x_t = fixedToFloat(stoi(x));
          float_t y_t = fixedToFloat(stoi(y));

          ping_msg.heading = calculateHeading(x_t, y_t);
          ping_msg.elevation = calculateElevation(x_t, y_t, stoi(frequency));

          pingPublisher_.publish(ping_msg);
        }
      }
      catch(...)
      {
        ROS_WARN_STREAM("Received bad Packet");
      }
    }
  }

  void ProviderHydrophoneNode::h6RegisterThread()
  {
    while(!ros::isShuttingDown())
    {
      std_msgs::UInt32MultiArray msg;
      std::string tmp;

      std::unique_lock<std::mutex> mlock(h6_mutex);
      h6_cond.wait(mlock);

      try
      {
        if(!h6_string.empty() && ConfirmChecksum(h6_string))
        {
          std::stringstream ss(h6_string);
          std::getline(ss, tmp, ',');
          
          msg.data.clear();

          for(uint8_t i = 0; i < 4; ++i)
          {
            std::getline(ss, tmp, ',');
            msg.data.push_back(std::stoi(tmp));
          }
          debugPublisher_.publish(msg);
        }
      }
      catch(...)
      {
        ROS_WARN_STREAM("Received bad Packet");
      }
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
    if (isAcquiring())
    {
      ROS_INFO_STREAM("We were acquiring data. Acquisition will stop for a moment");
      stopAcquireData();
    }
    
    switch (req.setting)
    {
    case req.SET_GAIN:
      result = setGain(req.data);
      res.applied_data = req.data;
      res.applied_setting = req.setting;
      break;
    
    case req.SET_SNR_THRESHOLD:
      result = setSNRThreshold(req.data);
      res.applied_data = req.data;
      res.applied_setting = req.setting;
      break;

    case req.SET_SIGNAL_THRESHOLD:
      result = setSignalThreshold(req.data);
      res.applied_data = req.data;
      res.applied_setting = req.setting;
      break;
    
    default:
      break;
    }
    
    // If we were acquiring data before, restart
    if (!isAcquiring())
    {
      ROS_INFO_STREAM("Settings requested has been setted. Acquisition restart");
      startAcquireData(active_register);
    }
    return result;
  }

  bool ProviderHydrophoneNode::changeMode(sonia_common::SetHydroMode::Request &req, sonia_common::SetHydroMode::Response &res)
  {
    bool result = false;

    // If is acquiring data, stop
    if (isAcquiring())
    {
      ROS_INFO_STREAM("We were acquiring data. Acquisition will stop for a moment");
      stopAcquireData();
    }

    if(req.register_selected == H1_REGISTER)
    {
      startAcquireData(H1_REGISTER);
      ROS_INFO_STREAM("Acquisition of H1 Register");
      result = true;
    }
    else if(req.register_selected == H6_REGISTER)
    {
      startAcquireData(H6_REGISTER);
      ROS_INFO_STREAM("Acquisition of H6 Register");
      result = true;
    }
    else
    {
      result = false;
    }
    res.action_accomplished = true;
    return result;
  }

  bool ProviderHydrophoneNode::ConfirmChecksum(std::string data)
  {
    try
    {
      std::string checksumData = data.substr(0, data.find("*", 0) + 1); // Include de * of the checksum
      uint8_t calculatedChecksum = CalculateChecksum(checksumData);
      uint8_t orignalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
      return orignalChecksum == calculatedChecksum;
    }
    catch(...)
    {
      ROS_INFO_STREAM("Hydro : Bad packet checksum");
      return false;
    }
  }

  uint8_t ProviderHydrophoneNode::CalculateChecksum(std::string data)
  {
    uint8_t check = 0;

    for(uint8_t i = 0; i < data.size(); ++i)
    {
      check ^= data[i];
    }
    return check;
  }

  bool ProviderHydrophoneNode::isAcquiring() 
  {
    return acquiringNormalData_ || acquiringDebugData_;
  }

  void ProviderHydrophoneNode::startAcquireData(std::string hydro_register)
  {
    ROS_DEBUG("Start acquiring data");

    if(isAcquiring()) return;

    if(hydro_register == H1_REGISTER)
    {
      serialConnection_.transmit(SET_NORMAL_MODE_COMMAND);
      acquiringNormalData_ = true;
    }
    else if(hydro_register == H6_REGISTER)
    {
      serialConnection_.transmit(SET_RAW_DATA_MODE_COMMAND);
      acquiringDebugData_ = true;
    }
    else
    {
      ROS_WARN_STREAM("Error with the requested register");
      return;
    }
    
    active_register = hydro_register;

    // Give time to board to execute command
    ros::Duration(0.5).sleep();

    serialConnection_.flush();
  }

  void ProviderHydrophoneNode::stopAcquireData() {

    ROS_DEBUG("Stop acquiring data");

    if (!isAcquiring()) return;

    serialConnection_.transmit(EXIT_COMMAND);

    acquiringNormalData_ = false;
    acquiringDebugData_ = false;
    // Give time to board to execute command
    ros::Duration(0.5).sleep();

    serialConnection_.flush();
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

    ROS_INFO("Gain has been setted : %d", gain);

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
    if(data & SIGNED_MASK)
    {
      return -1.0*((float_t) ((~data) & FIXED_POINT_DATA_MASK) / (float_t)(1 << FIXED_POINT_FRACTIONAL_BITS));
    }
    else
    {
      return ((float_t) (data & FIXED_POINT_DATA_MASK) / (float_t)(1 << FIXED_POINT_FRACTIONAL_BITS));
    }
  }
  
  float_t ProviderHydrophoneNode::calculateElevation(float_t x, float_t y, float_t frequency)
  {
      x = pow(x, 2.0);
      y = pow(y, 2.0);

      float_t frequency_2pi = 0.0, t1 = 0.0, t2 = 0.0;

      frequency_2pi = frequency * 2 * M_PI;

      t1 = frequency_2pi / constant;
      t2 = pow(t1, 2.0);

      t2 = t2 - y - x;
      t2 = sqrt(t2);

      return acos(t2/t1);
  }

  float_t ProviderHydrophoneNode::calculateHeading(float_t x, float_t y)
  {
      return atan2(y,x);
  }  

}  // namespace provider_hydrophone
