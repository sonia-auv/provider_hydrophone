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

#define POW2(x) x*x

#define MAX_BUFFER_SIZE 4096
#define MAX_NUMBER_ARGS 5

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
    createDOACommand(configuration_.getsnrThreshold(), configuration_.getLowSignalThreshold(), configuration_.getHighSignalThreshold());
    createAGCCommand(configuration_.getAGCactivation(), configuration_.getAGCThreshold(), configuration_.getAGCMaxThreshold());

    // Subscribers
    pingPublisher_ = nh_->advertise<sonia_common::PingMsg>("/provider_hydrophone/ping", 100);
    debugPublisher_ = nh_->advertise<std_msgs::UInt32MultiArray>("/provider_hydrophone/debug_ping", 100);
    settingsSubcriber_ = nh_->subscribe("/provider_hydrophone/change_settings", 10, &ProviderHydrophoneNode::changeSettings, this);

    // Threads
    readerThread = std::thread(std::bind(&ProviderHydrophoneNode::readThread, this));
    h1ParseThread = std::thread(std::bind(&ProviderHydrophoneNode::h1RegisterThread, this));
    h6ParseThread = std::thread(std::bind(&ProviderHydrophoneNode::h6RegisterThread, this));

    // Dynamic Reconfigure Server
    server.setCallback(boost::bind(&ProviderHydrophoneNode::CallBackDynamicReconfigure, this, _1, _2));
  }

  //------------------------------------------------------------------------------
  //
  ProviderHydrophoneNode::~ProviderHydrophoneNode() {
      serialConnection_.~Serial();
      stopReadThread = true;
      stoph1RegisterThread = true;
      stoph6RegisterThread = true;
  }

  //==============================================================================
  // M E T H O D   S E C T I O N
  //------------------------------------------------------------------------------
  //
  void ProviderHydrophoneNode::Spin() 
  {
    ros::Rate r(10);  // 10 hz

    changeMode(normalop);

    while (ros::ok()) 
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  void ProviderHydrophoneNode::CallBackDynamicReconfigure(provider_hydrophone::HydroConfig &config, uint32_t level)
  {
    operation_mode current_mode =  operation_mode_;
    bool mode_changed = false;

    // To block multiple change at the same time
    dynamic_reconfigure_mutex.lock();
    stopAcquireData();

    // Feature to add in the dynamic reconfigure
    if(current_mode != config.Mode)
    {
      ROS_INFO_STREAM("New mode : " << std::to_string(config.Mode) << " Currrent mode : " << std::to_string(operation_mode_));
      changeMode((operation_mode)config.Mode);
      mode_changed = true;
    }
    else if(gain_ != config.Gain)
    {
      ROS_INFO_STREAM("New gain : " << std::to_string(config.Gain) << " Old gain : " << std::to_string(gain_));
      setGain((uint8_t)config.Gain);
    }
    else if(snrThreshold_ != config.SNR || signalLowThreshold_ != config.Low_Threshold || signalHighThreshold_ != config.High_Threshold)
    {
      ROS_INFO_STREAM("New SNR : " << std::to_string(config.SNR) << " Old SNR : " << std::to_string(snrThreshold_));
      ROS_INFO_STREAM("New High Threshold : " << std::to_string(config.High_Threshold) << " Old High Threshold : " << std::to_string(signalHighThreshold_));
      ROS_INFO_STREAM("New Low Threshold : " << std::to_string(config.Low_Threshold) << " Old Low Threshold : " << std::to_string(signalLowThreshold_));
      createDOACommand((uint8_t)config.SNR, (uint16_t)config.Low_Threshold, (uint16_t)config.High_Threshold);
    }
    else if(agcToggleMode_ != config.Active_AGC || agcThreshold_ != config.Signal_Threshold || agcMaxThreshold_ != config.Limit_Signal_Threshold)
    {
      ROS_INFO_STREAM("AGC toggle : " << std::to_string(config.Active_AGC) << " Previous State : " << std::to_string(agcToggleMode_));
      ROS_INFO_STREAM("New Signal Threshold : " << std::to_string(config.Signal_Threshold) << " Old Signal Threshold : " << std::to_string(agcThreshold_));
      ROS_INFO_STREAM("New Limit Threshold : " << std::to_string(config.Limit_Signal_Threshold) << " Old Limit Threshold : " << std::to_string(agcMaxThreshold_));
      createAGCCommand((uint8_t)config.Active_AGC, (uint16_t)config.Signal_Threshold, (uint16_t)config.Limit_Signal_Threshold);
    }

    if(!(mode_changed)) changeMode(current_mode);
    dynamic_reconfigure_mutex.unlock();

    ROS_WARN_STREAM("REMINDER : Change the rosparam with the new settings!!!");
  }

  void ProviderHydrophoneNode::readThread()
  {
    ros::Rate r(5);
    char buffer[MAX_BUFFER_SIZE];
    ROS_INFO_STREAM("Read Thread started");

    while(!stopReadThread)
    {
      do
      {
        serialConnection_.readOnce(buffer, 0);
      } while (buffer[0] != 'H' && buffer[0] != '>');

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
      if(!strncmp(&buffer[0], ">", 1))
      {
        buffer[i-1] = '\0';
        ROS_INFO_STREAM("Board response " << buffer);
      }
    }
  }

  void ProviderHydrophoneNode::h1RegisterThread()
  {
    sonia_common::PingMsg ping_msg;
    std::string phaseRef = "";
    std::string phase1 = "";
    std::string phase2 = "";
    std::string phase3 = "";
    std::string index = "";
    std::string debug = "";

    while(!stoph1RegisterThread)
    {
      std::unique_lock<std::mutex> mlock(h1_mutex);
      h1_cond.wait(mlock);

      try
      {
        if(!h1_string.empty())
        {
          std::stringstream ss(h1_string);

          ping_msg.header.stamp = ros::Time::now();

          std::getline(ss, phase3, ',');
          std::getline(ss, phase3, ',');
          std::getline(ss, phaseRef, ',');
          std::getline(ss, phase1, ',');
          std::getline(ss, phase2, ',');
          std::getline(ss, debug, ',');
          std::getline(ss, index, '\n');

          float_t phase3_ = fixedToFloat(std::stoi(phase3));
          float_t phaseRef_ = fixedToFloat(std::stoi(phaseRef));
          float_t phase1_ = fixedToFloat(std::stoi(phase1));
          float_t phase2_ = fixedToFloat(std::stoi(phase2));

          ping_msg.phaseRef = phaseRef_;
          ping_msg.phase1 = phase1_;
          ping_msg.phase2 = phase2_;
          ping_msg.phase3 = phase3_;
          ping_msg.debug = stoi(debug);
          ping_msg.frequency = stoi(index) * 1000;
          
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
    std_msgs::UInt32MultiArray msg;
    std::string tmp;

    while(!stoph6RegisterThread)
    {  
      std::unique_lock<std::mutex> mlock(h6_mutex);
      h6_cond.wait(mlock);
      
      try
      {
        if(!h6_string.empty())
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

  void ProviderHydrophoneNode::changeSettings(const sonia_common::HydroSettings::ConstPtr& msg)
  {
    bool result = false;
    
    // To block multiple change at the same time
    dynamic_reconfigure_mutex.lock();

    if(msg->cmd == "op")
    {
      result = changeMode((operation_mode) msg->argv[0]);
    }
    else if(msg->cmd == "pga")
    {
      result = setGain((uint8_t) msg->argv[0]);
    }
    else if(msg->cmd == "doa")
    {
      result = createDOACommand(msg->argv[0], msg->argv[1], msg->argv[2]);
    }
    else if(msg->cmd == "agc")
    {
      result = createAGCCommand(msg->argv[0], msg->argv[1], msg->argv[2]);
    }
    if(result)
    {
      ROS_DEBUG_STREAM("Setting has been changed");
    }
    else
    {
      ROS_ERROR_STREAM("Error with the settings change");
    }

    dynamic_reconfigure_mutex.unlock();
  }

  bool ProviderHydrophoneNode::ConfirmChecksum(std::string data)
  {
    try
    {
      std::string checksumData = data.substr(0, data.find("*", 0)); // Include de * of the checksum + 1 TODO : devrait etre good
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

  void ProviderHydrophoneNode::sendCmd(std::string cmd, std::vector<uint16_t> *argv)
  {
    std::string send_string = cmd;
    size_t len = argv->size();

    if(cmd == OPERATION_CMD && argv->at(0) == (uint16_t)idle)
    {
      serialConnection_.transmit(EXIT_COMMAND);
    }
    else
    {
      for(uint8_t i = 0; i < len; ++i)
      {
        send_string += " " + std::to_string(argv->at(i));
      }
      send_string += ENTER_COMMAND;
      ROS_INFO_STREAM("CMD send : " << send_string);
      serialConnection_.transmit(send_string);
    }
    ros::Duration(1).sleep(); // Give time for the board receive and interpret data
  }

  bool ProviderHydrophoneNode::isAcquiring() 
  {
    return operation_mode_ != idle;
  }

  void ProviderHydrophoneNode::stopAcquireData() 
  {
    ROS_DEBUG_STREAM("Stop acquiring data");

    if (!isAcquiring()) return;

    changeMode(idle);
  }

  bool ProviderHydrophoneNode::changeMode(operation_mode mode)
  {
    std::vector<uint16_t> argv;
    argv.reserve(MAX_NUMBER_ARGS);

    ROS_DEBUG_STREAM("Changing Acquisition Mode");

    if(mode < idle || mode > raw_data)
    {
      ROS_ERROR_STREAM("Error with the requested operation mode"); 
      return false;
    }

    if(mode == operation_mode_)
    {
      ROS_INFO_STREAM("Already in the right mode");
      return true;
    }

    operation_mode_ = mode;
    argv.push_back((uint16_t)mode);
    sendCmd(OPERATION_CMD, &argv);

    ROS_INFO_STREAM("Mode has been changed : " << std::to_string((uint8_t)operation_mode_));
    return true;
  }

  bool ProviderHydrophoneNode::setGain(uint8_t gain) 
  {
    std::vector<uint16_t> argv;
    argv.reserve(MAX_NUMBER_ARGS);

    ROS_DEBUG("Setting a new gain");

    if (gain > 7 || gain < 0)
    {
      ROS_ERROR_STREAM("Error with the requested gain");
      return false;
    }
    gain_ = gain;
    argv.push_back((uint16_t)gain);
    sendCmd(PGA_CMD, &argv);

    ROS_INFO_STREAM("Gain has been setted : " << std::to_string(gain_));
    return true;
  }

  bool ProviderHydrophoneNode::createDOACommand(uint8_t snr, uint16_t lowth, uint16_t highth)
  {
    bool result = setSNRThreshold(snr) && setSignalLowThreshold(lowth) && 
                  setSignalHighThreshold(highth);
    createDOACommand();
    return result;
  }

  void ProviderHydrophoneNode::createDOACommand()
  {
    std::vector<uint16_t> argv;
    argv.reserve(MAX_NUMBER_ARGS);

    argv.push_back(snrThreshold_);
    argv.push_back(signalLowThreshold_);
    argv.push_back(signalHighThreshold_);
    
    sendCmd(DOA_CMD, &argv);
  }

  bool ProviderHydrophoneNode::setSNRThreshold(uint8_t threshold)
  {
    ROS_DEBUG_STREAM("Setting new SNR Threshold");
    
    if(threshold < 0 || threshold > 255)
    {
      ROS_ERROR_STREAM("Error with the requested SNR threshold");
      return false;
    }
    snrThreshold_ = threshold;
    ROS_INFO_STREAM("SNR Threshold has been setted : " << std::to_string(threshold));
    return true;
  }

  bool ProviderHydrophoneNode::setSignalLowThreshold(uint16_t threshold)
  {
    ROS_DEBUG_STREAM("Setting new Signal Low Threshold");

    if(threshold < 0 || threshold > 65535)
    {
      ROS_ERROR_STREAM("Error with the requested Signal Low Threshold");
      return false;
    }
    signalLowThreshold_ = threshold;
    ROS_INFO_STREAM("Signal Low Threshold has been setted : " << std::to_string(threshold));
    return true;
  }

  bool ProviderHydrophoneNode::setSignalHighThreshold(uint16_t threshold)
  {
    ROS_DEBUG_STREAM("Setting new Signal High Threshold");

    if(threshold < 0 || threshold > 65535)
    {
      ROS_ERROR_STREAM("Error with the requested Signal High Threshold");
      return false;
    }
    signalHighThreshold_ = threshold;
    ROS_INFO_STREAM("Signal High Threshold has been setted : " << std::to_string(threshold));
    return true;
  }

  bool ProviderHydrophoneNode::createAGCCommand(uint8_t toggle, uint16_t signalth, uint16_t limitth)
  {
    bool result = setAGCToggle(toggle) && setSignalThreshold(signalth) &&
                  setLimitSignalThreshold(limitth);
    createAGCCommand();
    return result;
  }

  void ProviderHydrophoneNode::createAGCCommand()
  {
    std::vector<uint16_t> argv;
    argv.reserve(MAX_NUMBER_ARGS);

    argv.push_back(agcToggleMode_);
    argv.push_back(agcThreshold_);
    argv.push_back(agcMaxThreshold_);
    
    sendCmd(AGC_CMD, &argv);
  }

  bool ProviderHydrophoneNode::setAGCToggle(uint8_t toggle)
  {
    ROS_DEBUG_STREAM("Changing the mode for the AGC");

    if(toggle != 0 && toggle != 1)
    {
      ROS_ERROR_STREAM("Error with the request to toggle the AGC");
      return false;
    }
    agcToggleMode_ = toggle;
    ROS_INFO_STREAM("The AGC has been toggle with the value " << std::to_string(toggle));
    return true;
  }

  bool ProviderHydrophoneNode::setSignalThreshold(uint16_t threshold)
  {
    ROS_DEBUG_STREAM("Setting new Signal Threshold (AGC)");

    if(threshold < 0 || threshold > 65535)
    {
      ROS_ERROR_STREAM("Error with the requested Signal Threshold (AGC)");
      return false;
    }
    agcThreshold_ = threshold;
    ROS_INFO_STREAM("Signal Threshold (AGC) has been setted : " << std::to_string(threshold));
    return true;
  }

  bool ProviderHydrophoneNode::setLimitSignalThreshold(uint16_t threshold)
  {
    ROS_DEBUG_STREAM("Setting new Limit Signal Threshold");

    if(threshold < 0 || threshold > 65535)
    {
      ROS_ERROR_STREAM("Error with the requested Signal High Threshold");
      return false;
    }
    agcMaxThreshold_ = threshold;
    ROS_INFO_STREAM("Limit Signal Threshold has been setted : " << std::to_string(threshold));
    return true;
  }

  float_t ProviderHydrophoneNode::fixedToFloat(uint32_t data)
  {   
    float_t value = 0.0, sign = 1.0;
    
    if(data & SIGNED_MASK)
    {
      sign = -1.0;
      value = (float_t)((~data) & FIXED_POINT_DATA_MASK);
    }
    else
    {
      value = (float_t)(data & FIXED_POINT_DATA_MASK);
    }
    value /= pow(2.0, FIXED_POINT_FRACTIONAL_BITS); 
    return value * sign;
  }

}  // namespace provider_hydrophone
