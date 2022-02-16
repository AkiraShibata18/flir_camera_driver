/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution
Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.
Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
   @file nodelet.cpp
   @author Chad Rockey
   @date July 13, 2011
   @brief ROS nodelet for the Point Grey Chameleon Camera

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

/**
   @file nodelet.cpp
   @author Teyvonia Thomas
   @date August 28, 2017
   @brief ROS nodelet for the Point Grey Chameleon Camera - Updated to use Spinnaker driver insteady of Flycapture
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "spinnaker_camera_driver/SpinnakerCamera.h"  // The actual standalone library for the Spinnakers
#include "spinnaker_camera_driver/diagnostics.h"

#include <image_transport/image_transport.h>          // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h>  // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h>                   // ROS message header for CameraInfo

#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h>  // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h>  // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp>  // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h>  // Needed for the dynamic_reconfigure gui service to run

#include <fstream>
#include <memory>
#include <string>
#include <utility>

namespace spinnaker_camera_driver
{
class SpinnakerStereoCameraNodelet : public nodelet::Nodelet
{
public:
  SpinnakerStereoCameraNodelet()
  {
  }

  ~SpinnakerStereoCameraNodelet()
  {
    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    if (diagThread_)
    {
      diagThread_->interrupt();
      diagThread_->join();
    }

    if (pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();

      try
      {
        NODELET_DEBUG_ONCE("Stopping camera capture.");
        spinnaker_left_.stop();
        spinnaker_right_.stop();
        NODELET_DEBUG_ONCE("Disconnecting from camera.");
        spinnaker_left_.disconnect();
        spinnaker_right_.disconnect();
      }
      catch (const std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object
  * to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver
  * is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */

  void leftParamCallback(spinnaker_camera_driver::SpinnakerConfig& config, uint32_t level)
  {
    config_left_ = config;

    try
    {
      NODELET_DEBUG_ONCE("Dynamic reconfigure callback with level: %u", level);
      spinnaker_left_.setNewConfiguration(config, level);

      publish_diagnostics_ = config.publish_diagnostics;
      diag_pub_rate_ = config.diagnostic_publish_rate;

      // Store needed parameters for the metadata message
      left_camera_settings.gain_ = config.gain;
      left_camera_settings.wb_blue_ = config.white_balance_blue_ratio;
      left_camera_settings.wb_red_ = config.white_balance_red_ratio;

      // No separate param in CameraInfo for binning/decimation
      left_camera_settings.binning_x_ = config.image_format_x_binning * config.image_format_x_decimation;
      left_camera_settings.binning_y_ = config.image_format_y_binning * config.image_format_y_decimation;

      // Store CameraInfo RegionOfInterest information
      // TODO(mhosmar): Not compliant with CameraInfo message: "A particular ROI always denotes the
      //                same window of pixels on the camera sensor, regardless of binning settings."
      //                These values are in the post binned frame.

      if ((config.image_format_roi_width + config.image_format_roi_height) > 0 &&
          (config.image_format_roi_width < spinnaker_left_.getWidthMax() ||
           config.image_format_roi_height < spinnaker_left_.getHeightMax()))
      {
        left_camera_settings.roi_x_offset_ = config.image_format_x_offset;
        left_camera_settings.roi_y_offset_ = config.image_format_y_offset;
        left_camera_settings.roi_width_ = config.image_format_roi_width;
        left_camera_settings.roi_height_ = config.image_format_roi_height;
        left_camera_settings.do_rectify_ = true;  // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        left_camera_settings.roi_x_offset_ = 0;
        left_camera_settings.roi_y_offset_ = 0;
        left_camera_settings.roi_height_ = 0;
        left_camera_settings.roi_width_ = 0;
        left_camera_settings.do_rectify_ = false;  // Set to false if the whole image is captured.
      }
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  void rightParamCallback(spinnaker_camera_driver::SpinnakerConfig& config, uint32_t level)
  {
    config_right_ = config;

    try
    {
      NODELET_DEBUG_ONCE("Dynamic reconfigure callback with level: %u", level);
      spinnaker_right_.setNewConfiguration(config, level);

      publish_diagnostics_ = config.publish_diagnostics;
      diag_pub_rate_ = config.diagnostic_publish_rate;

      // Store needed parameters for the metadata message
      right_camera_settings.gain_ = config.gain;
      right_camera_settings.wb_blue_ = config.white_balance_blue_ratio;
      right_camera_settings.wb_red_ = config.white_balance_red_ratio;

      // No separate param in CameraInfo for binning/decimation
      right_camera_settings.binning_x_ = config.image_format_x_binning * config.image_format_x_decimation;
      right_camera_settings.binning_y_ = config.image_format_y_binning * config.image_format_y_decimation;

      // Store CameraInfo RegionOfInterest information
      // TODO(mhosmar): Not compliant with CameraInfo message: "A particular ROI always denotes the
      //                same window of pixels on the camera sensor, regardless of binning settings."
      //                These values are in the post binned frame.

      if ((config.image_format_roi_width + config.image_format_roi_height) > 0 &&
          (config.image_format_roi_width < spinnaker_right_.getWidthMax() ||
           config.image_format_roi_height < spinnaker_right_.getHeightMax()))
      {
        right_camera_settings.roi_x_offset_ = config.image_format_x_offset;
        right_camera_settings.roi_y_offset_ = config.image_format_y_offset;
        right_camera_settings.roi_width_ = config.image_format_roi_width;
        right_camera_settings.roi_height_ = config.image_format_roi_height;
        right_camera_settings.do_rectify_ = true;  // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        right_camera_settings.roi_x_offset_ = 0;
        right_camera_settings.roi_y_offset_ = 0;
        right_camera_settings.roi_height_ = 0;
        right_camera_settings.roi_width_ = 0;
        right_camera_settings.do_rectify_ = false;  // Set to false if the whole image is captured.
      }
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  void diagCb()
  {
    if (publish_diagnostics_ && !diagThread_)  // We need to connect
    {
      // Start the thread to loop through and publish messages
      diagThread_.reset(
          new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::diagPoll, this)));
    }
  }

  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void leftConnectCb()
  {
    is_left_connected_ = true;
    NODELET_INFO_STREAM("leftConnectCb!! flag: " << is_left_connected_);
    if (!pubThread_ && is_right_connected_)  // We need to connect
    {
      NODELET_INFO_STREAM("leftConnectCb thread started");
      // Start the thread to loop through and publish messages
      pubThread_.reset(
          new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::devicePoll, this)));
    }

    // @tthomas - removing subscriber check and logic below as it's leading to mutex locks and crashes currently
    /*
    NODELET_DEBUG_ONCE("Connect callback!");
    std::lock_guard<std::mutex> scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing
    before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_left_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
    {
      if (pubThread_)
      {
        NODELET_DEBUG_ONCE("Disconnecting.");
        pubThread_->interrupt();
        scopedLock.unlock();
        pubThread_->join();
        scopedLock.lock();
        pubThread_.reset();
        sub_.shutdown();

        try
        {
          NODELET_DEBUG_ONCE("Stopping camera capture.");
          spinnaker_.stop();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }

        try
        {
          NODELET_DEBUG_ONCE("Disconnecting from camera.");
          spinnaker_.disconnect();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }
      }
    }
    else if(!pubThread_)     // We need to connect
    {
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::devicePoll,
    this)));
    }
    else
    {
      NODELET_DEBUG_ONCE("Do nothing in callback.");
    }
    */
  }

  void rightConnectCb()
  {
    is_right_connected_ = true;
    NODELET_INFO_STREAM("rightConnectCb!! flag: " << is_right_connected_);
    if (!pubThread_ && is_left_connected_)  // We need to connect
    {
      // Start the thread to loop through and publish messages
      NODELET_INFO_STREAM("rightConnectCb thread started");
      pubThread_.reset(
          new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::devicePoll, this)));
    }

    // @tthomas - removing subscriber check and logic below as it's leading to mutex locks and crashes currently
    /*
    NODELET_DEBUG_ONCE("Connect callback!");
    std::lock_guard<std::mutex> scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing
    before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_right_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
    {
      if (pubThread_)
      {
        NODELET_DEBUG_ONCE("Disconnecting.");
        pubThread_->interrupt();
        scopedLock.unlock();
        pubThread_->join();
        scopedLock.lock();
        pubThread_.reset();
        sub_.shutdown();

        try
        {
          NODELET_DEBUG_ONCE("Stopping camera capture.");
          spinnaker_.stop();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }

        try
        {
          NODELET_DEBUG_ONCE("Disconnecting from camera.");
          spinnaker_.disconnect();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }
      }
    }
    else if(!pubThread_)     // We need to connect
    {
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::devicePoll,
    this)));
    }
    else
    {
      NODELET_DEBUG_ONCE("Do nothing in callback.");
    }
    */
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking
  * functions here.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle& nh = getMTNodeHandle();
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();

    ros::NodeHandle left_pnh (pnh, "left");
    ros::NodeHandle right_pnh (pnh, "right");

    // Get a serial number through ros
    int serial_left = 0;
    int serial_right = 0;

    pnh.param<bool>("reset_on_start", reset_on_start_, false);
    pnh.param<bool>("shutdown_on_error", shutdown_on_error_, false);
    pnh.param<bool>("use_device_timestamp", use_device_timestamp_, false);

    // set left camera serial number
    XmlRpc::XmlRpcValue serial_xmlrpc_left;
    pnh.getParam("serial_left", serial_xmlrpc_left);
    if (serial_xmlrpc_left.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>("serial_left", serial_left, 0);
    }
    else if (serial_xmlrpc_left.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str_left;
      pnh.param<std::string>("serial_left", serial_str_left, "0");
      std::istringstream(serial_str_left) >> serial_left;
    }
    else
    {
      NODELET_DEBUG_ONCE("Serial XMLRPC type.");
      serial_left = 0;
    }

    std::string camera_serial_path_left;
    pnh.param<std::string>("camera_serial_path_left", camera_serial_path_left, "");
    NODELET_DEBUG_ONCE("Camera serial path left %s", camera_serial_path_left.c_str());
    // If serial has been provided directly as a param, ignore the path
    // to read in the serial from.
    while (serial_left == 0 && !camera_serial_path_left.empty())
    {
      serial_left = readSerialAsHexFromFile(camera_serial_path_left);
      if (serial_left == 0)
      {
        NODELET_WARN_ONCE("Waiting for camera serial path to become available");
        ros::Duration(1.0).sleep();  // Sleep for 1 second, wait for serial device path to become available
      }
    }

    NODELET_DEBUG_ONCE("Using left camera serial %d", serial_left);

    spinnaker_left_.setDesiredCamera((uint32_t)serial_left);

    // set right camera serial number
    XmlRpc::XmlRpcValue serial_xmlrpc_right;
    pnh.getParam("serial_right", serial_xmlrpc_right);
    if (serial_xmlrpc_right.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>("serial_right", serial_right, 0);
    }
    else if (serial_xmlrpc_right.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str_right;
      pnh.param<std::string>("serial_right", serial_str_right, "0");
      std::istringstream(serial_str_right) >> serial_right;
    }
    else
    {
      NODELET_DEBUG_ONCE("Serial XMLRPC type.");
      serial_right = 0;
    }

    std::string camera_serial_path_right;
    pnh.param<std::string>("camera_serial_path_right", camera_serial_path_right, "");
    NODELET_DEBUG_ONCE("Camera serial path right %s", camera_serial_path_right.c_str());
    // If serial has been provided directly as a param, ignore the path
    // to read in the serial from.
    while (serial_right == 0 && !camera_serial_path_right.empty())
    {
      serial_right = readSerialAsHexFromFile(camera_serial_path_right);
      if (serial_right == 0)
      {
        NODELET_WARN_ONCE("Waiting for camera serial path to become available");
        ros::Duration(1.0).sleep();  // Sleep for 1 second, wait for serial device path to become available
      }
    }

    NODELET_DEBUG_ONCE("Using camera serial %d", serial_right);

    spinnaker_right_.setDesiredCamera((uint32_t)serial_right);

    // Get GigE camera parameters:
    pnh.param<int>("packet_size", packet_size_, 1400);
    pnh.param<bool>("auto_packet_size", auto_packet_size_, true);
    pnh.param<int>("packet_delay", packet_delay_, 4000);

    // TODO(mhosmar):  Set GigE parameters:
    // spinnaker_.setGigEParameters(auto_packet_size_, packet_size_, packet_delay_);

    // Get the location of our camera config yaml
    std::string camera_info_url_left;
    std::string camera_info_url_right;
    pnh.param<std::string>("camera_info_url_left", camera_info_url_left, "");
    pnh.param<std::string>("camera_info_url_right", camera_info_url_right, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id_left", frame_id_left_, "left_camera");
    pnh.param<std::string>("frame_id_right", frame_id_right_, "right_camera");
    // Do not call the connectCb function until after we are done initializing.
    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_left_ = std::make_shared<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> >(left_pnh);
    srv_right_ = std::make_shared<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> >(right_pnh);
    dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>::CallbackType f_left =
        boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::leftParamCallback, this, _1, _2);
    dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>::CallbackType f_right =
        boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::rightParamCallback, this, _1, _2);

    srv_left_->setCallback(f_left);
    srv_right_->setCallback(f_right);

    // queue size of ros publisher
    int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name_left;
    cinfo_name_left << serial_left;
    cinfo_left_.reset(new camera_info_manager::CameraInfoManager(left_pnh, cinfo_name_left.str(), camera_info_url_left));
    std::stringstream cinfo_name_right;
    cinfo_name_right << serial_right;
    cinfo_right_.reset(new camera_info_manager::CameraInfoManager(right_pnh, cinfo_name_right.str(), camera_info_url_right));

    // initialize connectCb flag
    is_left_connected_ = false;
    is_right_connected_ = false;

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_left_.reset(new image_transport::ImageTransport(nh));
    it_right_.reset(new image_transport::ImageTransport(nh));
    image_transport::SubscriberStatusCallback left_cb = boost::bind(&SpinnakerStereoCameraNodelet::leftConnectCb, this);
    image_transport::SubscriberStatusCallback right_cb = boost::bind(&SpinnakerStereoCameraNodelet::rightConnectCb, this);
    it_pub_left_ = it_left_->advertiseCamera("left/image_raw", queue_size, left_cb, left_cb);
    it_pub_right_ = it_right_->advertiseCamera("right/image_raw", queue_size, right_cb, right_cb);

    // Set up diagnostics
    updater_.setHardwareID("spinnaker_camera " + cinfo_name_left.str());

    // Set up a diagnosed publisher
    double desired_freq;
    pnh.param<double>("desired_freq", desired_freq, 30.0);
    pnh.param<double>("min_freq", min_freq_, desired_freq);
    pnh.param<double>("max_freq", max_freq_, desired_freq);
    double freq_tolerance;  // Tolerance before stating error on publish frequency, fractional percent of desired
                            // frequencies.
    pnh.param<double>("freq_tolerance", freq_tolerance, 0.1);
    int window_size;  // Number of samples to consider in frequency
    pnh.param<int>("window_size", window_size, 100);
    double min_acceptable;  // The minimum publishing delay (in seconds) before warning.  Negative values mean future
                            // dated messages.
    pnh.param<double>("min_acceptable_delay", min_acceptable, 0.0);
    double max_acceptable;  // The maximum publishing delay (in seconds) before warning.
    pnh.param<double>("max_acceptable_delay", max_acceptable, 0.2);
    ros::SubscriberStatusCallback left_cb2 = boost::bind(&SpinnakerStereoCameraNodelet::leftConnectCb, this);
    pub_left_.reset(
        new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(
            nh.advertise<wfov_camera_msgs::WFOVImage>("image", queue_size, left_cb2, left_cb2),
            updater_, diagnostic_updater::FrequencyStatusParam(
                          &min_freq_, &max_freq_, freq_tolerance, window_size),
            diagnostic_updater::TimeStampStatusParam(min_acceptable,
                                                     max_acceptable)));

    // Set up diagnostics aggregator publisher and diagnostics manager
    ros::SubscriberStatusCallback diag_cb =
        boost::bind(&SpinnakerStereoCameraNodelet::diagCb, this);
    diagnostics_pub_left_.reset(
        new ros::Publisher(nh.advertise<diagnostic_msgs::DiagnosticArray>(
            "/diagnostics", 1, diag_cb, diag_cb)));

    diag_man = std::unique_ptr<DiagnosticsManager>(new DiagnosticsManager(
        frame_id_left_, std::to_string(spinnaker_left_.getSerial()), diagnostics_pub_left_));
    diag_man->addDiagnostic("DeviceTemperature", true, std::make_pair(0.0f, 90.0f), -10.0f, 95.0f);
    diag_man->addDiagnostic("AcquisitionResultingFrameRate", true, std::make_pair(10.0f, 60.0f), 5.0f, 90.0f);
    diag_man->addDiagnostic("PowerSupplyVoltage", true, std::make_pair(4.5f, 5.2f), 4.4f, 5.3f);
    diag_man->addDiagnostic("PowerSupplyCurrent", true, std::make_pair(0.4f, 0.6f), 0.3f, 1.0f);
    diag_man->addDiagnostic<int>("DeviceUptime");
    diag_man->addDiagnostic<int>("U3VMessageChannelID");
  }

  /**
   * @brief Reads in the camera serial from a specified file path.
   * The format of the serial is expected to be base 16.
   * @param camera_serial_path The path of where to read in the serial from. Generally this
   * is a USB device path to the serial file.
   * @return int The serial number for the given path, 0 if failure.
   */
  int readSerialAsHexFromFile(std::string camera_serial_path)
  {
    NODELET_DEBUG_ONCE("Reading camera serial file from: %s", camera_serial_path.c_str());

    std::ifstream serial_file(camera_serial_path.c_str());
    std::stringstream buffer;
    int serial = 0;

    if (serial_file.is_open())
    {
      std::string serial_str((std::istreambuf_iterator<char>(serial_file)), std::istreambuf_iterator<char>());
      NODELET_DEBUG_ONCE("Serial file contents: %s", serial_str.c_str());
      buffer << std::hex << serial_str;
      buffer >> serial;
      NODELET_DEBUG_ONCE("Serial discovered %d", serial);

      return serial;
    }

    NODELET_WARN_ONCE("Unable to open serial path: %s", camera_serial_path.c_str());
    return 0;
  }

  void diagPoll()
  {
    ros::Rate r(diag_pub_rate_);
    while (!boost::this_thread::interruption_requested())  // Block until we need
                                                           // to stop this
                                                           // thread.
    {
      diag_man->processDiagnostics(&spinnaker_left_);
      r.sleep();
    }
  }

  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing
  * them.
  */
  void devicePoll()
  {
    ROS_INFO_ONCE("devicePoll");

    enum State
    {
      NONE,
      ERROR,
      STOPPED,
      DISCONNECTED,
      CONNECTED,
      STARTED,
      SHUTDOWN,
    };

    State left_state = DISCONNECTED;
    State right_state = DISCONNECTED;
    State previous_left_state = NONE;
    State previous_right_state = NONE;

    if (reset_on_start_)
    {
      try
      {
        NODELET_INFO("Resetting device");
        spinnaker_left_.reset();
        spinnaker_right_.reset();
        spinnaker_left_.disconnect();
        spinnaker_right_.disconnect();
        while (!boost::this_thread::interruption_requested())
        {
          try
          {
            spinnaker_left_.connect();
            spinnaker_right_.connect();
            break;
          }
          catch (const std::runtime_error &e)
          {
            spinnaker_left_.disconnect();
            spinnaker_right_.disconnect();
            NODELET_INFO_STREAM("Waiting for device reset...");
            boost::this_thread::sleep_for(boost::chrono::seconds(1));
          }
        }
        NODELET_INFO("Device is reset");
      }
      catch (const std::runtime_error &e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }

    while (!boost::this_thread::interruption_requested())  // Block until we need to stop this thread.
    {
      bool left_state_changed = left_state != previous_left_state;
      bool right_state_changed = right_state != previous_right_state;

      previous_left_state = left_state;
      previous_right_state = right_state;

      // left state
      switch (left_state)
      {
        case SHUTDOWN:
          ros::shutdown();
          break;
        case ERROR:
// Generally there's no need to stop before disconnecting after an
// error. Indeed, stop will usually fail.
// #if STOP_ON_ERROR
          // Try stopping the camera
          {
            std::lock_guard<std::mutex> scopedLock(connect_mutex_);
            sub_.shutdown();
            sub_roi_.shutdown();
          }

          try
          {
            NODELET_DEBUG_ONCE("Stopping camera.");
            spinnaker_left_.stop();
            NODELET_DEBUG_ONCE("Stopped camera.");

            left_state = STOPPED;
          }
          catch (std::runtime_error& e)
          {
            if (left_state_changed)
            {
              NODELET_ERROR("Failed to stop with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            left_state = ERROR;
          }

          break;
// #endif
        case STOPPED:
          // Try disconnecting from the camera
          try
          {
            NODELET_DEBUG("Disconnecting from camera.");
            spinnaker_left_.disconnect();
            NODELET_DEBUG("Disconnected from camera.");

            left_state = DISCONNECTED;
          }
          catch (std::runtime_error& e)
          {
            if (left_state_changed)
            {
              NODELET_ERROR("Failed to disconnect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            left_state = ERROR;
          }

          break;
        case DISCONNECTED:
          // Try connecting to the camera
          try
          {
            NODELET_DEBUG("Connecting to camera.");

            spinnaker_left_.connect();

            NODELET_DEBUG("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            leftParamCallback(config_left_, SpinnakerCamera::LEVEL_RECONFIGURE_STOP);

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG_ONCE("Setting timeout to: %f.", timeout);
              spinnaker_left_.setTimeout(timeout);
            }
            catch (const std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
	      ros::NodeHandle left_pnh (getMTPrivateNodeHandle(), "left");
              std::lock_guard<std::mutex> scopedLock(connect_mutex_);
              sub_ =
                  left_pnh.subscribe("image_exposure_sequence", 1,
                                     &spinnaker_camera_driver::SpinnakerStereoCameraNodelet::leftGainWBCallback, this);
              sub_roi_ =
                  left_pnh.subscribe("set_roi", 1,
                                     &spinnaker_camera_driver::SpinnakerStereoCameraNodelet::leftRoiCallback, this);
            }

            left_state = CONNECTED;
          }
          catch (const std::runtime_error& e)
          {
            if (left_state_changed)
            {
              NODELET_ERROR("Failed to connect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            left_state = ERROR;
          }

          break;
        case CONNECTED:
          // Try starting the camera
          try
          {
            NODELET_DEBUG("Starting camera.");
            spinnaker_left_.start();
            NODELET_DEBUG("Started camera.");
            NODELET_DEBUG("Attention: if nothing subscribes to the camera topic, the camera_info is not published "
                          "on the correspondent topic.");
            left_state = STARTED;
          }
          catch (std::runtime_error& e)
          {
            if (left_state_changed)
            {
              NODELET_ERROR("Failed to start with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            left_state = ERROR;
          }

          break;
        case STARTED:
          NODELET_DEBUG("left camera started.");
          break;
        default:
          NODELET_ERROR("Unknown left camera state %d!", left_state);
      }

      // right state
      switch (right_state)
      {
        case SHUTDOWN:
          ros::shutdown();
          break;
        case ERROR:
// Generally there's no need to stop before disconnecting after an
// error. Indeed, stop will usually fail.
// #if STOP_ON_ERROR
          // Try stopping the camera
          {
            std::lock_guard<std::mutex> scopedLock(connect_mutex_);
            sub_.shutdown();
            sub_roi_.shutdown();
          }

          try
          {
            NODELET_DEBUG_ONCE("Stopping camera.");
            spinnaker_right_.stop();
            NODELET_DEBUG_ONCE("Stopped camera.");

            right_state = STOPPED;
          }
          catch (std::runtime_error& e)
          {
            if (right_state_changed)
            {
              NODELET_ERROR("Failed to stop with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            right_state = ERROR;
          }

          break;
// #endif
        case STOPPED:
          // Try disconnecting from the camera
          try
          {
            NODELET_DEBUG("Disconnecting from camera.");
            spinnaker_right_.disconnect();
            NODELET_DEBUG("Disconnected from camera.");

            right_state = DISCONNECTED;
          }
          catch (std::runtime_error& e)
          {
            if (right_state_changed)
            {
              NODELET_ERROR("Failed to disconnect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            right_state = ERROR;
          }

          break;
        case DISCONNECTED:
          // Try connecting to the camera
          try
          {
            NODELET_DEBUG("Connecting to camera.");

            spinnaker_right_.connect();

            NODELET_DEBUG("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            rightParamCallback(config_right_, SpinnakerCamera::LEVEL_RECONFIGURE_STOP);

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG_ONCE("Setting timeout to: %f.", timeout);
              spinnaker_right_.setTimeout(timeout);
            }
            catch (const std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
	      ros::NodeHandle right_pnh (getMTPrivateNodeHandle(), "right");
              std::lock_guard<std::mutex> scopedLock(connect_mutex_);
              sub_ =
                  right_pnh.subscribe("image_exposure_sequence", 1,
                                      &spinnaker_camera_driver::SpinnakerStereoCameraNodelet::rightGainWBCallback, this);
              sub_roi_ =
                  right_pnh.subscribe("set_roi", 1,
                                      &spinnaker_camera_driver::SpinnakerStereoCameraNodelet::rightRoiCallback, this);
            }

            right_state = CONNECTED;
          }
          catch (const std::runtime_error& e)
          {
            if (right_state_changed)
            {
              NODELET_ERROR("Failed to connect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            right_state = ERROR;
          }

          break;
        case CONNECTED:
          // Try starting the camera
          try
          {
            NODELET_DEBUG("Starting camera.");
            spinnaker_right_.start();
            NODELET_DEBUG("Started camera.");
            NODELET_DEBUG("Attention: if nothing subscribes to the camera topic, the camera_info is not published "
                          "on the correspondent topic.");
            right_state = STARTED;
          }
          catch (std::runtime_error& e)
          {
            if (right_state_changed)
            {
              NODELET_ERROR("Failed to start with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            right_state = ERROR;
          }

          break;
        case STARTED:
          // only publish images if both left and right state are STARTED
          if (left_state == STARTED)
          {
            try
            {
              wfov_camera_msgs::WFOVImagePtr left_wfov_image(new wfov_camera_msgs::WFOVImage);
              wfov_camera_msgs::WFOVImagePtr right_wfov_image(new wfov_camera_msgs::WFOVImage);
              // Get the image from the camera library
              NODELET_DEBUG_ONCE("Starting a new grab from camera with serial {%d}.", spinnaker_left_.getSerial());
              NODELET_DEBUG_ONCE("Starting a new grab from camera with serial {%d}.", spinnaker_right_.getSerial());
              spinnaker_left_.grabImage(&left_wfov_image->image, frame_id_left_, use_device_timestamp_);
              spinnaker_right_.grabImage(&right_wfov_image->image, frame_id_right_, use_device_timestamp_);

              // Set other values
              left_wfov_image->header.frame_id = frame_id_left_;
              right_wfov_image->header.frame_id = frame_id_right_;

              left_wfov_image->gain = left_camera_settings.gain_;
              right_wfov_image->gain = right_camera_settings.gain_;
              left_wfov_image->white_balance_blue = left_camera_settings.wb_blue_;
              right_wfov_image->white_balance_blue = right_camera_settings.wb_blue_;
              left_wfov_image->white_balance_red = left_camera_settings.wb_red_;
              right_wfov_image->white_balance_red = right_camera_settings.wb_red_;

              // wfov_image->temperature = spinnaker_right_.getCameraTemperature();

              if (use_device_timestamp_)
              {
                left_wfov_image->header.stamp = left_wfov_image->image.header.stamp;
                right_wfov_image->header.stamp = right_wfov_image->image.header.stamp;
              }
              else
              {
                const ros::Time time = ros::Time::now();
                left_wfov_image->header.stamp = time;
                left_wfov_image->image.header.stamp = time;
                right_wfov_image->header.stamp = time;
                right_wfov_image->image.header.stamp = time;
              }

              // Set the Left CameraInfo message
              ci_left_.reset(new sensor_msgs::CameraInfo(cinfo_left_->getCameraInfo()));
              ci_left_->header.stamp = left_wfov_image->image.header.stamp;
              ci_left_->header.frame_id = left_wfov_image->header.frame_id;
              // The width/height in sensor_msgs/CameraInfo is full camera resolution in pixels,
              // which is unchanged regardless of binning settings.
              ci_left_->width = ci_left_->width == 0 ? spinnaker_left_.getWidthMax() * left_camera_settings.binning_x_ : ci_left_->width;
              ci_left_->height = ci_left_->height == 0 ? spinnaker_left_.getHeightMax() * left_camera_settings.binning_y_ : ci_left_->height;
              // The height, width, distortion model, and parameters are all filled in by camera info manager.
              ci_left_->binning_x = left_camera_settings.binning_x_;
              ci_left_->binning_y = left_camera_settings.binning_y_;
              // NOTE: The ROI offset/size in Spinnaker driver is the values, given in binned image coordinates,
              //       in sensor_msgs/CameraInfo, on the other hand, given in un-binned image coordinates.
              ci_left_->roi.x_offset = left_camera_settings.roi_x_offset_ * left_camera_settings.binning_x_;
              ci_left_->roi.y_offset = left_camera_settings.roi_y_offset_ * left_camera_settings.binning_y_;
              ci_left_->roi.height = left_camera_settings.roi_height_ * left_camera_settings.binning_y_;
              ci_left_->roi.width = left_camera_settings.roi_width_ * left_camera_settings.binning_x_;
              ci_left_->roi.do_rectify = left_camera_settings.do_rectify_;

              // Set the Right CameraInfo message
              ci_right_.reset(new sensor_msgs::CameraInfo(cinfo_right_->getCameraInfo()));
              ci_right_->header.stamp = right_wfov_image->image.header.stamp;
              ci_right_->header.frame_id = right_wfov_image->header.frame_id;
              // The width/height in sensor_msgs/CameraInfo is full camera resolution in pixels,
              // which is unchanged regardless of binning settings.
              ci_right_->width = ci_right_->width == 0 ? spinnaker_right_.getWidthMax() * right_camera_settings.binning_x_ : ci_right_->width;
              ci_right_->height = ci_right_->height == 0 ? spinnaker_right_.getHeightMax() * right_camera_settings.binning_y_ : ci_right_->height;
              // The height, width, distortion model, and parameters are all filled in by camera info manager.
              ci_right_->binning_x = right_camera_settings.binning_x_;
              ci_right_->binning_y = right_camera_settings.binning_y_;
              // NOTE: The ROI offset/size in Spinnaker driver is the values, given in binned image coordinates,
              //       in sensor_msgs/CameraInfo, on the other hand, given in un-binned image coordinates.
              ci_right_->roi.x_offset = right_camera_settings.roi_x_offset_ * right_camera_settings.binning_x_;
              ci_right_->roi.y_offset = right_camera_settings.roi_y_offset_ * right_camera_settings.binning_y_;
              ci_right_->roi.height = right_camera_settings.roi_height_ * right_camera_settings.binning_y_;
              ci_right_->roi.width = right_camera_settings.roi_width_ * right_camera_settings.binning_x_;
              ci_right_->roi.do_rectify = right_camera_settings.do_rectify_;

              left_wfov_image->info = *ci_left_;
              right_wfov_image->info = *ci_right_;

              // Publish the full message
              pub_left_->publish(left_wfov_image);
              pub_right_->publish(right_wfov_image);

              // Publish the message using standard image transport
              if (it_pub_left_.getNumSubscribers() > 0 && it_pub_right_.getNumSubscribers() > 0)
              {
                sensor_msgs::ImagePtr left_image(new sensor_msgs::Image(left_wfov_image->image));
                sensor_msgs::ImagePtr right_image(new sensor_msgs::Image(right_wfov_image->image));
                it_pub_left_.publish(left_image, ci_left_);
                it_pub_right_.publish(right_image, ci_right_);
              }
            }
            catch (CameraTimeoutException& e)
            {
              NODELET_WARN("%s", e.what());
            }
            catch (const IncompleteImageException& e)
            {
              if (config_left_.ignore_incomplete_image && config_right_.ignore_incomplete_image)
              {
                NODELET_WARN("%s", e.what());
              }
              else
              {
                NODELET_ERROR("%s", e.what());
                left_state = ERROR;
                right_state = ERROR;
              }
            }
            catch (std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
              if (shutdown_on_error_)
              {
                NODELET_ERROR("Shutting down this process");
                left_state = SHUTDOWN;
                right_state = SHUTDOWN;
              }
              else
              {
                left_state = SHUTDOWN;
                right_state = SHUTDOWN;
              }
            }
          }
          break;
        default:
          NODELET_ERROR("Unknown right camera state %d!", right_state);
      }

      // Update diagnostics
      updater_.update();
    }
    NODELET_DEBUG_ONCE("Leaving thread.");
  }

  void leftGainWBCallback(const image_exposure_msgs::ExposureSequence& msg)
  {
    try
    {
      NODELET_DEBUG_ONCE("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain,
                         msg.white_balance_blue, msg.white_balance_red);
      left_camera_settings.gain_ = msg.gain;

      spinnaker_left_.setGain(static_cast<float>(left_camera_settings.gain_));
      left_camera_settings.wb_blue_ = msg.white_balance_blue;
      left_camera_settings.wb_red_ = msg.white_balance_red;

      // TODO(mhosmar):
      // spinnaker_left_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  void rightGainWBCallback(const image_exposure_msgs::ExposureSequence& msg)
  {
    try
    {
      NODELET_DEBUG_ONCE("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain,
                         msg.white_balance_blue, msg.white_balance_red);
      right_camera_settings.gain_ = msg.gain;

      spinnaker_right_.setGain(static_cast<float>(right_camera_settings.gain_));
      right_camera_settings.wb_blue_ = msg.white_balance_blue;
      right_camera_settings.wb_red_ = msg.white_balance_red;

      // TODO(mhosmar):
      // spinnaker_right_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  void leftRoiCallback(const sensor_msgs::RegionOfInterest::ConstPtr &msg)
  {
    if ((msg->width + msg->height) > 0 &&
        (static_cast<int>(msg->width) < spinnaker_left_.getWidthMax() ||
         static_cast<int>(msg->height) < spinnaker_left_.getHeightMax()))
    {
      left_camera_settings.roi_x_offset_ = msg->x_offset;
      left_camera_settings.roi_y_offset_ = msg->y_offset;
      left_camera_settings.roi_width_ = msg->width;
      left_camera_settings.roi_height_ = msg->height;
      left_camera_settings.do_rectify_ = true;
    }
    else
    {
      // Zeros mean the full resolution was captured.
      left_camera_settings.roi_x_offset_ = 0;
      left_camera_settings.roi_y_offset_ = 0;
      left_camera_settings.roi_height_ = 0;
      left_camera_settings.roi_width_ = 0;
      left_camera_settings.do_rectify_ = false;  // Set to false if the whole image is captured.
    }
    spinnaker_left_.setROI(left_camera_settings.roi_x_offset_,
			   left_camera_settings.roi_y_offset_,
			   left_camera_settings.roi_width_,
			   left_camera_settings.roi_height_);
  }

  void rightRoiCallback(const sensor_msgs::RegionOfInterest::ConstPtr &msg)
  {
    if ((msg->width + msg->height) > 0 &&
        (static_cast<int>(msg->width) < spinnaker_right_.getWidthMax() ||
         static_cast<int>(msg->height) < spinnaker_right_.getHeightMax()))
    {
      right_camera_settings.roi_x_offset_ = msg->x_offset;
      right_camera_settings.roi_y_offset_ = msg->y_offset;
      right_camera_settings.roi_width_ = msg->width;
      right_camera_settings.roi_height_ = msg->height;
      right_camera_settings.do_rectify_ = true;
    }
    else
    {
      // Zeros mean the full resolution was captured.
      right_camera_settings.roi_x_offset_ = 0;
      right_camera_settings.roi_y_offset_ = 0;
      right_camera_settings.roi_height_ = 0;
      right_camera_settings.roi_width_ = 0;
      right_camera_settings.do_rectify_ = false;  // Set to false if the whole image is captured.
    }
    spinnaker_right_.setROI(right_camera_settings.roi_x_offset_,
			   right_camera_settings.roi_y_offset_,
			   right_camera_settings.roi_width_,
			   right_camera_settings.roi_height_);
  }

  /* Class Fields */
  std::shared_ptr<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> > srv_left_;
  std::shared_ptr<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> > srv_right_;  ///< Needed to
                                                                                                 ///  initialize
                                                                                                 ///  and keep the
  /// dynamic_reconfigure::Server
  /// in scope.

  // left camera
  std::shared_ptr<image_transport::ImageTransport> it_left_;  ///< Needed to initialize and keep the ImageTransport in
                                                         /// scope.
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;  ///< Needed to initialize and keep the
                                                                   /// CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_left_;                        ///< CameraInfoManager ROS publisher
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_left_;  ///< Diagnosed
  std::shared_ptr<ros::Publisher> diagnostics_pub_left_;

  // right camera
  std::shared_ptr<image_transport::ImageTransport> it_right_;  ///< Needed to initialize and keep the ImageTransport in
                                                         /// scope.
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;  ///< Needed to initialize and keep the
                                                                   /// CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_right_;                        ///< CameraInfoManager ROS publisher
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_right_;  ///< Diagnosed
  std::shared_ptr<ros::Publisher> diagnostics_pub_right_;
  /// publisher, has to be
  /// a pointer because of
  /// constructor
  /// requirements
  ros::Subscriber sub_;  ///< Subscriber for gain and white balance changes.
  ros::Subscriber sub_roi_;   ///< Subscriber for setting ROI

  std::mutex connect_mutex_;

  diagnostic_updater::Updater updater_;  ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  // left camera
  SpinnakerCamera spinnaker_left_;      ///< Instance of the SpinnakerCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_left_;  ///< Camera Info message.

  // right camera
  SpinnakerCamera spinnaker_right_;      ///< Instance of the SpinnakerCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_right_;  ///< Camera Info message.

  std::string frame_id_left_;           ///< Frame id for the camera messages, defaults to 'left_camera'
  std::string frame_id_right_;           ///< Frame id for the camera messages, defaults to 'right_camera'
  std::shared_ptr<boost::thread> pubThread_;  ///< The thread that reads and publishes the images.
  std::shared_ptr<boost::thread> diagThread_;  ///< The thread that reads and publishes the diagnostics.

  bool is_left_connected_;
  bool is_right_connected_;

  bool reset_on_start_;
  bool shutdown_on_error_;
  bool use_device_timestamp_;
  bool publish_diagnostics_;
  double diag_pub_rate_;
  std::unique_ptr<DiagnosticsManager> diag_man;

  struct CameraSettings{
    double gain_;
    uint16_t wb_blue_;
    uint16_t wb_red_;

    // Parameters for cameraInfo
    size_t binning_x_;     ///< Camera Info pixel binning along the image x axis.
    size_t binning_y_;     ///< Camera Info pixel binning along the image y axis.
    size_t roi_x_offset_;  ///< Camera Info ROI x offset
    size_t roi_y_offset_;  ///< Camera Info ROI y offset
    size_t roi_height_;    ///< Camera Info ROI height
    size_t roi_width_;     ///< Camera Info ROI width
    bool do_rectify_;  ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in
		       /// ROI mode.
  };

  CameraSettings left_camera_settings;
  CameraSettings right_camera_settings;

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  int packet_size_;
  /// GigE packet delay:
  int packet_delay_;

  /// Configuration:
  spinnaker_camera_driver::SpinnakerConfig config_left_;
  spinnaker_camera_driver::SpinnakerConfig config_right_;
};

PLUGINLIB_EXPORT_CLASS(spinnaker_camera_driver::SpinnakerStereoCameraNodelet,
                       nodelet::Nodelet)  // Needed for Nodelet declaration
}  // namespace spinnaker_camera_driver
