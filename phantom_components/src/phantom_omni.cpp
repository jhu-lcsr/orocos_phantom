#include <time.h>
#include <iostream>

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include <kdl/frames_io.hpp>

// Sensable Headers
#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

// ROS Headers
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

#include <rtt_ros_tools/time.h>
#include <rtt_ros_tools/throttles.h>

#include "phantom_omni.h"

using namespace phantom_components;

bool phantom_omni_calibration()
{
  int calibrationStyle;
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
  {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET...");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
  {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL...");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
  {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO...");
  }

  ros::Rate rate(1.0);
  do 
  {
    hdUpdateCalibration(calibrationStyle);
    ROS_WARN("Phantom Omni needs to be calibrated, please put the stylus in the well.");
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
      ROS_ERROR_STREAM("Calibration failed: "<<error);
      return false;
    }
    rate.sleep();
  }   while (hdCheckCalibration() != HD_CALIBRATION_OK && ros::ok());

  ROS_INFO("Phantom Omni calibration complete.");

  return true;
}

static inline void posetwist_to_framevel(
    const cartesian_trajectory_msgs::PoseTwist &pt, 
    KDL::FrameVel &ft)
{
  using namespace tf;

  KDL::Frame T;
  KDL::Twist t;
  PoseMsgToKDL(pt.pose, T);
  TwistMsgToKDL(pt.twist, t);

  ft = KDL::FrameVel(T,t);
}

static inline void framevel_to_posetwist(
    const KDL::FrameVel &fv, 
    cartesian_trajectory_msgs::PoseTwist &pt)
{
  using namespace tf;

  PoseKDLToMsg(fv.value(), pt.pose);
  TwistKDLToMsg(fv.deriv(), pt.twist);
}

// Convert a 4x4 array (column-major) into a KDL frame
static inline void array_to_frame(
    const double* array,
    KDL::Frame &frame)
{
  // Initialize frame
  frame = KDL::Frame(
      KDL::Rotation(
        array[0],array[4],array[8],
        array[1],array[5],array[9],
        array[2],array[6],array[10]),
      KDL::Vector(array[12],array[13],array[14]));
}

HDCallbackCode HDCALLBACK phantom_omni_cb(void *pUserData)
{
  HDErrorInfo error;

  // Get poiinter to omni task
  PhantomOmni *phantom_omni_task = static_cast<PhantomOmni*>(pUserData);

  // Get the device, begin the interaction frame
  hdBeginFrame(hdGetCurrentDevice());

  // Get omni orientation
  hduVector3Dd position(0,0,0); 
  hduVector3Dd orientation(0,0,0);
  hduVector3Dd force(0,0,0);

  HDdouble ee_transform[16];

  hdGetDoublev(HD_CURRENT_POSITION, position);
  if(HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR_STREAM("Failed to get position! Error: "<<error);
  }
  //hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, orientation);      
  hdGetDoublev(HD_CURRENT_TRANSFORM, ee_transform);      

  //Get buttons
  int button_bits = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &button_bits);
  bool button_1 = button_bits & HD_DEVICE_BUTTON_1;
  bool button_2 = button_bits & HD_DEVICE_BUTTON_2;

  // Zero out the force
  hdSetDoublev(HD_CURRENT_FORCE, force);
  // End the interaction frame
  hdEndFrame(hdGetCurrentDevice());

  static rtt_ros_tools::PeriodicThrottle pthrottle(0.019);

  // Publish the telemanip message
  if(pthrottle.ready()) {
    // Working variables
    KDL::Frame ee_transform_kdl = KDL::Frame::Identity();

    // Construct the telemanip message
    telemanip_msgs::TelemanipCommand cmd;
    cmd.header.frame_id = "/omni_base";
    cmd.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    cmd.grasp_opening = button_1 ? 0.0 : 1.0;
    cmd.deadman_engaged = button_2;
    
    // Convert to meters
    ee_transform[12] /= 1000.0/phantom_omni_task->scale();
    ee_transform[13] /= 1000.0/phantom_omni_task->scale();
    ee_transform[14] /= 1000.0/phantom_omni_task->scale();
    // Convert to KDL frame
    array_to_frame(ee_transform,ee_transform_kdl);

    // Rotate the frame so that Z points in the direction of the tip
    KDL::Frame local_rotx = KDL::Frame(KDL::Rotation::RotX(M_PI));
    KDL::Frame local_roty = KDL::Frame(KDL::Rotation::RotY(M_PI));
    KDL::Frame local_rotz = KDL::Frame(KDL::Rotation::RotZ(-M_PI/2.0));

    ee_transform_kdl = ee_transform_kdl*local_rotx*local_rotz;//*KDL::Frame(local_roty)*KDL::Frame(local_rotz);

    // Construct framevel
    KDL::FrameVel fv = KDL::FrameVel(
        ee_transform_kdl,
        KDL::Twist::Zero());
    framevel_to_posetwist(fv, cmd.posetwist);

    //ROS_INFO_STREAM("GIMBALS: "<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]);
    cmd.header.stamp = ros::Time::now();
    phantom_omni_task->telemanip_cmd_out_.write(cmd);
    
    // Broadcast a TF frame
    tf::Transform T_tf;
    tf::PoseKDLToTF(fv.value(),T_tf);

    geometry_msgs::TransformStamped tform;
    tf::transformTFToMsg(T_tf,tform.transform);
    tform.header = cmd.header;
    tform.child_frame_id = "/omni_hip";
    phantom_omni_task->broadcast_(tform);
  }

  return HD_CALLBACK_CONTINUE;
}

PhantomOmni::PhantomOmni(std::string const& name) : 
  TaskContext(name)
  ,broadcast_("broadcastTransform")
  ,scale_(10.0)
{
  std::cout<<"PhantomOmni constructed!" <<std::endl;

  // Set up RTT properties
  this->addProperty("scale",scale_).doc("The cartesian scale.");

  // Set up RTT interface
  this->ports()->addPort("telemanip_cmd_out",telemanip_cmd_out_)
    .doc("Current pose and twist of the omni haptic interface point (HIP).");

  this->addOperation("getLoopRate",
      &PhantomOmni::get_loop_rate, this, RTT::OwnThread)
    .doc("Get the loop rate (Hz)");

  this->requires("tf")->addOperationCaller(broadcast_);
}

bool PhantomOmni::configureHook()
{
  return true;
}

bool PhantomOmni::startHook()
{
  HDErrorInfo error;

  // Initialize device
  haptic_device_handle_ = hdInitDevice(HD_DEFAULT_DEVICE);
  if(HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR_STREAM("Failed to initialize haptic device! Error: "<<error);
    return false;
  }
  ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

  // Enable force output
  hdEnable(HD_FORCE_OUTPUT);

  // Start the scheduler
  hdStartScheduler(); 
  if(HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR_STREAM("Failed to start the HDAPI scheduler! Error: "<<error);
    return false;           
  }

  if(!phantom_omni_calibration()) {
    return false;
  }

  // Start the callback scheduler
  hdScheduleAsynchronous(phantom_omni_cb, this, HD_MAX_SCHEDULER_PRIORITY);

  return true;
}

void PhantomOmni::updateHook()
{
}

void PhantomOmni::stopHook()
{
  ROS_INFO("Stopping HDAPI scheduler...");
  hdStopScheduler();

  ROS_INFO("Disabling haptic device...");
  hdDisableDevice(haptic_device_handle_);
}

void PhantomOmni::cleanupHook()
{
}

