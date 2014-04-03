#include <time.h>
#include <iostream>

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

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

#include <rtt_ros_tools/throttles.h>
#include <rtt_rosclock/rtt_rosclock.h>

#include "phantom_omni.h"

using namespace oro_phantom_hw;

bool phantom_omni_calibration()
{
  RTT::Logger::In("phantom_omni_calibration");

  int calibrationStyle;
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
  {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    RTT::log(RTT::Info) << "HD_CALIBRATION_ENCODER_RESET..." << RTT::endlog();
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
  {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    RTT::log(RTT::Info) << "HD_CALIBRATION_INKWELL..." << RTT::endlog();
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
  {
    calibrationStyle = HD_CALIBRATION_AUTO;
    RTT::log(RTT::Info) << "HD_CALIBRATION_AUTO..." << RTT::endlog();
  }

  ros::Rate rate(1.0);
  do 
  {
    hdUpdateCalibration(calibrationStyle);
    RTT::log(RTT::Warning) << "Phantom Omni needs to be calibrated, please put the stylus in the well." << RTT::endlog();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
      RTT::log(RTT::Error) << "Calibration failed: "<<error << RTT::endlog();
      return false;
    }
    rate.sleep();
  }   while (hdCheckCalibration() != HD_CALIBRATION_OK && ros::ok());

  RTT::log(RTT::Info) << "Phantom Omni calibration complete." << RTT::endlog();

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

static inline void SO3toso3(
    const KDL::Rotation &rot,
    KDL::Vector &vec)
{
  KDL::Vector axis;
  const double angle = rot.GetRotAngle(axis);

  if(std::abs(angle) < 1E-6) {
    vec.x(0);
    vec.y(0);
    vec.z(0);
    return;
  }

  typedef Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > RotMap;
  RotMap map = RotMap(rot.data);

  Eigen::Matrix3d logR = angle/2.0/sin(angle)*(map - map.transpose());

  vec.x(logR(2,1));
  vec.y(logR(0,2));
  vec.z(logR(1,0));
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

PhantomOmni::PhantomOmni(std::string const& name) : 
  TaskContext(name)
  ,initialized_(false)
  ,broadcast_("broadcastTransform")
  ,scale_(10.0)
  ,damping_(0.2)
  ,hip_support_force_(0.5)
  ,pose_(KDL::Frame::Identity())
  ,twist_(KDL::Twist::Zero())
  ,force_(KDL::Vector::Zero())
{
  std::cout<<"PhantomOmni constructed!" <<std::endl;

  // Set up RTT properties
  this->addProperty("scale",scale_).doc("The cartesian scale.");
  this->addProperty("damping",damping_).doc("The cartesian damping.");
  this->addProperty("force",force_).doc("The cartesian force.");
  this->addProperty("hip_support_force",hip_support_force_).doc("Gravity compensation force.");

  // Set up RTT interface
  this->ports()->addPort("cart_force_in",cart_force_in_);
  this->ports()->addPort("cart_pose_out",cart_pose_out_);
  this->ports()->addPort("cart_twist_out",cart_twist_out_);

  this->ports()->addPort("telemanip_cmd_out",telemanip_cmd_out_)
    .doc("Current pose and twist of the omni haptic interface point (HIP).");

  this->addOperation("getLoopRate",
      &PhantomOmni::get_loop_rate, this, RTT::ClientThread)
    .doc("Get the loop rate (Hz)");

  this->requires("tf")->addOperationCaller(broadcast_);
}

bool PhantomOmni::configureHook()
{
  RTT::Logger::In("PhantomOmni::configureHook");
  HDErrorInfo error;

  // Initialize device
  if(!initialized_) {
    haptic_device_handle_ = hdInitDevice(HD_DEFAULT_DEVICE);
    if(HD_DEVICE_ERROR(error = hdGetError())) {
      ROS_ERROR_STREAM("Failed to initialize haptic device! Error: "<<error);
      return false;
    }
  }

  RTT::log(RTT::Info) << "Using " << hdGetString(HD_DEVICE_MODEL_TYPE) << RTT::endlog();

  return true;
}

HDCallbackCode HDCALLBACK phantom_omni_cb(void *pUserData)
{
  // Get poiinter to omni task
  PhantomOmni *omni_task = static_cast<PhantomOmni*>(pUserData);

  // Trigger the task
  if(omni_task) {
    if(omni_task->getTaskState() == RTT::TaskContext::Running) {
      omni_task->hapticHook();
    } else {
      return HD_CALLBACK_DONE;
    }
  }

  return HD_CALLBACK_CONTINUE;
}

bool PhantomOmni::startHook()
{
  RTT::Logger::In("PhantomOmni::startHook");
  HDErrorInfo error;

  // Start the scheduler
  hdStartScheduler(); 
  if(HD_DEVICE_ERROR(error = hdGetError())) {
    RTT::log(RTT::Error) << "Failed to start the HDAPI scheduler! Error: " << error << RTT::endlog();
    return false;           
  }

  // Calibrate the joints
  if(phantom_omni_calibration()) {
    calibrated_ = true;
  } else {
    return false;
  }

  // Blocking call through to the HDAPI layer
  hdScheduleAsynchronous(phantom_omni_cb, this, HD_MAX_SCHEDULER_PRIORITY);

  return true;
}

void PhantomOmni::updateHook()
{
}

bool PhantomOmni::hapticHook()
{
  // Rotate the frame so that the world Z points up 
  static const KDL::Frame z_up = KDL::Frame(KDL::Rotation::RotX(M_PI/2.0));

  HDErrorInfo error;

  loop_period_ = RTT::os::TimeService::Instance()->secondsSince(last_loop_time_);
  last_loop_time_ = RTT::os::TimeService::Instance()->getTicks();

  // HDAPI structures
  hduVector3Dd position(0,0,0); 
  hduVector3Dd orientation(0,0,0);
  HDdouble ee_transform[16];

  // Read inputs
  force_ = KDL::Vector::Zero();
  cart_force_in_.readNewest(force_);

  // Add damping
  force_ -= damping_ * twist_.vel;

  // Add gravity compensation
  force_[2] += hip_support_force_;

  ////////////////////////////////////////////////////////////////////////////
  // Get the device, begin the interaction frame
  hdBeginFrame(haptic_device_handle_);

  // Enable force output
  hdEnable(HD_FORCE_OUTPUT);

  // Transform to y-up
  KDL::Vector hip_force = z_up.Inverse()*force_;
  // Set the force
  hdSetDoublev(HD_CURRENT_FORCE, hip_force.data);

  // Get omni position and orientation
  hdGetDoublev(HD_CURRENT_TRANSFORM, ee_transform);      

  //Get buttons
  int button_bits = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &button_bits);
  button_1_ = button_bits & HD_DEVICE_BUTTON_1;
  button_2_ = button_bits & HD_DEVICE_BUTTON_2;

  // End the interaction frame
  hdEndFrame(haptic_device_handle_);
  ////////////////////////////////////////////////////////////////////////////

  // Working variables
  KDL::Frame pose = KDL::Frame::Identity();

  // Convert to meters
  ee_transform[12] /= 1000.0/scale_;
  ee_transform[13] /= 1000.0/scale_;
  ee_transform[14] /= 1000.0/scale_;
  // Convert to KDL frame
  KDL::Frame new_pose;
  array_to_frame(ee_transform,new_pose);

  // Transform to z-up
  new_pose = z_up*new_pose;

  // Compute twist
  KDL::Vector dp = new_pose.p - pose_.p;
  KDL::Rotation dM = pose_.M.Inverse()*new_pose.M;
  KDL::Vector dw;
  SO3toso3(dM, dw);

  // Compute the new twist measurement
  KDL::Twist new_twist = KDL::Twist(dp,dw) / loop_period_;

  // Update the twist
  static const double alpha = 0.1;
  twist_ = (1.0-alpha)*twist_ + (alpha)*new_twist;

  // Update the pose
  pose_ = new_pose;

  // Construct framevel
  KDL::FrameVel fv = KDL::FrameVel(
      pose_,
      twist_);

  // Write data ports
  cart_pose_out_.write(pose_);
  cart_twist_out_.write(twist_);

  static rtt_ros_tools::PeriodicThrottle pthrottle(0.019);

  // Publish the telemanip message
  if(pthrottle.ready()) {
    // Construct the telemanip message
    telemanip_msgs::TelemanipCommand cmd;
    cmd.header.frame_id = "/omni_base";
    //cmd.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    cmd.grasp_opening = button_1_ ? 0.0 : 1.0;
    cmd.deadman_engaged = button_2_;
    
    //ROS_INFO_STREAM("GIMBALS: "<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]);
    cmd.header.stamp = rtt_rosclock::host_now();
    framevel_to_posetwist(fv, cmd.posetwist);
    telemanip_cmd_out_.write(cmd);
    
    // Broadcast a TF frame
    tf::Transform T_tf;
    tf::PoseKDLToTF(fv.value(),T_tf);

    geometry_msgs::TransformStamped tform;
    tf::transformTFToMsg(T_tf,tform.transform);
    tform.header = cmd.header;
    tform.child_frame_id = "/omni_hip";
    broadcast_(tform);
  }

  return true;
}

void PhantomOmni::stopHook()
{
  RTT::Logger::In("PhantomOmni::stopHook");
  HDErrorInfo error;

  RTT::log(RTT::Info) << ("Stopping HDAPI scheduler...") << RTT::endlog();
  hdStopScheduler();
  if(HD_DEVICE_ERROR(error = hdGetError())) {
    RTT::log(RTT::Error) << "Failed to stop the HDAPI scheduler! Error: " << error << RTT::endlog();
    return;
  }
}

void PhantomOmni::cleanupHook()
{
  RTT::log(RTT::Info) << "Disabling haptic device..." << RTT::endlog();
  //FIXME: This causes a segfault
  //hdDisableDevice(haptic_device_handle_);
  RTT::log(RTT::Warning) << "The haptic device has been left enabled." << RTT::endlog();
}

