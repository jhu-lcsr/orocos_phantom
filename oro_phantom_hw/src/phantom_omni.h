#ifndef __PHANTOM_COMPONENTS_PHANTOM_OMNI_H
#define __PHANTOM_COMPONENTS_PHANTOM_OMNI_H

// RTT Headers
#include <rtt/RTT.hpp>

// KDL Headers
#include <kdl/framevel.hpp>

// ROS Headers
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <telemanip_msgs/TelemanipCommand.h>

// Sensable Headers
#include <HL/hl.h>
#include <HD/hd.h>

namespace oro_phantom_hw {
  class PhantomOmni : public RTT::TaskContext {
  public:
    // RTT Ports
    RTT::InputPort<KDL::Vector> cart_force_in_;
    RTT::OutputPort<KDL::Frame> cart_pose_out_;
    RTT::OutputPort<KDL::Twist> cart_twist_out_;

    RTT::OutputPort<telemanip_msgs::TelemanipCommand> telemanip_cmd_out_;
   
    // RTT Operations
    RTT::OperationCaller
      <void(const geometry_msgs::TransformStamped &)> broadcast_;

    PhantomOmni(std::string const& name);

    // RTT Interface
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool hapticHook();

    double get_loop_rate() const {
      return 1.0/loop_period_;
    }

    double scale() const {
      return scale_;
    }

  private:
    bool initialized_;
    bool calibrated_;
    bool running_;
    HHD haptic_device_handle_;
    HDSchedulerHandle scheduler_handle_;
    double scale_;
    double damping_;
    double hip_support_force_;

    // State
    KDL::Frame pose_;
    KDL::Twist twist_;
    KDL::Vector force_;
    bool button_1_;
    bool button_2_;

    RTT::os::TimeService::ticks last_loop_time_;
    RTT::os::TimeService::Seconds loop_period_;
  };
}
#endif // ifndef __PHANTOM_COMPONENTS_PHANTOM_OMNI_H
