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

// Typekits TODO: this supposedly speeds up compile times, but it just seems to cause linker errors
//#include <geometry_msgs/typekit/Types.hpp> 
//#include <dsat_msgs/typekit/Types.hpp> 

// Sensable Headers
#include <HL/hl.h>
#include <HD/hd.h>

namespace phantom_components {
  class PhantomOmni : public RTT::TaskContext {
  public:
    // RTT Ports
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

    double get_loop_rate() const {
      return 1.0/loop_period_;
    }

    double scale() const {
      return scale_;
    }

  private:
    HHD haptic_device_handle_;
    double scale_;

    // Event calbacks
    void kdl_pose_twist_cb();
    void msg_pose_twist_cb();

    // Command / goal structures
    KDL::FrameVel pt_goal_;
    ros::Time pt_goal_stamp_;

    // Proxies to RTI values
    KDL::Twist twist_;
    KDL::Frame pose_;

    // Nutex for locking commanded cartesian goal interface
    RTT::os::Mutex cmd_mutex_;

    RTT::os::TimeService::ticks last_loop_time_;
    RTT::os::TimeService::Seconds loop_period_;
  };
}
#endif // ifndef __PHANTOM_COMPONENTS_PHANTOM_OMNI_H
