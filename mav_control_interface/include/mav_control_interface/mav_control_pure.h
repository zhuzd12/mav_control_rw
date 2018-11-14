#ifndef MAV_CONTROL_PURE_H_
#define MAV_CONTROL_PURE_H_

#include <memory>

#include <mav_control_interface/position_controller_interface.h>
#include <mav_control_interface/rc_interface.h>
#include <ros/ros.h>

namespace mav_control_interface {

class MavControlInterfacePure;

class MavControlPure
{
 public:
  MavControlPure(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                      std::shared_ptr<PositionControllerInterface> controller);

  ~MavControlPure();
 private:
  std::unique_ptr<MavControlInterfacePure> mav_control_interface_pure_;
};

} /* namespace mav_control_interface */

#endif /* MAV_CONTROL_PURE_H_ */