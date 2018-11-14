#include <mav_control_interface/mav_control_pure.h>

#include "mav_control_interface_pure.h"

namespace mav_control_interface {

MavControlPure::MavControlPure(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                         std::shared_ptr<PositionControllerInterface> controller)
{
  mav_control_interface_pure_.reset(new MavControlInterfacePure(nh, private_nh, controller));
}

MavControlPure::~MavControlPure()
{
}

} /* namespace mav_control_interface */
