#include "pyro_infantry1_gimbal.h"

namespace pyro
{
void infantry1_gimbal_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.gimbal_cfg_t.motor.pitch->disable();
    owner->_ctx.gimbal_cfg_t.motor.yaw->disable();
}

void infantry1_gimbal_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.gimbal_cfg_t.motor.pitch->send_torque(0);
    owner->_ctx.gimbal_cfg_t.motor.yaw->send_torque(0);
}

void infantry1_gimbal_t::state_passive_t::exit(owner *owner) {}
} // namespace pyro