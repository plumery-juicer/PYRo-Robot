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

    owner->_ctx.gimbal_cfg_t.motor.yaw->send_torque(0);
}

void infantry1_gimbal_t::state_passive_t::exit(owner *owner){
    owner->_ctx.data.target_pitch_rad=owner->_ctx.data.current_pitch_rad;
}
} // namespace pyro