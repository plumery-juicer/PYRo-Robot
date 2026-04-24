#include "pyro_infantry1_gimbal.h"

namespace pyro
{
void infantry1_gimbal_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.gimbal_cfg_t.motor.pitch->enable();
    owner->_ctx.gimbal_cfg_t.motor.yaw->enable();
}

void infantry1_gimbal_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.drive_mode == drive_mode_t::MOVING){
        change_state(&_moving_state);
    }

    owner->_gimbal_control();
   // _send_motor_command(&owner->_ctx);
}

void infantry1_gimbal_t::fsm_active_t::on_exit(owner *owner) {}

} // namespace pyro