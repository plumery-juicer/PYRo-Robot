#include "pyro_rudder_chassis.h"

namespace pyro
{
    void rudder_chassis_t::fsm_active_t::on_enter(owner *owner)
    {
        owner->_ctx.rudder_config.motor.rudder[0]->enable();
        owner->_ctx.rudder_config.motor.rudder[1]->enable();
        owner->_ctx.rudder_config.motor.rudder[2]->enable();
        owner->_ctx.rudder_config.motor.rudder[3]->enable();
        owner->_ctx.rudder_config.motor.wheel[0]->enable();
        owner->_ctx.rudder_config.motor.wheel[1]->enable();
        owner->_ctx.rudder_config.motor.wheel[2]->enable();
        owner->_ctx.rudder_config.motor.wheel[3]->enable();
      
        owner->_ctx.drive_mode = drive_mode_t::MOVING; // 默认进入MOVING模式
    }

    void rudder_chassis_t::fsm_active_t::on_execute(owner *owner)
    {
        if(rudder_chassis_t::drive_mode_t::MOVING == owner->_ctx.drive_mode)
        {
            change_state(&_moving_state);
        }
        owner->_kinematics_solve();
    }

    void rudder_chassis_t::fsm_active_t::on_exit(owner *owner)
    {

    }
}