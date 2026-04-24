#include "pyro_rudomni_chassis.h"

namespace pyro
{
    void rudomni_chassis_t::fsm_active_t::state_moving_t::enter(owner *owner){}

    void rudomni_chassis_t::fsm_active_t::state_moving_t::execute(owner *owner)
    {
        // 更新舵机角度记录

        _chassis_control(&owner->_ctx,&owner->_yaw_data);

        _send_motor_command(&owner->_ctx,&owner->_yaw_data);
    }

    void rudomni_chassis_t::fsm_active_t::state_moving_t::exit(owner *owner)
    {   

    }

}