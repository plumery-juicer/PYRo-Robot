#include "pyro_infantry1_gimbal.h"

namespace pyro
{
    void infantry1_gimbal_t::fsm_active_t::state_moving_t::enter(owner *owner){}

    void infantry1_gimbal_t::fsm_active_t::state_moving_t::execute(owner *owner)
    {
        _send_motor_command(&owner->_ctx);
    }

    void infantry1_gimbal_t::fsm_active_t::state_moving_t::exit(owner *owner)
    {   

    }

}