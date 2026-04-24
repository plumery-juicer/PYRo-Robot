#include "pyro_rudder_chassis.h"

namespace pyro
{
    void rudder_chassis_t::fsm_active_t::state_moving_t::enter(owner *owner){}

    void rudder_chassis_t::fsm_active_t::state_moving_t::execute(owner *owner)
    {
        _chassis_control(&owner->_ctx);

        _send_motor_command(&owner->_ctx);
    }

    void rudder_chassis_t::fsm_active_t::state_moving_t::exit(owner *owner)
    {   

    }

}