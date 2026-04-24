#include "pyro_rudder_chassis.h"

namespace pyro
{
    void rudder_chassis_t::state_passive_t::enter(owner *owner)
    {
         owner->_ctx.rudder_config.motor.rudder[0]->disable();
         owner->_ctx.rudder_config.motor.rudder[1]->disable();
         owner->_ctx.rudder_config.motor.rudder[2]->disable();
         owner->_ctx.rudder_config.motor.rudder[3]->disable();


         owner->_ctx.rudder_config.motor.wheel[0]->disable();
         owner->_ctx.rudder_config.motor.wheel[1]->disable();
         owner->_ctx.rudder_config.motor.wheel[2]->disable();
         owner->_ctx.rudder_config.motor.wheel[3]->disable();
            
    }

    void rudder_chassis_t::state_passive_t::execute(owner *owner)
    {
        owner->_ctx.rudder_config.motor.rudder[0]->send_torque(0);
        owner->_ctx.rudder_config.motor.rudder[1]->send_torque(0);
        owner->_ctx.rudder_config.motor.rudder[2]->send_torque(0);
        owner->_ctx.rudder_config.motor.rudder[3]->send_torque(0);
        owner->_ctx.rudder_config.motor.wheel[0]->send_torque(0);
        owner->_ctx.rudder_config.motor.wheel[1]->send_torque(0);
        owner->_ctx.rudder_config.motor.wheel[2]->send_torque(0);
        owner->_ctx.rudder_config.motor.wheel[3]->send_torque(0);
    }

    void rudder_chassis_t::state_passive_t::exit(owner *owner)
    {   

    }

}