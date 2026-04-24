#include "pyro_rudomni_chassis.h"

namespace pyro
{
    void rudomni_chassis_t::state_passive_t::enter(owner *owner)
    {
         owner->_ctx.rudomni_config.motor.rudder[0]->disable();
         owner->_ctx.rudomni_config.motor.rudder[1]->disable();

         owner->_ctx.rudomni_config.motor.wheel[0]->disable();
         owner->_ctx.rudomni_config.motor.wheel[1]->disable();
         owner->_ctx.rudomni_config.motor.wheel[2]->disable();
         owner->_ctx.rudomni_config.motor.wheel[3]->disable();
         owner->_ctx.rudomni_config.motor.yaw->disable();    
    }

    void rudomni_chassis_t::state_passive_t::execute(owner *owner)
    {
        owner->_ctx.rudomni_config.motor.rudder[0]->send_torque(0);
        owner->_ctx.rudomni_config.motor.rudder[1]->send_torque(0);
        owner->_ctx.rudomni_config.motor.wheel[0]->send_torque(0);
        owner->_ctx.rudomni_config.motor.wheel[1]->send_torque(0);
        owner->_ctx.rudomni_config.motor.wheel[2]->send_torque(0);
        owner->_ctx.rudomni_config.motor.wheel[3]->send_torque(0);
        owner->_ctx.rudomni_config.motor.yaw->send_torque(0);
    }

    void rudomni_chassis_t::state_passive_t::exit(owner *owner)
    {   

    }

}