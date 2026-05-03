#include "pyro_booster.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void booster_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.booster_cfg.motor.trigger_wheel->enable();
    owner->_ctx.booster_cfg.motor.fric_wheels[0]->enable();
    owner->_ctx.booster_cfg.motor.fric_wheels[1]->enable();
    change_state(&_ready_state);
}

void booster_t::fsm_active_t::on_execute(owner *owner)
{
    owner->_speed_control();
    if (owner->_ctx.cmd->fric_on)
    {
        owner->_ctx.data.target_fric_mps[0] = owner->_ctx.shoot_data.fric1_mps; 
        owner->_ctx.data.target_fric_mps[1] = owner->_ctx.shoot_data.fric2_mps;
        owner->_fric_control();      
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;

        owner->_fric_control();
        if(fabsf(owner->_ctx.data.current_fric_mps[0])<60.0f)
        {
            owner->_ctx.data.out_fric_torque[0]=0.0;
        }
        if(fabsf(owner->_ctx.data.current_fric_mps[1])<60.0f)
        {
            owner->_ctx.data.out_fric_torque[1]=0.0;
        }
    }
    owner->_send_fric_command();
}

void booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro