#include "pyro_dwt_drv.h"
#include "pyro_booster.h"


namespace pyro
{
void booster_t::fsm_active_t::state_continue_t::enter(owner *owner)
{
     owner->_ctx.data.target_trig_radps = owner->_ctx.data.target_trig_continue_radps; // 
}

void booster_t::fsm_active_t::state_continue_t::execute(owner *owner)
{
   
    owner->_trigger_speed_control();
    owner->_send_trigger_command();
    if(!owner->_ctx.cmd->continue_on)
        request_switch(&owner->_state_active._ready_state);
}

void booster_t::fsm_active_t::state_continue_t::exit(owner *owner)
{
    owner->_ctx.data.target_trig_position   = owner->_ctx.data.current_trig_position;
    owner->_ctx.data.target_trig_radps = 0.0f;
}
} // namespace pyro