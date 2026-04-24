#include "pyro_booster.h"

namespace pyro
{

void booster_t::fsm_active_t::state_single_t::enter(owner *owner)
{
    owner->_ctx.data.target_trig_position=_normalize_angle(owner->_ctx.data.target_trig_position+PI/4.0f);
}

void booster_t::fsm_active_t::state_single_t::execute(owner *owner)
{
   
    owner->_trigger_position_control();
    owner->_send_trigger_command();
    if(fabsf(owner->_ctx.data.target_trig_position-owner->_ctx.data.target_trig_position)<0.1f)
    {
        request_switch(&owner->_state_active._ready_state);
    }
}

void booster_t::fsm_active_t::state_single_t::exit(owner *owner)
{
}
}