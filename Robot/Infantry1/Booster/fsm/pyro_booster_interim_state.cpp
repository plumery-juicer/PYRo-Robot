#include "pyro_dwt_drv.h"
#include "pyro_booster.h"


namespace pyro
{
void booster_t::fsm_active_t::state_interim_t::enter(owner *owner)
{
}

void booster_t::fsm_active_t::state_interim_t::execute(owner *owner)
{

    if (abs(owner->_ctx.data.current_fric_mps[0] -
            owner->_ctx.data.target_fric_mps[0]) < 0.5f &&
        abs(owner->_ctx.data.current_fric_mps[1] -
            owner->_ctx.data.target_fric_mps[1]) < 0.5f)
    {
        request_switch(&owner->_state_active._ready_state);
    }
    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void booster_t::fsm_active_t::state_interim_t::exit(owner *owner)
{
}
} // namespace pyro