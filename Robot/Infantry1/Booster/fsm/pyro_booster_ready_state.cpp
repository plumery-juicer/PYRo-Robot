#include "pyro_dwt_drv.h"
#include "pyro_booster.h"

namespace pyro
{
void booster_t::fsm_active_t::state_ready_t::enter(owner *owner)
{
    owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
}

void booster_t::fsm_active_t::state_ready_t::execute(owner *owner)
{

    if(owner->_ctx.cmd->continue_on)
    {
        request_switch(&owner->_state_active._continue_state);
    }

    else if (owner->_ctx.cmd->fire_count != owner->_ctx.data.internal_fire_count)
    {
        owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
        request_switch(&owner->_state_active._single_state);
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void booster_t::fsm_active_t::state_ready_t::exit(owner *owner)
{

}
} // namespace pyro