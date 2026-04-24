#include "pyro_rudder_chassis.h"
#include "pyro_core_def.h"
#include "pyro_module_base.h"
#include "pyro_task.h"


float debugbuf2[6];
namespace pyro 
{
    rudder_chassis_t::rudder_chassis_t() 
    : module_base_t("rudder",512,512,task_base_t::priority_t::HIGH)
    {
        _ctx.data  = {};
    }
    status_t rudder_chassis_t::_init()
    {
        _kinematics = new rudder_kin_t(0.36,0.36);
        _ctx.rudder_config = _module_deps;
        return PYRO_OK;
    }

    void rudder_chassis_t::yaw_angle_ch(float current_angle, float& target_angle)
    {
        float angle_diff = target_angle - current_angle;
    
        while (angle_diff > PI)
            angle_diff -= 2.0f * PI;
        while (angle_diff < -PI)
            angle_diff += 2.0f * PI;

        if(fabsf(angle_diff) > PI / 2)
        {
            target_angle += PI;
        while (target_angle > PI)
            target_angle -= 2.0f * PI;
        while (target_angle < -PI)
            target_angle += 2.0f * PI;
        }

        angle_diff = target_angle - current_angle;
    
        while (angle_diff > PI)
            angle_diff -= 2.0f * PI;
        while (angle_diff < -PI)
            angle_diff += 2.0f * PI;

        target_angle = current_angle + angle_diff;
    }

    void rudder_chassis_t::_update_feedback()
    {
        _ctx.rudder_config.motor.rudder[0]->update_feedback();
        _ctx.rudder_config.motor.rudder[1]->update_feedback();
        _ctx.rudder_config.motor.wheel[0]->update_feedback();
        _ctx.rudder_config.motor.wheel[1]->update_feedback();
        _ctx.rudder_config.motor.wheel[2]->update_feedback();
        _ctx.rudder_config.motor.wheel[3]->update_feedback();
    

        // 1. 两个舵机的角度和角速度
        // 舵机当前角度（-PI ~ PI）
        for(int i = 0; i < 4; i++)
        {
            _ctx.data.current_states.modules[i].angle =
                _ctx.rudder_config.motor.rudder[i]->get_current_position() - _ctx.rudder_config.rud_pos_moving_offset[i];
            _ctx.data.current_rud_radps[i] =
            _ctx.rudder_config.motor.rudder[i]->get_current_rotate();
           // debugbuf2[i+2] =  _ctx.rudder_config.motor.rudder[i]->get_current_position();
        }
        for (int i = 0; i < 4; i++)
        {
            if (_ctx.data.current_states.modules[i].angle > PI)
                _ctx.data.current_states.modules[i].angle -= 2 * PI;
            else if (_ctx.data.current_states.modules[i].angle < -PI)
                _ctx.data.current_states.modules[i].angle += 2 * PI;
        }

        // 2. 四个轮子的转速
        for (int i = 0; i < 4; i++)
            _ctx.data.current_states.modules[i].speed =
                _ctx.rudder_config.motor.wheel[i]->get_current_rotate();
    }

    void rudder_chassis_t::_kinematics_solve()
    {

        _ctx.data.target_states = _kinematics->solve(
            _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.data.current_states);
    }

    void rudder_chassis_t::_chassis_control(rudder_ctx_t *ctx)
    {
        for (int i = 0; i < 4; i++)
        {
            // 舵机位置环
            const float rud_pos_output =
                ctx->rudder_config.pid.rud_pos_pid[i]->calculate(
                    ctx->data.target_states.modules[i].angle,
                    ctx->data.current_states.modules[i].angle);
            //debugbuf2[i] = ctx->data.target_states.modules[i].angle;
            // 舵机速度环
            ctx->data.out_rud_torque[i] =
                ctx->rudder_config.pid.rud_spd_pid[i]->calculate(
                    rud_pos_output, ctx->data.current_rud_radps[i]);
        }
        for(int i = 0; i < 4; i++)
        {
            // 轮子速度环
            ctx->data.out_wheel_torque[i] =
                ctx->rudder_config.pid.wheel_pid[i]->calculate(
                    ctx->data.target_states.modules[i].speed,
                    ctx->data.current_states.modules[i].speed);
        }

    }

    void rudder_chassis_t::_send_motor_command(rudder_ctx_t *ctx)
    {
        // 发送舵机扭矩命令
       
            ctx->rudder_config.motor.rudder[0]->send_torque(
                ctx->data.out_rud_torque[0]);
            ctx->rudder_config.motor.rudder[1]->send_torque(
                ctx->data.out_rud_torque[1]);
            ctx->rudder_config.motor.rudder[2]->send_torque(
                ctx->data.out_rud_torque[2]);
            ctx->rudder_config.motor.rudder[3]->send_torque(
                ctx->data.out_rud_torque[3]);

        // 发送轮子扭矩命令
        for (int i = 0; i < 4; i++)
        {
            ctx->rudder_config.motor.wheel[i]->send_torque(
                ctx->data.out_wheel_torque[i]);
        }
    }

    void rudder_chassis_t::_fsm_execute()
    {
        _ctx.cmd = &_current_cmd;
        if(cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
            _main_fsm.change_state(&_state_passive);
        else if(cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
            _main_fsm.change_state(&_state_active);
       _main_fsm.execute(this);
    }

}