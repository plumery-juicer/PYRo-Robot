#include "pyro_rudomni_chassis.h"
#include "pyro_core_def.h"
#include "pyro_module_base.h"
#include "pyro_task.h"


float debugbuf2[6];
namespace pyro 
{
    rudomni_chassis_t::rudomni_chassis_t() 
    : module_base_t("rudomni",512,512,task_base_t::priority_t::HIGH)
    {
        _ctx.data  = {};
        _ctx.data.current_states.modules[0].direction = -1;
        _ctx.data.current_states.modules[1].direction = 1;
        _ctx.data.current_states.modules[2].direction = -1;
        _ctx.data.current_states.modules[3].direction = -1;
    }
    status_t rudomni_chassis_t::_init()
    {
        _kinematics = new rudomni_kin_t(0.183,0.168,0.178,0.164);
        _ctx.rudomni_config = _module_deps;
        return PYRO_OK;
    }

    void rudomni_chassis_t::yaw_angle_ch(float current_angle, float& target_angle)
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

    void rudomni_chassis_t::_update_feedback()
    {
        _ctx.rudomni_config.motor.rudder[0]->update_feedback();
        _ctx.rudomni_config.motor.rudder[1]->update_feedback();
        _ctx.rudomni_config.motor.wheel[0]->update_feedback();
        _ctx.rudomni_config.motor.wheel[1]->update_feedback();
        _ctx.rudomni_config.motor.wheel[2]->update_feedback();
        _ctx.rudomni_config.motor.wheel[3]->update_feedback();
        _ctx.rudomni_config.motor.yaw->update_feedback();

        // 1. 两个舵机的角度和角速度
        // 舵机当前角度（-PI ~ PI）
        _ctx.data.current_states.modules[2].angle =
            _ctx.rudomni_config.motor.rudder[0]->get_current_position() - _ctx.rudomni_config.rud_pos_moving_offset[0];
        _ctx.data.current_states.modules[3].angle =
            _ctx.rudomni_config.motor.rudder[1]->get_current_position() - _ctx.rudomni_config.rud_pos_moving_offset[1];
            debugbuf2[2] =  _ctx.rudomni_config.motor.rudder[0]->get_current_position();
            debugbuf2[3] =  _ctx.rudomni_config.motor.rudder[1]->get_current_position();
        for (int i = 0; i < 2; i++)
        {
            if (_ctx.data.current_states.modules[i+2].angle > PI)
                _ctx.data.current_states.modules[i+2].angle -= 2 * PI;
            else if (_ctx.data.current_states.modules[i+2].angle < -PI)
                _ctx.data.current_states.modules[i+2].angle += 2 * PI;
        }
        //yaw当前速度角度
        _yaw_data.current_yaw_rads = _ctx.rudomni_config.motor.yaw->get_current_rotate();
        debugbuf2[4] = _ctx.rudomni_config.motor.yaw->get_current_position();
        _yaw_data.current_yaw_pos = _ctx.rudomni_config.motor.yaw->get_current_position() - _ctx.rudomni_config.rud_pos_moving_offset[2];
        if(_yaw_data.current_yaw_pos > PI)
            _yaw_data.current_yaw_pos -= 2 * PI;
        else if(_yaw_data.current_yaw_pos < -PI)
            _yaw_data.current_yaw_pos += 2 * PI;
        // 舵机当前角速度
        _ctx.data.current_rud_radps[0] =
            _ctx.rudomni_config.motor.rudder[0]->get_current_rotate();
        _ctx.data.current_rud_radps[1] =
            _ctx.rudomni_config.motor.rudder[1]->get_current_rotate();

        // 2. 四个轮子的转速
        for (int i = 0; i < 4; i++)
            _ctx.data.current_states.modules[i].speed =
                _ctx.rudomni_config.motor.wheel[i]->get_current_rotate();
    }

    void rudomni_chassis_t::_kinematics_solve()
    {
        float vy2=_ctx.cmd->vy,vx2=_ctx.cmd->vx;

        _ctx.cmd->vx = vx2 * cos(_yaw_data.current_yaw_pos) - vy2 * sin(_yaw_data.current_yaw_pos);
        _ctx.cmd->vy = vx2 * sin(_yaw_data.current_yaw_pos) + vy2 * cos(_yaw_data.current_yaw_pos);

        if(_ctx.cmd->follow_yaw == false)
        {
            _yaw_data.target_insyaw += _ctx.cmd->wz2;
           rudomni_chassis_t::yaw_angle_ch(_yaw_data.current_insyaw, _yaw_data.target_insyaw);
        } 

        _ctx.data.target_states = _kinematics->solve(
            _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.data.current_states);
    }

    void rudomni_chassis_t::_chassis_control(rudomni_ctx_t *ctx, yaw_data_t *yaw_data)
    {
        for (int i = 0; i < 2; i++)
        {
            // 舵机位置环
            const float rud_pos_output =
                ctx->rudomni_config.pid.rud_pos_pid[i]->calculate(
                    ctx->data.target_states.modules[i+2].angle,
                    ctx->data.current_states.modules[i+2].angle);
            debugbuf2[i] = ctx->data.target_states.modules[i+2].angle;
            // 舵机速度环
            ctx->data.out_rud_torque[i] =
                ctx->rudomni_config.pid.rud_spd_pid[i]->calculate(
                    rud_pos_output, ctx->data.current_rud_radps[i]);
        }
        for(int i = 0; i < 4; i++)
        {
            // 轮子速度环
            ctx->data.out_wheel_torque[i] =
                ctx->rudomni_config.pid.wheel_pid[i]->calculate(
                    ctx->data.target_states.modules[i].speed,
                    ctx->data.current_states.modules[i].speed);
        }

        if(ctx->cmd->follow_yaw == false)
        {
            yaw_data->target_yaw_rads =
            ctx->rudomni_config.pid.yaw_pos_pid->calculate(
                yaw_data->target_insyaw, yaw_data->current_insyaw);
            yaw_data->out_yaw_torque =
            ctx->rudomni_config.pid.yaw_spd_pid->calculate(
                yaw_data->target_yaw_rads, yaw_data->current_yaw_rads);
        }
        else
        {
           yaw_data->target_yaw_rads =
            ctx->rudomni_config.pid.yaw_pos_pid->calculate(
                0.0f, yaw_data->current_yaw_pos);
            yaw_data->out_yaw_torque =
            ctx->rudomni_config.pid.yaw_spd_pid->calculate(
                yaw_data->target_yaw_rads, yaw_data->current_yaw_rads);
        }

    }

    void rudomni_chassis_t::_send_motor_command(rudomni_ctx_t *ctx, yaw_data_t *yaw_data)
    {
        // 发送舵机扭矩命令
       
            ctx->rudomni_config.motor.rudder[0]->send_torque(
                ctx->data.out_rud_torque[0]);
            ctx->rudomni_config.motor.rudder[1]->send_torque(
                ctx->data.out_rud_torque[1]);

        // 发送轮子扭矩命令
        for (int i = 0; i < 4; i++)
        {
            ctx->rudomni_config.motor.wheel[i]->send_torque(
                ctx->data.out_wheel_torque[i]);
        }

        ctx->rudomni_config.motor.yaw->send_torque(yaw_data->out_yaw_torque);
    }

    void rudomni_chassis_t::_fsm_execute()
    {
        _ctx.cmd = &_current_cmd;
        if(cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
            _main_fsm.change_state(&_state_passive);
        else if(cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
            _main_fsm.change_state(&_state_active);
       _main_fsm.execute(this);
    }

}