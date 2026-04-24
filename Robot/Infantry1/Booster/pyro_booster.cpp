#include "pyro_booster.h"
#include "pyro_algo_common.h"
#include "pyro_com_canrx.h"
#include "pyro_dwt_drv.h"
#include <cmath>

float bulletspeed;

namespace pyro
{

booster_t::booster_t() : module_base_t("booster",512,512,task_base_t::priority_t::HIGH)
{
    _ctx = {};
    _ctx.data.current_trig_position=0;
}

status_t booster_t::_init()
{

    _ctx.booster_cfg = _module_deps;
    // 3. 弹速控制初始化
    can_rx_drv_t::subscribe(can_hub_t::can1, 0x133);
    return PYRO_OK;
}

float booster_t::_normalize_angle(float angle)
{
    // 归一化到 [-PI, PI]
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

void booster_t::_update_feedback()
{
    // 1. 摩擦轮反馈
    for (int i = 0; i < 2; i++)
    {
        _ctx.booster_cfg.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_torque[i] = _ctx.booster_cfg.motor.fric_wheels[i]->get_current_torque();
    }
    _ctx.data.current_fric_mps[0] = _ctx.booster_cfg.motor.fric_wheels[0]->get_current_rotate();
    _ctx.data.current_fric_mps[1] = (0.4f*_ctx.booster_cfg.motor.fric_wheels[1]->get_current_rotate()+0.6f*_ctx.data.current_fric_mps[1]);
    // 2. 拨弹反馈
    _ctx.booster_cfg.motor.trigger_wheel->update_feedback();

    // --- A. 速度反馈 ---
    _ctx.data.current_trig_radps =
        _ctx.booster_cfg.motor.trigger_wheel->get_current_rotate();

    // --- B. 扭矩反馈 ---
    _ctx.data.current_trig_torque =
        _ctx.booster_cfg.motor.trigger_wheel->get_current_torque();

    // --- C. 角度反馈 (-PI ~ PI) ---
    
    _ctx.data.last_trig_wheelrad=_ctx.data.current_trig_wheelrad;
    _ctx.data.current_trig_wheelrad = _ctx.booster_cfg.motor.trigger_wheel->get_current_position();
    float delt_angle=_normalize_angle(_ctx.data.current_trig_wheelrad- _ctx.data.last_trig_wheelrad)/36.0f;
    _ctx.data.current_trig_position=_normalize_angle(_ctx.data.current_trig_position+delt_angle);

}


void booster_t::_speed_control()
{
    std::array<uint8_t, 8> raw_data{};

    // 仅在成功接收到新弹速的这一帧，才进行闭环计算
    if (can_rx_drv_t::get_data(pyro::can_hub_t::can1, 0x133, raw_data))
    {
        // 1. 更新弹速历史数据
        _ctx.shoot_data.ball_speed[2] = _ctx.shoot_data.ball_speed[1];
        _ctx.shoot_data.ball_speed[1] = _ctx.shoot_data.ball_speed[0];
        _ctx.shoot_data.ball_speed[0] =(raw_data[0]+raw_data[1]/100.0f);
        bulletspeed=_ctx.shoot_data.ball_speed[0];
        for (int i = 0; i < 3; i++)
        {
            if (_ctx.shoot_data.ball_speed[i] == 0.0f)
            {
                _ctx.shoot_data.ball_speed[i] = _ctx.cmd->target_speed;
            }
        }

        // 2. 确保目标弹速有效，避免启动时出现误动作
        if (_ctx.cmd->target_speed > 7.5f)
        {
            // --- A. 定义近期弹速的权重 ---
            // 越新的弹速参考价值越大
            constexpr float w0 = 0.72f; // 最新一发
            constexpr float w1 = 0.21f; // 上一发
            constexpr float w2 = 0.07f; // 上上发

            // --- B. 计算带符号的均方误差 ---
            float e0 = _ctx.shoot_data.ball_speed[0] - _ctx.cmd->target_speed;
            float e1 = _ctx.shoot_data.ball_speed[1] - _ctx.cmd->target_speed;
            float e2 = _ctx.shoot_data.ball_speed[2] - _ctx.cmd->target_speed;

            // 采用 e * |e| 保留误差方向 (加速或减速)
            float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                        (w1 * e1 * std::abs(e1)) +
                                        (w2 * e2 * std::abs(e2));

            // --- C. PID 计算速度增量 ---
            // 由于 signed_weighted_mse 本身已经是误差值，直接将其作为
            // target，current 设为 0
            float speed_increment =
                _ctx.booster_cfg.pid.ball_speed_pid->calculate(0.0f, signed_weighted_mse);

            // --- D. 累加到 fric1 的基础转速上 ---
        /*    _ctx.shoot_data.fric1_mps += speed_increment;
            _ctx.shoot_data.fric1_mps -= speed_increment;
            // --- E. 安全限幅 (非常重要) ---
            // 避免闭环异常导致单侧摩擦轮转速过高或过低，导致卡弹或弹道严重偏斜
            // 这里的限幅值请根据你实际的摩擦轮物理极限进行调整
            constexpr float MAX_FRIC1_MPS = 750.0f;
            constexpr float MIN_FRIC1_MPS = 600.0f;

            if (_ctx.shoot_data.fric1_mps > MAX_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MAX_FRIC1_MPS;
            }
            else if (_ctx.shoot_data.fric1_mps < MIN_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MIN_FRIC1_MPS;
            }

            if (_ctx.shoot_data.fric1_mps < -MAX_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = -MAX_FRIC1_MPS;
            }
            else if (_ctx.shoot_data.fric1_mps >- MIN_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = -MIN_FRIC1_MPS;
            }
        */
        }
    }
}


void booster_t::_fric_control()
{
    for (int i = 0; i < 2; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.booster_cfg.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void booster_t::_trigger_position_control()
{
    const float error = _ctx.data.target_trig_position - _ctx.data.current_trig_position;
    // 处理过零点问题，选择最短路径
    if (error > PI)
    {
        _ctx.data.target_trig_position -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        _ctx.data.target_trig_position += 2.0f * PI;
    }

    // 拨弹 PID 计算
    // 使用归一化后的 -PI~PI 角度进行控制
    _ctx.data.target_trig_radps = _ctx.booster_cfg.pid.trigger_pos_pid->calculate(
        _ctx.data.target_trig_position, _ctx.data.current_trig_position);

    _ctx.data.out_trig_torque = _ctx.booster_cfg.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void booster_t::_trigger_speed_control()
{
    _ctx.data.out_trig_torque = _ctx.booster_cfg.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void booster_t::_send_fric_command() const
{
   _ctx.booster_cfg.motor.fric_wheels[0]->send_torque(_ctx.data.out_fric_torque[0]);
   _ctx.booster_cfg.motor.fric_wheels[1]->send_torque(_ctx.data.out_fric_torque[1]);
}

void booster_t::_send_trigger_command() const
{
    _ctx.booster_cfg.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

booster_t::booster_ctx_t booster_t::get_ctx() const
{
    return _ctx;
}

void booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}


} // namespace pyro