#include "pyro_infantry1_gimbal.h"
#include "pyro_core_def.h"
#include "pyro_ins.h"
#include "pyro_com_canrx.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"

#include <algorithm>

float debugbuf_fire1;
float debugbuf_fire11;
float debugbuf_fire2;
float debugbuf_fire22;
float debugbuf1[4];

namespace pyro
{
#define minpitch_rad (-0.11f) //pitch电机的反馈角度于pitch轴的实际角度相反，因此所谓min是指电机角度的min
#define maxpitch_rad (0.205f)

#define imu_pitch_min (-0.583625f)
#define imu_pitch_max (0.6967f)


// =========================================================
// 构造与初始化
// =========================================================

infantry1_gimbal_t::infantry1_gimbal_t() 
: module_base_t("infantry1_gimbal",512,512,task_base_t::priority_t::HIGH)
{
    _ctx = {};
    init_fleg=false;
}

status_t infantry1_gimbal_t::_init()
{
    _ctx.gimbal_cfg_t = _module_deps;
    _ctx.data = {};
    return PYRO_OK;
}

// =========================================================
// 核心循环回调
// =========================================================

void infantry1_gimbal_t::_update_feedback()
{
    _ctx.gimbal_cfg_t.motor.pitch->update_feedback();
    _ctx.gimbal_cfg_t.motor.yaw->update_feedback();

    // =========================================================
    // 处理 Pitch 增量式电机过零点套圈逻辑
    // =========================================================
    _ctx.data.current_pitch_rad= _ctx.gimbal_cfg_t.motor.pitch->get_current_position()+_ctx.gimbal_cfg_t.pitch_pos_offset;
    _ctx.data.current_pitch_radps= _ctx.gimbal_cfg_t.motor.pitch->get_current_rotate();

    _ctx.data.current_yaw_rad= _ctx.gimbal_cfg_t.motor.yaw->get_current_position()+_ctx.gimbal_cfg_t.yaw_pos_offset;
    _ctx.data.current_yaw_radps= _ctx.gimbal_cfg_t.motor.yaw->get_current_rotate()*0.4f+_ctx.data.current_yaw_radps*0.6f;

    if (_ctx.data.current_pitch_rad > PI)
    {
        _ctx.data.current_pitch_rad -= 2.0f * PI;
    }
    else if (_ctx.data.current_pitch_rad < -PI)
    {
        _ctx.data.current_pitch_rad += 2.0f * PI;
    }

    if (_ctx.data.current_yaw_rad > PI)
    {
        _ctx.data.current_yaw_rad -= 2.0f * PI;
    }
    else if (_ctx.data.current_yaw_rad < -PI)
    {
        _ctx.data.current_yaw_rad += 2.0f * PI;
    }

    // 读取 IMU 数据作为云台姿态反馈
    float _pitch_rad;
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.yaw_imu_rad,
                                          &_pitch_rad,
                                          &_ctx.data.roll_imu_rad);
    _ctx.data.pitch_imu_rad=_ctx.data.pitch_imu_rad*0.3f+_pitch_rad*0.7f;

    _communicate_chassis();

    
}

void infantry1_gimbal_t::yaw_angle_ch(float current_angle, float& target_angle)
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

float pyro::infantry1_gimbal_t::imu2motor_pitch(float imu_pitch)
{
    float motor_pitch = -0.2473404331f*imu_pitch+0.066523596732f; // pitch轴电机与imu的转换关系；
    return motor_pitch;
}

float pyro::infantry1_gimbal_t::motor2imu_pitch(float motor_pitch)
{
    float imu_pitch = (motor_pitch-0.066523596732f)/(-0.2473404331f); // pitch轴电机与imu的转换关系；
    return imu_pitch;
}

void infantry1_gimbal_t::_gimbal_control()
{
    if(!init_fleg) {
        _ctx.data.target_pitch_rad=_ctx.data.current_pitch_rad;
        _ctx.data.target_imupitch_rad=_ctx.data.pitch_imu_rad;
        init_fleg=true;
    }
    _ctx.data.target_imuyaw_rad += _ctx.cmd->yaw_angle;

    _ctx.data.target_imupitch_rad += _ctx.cmd->pitch_angle;

    if (_ctx.data.target_imupitch_rad > imu_pitch_max)
    {
        _ctx.data.target_imupitch_rad = imu_pitch_max;
    }
    else if (_ctx.data.target_imupitch_rad < imu_pitch_min)
    {
        _ctx.data.target_imupitch_rad = imu_pitch_min;
    }

    _ctx.data.target_pitch_rad=imu2motor_pitch(_ctx.data.target_imupitch_rad+motor2imu_pitch(_ctx.data.current_pitch_rad)-_ctx.data.pitch_imu_rad);
    debugbuf1[0]=2.0f*_ctx.data.target_imupitch_rad-_ctx.data.pitch_imu_rad;
    if (_ctx.data.target_imuyaw_rad > PI)
    {
        _ctx.data.target_imuyaw_rad -= 2.0f * PI;
    }
    else if (_ctx.data.target_imuyaw_rad < -PI)
    {
        _ctx.data.target_imuyaw_rad += 2.0f * PI;
    }

    if (_ctx.data.target_pitch_rad > maxpitch_rad)
    {
        _ctx.data.target_pitch_rad = maxpitch_rad;
    }
    else if (_ctx.data.target_pitch_rad < minpitch_rad)
    {
        _ctx.data.target_pitch_rad = minpitch_rad;
    }

    yaw_angle_ch(_ctx.data.yaw_imu_rad, _ctx.data.target_imuyaw_rad);
    yaw_angle_ch(_ctx.data.pitch_imu_rad, _ctx.data.target_pitch_rad);

    float pitch_pos_output,yaw_pos_output;
    pitch_pos_output=_ctx.gimbal_cfg_t.pid.pitch_pos->calculate(
        _ctx.data.target_pitch_rad, _ctx.data.current_pitch_rad);

    _ctx.data.out_pitch_torque=_ctx.gimbal_cfg_t.pid.pitch_spd->calculate(
        pitch_pos_output, _ctx.data.current_pitch_radps);

    yaw_pos_output=_ctx.gimbal_cfg_t.pid.yaw_pos->calculate(
        _ctx.data.target_imuyaw_rad, _ctx.data.yaw_imu_rad);
    _ctx.data.out_yaw_torque=_ctx.gimbal_cfg_t.pid.yaw_spd->calculate(
        yaw_pos_output, _ctx.data.current_yaw_radps);

}

infantry1_gimbal_t::gimbal_ctx_t infantry1_gimbal_t::get_ctx() const
{
    return _ctx;
}

void infantry1_gimbal_t::_send_motor_command(gimbal_ctx_t *ctx)
{

    ctx->gimbal_cfg_t.motor.pitch->send_mit_ctrl(ctx->data.target_pitch_rad, 0, 0);
    ctx->gimbal_cfg_t.motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void infantry1_gimbal_t::_communicate_chassis()
{
    std::array<uint8_t, 8> raw_data{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x103, raw_data))
    {
        //auto *src = reinterpret_cast<int16_t *>(raw_data.data());
    }
}

void infantry1_gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if(cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
    _main_fsm.change_state(&_state_passive);
    else if(cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
    _main_fsm.change_state(&_state_active);
    _main_fsm.execute(this);
}

float infantry1_gimbal_t::get_yaw()
{
    return _ctx.data.current_yaw_rad;
}

} // namespace pyro