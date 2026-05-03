#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_virtual_rc.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_infantry1_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include <cmath>

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_SPINNING_TOGGLE = (1 << 0);
constexpr uint32_t EVENT_BIT_LEG_TOGGLE   = (1 << 1);

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::infantry1_gimbal_t *infantry1_gimbal_ptr         = nullptr;
static pyro::infantry1_gimbal_cmd_t *infantry1_gimbal_cmd_ptr = nullptr;
static pyro::infantry1_gimbal_cfg_t *infantry1_gimbal_cfg_ptr   = nullptr;
static pyro::pid_t chassis_yaw_pid(0.8,0.2,0.05,0.2,1.0);

virtual_rc_t data1;
float data2;

static void gimbal_dr162cmd();
static void chassis_dr162cmd();
static void gimbal_vt032cmd();
static void chassis_vt032cmd(uint32_t notify_val);
static void gimbal_config();
static void chassis_gimbal_can();
//代办---根据底盘wz做云台的前馈控制。
extern "C"
{
    void infantry1_gimbal_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            chassis_gimbal_can();
            if (vt03_drv_t::instance().check_online())
            {
                chassis_vt032cmd(notify_val);
                gimbal_vt032cmd();
            }
            else if (dr16_drv_t::instance().check_online())
            {
               // chassis_dr162cmd();
               // gimbal_dr162cmd();
            }
            else
            {
                infantry1_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }
        
            infantry1_gimbal_ptr->set_command(*infantry1_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void infantry1_gimbal_init(void *argument)
    {
        infantry1_gimbal_cmd_ptr = new pyro::infantry1_gimbal_cmd_t();
        infantry1_gimbal_ptr     = pyro::infantry1_gimbal_t::instance();
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can1, 0x133);
    
        gimbal_config();
        infantry1_gimbal_ptr->configure(*infantry1_gimbal_cfg_ptr);
        infantry1_gimbal_ptr->start();

        xTaskCreate(infantry1_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        auto &vrc = pyro::rc_drv_t::read();
        pyro::btn_broker::subscribe(&vrc.buttons.pause, pyro::btn_event_t::PRESS_DOWN, gimbal_task_handle, EVENT_BIT_SPINNING_TOGGLE);
        vTaskDelete(nullptr);
    }
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        infantry1_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        infantry1_gimbal_cmd_ptr->pitch_angle = 0;
        infantry1_gimbal_cmd_ptr->yaw_angle   = 0;
        return;
    }
    infantry1_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    infantry1_gimbal_cmd_ptr->pitch_angle = -vrc.axes.ry * 0.03f;
    infantry1_gimbal_cmd_ptr->yaw_angle   = -vrc.axes.rx * 0.0035f;
}


void gimbal_vt032cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();
    data1=vrc;
    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        infantry1_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        infantry1_gimbal_cmd_ptr->pitch_angle = 0;
        infantry1_gimbal_cmd_ptr->yaw_angle   = 0;
        return;
    }
    else if (pyro::sw_pos_t::MID == vrc.switches.gear.current_pos)
    {
        infantry1_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
        infantry1_gimbal_cmd_ptr->pitch_angle = vrc.axes.ry * 0.002f+vrc.mouse_axes.y*0.8f;
        infantry1_gimbal_cmd_ptr->yaw_angle   = -vrc.axes.rx * 0.005f-vrc.mouse_axes.x*0.5f;
        return;
    }

}

void chassis_vt032cmd(uint32_t notify_val)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();
   static bool wz_schmit=false;

    float angle_yaw=infantry1_gimbal_ptr->get_yaw();

    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static bool active      = false;
    static bool spinning    = false;

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        vx        = 0;
        vy        = 0;
        wz        = 0;
        active    = false;
    }
    else if (pyro::sw_pos_t::MID == vrc.switches.gear.current_pos)
    {
        vx        = static_cast<int8_t>((vrc.axes.ly*cosf(angle_yaw)-vrc.axes.lx*sinf(angle_yaw))*127*0.5f);//无论现在云台和底盘是否角度对齐都以云台角度作为速度的基准
        vy        = static_cast<int8_t>((vrc.axes.lx*cosf(angle_yaw)+vrc.axes.ly*sinf(angle_yaw))*127*0.5f);
        wz        = static_cast<int8_t>(-chassis_yaw_pid.calculate(0.0f, infantry1_gimbal_ptr->get_yaw())*127);
        active    = true;
    }

    if(!wz_schmit){
        if(fabs(infantry1_gimbal_ptr->get_yaw())>0.3f)
        {
            wz_schmit=true;
        }
    }
    else if(wz_schmit){
        if(fabs(infantry1_gimbal_ptr->get_yaw())<0.1f)
        {
            wz_schmit=false;
            wz=0;
        }
    }

    if(notify_val==EVENT_BIT_SPINNING_TOGGLE)
    {
        spinning=!spinning;
    }

    if(spinning) wz=static_cast<int8_t>(80.0f);
    pyro::can_tx_drv_t::clear(0x123);


    pyro::can_tx_drv_t::add_data(0x123, 8, vx);
    pyro::can_tx_drv_t::add_data(0x123, 8, vy);
    pyro::can_tx_drv_t::add_data(0x123, 8, wz);
    pyro::can_tx_drv_t::add_data(0x123, 1, active);
    pyro::can_tx_drv_t::send(0x123,
                              pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                  pyro::can_hub_t::which_can::can1));
}

void chassis_gimbal_can()
{
    std::array<uint8_t, 8> raw_data{};
    if(can_rx_drv_t::get_data(can_hub_t::which_can::can1, 0x133, raw_data))
    {
        bullet_speed=raw_data[0]+raw_data[1]/100.0f;
        data2=bullet_speed;
    }
}

void gimbal_config()
{
    infantry1_gimbal_cfg_ptr = new pyro::infantry1_gimbal_cfg_t();
    
    infantry1_gimbal_cfg_ptr->motor.pitch =
        new dm_motor_drv_t(0x01, 0x00, can_hub_t::can2); // Pitch 轴使用 DM 电机
    infantry1_gimbal_cfg_ptr->motor.pitch->set_position_range(-PI, PI);
    infantry1_gimbal_cfg_ptr->motor.pitch->set_runtime_kp(10.0f);
    infantry1_gimbal_cfg_ptr->motor.pitch->set_runtime_kd(1.0f);
    infantry1_gimbal_cfg_ptr->motor.pitch->set_rotate_range(-30.0f, 30.0f);
    infantry1_gimbal_cfg_ptr->motor.pitch->set_torque_range(-10.0f, 10.0f);
    
    infantry1_gimbal_cfg_ptr->motor.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_5 , can_hub_t::can1);

    infantry1_gimbal_cfg_ptr->yaw_pos_offset =-1.00629139f;
    infantry1_gimbal_cfg_ptr->pitch_pos_offset = 0.0f;
    // 3. 初始化串级 PID
    infantry1_gimbal_cfg_ptr->pid.pitch_pos =
        new pid_t(30.0f, 0.2f, 0.05f, 10.0f, 45.0f); // 位置环输出为 rad/s，限制在电机可接受范围内
    infantry1_gimbal_cfg_ptr->pid.pitch_spd =
        new pid_t(1.0f, 0.1f, 0.00f, 1.0f, 10.0f); // 输出限制匹配电机 Nm 级

    infantry1_gimbal_cfg_ptr->pid.yaw_pos =
        new pid_t(60.2f, 0.0f, 2.5f, 10.0f, 80.0f);
    infantry1_gimbal_cfg_ptr->pid.yaw_spd =
        new pid_t(1.5f, 0.005f, 0.002f, 3.0f, 24.0f);
}