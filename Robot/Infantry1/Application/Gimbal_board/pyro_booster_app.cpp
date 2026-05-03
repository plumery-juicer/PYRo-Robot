#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_infantry1_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_infantry1_gimbal.h"

using namespace pyro;

extern float read_time;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_ON = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE_CONTINUE  = (1 << 1);
constexpr uint32_t EVENT_BIT_FIRE_UP        = (1 << 2);
constexpr uint32_t EVENT_BIT_FIRE_SINGLE  = (1 << 3);

static TaskHandle_t booster_task_handle               = nullptr;
static pyro::booster_t *booster_ptr              = nullptr;
static pyro::booster_cmd_t *booster_cmd_ptr           = nullptr;
static pyro::booster_cfg_t *booster_cfg_ptr           = nullptr;
static void deps_init();
uint16_t firg_time=0;

float data3=0;

extern "C"
{
    

    void booster_dr162cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
            booster_cmd_ptr->fric_on     = false;
            // 移除手动清零，交由底层状态机自动同步处理
            return;
        }

        booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
        booster_cmd_ptr->target_speed = 11.5f; // 可调节

        // 根据通知处理摩擦轮翻转
        if (notify_val & EVENT_BIT_FRIC_ON)
        {
            booster_cmd_ptr->fric_on = !booster_cmd_ptr->fric_on;
        }

        // 仅在手动模式（中档）下响应射击
        if (pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
        {
            if (notify_val & EVENT_BIT_FIRE_UP)
            {
                // [修改] 发射脉冲触发时，计数器自增
                booster_cmd_ptr->fire_count++;
            }
        }
    }

    void booster_vt032cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
        { 
            booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
            booster_cmd_ptr->fric_on     = false;
            return;
        }

        booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
        booster_cmd_ptr->target_speed = 23.5f; // 可调节


        if (notify_val & EVENT_BIT_FRIC_ON)
        {
            booster_cmd_ptr->fric_on = !booster_cmd_ptr->fric_on;
        }

        
        if(notify_val & EVENT_BIT_FIRE_SINGLE)
            booster_cmd_ptr->fire_count++;

        if (notify_val & EVENT_BIT_FIRE_CONTINUE)
            booster_cmd_ptr->continue_on=true;
        
        if(notify_val & EVENT_BIT_FIRE_UP)
            booster_cmd_ptr->continue_on=false;
    }

    void infantry1_booster_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            //booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;

            //if(notify_val&EVENT_BIT_FIRE) data3++;
            if (vt03_drv_t::instance().check_online())
            {
                booster_vt032cmd(notify_val);
            }
            else if (dr16_drv_t::instance().check_online())
            {
                //booster_dr162cmd(notify_val);
            }
            booster_ptr->set_command(*booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void infantry1_booster_init(void *argument)
    {
        booster_ptr     = pyro::booster_t::instance();
        booster_cmd_ptr = new pyro::booster_cmd_t();

        deps_init();
        booster_ptr->configure(*booster_cfg_ptr);
        

        xTaskCreate(infantry1_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC_ON);
       // pyro::btn_broker::subscribe(&vrc.keys.q, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FIRE_SINGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::LONG_PRESS_START, booster_task_handle, EVENT_BIT_FIRE_CONTINUE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_UP, booster_task_handle, EVENT_BIT_FIRE_UP);
        //pyro::btn_broker::subscribe(&vrc.buttons.press_l, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FIRE);

        // --- DR16 拨杆绑定 ---
       // pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID, booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
       // pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID, booster_task_handle, EVENT_BIT_FIRE);

        booster_ptr->start();
        vTaskDelete(nullptr);
    }
}

void deps_init()
{
    booster_cfg_ptr = new pyro::booster_cfg_t();
    booster_cfg_ptr->motor.fric_wheels[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can2); // Fric l
    booster_cfg_ptr->motor.fric_wheels[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can2); // Fric r
    booster_cfg_ptr->motor.trigger_wheel =
        new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,pyro::can_hub_t::can1);


    booster_cfg_ptr->pid.fric_pid[0] =
        new pid_t(0.79f, 0.7f, 0.003f, 10.0f, 20);
    booster_cfg_ptr->pid.fric_pid[1] =
        new pid_t(0.45f, 0.55f, 0.003f, 10.0f, 20);

    booster_cfg_ptr->pid.trigger_pos_pid =
        new pid_t(1500.0f, 0.0f, 0.0f, 10.0f, 500.0f);
    booster_cfg_ptr->pid.trigger_spd_pid =
        new pid_t(0.038f, 0.01f, 0.0001f, 5.2f, 10.0f);

    booster_cfg_ptr->pid.ball_speed_pid =
        new pid_t(0.0005f, 0.0f, 0.0f, 0.0f, 20.0f);
}
