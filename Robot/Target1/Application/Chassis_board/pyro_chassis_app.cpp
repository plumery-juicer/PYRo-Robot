#include "pyro_core_config.h"


#if ROBOT_ID == TARGET_ID

#include "pyro_module_base.h"
#include "pyro_rudomni_chassis.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_ins.h"

using namespace pyro;

rudomni_chassis_t *rudomni_chassis_ptr             = nullptr;
rudomni_cmd_t *rudomni_cmd_ptr                     = nullptr;
rudomni_cfg_t *rudomni_cfg_ptr                     = nullptr;
pyro::ins_drv_t *ins_drive;


float rcdata_fliter(float data)
{
    if(fabsf(data) < 0.1f)
        return 0.0f;
    else
        return data;
}

void chassis_config(rudomni_cfg_t &rudomni_cfg_ptr)
{
    ins_drive = pyro::ins_drv_t::get_instance();

    rudomni_cfg_ptr.motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                    can_hub_t::can1); // 
    rudomni_cfg_ptr.motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                    can_hub_t::can1); // 

    rudomni_cfg_ptr.motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can1); // FL Wheel
    rudomni_cfg_ptr.motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // BL Wheel
    rudomni_cfg_ptr.motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Wheel
    rudomni_cfg_ptr.motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can1); // FR Wheel

    rudomni_cfg_ptr.motor.yaw =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                    can_hub_t::can2); // Yaw Motor

    for (int i = 0; i < 4; i++)
        rudomni_cfg_ptr.pid.wheel_pid[i] =
            new pid_t(0.2f, 0.1f, 0.00f, 1.00f, 20.0f);

    
    rudomni_cfg_ptr.pid.rud_pos_pid[0] =
            new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudomni_cfg_ptr.pid.rud_spd_pid[0]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);

    rudomni_cfg_ptr.pid.rud_pos_pid[1] =
            new pid_t(18.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudomni_cfg_ptr.pid.rud_spd_pid[1]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);

    rudomni_cfg_ptr.pid.yaw_pos_pid =
        new pid_t(32.8f, 0.1f, 0.0f, 1.0f, 50.0f);
    rudomni_cfg_ptr.pid.yaw_spd_pid =
        new pid_t(0.1f, 0.00001f, 0.0000003f, 10.0f, 20.0f);
    rudomni_cfg_ptr.rud_pos_moving_offset[0] = -0.662679672f;
    rudomni_cfg_ptr.rud_pos_moving_offset[1] = 2.95061207f;
    rudomni_cfg_ptr.rud_pos_moving_offset[2] = -2.12686443f;
}

extern "C"
{
    void chassis_rccmd()
    {
            
         // 读取遥控器数据
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if(pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            rudomni_cmd_ptr->mode       = cmd_base_t::mode_t::PASSIVE;
            rudomni_cmd_ptr->follow_yaw = false;
            rudomni_cmd_ptr->timestamp  = 0;
            rudomni_cmd_ptr->vx         = 0.0f;
            rudomni_cmd_ptr->vy         = 0.0f;
            rudomni_cmd_ptr->wz         = 0.0f;
            rudomni_cmd_ptr->wz2        = 0.0f;
        }
        else if(pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
        {
            rudomni_cmd_ptr->mode       = cmd_base_t::mode_t::ACTIVE;
            rudomni_cmd_ptr->follow_yaw = false;
            rudomni_cmd_ptr->timestamp  = 0;
            rudomni_cmd_ptr->vx         = rcdata_fliter(vrc.axes.lx*2.0f);
            rudomni_cmd_ptr->vy         = rcdata_fliter(vrc.axes.ly * 2.0f);
            rudomni_cmd_ptr->wz         = rcdata_fliter(-vrc.axes.rx * 2.0f);
            rudomni_cmd_ptr->wz2        = rcdata_fliter(vrc.axes.ry *0.5f)*0.02f;
        }
        else if(pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            rudomni_cmd_ptr->mode       = cmd_base_t::mode_t::ACTIVE;
            rudomni_cmd_ptr->follow_yaw = true;
            rudomni_cmd_ptr->timestamp  = 0;
            rudomni_cmd_ptr->vx         = rcdata_fliter(vrc.axes.lx*2.0f);
            rudomni_cmd_ptr->vy         = rcdata_fliter(vrc.axes.ly * 2.0f);
            rudomni_cmd_ptr->wz         = rcdata_fliter(-vrc.axes.rx * 2.0f);
            rudomni_cmd_ptr->wz2        = 0.0f;
        }
        if(pyro::sw_pos_t::MID == vrc.switches.left.current_pos)
        {
            rudomni_cmd_ptr->follow_yaw = false;
            rudomni_cmd_ptr->wz        = 2.0f;
        }
        else if(pyro::sw_pos_t::DOWN == vrc.switches.left.current_pos)
        {
            rudomni_cmd_ptr->follow_yaw = false;
            rudomni_cmd_ptr->wz        = -2.0f;
        }
        if(rudomni_cmd_ptr->follow_yaw==false)
        {
            ins_drive->get_rads_b(&(rudomni_chassis_ptr->_yaw_data.current_insyaw), 
                            &(rudomni_chassis_ptr->_yaw_data.current_inspitch), 
                             &(rudomni_chassis_ptr->_yaw_data.current_insroll));
        }
    }

    void target_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rccmd();
            rudomni_chassis_ptr->set_command(*rudomni_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void target_chassis_init(void *argument)
    {
        rudomni_cmd_ptr = new rudomni_cmd_t();
        rudomni_cfg_ptr = new rudomni_cfg_t();  
        
        rudomni_chassis_ptr = rudomni_chassis_t::instance();
        chassis_config(*rudomni_cfg_ptr);
        rudomni_chassis_ptr->configure(*rudomni_cfg_ptr);
        rudomni_chassis_ptr->start();
        xTaskCreate(target_chassis_thread, "Target Chassis Thread", 512,
             nullptr, configMAX_PRIORITIES-1, nullptr);
        vTaskDelete(nullptr);
    }
}

#endif


