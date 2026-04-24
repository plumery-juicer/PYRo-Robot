#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_rudder_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_com_cantx.h"

using namespace pyro;

rudder_chassis_t *rudder_chassis_ptr             = nullptr;
rudder_cmd_t *rudder_cmd_ptr                     = nullptr;
rudder_cfg_t *rudder_cfg_ptr                     = nullptr;

static void chassis_rxcmd();
static void chassis_dr162cmd();
static void chassis_config(rudder_cfg_t &rudder_cfg_ptr);


extern "C"
{
    void infantry1_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rxcmd();
            // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
            // chassis_dr162cmd();
            rudder_chassis_ptr->set_command(*rudder_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void infantry1_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can1, 0x101);
        rudder_cmd_ptr     = new pyro::rudder_cmd_t();
        rudder_cfg_ptr     = new pyro::rudder_cfg_t();

        rudder_chassis_ptr = pyro::rudder_chassis_t::instance();
        chassis_config(*rudder_cfg_ptr);
        rudder_chassis_ptr->configure(*rudder_cfg_ptr);
        rudder_chassis_ptr->start();

        xTaskCreate(infantry1_chassis_thread, "start_infantry1_chassis_thread", 128,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}

void chassis_rxcmd()
{
    std::array<uint8_t, 8> raw_data{};
    pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x101, raw_data);

    rudder_cmd_ptr->vx =
        2.5f * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
    rudder_cmd_ptr->vy =
        2.5f * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
    rudder_cmd_ptr->mode =
        static_cast<pyro::cmd_base_t::mode_t>(raw_data[3] & 0x01);

}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
    }
    else
    {
    }
}

void chassis_config(rudder_cfg_t &rudder_cfg_ptr)
{

    rudder_cfg_ptr.motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                    can_hub_t::can1); // 
    rudder_cfg_ptr.motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                    can_hub_t::can1); // 
    rudder_cfg_ptr.motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                    can_hub_t::can1); // 
    rudder_cfg_ptr.motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                    can_hub_t::can1); // 

    rudder_cfg_ptr.motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can1); // FL Wheel
    rudder_cfg_ptr.motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // BL Wheel
    rudder_cfg_ptr.motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Wheel
    rudder_cfg_ptr.motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can1); // FR Wheel


    for (int i = 0; i < 4; i++)
        rudder_cfg_ptr.pid.wheel_pid[i] =
            new pid_t(0.2f, 0.1f, 0.00f, 1.00f, 20.0f);

    
    rudder_cfg_ptr.pid.rud_pos_pid[0] =
            new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudder_cfg_ptr.pid.rud_spd_pid[0]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);
    rudder_cfg_ptr.pid.rud_pos_pid[1] =
            new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudder_cfg_ptr.pid.rud_spd_pid[1]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);
    rudder_cfg_ptr.pid.rud_pos_pid[2] =
            new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudder_cfg_ptr.pid.rud_spd_pid[2]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);
    rudder_cfg_ptr.pid.rud_pos_pid[3] =
            new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    rudder_cfg_ptr.pid.rud_spd_pid[3]=
            new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);

    rudder_cfg_ptr.rud_pos_moving_offset[0] = -0.662679672f;
    rudder_cfg_ptr.rud_pos_moving_offset[1] = 2.95061207f;
    rudder_cfg_ptr.rud_pos_moving_offset[2] = -2.12686443f;
    rudder_cfg_ptr.rud_pos_moving_offset[3] = 0.0f;
}

