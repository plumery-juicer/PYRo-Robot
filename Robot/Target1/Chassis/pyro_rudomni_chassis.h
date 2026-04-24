#ifndef __PYRO_RUDOMNI_CHASSIS_H__
#define __PYRO_RUDOMNI_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_kin_rudomni.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"
#include "pyro_power_control_drv.h"

namespace pyro 
{
    struct rudomni_cmd_t : cmd_base_t
    {
        float vx, vy, wz,wz2;
        bool follow_yaw;
        rudomni_cmd_t() : vx(0), vy(0), wz(0), wz2(0), follow_yaw(false)
        {
        }
    };//这些命令均以底盘为坐标系，传命令前请根据云台角度将原始指令进行换算
    struct rudomni_cfg_t
    {
        // 电机句柄
        struct motor_cfg_t
        {
            motor_base_t *rudder[2]{nullptr};
            motor_base_t *wheel[4]{nullptr};
            motor_base_t *yaw{nullptr};
        };

        struct pid_cfg_t
        {
            pid_t *rud_pos_pid[2]{nullptr};
            pid_t *rud_spd_pid[2]{nullptr};
            pid_t *wheel_pid[4]{nullptr};
            pid_t *yaw_pos_pid{nullptr};
            pid_t *yaw_spd_pid{nullptr};
        };

        motor_cfg_t motor;
        pid_cfg_t pid;
        float rud_pos_moving_offset[4]{};//舵机角度偏移，分别为[0]底盘2号轮的舵，[1]底盘3号轮的舵，[2]yaw的舵 //yaw的舵零点指向底盘y轴正方向
    };

    class rudomni_chassis_t final 
        : public module_base_t<rudomni_chassis_t, rudomni_cmd_t, rudomni_cfg_t>
    {
        friend class module_base_t;
        friend class vofa_drv_t;

        struct motor_ctx_t;
        struct pid_ctx_t;
        struct data_ctx_t;
        struct rudomni_ctx_t;

      public:
        rudomni_chassis_t(const rudomni_chassis_t &)            = delete;
        rudomni_chassis_t &operator=(const rudomni_chassis_t &) = delete;
        
        struct yaw_data_t
        {
            float current_insyaw;
            float current_inspitch;
            float current_insroll;

            float target_insyaw;

            float current_yaw_pos;
            float current_yaw_rads;
            float target_yaw_rads;
            float out_yaw_torque;
        };

        yaw_data_t _yaw_data;

      private:
        rudomni_chassis_t();
        ~rudomni_chassis_t() override = default;

        // --- 基类接口实现 ---
        status_t _init() override;
        void _update_feedback() override;
        void _fsm_execute() override;

        // --- 私有辅助方法 ---
        void _kinematics_solve();
        static void _chassis_control(rudomni_ctx_t *ctx,yaw_data_t *yaw_data);
        static void _send_motor_command(rudomni_ctx_t *ctx,yaw_data_t *yaw_data);

        void yaw_angle_ch(float current_yaw, float &target_yaw);

        rudomni_kin_t *_kinematics{nullptr};

    struct data_ctx_t
    {
        rudomni_kin_t::rudomni_states_t current_states{};
        rudomni_kin_t::rudomni_states_t target_states{};

        float current_rud_radps[2];

        float out_rud_torque[2]{};
        float out_wheel_torque[4]{};

    };

    enum class drive_mode_t
    {

        MOVING,  // Normal driving mode
    };

    struct rudomni_ctx_t
    {
        rudomni_cfg_t rudomni_config;
        data_ctx_t data;
        rudomni_cmd_t *cmd;
        drive_mode_t drive_mode;
    };
    rudomni_ctx_t _ctx;
    

    //--------------------------- FSM 状态定义 -----------------//
    using owner = rudomni_chassis_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t : public fsm_t<owner>
    {
        struct state_moving_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

        private:
        state_moving_t _moving_state;

    };

    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

    };
   
}
#endif