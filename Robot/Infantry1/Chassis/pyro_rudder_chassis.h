#ifndef __PYRO_RUDOMNI_CHASSIS_H__
#define __PYRO_RUDOMNI_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"
//#include "pyro_powermeter.h"
//#include "pyro_power_control_drv.h"

namespace pyro 
{
    struct rudder_cmd_t : cmd_base_t
    {
        float vx, vy, wz;
        bool follow_yaw;
        rudder_cmd_t() : vx(0), vy(0), wz(0), follow_yaw(false)
        {
        }
    };//这些命令均以底盘为坐标系，传命令前请根据云台角度将原始指令进行换算(在底盘不跟随云台时)
    struct rudder_cfg_t
    {
        // 电机句柄
        struct motor_cfg_t
        {
            motor_base_t *rudder[4]{nullptr};
            motor_base_t *wheel[4]{nullptr};
        };

        struct pid_cfg_t
        {
            pid_t *rud_pos_pid[4]{nullptr};
            pid_t *rud_spd_pid[4]{nullptr};
            pid_t *wheel_pid[4]{nullptr};
        };

        motor_cfg_t motor;
        pid_cfg_t pid;
        float rud_pos_moving_offset[4]{};//舵机角度偏移，分别为[0]底盘2号轮的舵，[1]底盘3号轮的舵，[2]yaw的舵 //yaw的舵零点指向底盘y轴正方向
    };

    class rudder_chassis_t final 
        : public module_base_t<rudder_chassis_t, rudder_cmd_t, rudder_cfg_t>
    {
        friend class module_base_t;
        friend class vofa_drv_t;

        struct motor_ctx_t;
        struct pid_ctx_t;
        struct data_ctx_t;
        struct rudder_ctx_t;

      public:
        rudder_chassis_t(const rudder_chassis_t &)            = delete;
        rudder_chassis_t &operator=(const rudder_chassis_t &) = delete;
        
      private:
        rudder_chassis_t();
        ~rudder_chassis_t() override = default;

        // --- 基类接口实现 ---
        status_t _init() override;
        void _update_feedback() override;
        void _fsm_execute() override;

        // --- 私有辅助方法 ---
        void _kinematics_solve();
        static void _chassis_control(rudder_ctx_t *ctx);
        static void _send_motor_command(rudder_ctx_t *ctx);

        void yaw_angle_ch(float current_yaw, float &target_yaw);

        rudder_kin_t *_kinematics{nullptr};

    struct data_ctx_t
    {
        rudder_kin_t::rudder_states_t current_states{};
        rudder_kin_t::rudder_states_t target_states{};

        float current_rud_radps[4]{};

        float out_rud_torque[4]{};
        float out_wheel_torque[4]{};

    };

    enum class drive_mode_t
    {
        MOVING,  // Normal driving mode
    };

    struct rudder_ctx_t
    {
        rudder_cfg_t rudder_config;
        data_ctx_t data;
        rudder_cmd_t *cmd;
        drive_mode_t drive_mode;
    };
    rudder_ctx_t _ctx;
    

    //--------------------------- FSM 状态定义 -----------------//
    using owner = rudder_chassis_t;

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