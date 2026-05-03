#ifndef __PYRO_SCREW_GIMBAL_H__
#define __PYRO_SCREW_GIMBAL_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h" // 新增 DM 电机驱动
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
static float bullet_speed;
// =========================================================
// 1. 命令定义
// =========================================================
struct infantry1_gimbal_cmd_t final : public cmd_base_t
{
    float pitch_angle; // 目标 Pitch 角度 (rad)
    float yaw_angle;   // 目标 Yaw 角度 (rad)

    // 自瞄数据
    bool auto_aim;
    float auto_pitch;
    float auto_yaw;

    infantry1_gimbal_cmd_t()
        : pitch_angle(0.0f), yaw_angle(0.0f), auto_aim(false), auto_pitch(0.0f), auto_yaw(0.0f)
    {
    }
};

struct infantry1_gimbal_cfg_t
{
    // 电机句柄
    struct motor_cfg_t
    {
        dm_motor_drv_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_deps_t
    {
        pid_t *pitch_pos{nullptr};
        pid_t *pitch_spd{nullptr};
        pid_t *yaw_pos{nullptr};
        pid_t *yaw_spd{nullptr};
    };

    motor_cfg_t motor{};
    pid_deps_t pid{};
    float pitch_pos_offset{0};
    float yaw_pos_offset{0};
};

// =========================================================
// 2. 云台类
// =========================================================
class infantry1_gimbal_t final
    : public module_base_t<infantry1_gimbal_t, infantry1_gimbal_cmd_t,
                           infantry1_gimbal_cfg_t>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_ctx_t;

  public:
    [[nodiscard]] gimbal_ctx_t get_ctx() const;
    float get_yaw();

  private:
    infantry1_gimbal_t();
    ~infantry1_gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    void _gimbal_control();
    static void _send_motor_command(gimbal_ctx_t *ctx);
    void _communicate_chassis();
    void yaw_angle_ch(float current_yaw, float &target_yaw);
    bool init_fleg;
    float imu2motor_pitch(float imu_pitch);
    float motor2imu_pitch(float motor_pitch);
    // 运行时数据
    struct data_ctx_t
    {
        float pitch_imu_rad{0};
        float yaw_imu_rad{0};
        float roll_imu_rad{0};

        float target_pitch_rad{0};//电机的
        float target_pitch_radps{0};
        float target_imupitch_rad{0};//imu
        float target_imuyaw_rad{0};//imu

        float current_pitch_rad{0};
        float current_pitch_radps{0};
        float current_yaw_rad{0};
        float current_yaw_radps{0};

        float out_pitch_torque{0};
        float out_yaw_torque{0};
    };


     enum class drive_mode_t
    {
        MOVING,  // Normal driving mode
    };


    // 总 Context
    struct gimbal_ctx_t
    {
        infantry1_gimbal_cfg_t gimbal_cfg_t;
        data_ctx_t data;
        infantry1_gimbal_cmd_t *cmd{};
        drive_mode_t drive_mode;
    };

    gimbal_ctx_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = infantry1_gimbal_t;

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

} // namespace pyro

#endif