#ifndef __PYRO_QUAD_BOOSTER_H__
#define __PYRO_QUAD_BOOSTER_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include <cmath>

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct booster_cmd_t final : public cmd_base_t
{
    bool fric_on;       // 摩擦轮开启
    uint8_t fire_count; // 拨弹计数器，替代 fire_enable
    float target_speed;
    bool continue_on;

    booster_cmd_t()
        : fric_on(false), fire_count(0), target_speed(23.5f), continue_on(false)
    {
    }
};

struct booster_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *fric_wheels[2]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *fric_pid[2]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
        pid_t *ball_speed_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
};

// =========================================================
// =========================================================
class booster_t final
    : public module_base_t<booster_t, booster_cmd_t,booster_cfg_t>
{
    friend class module_base_t<booster_t, booster_cmd_t,booster_cfg_t>;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    booster_t(const booster_t &)            = delete;
    booster_t &operator=(const booster_t &) = delete;
    [[nodiscard]] booster_ctx_t get_ctx() const;

  private:
    booster_t();
    ~booster_t() override = default;

    // --- 接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 内部辅助 ---
    void _speed_control();
    void _fric_control();
    void _trigger_position_control();
    void _trigger_speed_control();
    void _send_fric_command() const;
    void _send_trigger_command() const;

    // 角度归一化辅助函数
    static float _normalize_angle(float angle);

    // --- 成员变量 ---

    struct data_ctx_t
    {
        uint8_t internal_fire_count{0}; // 内部拨弹计数器追踪

        float launch_delay_timer[3]{}; // 发射延时计时器
        float signal_timer{0};         // 信号持续时间计时器
        float avg_launch_delay{0};      // 平均发射延时
        uint32_t fresh_timer{0};       // 发弹延迟计算的刷新计时器

        // 反馈
        float current_fric_mps[2]{};
        float current_trig_radps{0};
        float current_trig_torque{0};
        float current_trig_wheelrad{0}; // -PI ~ PI (归一化后的输出)
        float last_trig_wheelrad{0};
        float current_trig_position{0};
        // 目标
        float target_fric_mps[2]{};
        float target_trig_position{0};
        float target_trig_radps{0};

        float target_trig_continue_radps=550.0f;

        float target_fricom_mps[2]{};

        float current_fric_torque[2]{};
        // 输出
        float out_fric_torque[2]{};
        float out_trig_torque{0};
       
    };

    struct shoot_data_t
    {
        float ball_speed[3]{};
        float fric1_mps = 690.3f;
        float fric2_mps = -690.3f;
    };

    struct booster_ctx_t
    {
        booster_cfg_t booster_cfg;
        data_ctx_t data;
        shoot_data_t shoot_data{};
        booster_cmd_t *cmd{};
    };

    booster_ctx_t _ctx;

    // =====================================================
    // 状态机定义
    // =====================================================
    using owner = booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;

    private:
        bool _trigger_stopped{false}; // 用于确保拨弹盘完全停止后发0
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
       struct state_continue_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_interim_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_single_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_continue_t _continue_state;
        state_interim_t _interim_state;
        state_ready_t _ready_state;
        state_single_t _single_state;
        state_stall_t _stall_state;
    };
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

};

} // namespace pyro
#endif