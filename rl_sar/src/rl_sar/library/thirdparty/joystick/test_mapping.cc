// 手柄映射测试程序
//
// 用途：在不启动整个 rl_real_dmgo 的情况下，单独验证 /dev/input/js0
//      的按键/轴编号是否与 rl_real_dmgo.cpp 中 GetSysJoystick() 的映射一致。
//
// 构建：bash build-test-mapping.sh
// 运行：./test_mapping              使用默认 /dev/input/js0
//      ./test_mapping /dev/input/js0
//
// 操作建议：
//   1) 依次按 A/B/X/Y/LB/RB，确认终端看到的按钮编号是 0/1/2/3/4/5。
//   2) 按下左/右摇杆，确认是 9/10。
//   3) 拨左摇杆，确认轴 0/1 变化；拨右摇杆，确认轴 3 变化。
//   4) 按十字键，确认轴 6/7 变化（左右=6，上下=7）。
//   5) 测组合键：LB+X 应该看到 [LB_X -> Passive]；
//      RB+DPadUp 应该看到 [RB_DPadUp -> RLLocomotion]。

#include "joystick.hh"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <string>
#include <vector>

namespace {

constexpr int kMaxButtons = 32;
constexpr int kMaxAxes = 16;
constexpr int kAxisBits = 16;
constexpr float kAxisDeadzone = 0.05f;
constexpr int kPrintIntervalUs = 50 * 1000;

struct ButtonState {
    bool pressed = false;
    bool on_press = false;
    bool on_release = false;

    void update(bool state) {
        on_press = state ? (state != pressed) : false;
        on_release = state ? false : (state != pressed);
        pressed = state;
    }
};

const char* ButtonRoleByIndex(int idx) {
    switch (idx) {
        case 0:  return "A         (Passive->GetUp / GetDown->GetUp)";
        case 1:  return "B         (-> GetDown)";
        case 2:  return "X";
        case 3:  return "Y";
        case 4:  return "LB        (modifier)";
        case 5:  return "RB        (modifier)";
        case 9:  return "LStick    (press left stick)";
        case 10: return "RStick    (press right stick)";
        default: return "(not mapped in rl_real_dmgo.cpp)";
    }
}

const char* AxisRoleByIndex(int idx) {
    switch (idx) {
        case 0: return "left stick  X  -> control.y   (sign inverted)";
        case 1: return "left stick  Y  -> control.x   (sign inverted)";
        case 3: return "right stick X  -> control.yaw (sign inverted, *0.5)";
        case 6: return "DPad horizontal (>0 Left, <0 Right)";
        case 7: return "DPad vertical   (<0 Up,   >0 Down)";
        default: return "(not used in rl_real_dmgo.cpp)";
    }
}

void DescribeCombo(const ButtonState* btns, const int* axes, int axis_max) {
    auto stick_dir = [&](int axis_idx, const char* neg_dir, const char* pos_dir) -> const char* {
        if (axis_idx >= axis_max) return nullptr;
        if (axes[axis_idx] < 0) return neg_dir;
        if (axes[axis_idx] > 0) return pos_dir;
        return nullptr;
    };

    if (btns[4].pressed && btns[2].on_press) {
        std::printf("  >> COMBO  LB + X         -> RLFSMStatePassive (kill switch)\n");
    }
    if (btns[5].pressed && axis_max > 7 && axes[7] < 0) {
        std::printf("  >> COMBO  RB + DPadUp    -> RLFSMStateRLLocomotion\n");
    }
    if (btns[4].pressed) {
        if (btns[0].on_press) std::printf("  >> COMBO  LB + A         (LB_A)\n");
        if (btns[1].on_press) std::printf("  >> COMBO  LB + B         (LB_B)\n");
        if (btns[3].on_press) std::printf("  >> COMBO  LB + Y         (LB_Y)\n");
        if (btns[9].on_press) std::printf("  >> COMBO  LB + LStick    (LB_LStick)\n");
        if (btns[10].on_press) std::printf("  >> COMBO  LB + RStick    (LB_RStick)\n");
        if (auto* d = stick_dir(7, "DPadUp", "DPadDown"))
            if (axes[7] != 0) std::printf("  >> COMBO  LB + %-9s (LB_%s)\n", d, d);
        if (auto* d = stick_dir(6, "DPadRight", "DPadLeft"))
            if (axes[6] != 0) std::printf("  >> COMBO  LB + %-9s (LB_%s)\n", d, d);
    }
    if (btns[5].pressed) {
        if (btns[0].on_press) std::printf("  >> COMBO  RB + A         (RB_A)\n");
        if (btns[1].on_press) std::printf("  >> COMBO  RB + B         (RB_B)\n");
        if (btns[2].on_press) std::printf("  >> COMBO  RB + X         (RB_X)\n");
        if (btns[3].on_press) std::printf("  >> COMBO  RB + Y         (RB_Y)\n");
        if (auto* d = stick_dir(6, "DPadRight", "DPadLeft"))
            if (axes[6] != 0) std::printf("  >> COMBO  RB + %-9s (RB_%s)\n", d, d);
        if (axis_max > 7 && axes[7] > 0) std::printf("  >> COMBO  RB + DPadDown  (RB_DPadDown)\n");
    }
    if (btns[4].pressed && btns[5].on_press) {
        std::printf("  >> COMBO  LB + RB        (LB_RB)\n");
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string device = "/dev/input/js0";
    if (argc >= 2) device = argv[1];

    std::printf("=================================================================\n");
    std::printf("  Joystick mapping tester (compares against rl_real_dmgo.cpp)\n");
    std::printf("  Device: %s\n", device.c_str());
    std::printf("=================================================================\n\n");

    Joystick joystick(device);
    if (!joystick.isFound()) {
        std::fprintf(stderr, "ERROR: failed to open %s\n", device.c_str());
        std::fprintf(stderr, "Check the device with:  ls -l /dev/input/js*\n");
        return 1;
    }

    const float axis_max_value = static_cast<float>(1 << (kAxisBits - 1));

    ButtonState buttons[kMaxButtons];
    int axes[kMaxAxes];
    std::memset(axes, 0, sizeof(axes));

    int last_axis_idx = -1;
    int last_axis_value = 0;
    long since_last_print_us = 0;

    std::printf("Press buttons / move sticks. Press Ctrl-C to quit.\n\n");
    std::printf("Expected button mapping (Xbox-style):\n");
    for (int i : {0, 1, 2, 3, 4, 5, 9, 10}) {
        std::printf("  button[%2d]  = %s\n", i, ButtonRoleByIndex(i));
    }
    std::printf("\nExpected axis mapping:\n");
    for (int i : {0, 1, 3, 6, 7}) {
        std::printf("  axis[%2d]    = %s\n", i, AxisRoleByIndex(i));
    }
    std::printf("\n----- live events -----\n");

    while (true) {
        usleep(1000);
        since_last_print_us += 1000;

        for (int i = 0; i < kMaxButtons; ++i) {
            buttons[i].on_press = false;
            buttons[i].on_release = false;
        }

        bool any_event = false;
        JoystickEvent ev;
        while (joystick.sample(&ev)) {
            any_event = true;
            if (ev.isButton() && ev.number < kMaxButtons) {
                buttons[ev.number].update(ev.value != 0);
                std::printf("[BTN ] number=%2u value=%d  -> %s%s\n",
                            ev.number,
                            static_cast<int>(ev.value),
                            ButtonRoleByIndex(ev.number),
                            ev.isInitialState() ? "  (initial state)" : "");
            } else if (ev.isAxis() && ev.number < kMaxAxes) {
                double normalized = static_cast<double>(ev.value) / axis_max_value;
                if (std::abs(normalized) < kAxisDeadzone) {
                    axes[ev.number] = 0;
                } else {
                    axes[ev.number] = ev.value;
                }
                last_axis_idx = ev.number;
                last_axis_value = ev.value;
            }
        }

        if (last_axis_idx >= 0 && since_last_print_us >= kPrintIntervalUs) {
            double normalized = static_cast<double>(last_axis_value) / axis_max_value;
            std::printf("[AXIS] number=%2d raw=%6d  norm=%+0.3f  -> %s\n",
                        last_axis_idx, last_axis_value, normalized,
                        AxisRoleByIndex(last_axis_idx));
            last_axis_idx = -1;
            since_last_print_us = 0;
        }

        if (any_event) {
            DescribeCombo(buttons, axes, kMaxAxes);

            float ly = -static_cast<float>(axes[1]) / axis_max_value;
            float lx = -static_cast<float>(axes[0]) / axis_max_value;
            float rx = -static_cast<float>(axes[3]) / axis_max_value;
            if (ly != 0.0f || lx != 0.0f || rx != 0.0f) {
                std::printf("  >> CMD    control.x=%+0.3f  control.y=%+0.3f  control.yaw=%+0.3f\n",
                            ly, lx, rx * 0.5f);
            }
        }
    }
    return 0;
}
