// 鎵嬫焺鏄犲皠娴嬭瘯绋嬪簭
//
// 鐢ㄩ€旓細鍦ㄤ笉鍚姩鏁翠釜 rl_real_d1 鐨勬儏鍐典笅锛屽崟鐙獙璇?/dev/input/js0
//      鐨勬寜閿?杞寸紪鍙锋槸鍚︿笌 rl_real_d1.cpp 涓?GetSysJoystick() 鐨勬槧灏勪竴鑷淬€?//
// 鏋勫缓锛歜ash build-test-mapping.sh
// 杩愯锛?/test_mapping              浣跨敤榛樿 /dev/input/js0
//      ./test_mapping /dev/input/js0
//
// 鎿嶄綔寤鸿锛?//   1) 渚濇鎸?A/B/X/Y/LB/RB锛岀‘璁ょ粓绔湅鍒扮殑鎸夐挳缂栧彿鏄?0/1/2/3/4/5銆?//   2) 鎸変笅宸?鍙虫憞鏉嗭紝纭鏄?9/10銆?//   3) 鎷ㄥ乏鎽囨潌锛岀‘璁よ酱 0/1 鍙樺寲锛涙嫧鍙虫憞鏉嗭紝纭杞?3 鍙樺寲銆?//   4) 鎸夊崄瀛楅敭锛岀‘璁よ酱 6/7 鍙樺寲锛堝乏鍙?6锛屼笂涓?7锛夈€?//   5) 娴嬬粍鍚堥敭锛歀B+X 搴旇鐪嬪埌 [LB_X -> Passive]锛?//      RB+DPadUp 搴旇鐪嬪埌 [RB_DPadUp -> RLLocomotion]銆?
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
        default: return "(not mapped in rl_real_d1.cpp)";
    }
}

const char* AxisRoleByIndex(int idx) {
    switch (idx) {
        case 0: return "left stick  X  -> control.y   (sign inverted)";
        case 1: return "left stick  Y  -> control.x   (sign inverted)";
        case 3: return "right stick X  -> control.yaw (sign inverted, *0.5)";
        case 6: return "DPad horizontal (>0 Left, <0 Right)";
        case 7: return "DPad vertical   (<0 Up,   >0 Down)";
        default: return "(not used in rl_real_d1.cpp)";
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
    std::printf("  Joystick mapping tester (compares against rl_real_d1.cpp)\n");
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

