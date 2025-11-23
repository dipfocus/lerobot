"""Terminal utility to quickly verify Xbox controller buttons/axes.

Run ``PYTHONPATH=src python -m tests.xlerobot.test_xbox`` with the controller
plugged in (USB recommended) and watch the console output while pressing each
button/stick.
"""

from __future__ import annotations

import sys
from typing import Final

import pygame

# Typical Xbox mappings reported by pygame. Some models expose the triggers as
# axes instead of buttons, so we list them under axes for better visibility.
AXIS_NAMES: Final = {
    0: "左摇杆 X",
    1: "左摇杆 Y",
    2: "右摇杆 X",
    3: "右摇杆 Y",
    4: "LT 扳机",
    5: "RT 扳机",
}

BUTTON_NAMES: Final = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "View (视图键)",
    5: "Start",
    6: "Menu",
    7: "Left Stick",
    8: "Right Stick",
    9: "LB",
    10: "RB",
    11: "UP",
    12: "DOWN",
    13: "LEFT",
    14: "RIGHT",
    15: "Share",
}

HAT_NAMES: Final = {
    0: "十字键",
}


def init_joystick() -> pygame.joystick.Joystick | None:
    """Detect and initialize the first connected joystick."""

    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("没有检测到任何手柄，请确认 Xbox 手柄已连接（建议 USB 连接）。")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"检测到 {joystick_count} 个手柄，当前使用：{joystick.get_name()}")
    print(f"轴数量: {joystick.get_numaxes()}")
    print(f"按钮数量: {joystick.get_numbuttons()}")
    print(f"帽开关数量: {joystick.get_numhats()}")
    return joystick


def log_intro() -> None:
    print("\n开始测试 Xbox 手柄，按 Ctrl+C 退出。")
    print("建议依次测试：")
    print("  1）慢慢移动两个摇杆，查看轴数值是否变化")
    print("  2）按下 A/B/X/Y、LB/RB/Start 等按钮，查看按钮事件")
    print("  3）按下十字键方向，观察 hat 的 (x, y) 变化\n")


def main() -> None:
    joystick = init_joystick()
    if joystick is None:
        pygame.quit()
        sys.exit(1)

    log_intro()
    clock = pygame.time.Clock()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    axis_name = AXIS_NAMES.get(event.axis, f"轴 {event.axis}")
                    print(f"[AXIS ] {axis_name} (index {event.axis}): {event.value:.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    btn_name = BUTTON_NAMES.get(event.button, f"按钮 {event.button}")
                    print(f"[BUTTON  DOWN ] {btn_name} (index {event.button}) 被按下")
                elif event.type == pygame.JOYBUTTONUP:
                    btn_name = BUTTON_NAMES.get(event.button, f"按钮 {event.button}")
                    print(f"[BUTTON  UP ] {btn_name} (index {event.button}) 已抬起")
                elif event.type == pygame.JOYHATMOTION:
                    hat_name = HAT_NAMES.get(event.hat, f"帽 {event.hat}")
                    print(f"[HAT  ] {hat_name}: {event.value}")

            clock.tick(60)
    except KeyboardInterrupt:
        print("\n退出测试。")
    finally:
        pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
