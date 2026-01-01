import pygame
import time
import os

# 初始化 Pygame
pygame.init()
pygame.joystick.init()

# 检查手柄
if pygame.joystick.get_count() == 0:
    print("❌ 未检测到手柄，请检查 USB 或蓝牙连接！")
    exit()

# 初始化第一个手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("=" * 40)
print(f"🎮 已连接: {joystick.get_name()}")
print(f"   - 按钮数 (Buttons): {joystick.get_numbuttons()}")
print(f"   - 轴数 (Axes/LT/RT): {joystick.get_numaxes()}")
print(f"   - 苦力帽数 (Hats/D-Pad): {joystick.get_numhats()}")
print("=" * 40)
print("请按下任意键、移动摇杆或十字键... (按 Ctrl+C 退出)")

try:
    while True:
        # 处理事件队列 (Event Queue)
        # 这种方式比轮询(polling)更准确，不会漏掉按键
        for event in pygame.event.get():
            
            # --- 1. 检测普通按钮 (A, B, X, Y, LB, RB, Start, Select) ---
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"🟢 [按钮按下] ID: {event.button}")
            elif event.type == pygame.JOYBUTTONUP:
                print(f"⚪ [按钮松开] ID: {event.button}")

            # --- 2. 检测轴 (摇杆 + LT/RT) ---
            # 注意：摇杆稍微回弹不归零可能会一直触发，这里加了个阈值过滤噪音
            elif event.type == pygame.JOYAXISMOTION:
                axis_id = event.axis
                value = event.value
                
                # 只有当数值明显变化时才打印 (过滤轻微漂移)
                if abs(value) > 0.1: 
                    # 通常 Axis 2 是 LT, Axis 5 是 RT (具体看打印结果)
                    print(f"📈 [轴/扳机] ID: {axis_id} | 数值: {value:.2f}")

            # --- 3. 检测十字键 (D-Pad) ---
            elif event.type == pygame.JOYHATMOTION:
                hat_id = event.hat
                value = event.value # 返回元组 (x, y)
                
                # value 格式: (-1, 0)左, (1, 0)右, (0, 1)上, (0, -1)下
                direction = "中心"
                if value == (0, 1): direction = "⬆️ 上"
                elif value == (0, -1): direction = "⬇️ 下"
                elif value == (-1, 0): direction = "⬅️ 左"
                elif value == (1, 0): direction = "➡️ 右"
                
                print(f"➕ [十字键] ID: {hat_id} | 方向: {value} ({direction})")

        time.sleep(0.01) # 避免 CPU 占用过高

except KeyboardInterrupt:
    print("\n👋 测试结束")
    pygame.quit()