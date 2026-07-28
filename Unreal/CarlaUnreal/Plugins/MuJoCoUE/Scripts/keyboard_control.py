#!/usr/bin/env python3
"""
Matrix mc_ctrl 键盘遥控脚本 (uinput 虚拟罗技 F710 手柄)
=========================================================
通过 Linux uinput 创建虚拟 Logitech F710 手柄，完全模拟 sim_launcher 的 KeyListener。
mc_ctrl 的 GamepadReader 只识别此设备。

数据流:
  键盘 → 本脚本 → /dev/uinput → 虚拟F710(/dev/input/jsX) → mc_ctrl → UDP:25002 → UE

用法:
  sudo python3 keyboard_control.py

按键 (与 sim_launcher 完全一致):
  U       - 站立 (LB+Y 组合键)
  Space   - 蹲下/趴下 (RB+LB 组合键)
  W/S     - 前进/后退 (ABS_Y)
  A/D     - 左移/右移 (ABS_X)
  Q/E     - 左转/右转 (ABS_RX + ABS_Z)
  Esc     - 退出
"""

import os
import sys
import struct
import time
import fcntl
import threading
import socket

try:
    import keyboard
except ImportError:
    print("需要安装 keyboard 库: sudo pip3 install keyboard")
    sys.exit(1)

# ============ Linux input 常量 ============
# Event types
EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03

# Sync
SYN_REPORT = 0x00

# Buttons (Logitech F710)
BTN_SOUTH = 0x130    # A
BTN_EAST = 0x131     # B
BTN_WEST = 0x133     # X
BTN_NORTH = 0x134    # Y
BTN_TL = 0x136       # LB (Left Bumper)
BTN_TR = 0x137       # RB (Right Bumper)
BTN_SELECT = 0x13a   # Back
BTN_START = 0x13b    # Start
BTN_THUMBL = 0x13c   # L3
BTN_THUMBR = 0x13d   # R3

# Axes
ABS_X = 0x00         # Left stick X
ABS_Y = 0x01         # Left stick Y
ABS_RX = 0x03        # Right stick X
ABS_RY = 0x04        # Right stick Y
ABS_Z = 0x02         # Left trigger
ABS_RZ = 0x05        # Right trigger
ABS_HAT0X = 0x10     # D-pad X
ABS_HAT0Y = 0x11     # D-pad Y

# uinput ioctl
UI_SET_EVBIT = 0x40045564
UI_SET_KEYBIT = 0x40045565
UI_SET_ABSBIT = 0x40045567
UI_DEV_CREATE = 0x5501
UI_DEV_DESTROY = 0x5502

# Bus type
BUS_USB = 0x03

# ============ LCM 配置 ============
LCM_MULTICAST = "239.255.76.67"
LCM_PORT = 7667
LCM_MAGIC = 0x4C433032
LCM_CHANNEL = "des_vel_cmd"
# gamepad_lcmt wire hash (from _computeHash: 0xef429e39e2e19db1 rotated)
GAMEPAD_LCM_HASH = 0xDE853C73C5C33B63

# ============ 配置 ============
SEND_RATE_HZ = 100            # 发送频率 (Hz)
STICK_MAX = 32767             # 摇杆最大值 (sim_launcher 用满量程 ±32768/32767)


class VirtualF710Controller:
    """通过 uinput 创建虚拟 Logitech F710 手柄 (与 sim_launcher 完全一致)"""

    def __init__(self):
        self.fd = -1
        self._initialize()

    def _initialize(self):
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)

        # 设置事件类型
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_SYN)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)

        # 设置按键
        for btn in [BTN_SOUTH, BTN_EAST, BTN_WEST, BTN_NORTH,
                    BTN_TL, BTN_TR, BTN_SELECT, BTN_START,
                    BTN_THUMBL, BTN_THUMBR]:
            fcntl.ioctl(self.fd, UI_SET_KEYBIT, btn)

        # 设置轴
        for axis in [ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ, ABS_HAT0X, ABS_HAT0Y]:
            fcntl.ioctl(self.fd, UI_SET_ABSBIT, axis)

        # uinput_user_dev 结构体 (1116 bytes):
        # char name[80] + input_id(8) + ff_effects_max(4) +
        # absmax[64](256) + absmin[64](256) + absfuzz[64](256) + absflat[64](256)
        name = b"Logitech Gamepad F710 (Virtual)"
        dev = bytearray(1116)
        dev[0:len(name)] = name

        # input_id: bustype(u16) vendor(u16) product(u16) version(u16) at offset 80
        # Logitech: VID=0x046d, PID=0xc219
        struct.pack_into('<HHHH', dev, 80, BUS_USB, 0x046d, 0xc219, 0x0110)

        # ff_effects_max at offset 88
        struct.pack_into('<I', dev, 88, 0)

        # absmax[64] at offset 92 (each int32)
        absmax_offset = 92
        for axis in [ABS_X, ABS_Y, ABS_RX, ABS_RY]:
            struct.pack_into('<i', dev, absmax_offset + axis * 4, 32767)
        struct.pack_into('<i', dev, absmax_offset + ABS_Z * 4, 32767)
        struct.pack_into('<i', dev, absmax_offset + ABS_RZ * 4, 32767)
        struct.pack_into('<i', dev, absmax_offset + ABS_HAT0X * 4, 1)
        struct.pack_into('<i', dev, absmax_offset + ABS_HAT0Y * 4, 1)

        # absmin[64] at offset 348
        absmin_offset = 348
        for axis in [ABS_X, ABS_Y, ABS_RX, ABS_RY]:
            struct.pack_into('<i', dev, absmin_offset + axis * 4, -32768)
        struct.pack_into('<i', dev, absmin_offset + ABS_Z * 4, -32768)
        struct.pack_into('<i', dev, absmin_offset + ABS_RZ * 4, -32768)
        struct.pack_into('<i', dev, absmin_offset + ABS_HAT0X * 4, -1)
        struct.pack_into('<i', dev, absmin_offset + ABS_HAT0Y * 4, -1)

        # absfuzz[64] at offset 604 - all zeros
        # absflat[64] at offset 860 - all zeros

        os.write(self.fd, bytes(dev))
        fcntl.ioctl(self.fd, UI_DEV_CREATE)
        time.sleep(0.5)  # 等待设备创建
        print("  虚拟手柄已创建: Logitech Gamepad F710 (Virtual)")

    def _send_event(self, ev_type, code, value):
        # input_event on 64-bit Linux: int64 tv_sec + int64 tv_usec + u16 type + u16 code + s32 value = 24 bytes
        event = struct.pack('<qqHHi', 0, 0, ev_type, code, value)
        os.write(self.fd, event)

    def _sync(self):
        self._send_event(EV_SYN, SYN_REPORT, 0)

    def set_left_stick(self, x, y):
        """设置左摇杆 (-32768 ~ 32767)"""
        self._send_event(EV_ABS, ABS_X, x)
        self._send_event(EV_ABS, ABS_Y, y)
        self._sync()

    def set_right_stick_x(self, x):
        """设置右摇杆X + 左触发器 (sim_launcher: Q/E 同时写 ABS_RX 和 ABS_Z)"""
        self._send_event(EV_ABS, ABS_RX, x)
        self._send_event(EV_ABS, ABS_Z, x)
        self._sync()

    def set_right_stick_y(self, y):
        """设置右摇杆Y + 右触发器 (sim_launcher: R/F 同时写 ABS_RY 和 ABS_RZ)"""
        self._send_event(EV_ABS, ABS_RY, y)
        self._send_event(EV_ABS, ABS_RZ, y)
        self._sync()

    def press_button(self, button):
        self._send_event(EV_KEY, button, 1)
        self._sync()

    def release_button(self, button):
        self._send_event(EV_KEY, button, 0)
        self._sync()

    def tap_button(self, button, hold_ms=100):
        self.press_button(button)
        time.sleep(hold_ms / 1000.0)
        self.release_button(button)

    def tap_combo(self, btn1, btn2, hold_ms=150):
        """同时按下两个键（组合键），保持后同时释放"""
        self._send_event(EV_KEY, btn1, 1)
        self._send_event(EV_KEY, btn2, 1)
        self._sync()
        time.sleep(hold_ms / 1000.0)
        self._send_event(EV_KEY, btn1, 0)
        self._send_event(EV_KEY, btn2, 0)
        self._sync()

    def destroy(self):
        if self.fd >= 0:
            fcntl.ioctl(self.fd, UI_DEV_DESTROY)
            os.close(self.fd)
            self.fd = -1


class LcmVelocityPublisher:
    """通过 LCM UDP 多播发送 gamepad_lcmt 速度指令到 mc_ctrl"""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        self.seq = 0
        self.channel_bytes = LCM_CHANNEL.encode('ascii')
        print(f"  LCM 发布器已创建: {LCM_MULTICAST}:{LCM_PORT} ch=\"{LCM_CHANNEL}\"")

    def publish(self, vx=0.0, vy=0.0, yaw=0.0):
        """发送 gamepad_lcmt 消息
        Args:
            vx: 前进速度 (-1.0~1.0), 映射到 leftStickAnalog[1]
            vy: 横移速度 (-1.0~1.0), 映射到 leftStickAnalog[0]
            yaw: 偏航角速度 (-1.0~1.0), 映射到 rightStickAnalog[0]
        """
        # 编码 gamepad_lcmt payload (76 bytes, big-endian)
        # 13 int32 buttons + 2 float triggers + float[2] leftStick + float[2] rightStick + int32 flag
        payload = struct.pack('>Q', GAMEPAD_LCM_HASH)
        # Buttons: leftBumper, rightBumper, leftTriggerButton, rightTriggerButton,
        #          back, start, a, b, x, y, leftStickButton, rightStickButton, navigation_mode
        payload += struct.pack('>13i', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        # leftTriggerAnalog, rightTriggerAnalog
        payload += struct.pack('>2f', 0.0, 0.0)
        # leftStickAnalog[2]: [X=lateral, Y=forward]
        payload += struct.pack('>2f', vy, vx)
        # rightStickAnalog[2]: [X=yaw, Y=0]
        payload += struct.pack('>2f', yaw, 0.0)
        # robot_state_flag
        payload += struct.pack('>i', 0)

        # LCM UDP header: magic + seq + channel_len + channel
        header = struct.pack('>III', LCM_MAGIC, self.seq, len(self.channel_bytes))
        packet = header + self.channel_bytes + payload

        self.sock.sendto(packet, (LCM_MULTICAST, LCM_PORT))
        self.seq = (self.seq + 1) & 0xFFFFFFFF

    def close(self):
        self.sock.close()


class KeyboardController:
    def __init__(self, gamepad, lcm_pub=None):
        self.gamepad = gamepad
        self.lcm_pub = lcm_pub
        self.vx = 0
        self.vy = 0
        self.yaw_rate = 0
        self.running = True
        self.lock = threading.Lock()

    def update(self):
        """根据当前按键状态更新速度并发送手柄事件 (与 sim_launcher KeyListener 一致)"""
        with self.lock:
            # W/S → ABS_Y (sim_launcher: W=-32768, S=+32767)
            if keyboard.is_pressed('w'):
                self.vx = -32768
            elif keyboard.is_pressed('s'):
                self.vx = 32767
            else:
                self.vx = 0

            # A/D → ABS_X (sim_launcher: A=-32768, D=+32767)
            if keyboard.is_pressed('a'):
                self.vy = -32768
            elif keyboard.is_pressed('d'):
                self.vy = 32767
            else:
                self.vy = 0

            # Q/E → ABS_RX + ABS_Z (sim_launcher: Q=-32768, E=+32767)
            if keyboard.is_pressed('q'):
                self.yaw_rate = -32768
            elif keyboard.is_pressed('e'):
                self.yaw_rate = 32767
            else:
                self.yaw_rate = 0

            # 发送摇杆 (左摇杆: X=横移, Y=前后)
            self.gamepad.set_left_stick(self.vy, self.vx)
            # 右摇杆X + 左触发器 (偏航)
            self.gamepad.set_right_stick_x(self.yaw_rate)

            # 发送 LCM 速度指令 (mc_ctrl 通过 des_vel_cmd 通道接收行走速度)
            if self.lcm_pub:
                vx_norm = self.vx / 32768.0    # 前进 (-1~1)
                vy_norm = self.vy / 32768.0    # 横移 (-1~1)
                yaw_norm = self.yaw_rate / 32768.0  # 偏航 (-1~1)
                self.lcm_pub.publish(vx=vx_norm, vy=vy_norm, yaw=yaw_norm)

    def get_status(self):
        with self.lock:
            vx_norm = self.vx / 32768.0
            vy_norm = self.vy / 32768.0
            yaw_norm = self.yaw_rate / 32768.0
            return f"  L=({vy_norm:+.2f},{vx_norm:+.2f}) R_x={yaw_norm:+.2f}"


def main():
    print("=" * 55)
    print("  Matrix mc_ctrl 键盘遥控 (虚拟 Logitech F710)")
    print("=" * 55)

    # 创建虚拟手柄
    try:
        gamepad = VirtualF710Controller()
    except Exception as e:
        print(f"  创建虚拟手柄失败: {e}")
        print("  请确保: sudo modprobe uinput && 有 /dev/uinput 权限")
        sys.exit(1)

    print(f"  频率: {SEND_RATE_HZ} Hz")
    print("-" * 55)
    print("  U:     站立 (LB+Y 组合键)")
    print("  Space: 蹲下/趴下 (RB+LB 组合键)")
    print("  W/S:   前进/后退 (ABS_Y)")
    print("  A/D:   左移/右移 (ABS_X)")
    print("  Q/E:   左转/右转 (ABS_RX+ABS_Z)")
    print("  Esc:   退出")
    print("=" * 55)

    # 创建 LCM 速度发布器
    lcm_pub = LcmVelocityPublisher()

    ctrl = KeyboardController(gamepad, lcm_pub)

    # 按键回调 (与 sim_launcher KeyListener 完全一致)
    def on_stand():
        # sim_launcher: U → LB+Y
        gamepad.tap_combo(BTN_TL, BTN_NORTH)
        print("\n  → 站立! (LB+Y)")

    def on_lie_down():
        # sim_launcher: Space → RB+LB
        gamepad.tap_combo(BTN_TR, BTN_TL)
        print("\n  → 趴下! (RB+LB)")

    keyboard.on_press_key('u', lambda _: on_stand())
    keyboard.on_press_key('space', lambda _: on_lie_down())

    period = 1.0 / SEND_RATE_HZ
    packet_count = 0
    last_print = time.time()

    print("\n  发送中... (Esc 退出)\n")

    try:
        while ctrl.running:
            if keyboard.is_pressed('esc'):
                break

            ctrl.update()
            packet_count += 1

            # 每秒打印状态
            now = time.time()
            if now - last_print >= 1.0:
                status = ctrl.get_status()
                print(f"\r{status} | {packet_count} updates/s", end='', flush=True)
                packet_count = 0
                last_print = now

            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        # 归零摇杆
        gamepad.set_left_stick(0, 0)
        gamepad.set_right_stick_x(0)
        gamepad.set_right_stick_y(0)
        # 发送零速度 LCM
        if lcm_pub:
            lcm_pub.publish(0, 0, 0)
            lcm_pub.close()
        time.sleep(0.1)
        gamepad.destroy()
        keyboard.unhook_all()
        print("\n\n  已停止，虚拟手柄已销毁。")


if __name__ == "__main__":
    main()
