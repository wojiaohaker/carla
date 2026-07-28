#!/usr/bin/env python3
"""
LCM 流量捕获器 - 监听 239.255.76.67:7667 上的所有 LCM 消息
用于捕获 sim_launcher 发送给 mc_ctrl 的正确消息格式和 hash
"""
import socket
import struct
import time
import sys

LCM_MULTICAST = "239.255.76.67"
LCM_PORT = 7667
LCM_MAGIC = 0x4C433032

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', LCM_PORT))
    mreq = struct.pack('4sl', socket.inet_aton(LCM_MULTICAST), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(30.0)

    print(f"监听 LCM 多播 {LCM_MULTICAST}:{LCM_PORT} ...")
    print("请启动 sim_launcher 键盘模式，按 U/W 等键")
    print("捕获 30 秒后自动停止 (Ctrl+C 提前退出)\n")

    packets = {}
    start = time.time()

    try:
        while time.time() - start < 30.0:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                break

            if len(data) < 12:
                continue

            magic, msg_size, ch_size = struct.unpack('>III', data[:12])
            if magic != LCM_MAGIC:
                continue

            channel = data[12:12+ch_size].decode('ascii', errors='replace')
            payload = data[12+ch_size:]

            # 提取 LCM 消息 hash (前 8 字节 big-endian)
            if len(payload) >= 8:
                msg_hash = struct.unpack('>Q', payload[:8])[0]
            else:
                msg_hash = 0

            key = (channel, msg_hash)
            if key not in packets:
                packets[key] = {"count": 0, "size": len(payload), "first_data": payload.hex()}
            packets[key]["count"] += 1

            # 实时打印新发现的 channel
            if packets[key]["count"] == 1:
                print(f"  [NEW] channel=\"{channel}\" hash=0x{msg_hash:016x} size={len(payload)}B")

    except KeyboardInterrupt:
        pass

    sock.close()

    print(f"\n{'='*60}")
    print(f"捕获结果 ({len(packets)} 个唯一 channel+hash):")
    print(f"{'='*60}")
    for (ch, h), info in sorted(packets.items()):
        print(f"\n  Channel: \"{ch}\"")
        print(f"  Hash:    0x{h:016x}")
        print(f"  Size:    {info['size']} bytes")
        print(f"  Count:   {info['count']} packets")
        print(f"  First:   {info['first_data'][:80]}...")

    # 特别关注 gamepad 相关的
    print(f"\n{'='*60}")
    print("关键信息 (用于 keyboard_control.py):")
    for (ch, h), info in packets.items():
        if 'gamepad' in ch.lower() or 'vel' in ch.lower() or 'cmd' in ch.lower() or 'des' in ch.lower():
            # 反推原始 hash (unrotate)
            orig = ((h >> 1) | ((h & 1) << 63)) & 0xFFFFFFFFFFFFFFFF
            print(f"  channel = \"{ch}\"")
            print(f"  wire_hash = 0x{h:016x}")
            print(f"  orig_hash = 0x{orig:016x}")
            print(f"  payload_size = {info['size']}")
            print()


if __name__ == "__main__":
    main()
