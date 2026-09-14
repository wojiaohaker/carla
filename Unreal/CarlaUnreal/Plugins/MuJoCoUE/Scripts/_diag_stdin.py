#!/usr/bin/env python3
"""
诊断：在当前(可能是 sudo + SSH)环境下，stdin 能否逐字符读到按键。
用法（务必在你要测的那个终端窗口里跑）：
    sudo python3 _diag_stdin.py
    python3 _diag_stdin.py          # 再不加 sudo 跑一次做对比

跑起来后 15 秒内，在【同一个终端窗口】依次按： u  w  回车  q
把打印结果贴回来即可判断根因。
"""
import sys, os, select, termios, tty, time
import termios as T

fd = sys.stdin.fileno()
print("PID:", os.getpid(), " EUID:", os.geteuid(), "(0=root/sudo)")
print("stdin.isatty():", sys.stdin.isatty())
try:
    print("ttyname(stdin):", os.ttyname(fd),
          "  <- 和登录shell的 `tty` 输出对比；若不同=sudo套了新pty(use_pty)")
except Exception as e:
    print("ttyname err:", e)

if not sys.stdin.isatty():
    print("!! stdin 不是 tty —— terminal 模式无法工作（可能被重定向/管道）")
    sys.exit(1)

old = termios.tcgetattr(fd)
print("原始 lflag: ICANON(行缓冲)=", bool(old[3] & T.ICANON),
      " ECHO=", bool(old[3] & T.ECHO))
tty.setcbreak(fd)
new = termios.tcgetattr(fd)
print("cbreak 后 ICANON=", bool(new[3] & T.ICANON), "(应为 False)")
print("=" * 60)
print("现在请在【这个终端窗口】按键：u  w  回车  q(退出)")
print("  · 按 u/w 立刻出现 READ b'u' => stdin 逐字符正常(问题是焦点/别处按键)")
print("  · 按 u/w 无反应、按回车才出一串(如 b'u\\r') => sudo use_pty 行缓冲")
print("  · 完全没有任何 READ => 按键没到这个 stdin(焦点在别的窗口)")
print("=" * 60, flush=True)

t0 = time.time()
try:
    while time.time() - t0 < 15:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            data = os.read(fd, 64)
            if not data:
                print("  READ EOF (b'') —— stdin 被关闭", flush=True)
                break
            print(f"  READ {len(data)}B: hex={data.hex()} repr={data!r} "
                  f"t=+{time.time()-t0:.2f}s", flush=True)
            if b'q' in data:
                break
    else:
        print("  (15 秒超时，全程没读到任何按键)")
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print("终端已恢复。")
