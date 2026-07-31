#!/usr/bin/env python3
"""
串口对发测试 — 通信质量检测
=============================
测试两个串口之间的收发质量：丢包率、误码率、延迟。

接线:
  设备A TX → 设备B RX
  设备A RX → 设备B TX
  设备A GND → 设备B GND

用法:
  python3 serial_test.py

默认端口:
  TX端: /dev/ttyACM2
  RX端: /dev/ttyACM1
  可分别用 --tx-port 和 --rx-port 指定
"""

import sys, time, random, argparse
import threading
import serial

parser = argparse.ArgumentParser()
parser.add_argument('--tx-port', default='/dev/ttyACM2', help='发送端串口')
parser.add_argument('--rx-port', default='/dev/ttyACM1', help='接收端串口')
parser.add_argument('--baud', type=int, default=115200, help='波特率')
parser.add_argument('--duration', type=int, default=10, help='测试时长(秒)')
parser.add_argument('--interval', type=float, default=0.02, help='发送间隔(秒)')
args = parser.parse_args()

PACKET_SIZE = 32       # 每包字节数
SEQ_LEN = 4            # 序号字节数

# ── 统计 ──
sent_count = 0
recv_count = 0
byte_total = 0
errors = 0
latencies = []
running = True
recv_buf = b""

tx_ser = serial.Serial(args.tx_port, args.baud, timeout=1)
rx_ser = serial.Serial(args.rx_port, args.baud, timeout=0.01)

def build_packet(seq: int) -> bytes:
    """构造测试包: [4字节序号][4字节时间戳][24字节随机填充]"""
    ts = int(time.monotonic() * 1000000) & 0xFFFFFFFF
    payload = random.randbytes(PACKET_SIZE - SEQ_LEN - 4)
    return seq.to_bytes(SEQ_LEN, 'big') + ts.to_bytes(4, 'big') + payload

def parse_packet(data: bytes):
    """解析包，返回 (seq, tx_time_us, payload)"""
    seq = int.from_bytes(data[:SEQ_LEN], 'big')
    ts = int.from_bytes(data[SEQ_LEN:SEQ_LEN+4], 'big')
    return seq, ts, data[SEQ_LEN+4:]

def tx_thread():
    global sent_count, running
    seq = 0
    while running:
        pkt = build_packet(seq)
        try:
            tx_ser.write(pkt)
            tx_ser.flush()
            sent_count += 1
            seq += 1
        except Exception as e:
            print(f"[TX ERR] {e}")
        time.sleep(args.interval)

def rx_thread():
    global recv_count, byte_total, errors, latencies, recv_buf, running
    while running:
        try:
            chunk = rx_ser.read(256)
            if not chunk:
                time.sleep(0.001)
                continue
            recv_buf += chunk
            byte_total += len(chunk)
        except Exception:
            time.sleep(0.001)
            continue

        # 解包
        while len(recv_buf) >= PACKET_SIZE:
            pkt = recv_buf[:PACKET_SIZE]
            recv_buf = recv_buf[PACKET_SIZE:]
            try:
                seq, tx_ts, _ = parse_packet(pkt)
                now_us = int(time.monotonic() * 1000000) & 0xFFFFFFFF
                latency = (now_us - tx_ts) & 0xFFFFFFFF  # handle wrap
                if latency < 1000000:  # 过滤大于 1s 的异常值
                    latencies.append(latency / 1000.0)  # ms
                recv_count += 1
            except Exception:
                errors += 1

try:
    # 启动收发线程
    t_tx = threading.Thread(target=tx_thread, daemon=True)
    t_rx = threading.Thread(target=rx_thread, daemon=True)
    t_tx.start()
    t_rx.start()

    print("=" * 65)
    print("  串口通信质量测试")
    print(f"  TX: {args.tx_port} @ {args.baud}")
    print(f"  RX: {args.rx_port} @ {args.baud}")
    print(f"  包大小: {PACKET_SIZE}B  间隔: {args.interval*1000:.0f}ms")
    print(f"  测试时长: {args.duration}s")
    print("=" * 65)
    print()

    start = time.time()
    last_print = 0
    while time.time() - start < args.duration:
        now = time.time()
        if now - last_print >= 1.0:
            elapsed = now - start
            tx_speed = sent_count / elapsed if elapsed > 0 else 0
            rx_speed = recv_count / elapsed if elapsed > 0 else 0
            loss = sent_count - recv_count
            loss_pct = (loss / sent_count * 100) if sent_count > 0 else 0
            avg_lat = sum(latencies[-100:]) / len(latencies[-100:]) if latencies else 0

            print(f"  [{int(elapsed):2d}s] "
                  f"TX={sent_count:5d} ({tx_speed:.0f}pk/s)  "
                  f"RX={recv_count:5d} ({rx_speed:.0f}pk/s)  "
                  f"丢包={loss:4d} ({loss_pct:.1f}%)  "
                  f"延迟={avg_lat:.2f}ms  "
                  f"误码={errors}", end='\r')
            last_print = now
        time.sleep(0.1)

    running = False
    time.sleep(0.2)

    elapsed = time.time() - start
    loss = sent_count - recv_count
    loss_pct = (loss / sent_count * 100) if sent_count > 0 else 0
    avg_lat = sum(latencies) / len(latencies) if latencies else 0
    max_lat = max(latencies) if latencies else 0
    min_lat = min(latencies) if latencies else 0

    print("\n")
    print("=" * 65)
    print("  测试结果")
    print("=" * 65)
    print(f"  测试时间:       {elapsed:.1f}s")
    print(f"  发送包数:       {sent_count}")
    print(f"  接收包数:       {recv_count}")
    print(f"  接收字节:       {byte_total}")
    print(f"  ─────────────────────────────")
    print(f"  丢包:           {loss} ({loss_pct:.2f}%)")
    print(f"  误码帧:         {errors}")
    print(f"  ─────────────────────────────")
    print(f"  延迟平均:       {avg_lat:.3f}ms")
    print(f"  延迟最小:       {min_lat:.3f}ms")
    print(f"  延迟最大:       {max_lat:.3f}ms")
    print(f"  ─────────────────────────────")
    if loss_pct < 0.1 and avg_lat < 5:
        print(f"  ✅ 通信质量良好")
    elif loss_pct < 1.0 and avg_lat < 20:
        print(f"  ⚠️ 通信质量一般，可接受")
    else:
        print(f"  ❌ 通信质量差，检查接线/波特率")
    print("=" * 65)

finally:
    tx_ser.close()
    rx_ser.close()
