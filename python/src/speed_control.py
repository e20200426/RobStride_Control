#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride 速度模式控制脚本 (修复版)
模式: Mode 2 (Speed Control Mode)
通信: 使用 WRITE_PARAMETER 更新 VELOCITY_TARGET (spd_ref)

用法: python3 speed_control.py <motor_id>
"""

import sys
import os
import time
import threading
import signal
from typing import Optional

# 尝试导入 SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    # 假设当前目录结构
    try:
        from robstride_dynamics.bus import RobstrideBus, Motor
        from robstride_dynamics.protocol import ParameterType
    except ImportError as e:
        print(f"❌ 无法导入 SDK: {e}")
        sys.exit(1)

class SpeedController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # 互斥锁，防止Socket冲突
        
        self.running = True
        self.connected = False
        self.target_velocity = 0.0
        self.current_status = None
        
        # 默认参数
        self.max_velocity = 20.0  # rad/s 安全限制
        self.kp = 2.0
        self.ki = 0.5
        self.accel_limit = 10.0   # rad/s² — 加速度限制，防止电流冲击

        self._cmd_velocity = 0.0  # 当前已发送的速度指令（经过斜坡处理）
        self._last_sent_velocity: Optional[float] = None  # 脏标志：避免重复写相同值

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def connect(self):
        print(f"🔍 正在连接 CAN 通道 {self.channel}...")
        
        # 定义电机
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-00") # 模型型号请根据实际修改
        }
        
        # 简单的校准参数
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            # 激活电机
            print(f"⚡ 激活电机 ID: {self.motor_id} ...")
            self.bus.enable(self.motor_name)
            time.sleep(0.5)

            # 设置模式、PID、限速并归零 —— control_vel 一次完成
            print("⚙️ 设置速度控制模式并写入参数...")
            self.bus.control_vel(
                self.motor_name,
                vel=0.0,
                kp=self.kp,
                ki=self.ki,
                max_velocity=self.max_velocity,
            )
            self._cmd_velocity = 0.0
            self._last_sent_velocity = 0.0
            
            self.connected = True
            print("✅ 初始化完成！")
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def loop(self):
        """控制线程：持续发送心跳/速度指令并读取状态"""
        print("🔄 控制循环已启动")
        LOOP_DT = 0.05  # 20 Hz 目标周期

        while self.running and self.connected:
            t_start = time.perf_counter()
            try:
                with self.lock:
                    # ── 速度斜坡：以 accel_limit 逐步逼近目标，避免突变
                    delta = self.target_velocity - self._cmd_velocity
                    max_step = self.accel_limit * LOOP_DT
                    if abs(delta) <= max_step:
                        self._cmd_velocity = self.target_velocity
                    else:
                        self._cmd_velocity += max_step * (1 if delta > 0 else -1)

                    # ── 脏标志：只在值变化时才发送 CAN 帧
                    if self._cmd_velocity != self._last_sent_velocity:
                        self.bus.write(
                            self.motor_name,
                            ParameterType.VELOCITY_TARGET,
                            self._cmd_velocity,
                        )
                        self._last_sent_velocity = self._cmd_velocity

            except Exception as e:
                print(f"⚠️ 通信错误: {e}")

            # ── 精确定时：补偿本次循环耗时，维持稳定 20 Hz
            elapsed = time.perf_counter() - t_start
            sleep_time = LOOP_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def set_velocity(self, vel: float):
        """设置目标速度（带限幅）"""
        vel = max(-self.max_velocity, min(self.max_velocity, vel))
        self.target_velocity = vel
        print(f" -> 目标设定: {self.target_velocity:.2f} rad/s")

    def stop_and_exit(self):
        print("\n🛑 正在停止...")
        self.running = False
        self.target_velocity = 0.0
        
        if self.bus and self.connected:
            try:
                with self.lock:
                    # 先停
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                    time.sleep(0.2)
                    # 禁用
                    self.bus.disable(self.motor_name)
            except Exception:
                pass
            self.bus.disconnect()
        sys.exit(0)

    def run_interactive(self):
        # 启动后台发送线程
        t = threading.Thread(target=self.loop, daemon=True)
        t.start()

        print("\n" + "="*40)
        print(f"🎮 速度控制台 (ID: {self.motor_id})")
        print("="*40)
        print("👉 直接输入数字 (rad/s) 回车即可改变速度")
        print("👉 输入 '0' 停止")
        print("👉 输入 'q' 退出")
        print(f"⚠️  当前安全限速: ±{self.max_velocity} rad/s")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{self.target_velocity:.1f} rad/s] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                try:
                    vel = float(cmd)
                    self.set_velocity(vel)
                except ValueError:
                    print("❌ 无效输入，请输入数字")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("用法: python3 speed_control.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = SpeedController(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()