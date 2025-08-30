
# Project: ArmGravComp-Qt (6-DOF Robotic Arm Gravity Compensation & Visualization)
# File: damiao_motor_friction_detection.py
# Copyright (c) 2025, Chen XingYu. All rights reserved.
#
# License: Non-Commercial Use Only / 仅限非商业使用
# -----------------------------------------------------------------------------
# 本代码及其衍生作品仅允许用于个人学习、学术研究与教学等非商业场景。
# 严禁任何形式的商业使用，包括但不限于：出售、付费服务、SaaS/在线服务、
# 广告变现、集成到商业产品或用于商业咨询/竞赛/投标等。如需商业授权，请
# 先行获得版权所有者书面许可并签署授权协议。
#
# 允许的非商业使用条件：
# 1) 保留本版权与许可声明；
# 2) 在衍生作品/发表物中署名（Lu Yaoheng）并标明来源仓库；
# 3) 不得移除或修改本段声明。
#
# 免责声明：本代码按“现状”提供，不含任何明示或默示担保。
# 作者不对因使用本代码产生的任何直接或间接损失承担责任。
# 使用者需自行评估并承担风险。
#
# English Summary:
# This code (damiao motor friction detection) is provided for personal,
# academic, and research purposes only. Any commercial use (sale, paid service,
# SaaS, ad-monetization, integration into commercial products, consultancy,
# competitions, bids, etc.) is strictly prohibited without prior written
# permission from the copyright holder.
#
# Keep this notice intact and provide proper attribution in derived works.
# Provided "as is" without warranty of any kind. Use at your own risk.
#
# Contact / 商务与授权联系: <cdssywc@163.com>
# 首次使用：

# 设置好串口参数和电机ID,自动获取摩擦系数
# 固定好电机，让电机空载
# 点击"检查电机状态"按钮
# 查看日志中的电机信息，确认连接正常


# 实时监控：

# 检查成功后会自动启动实时状态显示
# 按钮会变成红色的"停止状态监控"
# 再次点击可停止监控


# 开始测试：

# 确认电机状态正常后
# 选择要进行的摩擦力识别测试
# 测试开始时会自动停止状态监控
# 可以多测几次

# 安全特性：
# 温度超过阈值时会用颜色警告
# 显示电机错误状态（过压、欠压、过流等）
# # # 健康状态一目了然（正常显示绿色，异常显示红色）
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import time
import serial
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QLineEdit, QPushButton, QComboBox, QGroupBox, 
                            QTabWidget, QTextEdit, QFileDialog, QProgressBar, QMessageBox,
                            QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox, QSpinBox,
                            QDoubleSpinBox, QFormLayout, QGridLayout, QSplitter, QFrame,
                            QSizePolicy, QScrollArea)

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QIcon, QPixmap, QColor, QPalette

import matplotlib
matplotlib.use('Qt5Agg')

# 导入电机控制库 - 注意: 确保DM_CAN库在同一目录下
try:
    from DM_CAN import *
except ImportError:
    print("警告: 无法导入DM_CAN库，请确保该库文件在正确路径下")

# 设置支持中文的字体（常见 Windows 字体）
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']  # 优先黑体/微软雅黑
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示为方块


# 电机状态检查线程
class MotorStatusThread(QThread):
    status_updated = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    
    def __init__(self, params):
        super().__init__()
        self.params = params
        self.running = False
        self.motor = None
        self.motor_control = None
        self.serial_device = None
        
    def setup_motor(self):
        try:
            # 初始化电机和串口
            motor_type_dict = {
                "DM4310": DM_Motor_Type.DM4310,
                "DM4340": DM_Motor_Type.DM4340,
                "DM6006": DM_Motor_Type.DM6006,
                "DM8006": DM_Motor_Type.DM8006,
                "DM8009": DM_Motor_Type.DM8009,
                "DM10010L": DM_Motor_Type.DM10010L,
                "DM10010": DM_Motor_Type.DM10010,
            }
            
            motor_type = motor_type_dict.get(self.params['motor_type'], DM_Motor_Type.DM4310)
            
            self.motor = Motor(motor_type, self.params['node_id'], self.params['master_id'])
            self.serial_device = serial.Serial(
                self.params['com_port'], 
                self.params['baud_rate'], 
                timeout=0.5
            )
            self.motor_control = MotorControl(self.serial_device)
            self.motor_control.addMotor(self.motor)
            
            return True
        except Exception as e:
            self.log_message.emit(f"电机连接失败: {str(e)}")
            return False
    
    def run(self):
        if not self.setup_motor():
            return
            
        self.running = True
        while self.running:
            try:
                # 刷新电机状态
                self.motor_control.refresh_motor_status(self.motor)
                
                # 获取基本状态信息
                position = self.motor.getPosition()
                velocity = self.motor.getVelocity()
                torque = self.motor.getTorque()
                t_mos = self.motor.getT_MOS() if hasattr(self.motor, 'getT_MOS') else 0.0
                t_rotor = self.motor.getT_Rotor() if hasattr(self.motor, 'getT_Rotor') else 0.0
                
                # 根据电机实际状态判断
                # 检查是否已使能
                is_enabled = abs(velocity) > 0.001 or abs(torque) > 0.001
                
                # 检查温度状态
                has_temp_warning = t_mos > 60 or t_rotor > 80
                has_temp_error = t_mos > 80 or t_rotor > 100
                
                # 判断状态
                if has_temp_error:
                    status_icon = "🔥"
                    status_text = "过温"
                    has_error = True
                elif has_temp_warning:
                    status_icon = "⚠️"
                    status_text = "温度警告"
                    has_error = False
                elif is_enabled:
                    status_icon = "✅"
                    status_text = "运行中"
                    has_error = False
                else:
                    status_icon = "⭕"
                    status_text = "待机"
                    has_error = False
                
                # 获取所有状态信息
                status_data = {
                    'position': position,
                    'velocity': velocity,
                    'torque': torque,
                    't_mos': t_mos,
                    't_rotor': t_rotor,
                    'status': 1 if is_enabled else 0,
                    'status_text': status_text,
                    'status_icon': status_icon,
                    'is_healthy': not has_error,
                    'has_error': has_error
                }
                
                self.status_updated.emit(status_data)
                
            except Exception as e:
                self.log_message.emit(f"状态更新错误: {str(e)}")
                # 发送默认状态
                status_data = {
                    'position': 0.0,
                    'velocity': 0.0,
                    'torque': 0.0,
                    't_mos': 0.0,
                    't_rotor': 0.0,
                    'status': 0,
                    'status_text': "连接错误",
                    'status_icon': "❌",
                    'is_healthy': False,
                    'has_error': True
                }
                self.status_updated.emit(status_data)
                
            time.sleep(0.1)  # 100ms更新一次
    
    def stop(self):
        self.running = False
        self.wait()
        
        if self.serial_device and self.serial_device.is_open:
            self.serial_device.close()


# 摩擦力识别工作线程
class FrictionIdentifierThread(QThread):
    update_progress = pyqtSignal(int, str)
    update_plot = pyqtSignal(object, str)
    update_results = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    test_completed = pyqtSignal()
    
    def __init__(self, params, test_type):
        super().__init__()
        self.params = params
        self.test_type = test_type  # 'coulomb', 'static', 'comprehensive'
        self.running = True
        self.results = {}
        
        # 检查参数
        required_params = [
            'motor_type', 'node_id', 'master_id', 'com_port', 'baud_rate',
            'viscous_coeff', 'inertia', 'test_speeds', 'duration', 'settling_time',
            'torque_increment', 'max_torque'
        ]
        
        for param in required_params:
            if param not in self.params:
                self.log_message.emit(f"缺少参数: {param}")
                return
    
    def setup_motor(self):
        try:
            # 初始化电机和串口
            self.log_message.emit("正在连接电机...")
            motor_type_dict = {
                "DM4310": DM_Motor_Type.DM4310,
                "DM4340": DM_Motor_Type.DM4340,
                "DM6006": DM_Motor_Type.DM6006,
                "DM8006": DM_Motor_Type.DM8006,
                "DM8009": DM_Motor_Type.DM8009,
                "DM10010L": DM_Motor_Type.DM10010L,
                "DM10010": DM_Motor_Type.DM10010,
            }
            
            motor_type = motor_type_dict.get(self.params['motor_type'], DM_Motor_Type.DM4310)
            
            self.motor = Motor(motor_type, self.params['node_id'], self.params['master_id'])
            self.serial_device = serial.Serial(
                self.params['com_port'], 
                self.params['baud_rate'], 
                timeout=0.5
            )
            self.motor_control = MotorControl(self.serial_device)
            self.motor_control.addMotor(self.motor)
            
            #切换到MIT控制模式
            current_mode = self.motor_control.read_motor_param(self.motor, DM_variable.CTRL_MODE)
            if current_mode != 1:
                self.log_message.emit("当前不是MIT模式，正在切换...")
                if not self.motor_control.switchControlMode(self.motor, Control_Type.MIT):
                    self.log_message.emit("MIT控制模式设置失败")
                    return False
                else:        
                    self.log_message.emit("电机已切换到MIT模式")
            else:
                self.log_message.emit("电机已经是MIT模式")
            
            # 读取电机参数
            motor_info = {
                'sub_ver': self.motor_control.read_motor_param(self.motor, DM_variable.sub_ver),
                'gear_ratio': self.motor_control.read_motor_param(self.motor, DM_variable.Gr),
                'max_pos': self.motor_control.read_motor_param(self.motor, DM_variable.PMAX),
                'max_vel': self.motor_control.read_motor_param(self.motor, DM_variable.VMAX),
                'max_torque': self.motor_control.read_motor_param(self.motor, DM_variable.TMAX),
            }
            
            self.results['motor_info'] = motor_info
            self.log_message.emit(f"电机连接成功, 版本: {motor_info['sub_ver']}, 最大力矩: {motor_info['max_torque']}N·m")
            
            # 使能电机
            self.motor_control.enable(self.motor)
            time.sleep(0.5)  # 等待电机稳定
            
            return True
        except Exception as e:
            self.log_message.emit(f"电机连接失败: {str(e)}")
            return False
    
    def cleanup(self):
        try:
            if hasattr(self, 'motor_control') and hasattr(self, 'motor'):
                self.motor_control.disable(self.motor)
            
            if hasattr(self, 'serial_device'):
                self.serial_device.close()
            
            self.log_message.emit("已关闭电机连接")
        except Exception as e:
            self.log_message.emit(f"关闭连接时发生错误: {str(e)}")
    
    def run(self):
        try:
            # 设置电机连接
            if not self.setup_motor():
                self.cleanup()
                return
            
            # 根据测试类型执行不同测试
            if self.test_type == 'coulomb' or self.test_type == 'comprehensive':
                self.identify_coulomb_friction()
            
            if self.test_type == 'static' or self.test_type == 'comprehensive':
                self.identify_static_friction()
            
            # 更新结果
            self.results['viscous_friction'] = self.params['viscous_coeff']
            self.results['inertia'] = self.params['inertia']
            self.results['timestamp'] = datetime.now().isoformat()
            
            # 发送最终结果
            self.update_results.emit(self.results)
            self.log_message.emit("测试完成！")
            
        except Exception as e:
            self.log_message.emit(f"测试过程中发生错误: {str(e)}")
        finally:
            self.cleanup()
            self.test_completed.emit()
    
    def identify_coulomb_friction(self):
        """识别库仑摩擦力矩"""
        self.log_message.emit("\n开始库仑摩擦力矩识别...")
        test_speeds = self.params['test_speeds']
        duration = self.params['duration']
        settling_time = self.params['settling_time']
        
        self.log_message.emit(f"将测试 {len(test_speeds)} 种不同速度，每种速度测试 {duration} 秒")
        
        # 数据存储
        speeds = []
        torques = []
        
        kv = 0.5  # 速度反馈增益
        
        # 对每个测试速度进行测试
        for i, target_speed in enumerate(test_speeds):
            if not self.running:
                self.log_message.emit("测试被中断")
                return
            
            # 计算进度
            progress = int((i / len(test_speeds)) * 100)
            self.update_progress.emit(progress, "库仑摩擦识别")
            
            self.log_message.emit(f"\n[{i+1}/{len(test_speeds)}] 设置电机速度为 {target_speed:.2f} rad/s")
            
            # 开始时间
            start_time = time.time()
            collected_torques = []
            collected_speeds = []
            
            # 先让电机达到目标速度并稳定
            self.log_message.emit(f"  电机加速中...")
            while (time.time() - start_time) < settling_time and self.running:
                # 速度控制模式 (零位置增益，只用速度反馈)
                self.motor_control.controlMIT(self.motor, 0, kv, 0, target_speed, 0)
                self.motor_control.refresh_motor_status(self.motor)
                time.sleep(0.01)
            
            if not self.running:
                self.log_message.emit("测试被中断")
                return
            
            self.log_message.emit(f"  开始收集数据...")
            
            # 在稳定后收集力矩数据
            data_collection_start = time.time()
            while (time.time() - data_collection_start) < duration and self.running:
                # 保持速度控制
                self.motor_control.controlMIT(self.motor, 0, kv, 0, target_speed, 0)
                self.motor_control.refresh_motor_status(self.motor)
                
                # 获取当前速度和力矩
                current_speed = self.motor.getVelocity()
                current_torque = self.motor.getTorque()
                
                # 存储数据
                collected_speeds.append(current_speed)
                collected_torques.append(current_torque)
                
                time.sleep(0.01)
            
            if not self.running:
                self.log_message.emit("测试被中断")
                return
            
            # 计算平均速度和力矩
            avg_speed = np.mean(collected_speeds)
            avg_torque = np.mean(collected_torques)
            std_torque = np.std(collected_torques)
            
            # 存储结果
            speeds.append(avg_speed)
            torques.append(avg_torque)
            
            self.log_message.emit(f"  速度 {avg_speed:.3f} rad/s 的平均力矩: {avg_torque:.5f} ± {std_torque:.5f} N·m")
        
        # 数据分析
        self.update_progress.emit(90, "库仑摩擦识别")
        self.log_message.emit("分析测试数据...")
        
        # 转换为numpy数组
        speeds = np.array(speeds)
        torques = np.array(torques)
        
        # 分离正负速度数据
        pos_speeds = speeds[speeds > 0]
        pos_torques = torques[speeds > 0]
        
        neg_speeds = speeds[speeds < 0]
        neg_torques = torques[speeds < 0]
        
        # 去除粘滞摩擦影响后的力矩
        viscous_coeff = self.params['viscous_coeff']
        
        if len(pos_speeds) > 1:
            pos_coulomb_torques = pos_torques - viscous_coeff * pos_speeds
            T_coulomb_pos = float(np.mean(pos_coulomb_torques))
        else:
            T_coulomb_pos = 0
            self.log_message.emit("警告: 正方向速度数据不足，无法准确估计正方向库仑摩擦")
        
        if len(neg_speeds) > 1:
            neg_coulomb_torques = neg_torques - viscous_coeff * neg_speeds
            T_coulomb_neg = float(-np.mean(neg_coulomb_torques))  # 注意负号，使T_coulomb_neg为正值
        else:
            T_coulomb_neg = 0
            self.log_message.emit("警告: 负方向速度数据不足，无法准确估计负方向库仑摩擦")
        
        # 计算平均库仑摩擦力矩
        T_coulomb = (T_coulomb_pos + T_coulomb_neg) / 2.0
        
        # 绘图
        self._plot_coulomb_friction(speeds, torques, T_coulomb_pos, T_coulomb_neg, viscous_coeff)
        
        # 更新结果
        self.results['coulomb_friction'] = float(T_coulomb)
        self.results['coulomb_friction_pos'] = float(T_coulomb_pos)
        self.results['coulomb_friction_neg'] = float(T_coulomb_neg)
        self.results['coulomb_raw_data'] = {
            'speeds': speeds.tolist(),
            'torques': torques.tolist()
        }
        
        self.log_message.emit("\n=== 库仑摩擦力矩识别结果 ===")
        self.log_message.emit(f"正方向库仑摩擦: {T_coulomb_pos:.5f} N·m")
        self.log_message.emit(f"负方向库仑摩擦: {T_coulomb_neg:.5f} N·m")
        self.log_message.emit(f"平均库仑摩擦力矩: {T_coulomb:.5f} N·m")
        
        # 完成
        self.update_progress.emit(100, "库仑摩擦识别")
    
    def identify_static_friction(self):
        """识别静摩擦力矩"""
        self.log_message.emit("\n开始静摩擦力矩识别...")
        torque_increment = self.params['torque_increment']
        max_torque = self.params['max_torque']
        test_directions = [1, -1]  # 正向和负向测试
        
        # 数据存储
        static_results = []
        
        for i, direction in enumerate(test_directions):
            if not self.running:
                self.log_message.emit("测试被中断")
                return
            
            # 进度
            progress = int((i / len(test_directions)) * 100)
            self.update_progress.emit(progress, "静摩擦识别")
            
            direction_str = "正向" if direction > 0 else "负向"
            self.log_message.emit(f"\n测试{direction_str}静摩擦...")
            
            # 先重置到指定位置
            self._reset_position(0.0)
            if not self.running:
                return
                
            time.sleep(1.0)  # 等待完全静止
            
            # 记录初始位置
            self.motor_control.refresh_motor_status(self.motor)
            initial_pos = self.motor.getPosition()
            self.log_message.emit(f"  初始位置: {initial_pos:.4f} rad")
            
            # 缓慢增加力矩直到电机开始移动
            current_torque = 0.0
            start_time = time.time()
            movement_detected = False
            pos_data = []
            torque_data = []
            time_data = []
            velocity_data = []
            movement_threshold = 0.05  # rad/s，认为开始移动的速度阈值
            
            while not movement_detected and current_torque < max_torque and (time.time() - start_time) < 30 and self.running:
                # 施加力矩
                self.motor_control.controlMIT(self.motor, 0, 0, 0, 0, direction * current_torque)
                
                # 读取状态
                self.motor_control.refresh_motor_status(self.motor)
                current_pos = self.motor.getPosition()
                current_vel = self.motor.getVelocity()
                elapsed = time.time() - start_time
                
                # 记录数据
                pos_data.append(current_pos)
                torque_data.append(current_torque)
                time_data.append(elapsed)
                velocity_data.append(current_vel)
                
                # 检查是否开始移动
                if abs(current_vel) > movement_threshold:
                    movement_detected = True
                    break
                
                # 增加力矩
                current_torque += torque_increment
                
                # 限制采样率
                time.sleep(0.01)
                
                # 每增加0.05N·m显示一次当前力矩
                if int(current_torque * 1000) % 50 == 0:
                    self.log_message.emit(f"  当前测试力矩: {current_torque:.4f} N·m, 速度: {current_vel:.4f} rad/s")
                    # 更新小进度
                    mini_progress = min(int((current_torque / max_torque) * 100), 99)
                    self.update_progress.emit(
                        progress + mini_progress // len(test_directions), 
                        f"{direction_str}静摩擦识别"
                    )
            
            if not self.running:
                self.log_message.emit("测试被中断")
                return
            
            # 记录结果
            if movement_detected:
                breakaway_torque = current_torque
                self.log_message.emit(f"  检测到开始移动! 脱离力矩: {breakaway_torque:.5f} N·m")
                static_results.append((direction, breakaway_torque))
            else:
                self.log_message.emit(f"  未检测到明确的移动，达到最大测试力矩: {max_torque} N·m")
                static_results.append((direction, np.nan))
            
            # 绘制测试过程图
            self._plot_static_test(time_data, torque_data, velocity_data, pos_data, direction_str)
            
            # 停止电机
            self.motor_control.controlMIT(self.motor, 0, 0, 0, 0, 0)
            time.sleep(0.5)
        
        # 分析结果
        T_static_pos = next((t for d, t in static_results if d > 0), np.nan)
        T_static_neg = next((t for d, t in static_results if d < 0), np.nan)
        
        if not np.isnan(T_static_pos) and not np.isnan(T_static_neg):
            T_static = (T_static_pos + T_static_neg) / 2.0
        elif not np.isnan(T_static_pos):
            T_static = T_static_pos
        elif not np.isnan(T_static_neg):
            T_static = T_static_neg
        else:
            T_static = np.nan
        
        # 更新结果
        self.results['static_friction'] = float(T_static) if not np.isnan(T_static) else None
        self.results['static_friction_pos'] = float(T_static_pos) if not np.isnan(T_static_pos) else None
        self.results['static_friction_neg'] = float(T_static_neg) if not np.isnan(T_static_neg) else None
        
        self.log_message.emit("\n=== 静摩擦力矩识别结果 ===")
        self.log_message.emit(f"正方向静摩擦: {T_static_pos:.5f} N·m" if not np.isnan(T_static_pos) else "正方向静摩擦: 识别失败")
        self.log_message.emit(f"负方向静摩擦: {T_static_neg:.5f} N·m" if not np.isnan(T_static_neg) else "负方向静摩擦: 识别失败")
        self.log_message.emit(f"平均静摩擦力矩: {T_static:.5f} N·m" if not np.isnan(T_static) else "平均静摩擦力矩: 识别失败")
        
        # 完成
        self.update_progress.emit(100, "静摩擦识别")
    
    def _reset_position(self, target_pos=0.0):
        """重置电机到指定位置"""
        self.log_message.emit(f"  重置电机位置到 {target_pos} rad...")
        
        # 获取当前位置
        self.motor_control.refresh_motor_status(self.motor)
        current_pos = self.motor.getPosition()
        
        # PD控制参数
        kp = 5.0
        kd = 0.5
        
        # 位置误差阈值
        pos_threshold = 0.01  # rad
        
        # 最大尝试时间
        max_time = 3.0  # 秒
        start_time = time.time()
        
        while abs(current_pos - target_pos) > pos_threshold and (time.time() - start_time) < max_time and self.running:
            # 控制电机
            self.motor_control.controlMIT(self.motor, kp, kd, target_pos, 0, 0)
            
            # 更新位置
            self.motor_control.refresh_motor_status(self.motor)
            current_pos = self.motor.getPosition()
            
            time.sleep(0.01)
        
        if not self.running:
            return
            
        # 停止电机但保持位置
        self.motor_control.controlMIT(self.motor, kp, kd, target_pos, 0, 0)
        self.log_message.emit(f"  位置重置完成，当前位置: {current_pos:.4f} rad")
    
    def _plot_coulomb_friction(self, speeds, torques, T_coulomb_pos, T_coulomb_neg, viscous_coeff):
        """绘制库仑摩擦识别结果图"""
        try:
            fig = plt.figure(figsize=(12, 8))
            
            # 原始数据点
            plt.scatter(speeds, torques, color='blue', s=50, alpha=0.7, label='实测数据点', zorder=5)
            
            # 理论模型线
            x_model = np.linspace(min(speeds) - 0.5, max(speeds) + 0.5, 200)
            y_model_pos = np.where(x_model > 0, viscous_coeff * x_model + T_coulomb_pos, 0)
            y_model_neg = np.where(x_model < 0, viscous_coeff * x_model - T_coulomb_neg, 0)
            y_model = y_model_pos + y_model_neg
            plt.plot(x_model, y_model, 'r-', linewidth=2.5, label='摩擦模型', zorder=4)
            
            # 标记库仑摩擦力矩
            plt.axhline(y=T_coulomb_pos, color='g', linestyle='--', linewidth=2, alpha=0.8,
                      label=f'正向库仑摩擦: {T_coulomb_pos:.5f} N·m', zorder=3)
            plt.axhline(y=-T_coulomb_neg, color='m', linestyle='--', linewidth=2, alpha=0.8,
                      label=f'负向库仑摩擦: {T_coulomb_neg:.5f} N·m', zorder=3)
            
            # 添加坐标轴线
            plt.axhline(y=0, color='k', linestyle='-', alpha=0.3, zorder=1)
            plt.axvline(x=0, color='k', linestyle='-', alpha=0.3, zorder=1)
            
            plt.xlabel('角速度 [rad/s]', fontsize=14)
            plt.ylabel('力矩 [N·m]', fontsize=14)
            plt.title('电机库仑摩擦力矩识别结果', fontsize=16, fontweight='bold')
            plt.legend(fontsize=12, loc='best')
            plt.grid(True, alpha=0.3)
            
            # 添加统计信息文本框
            stats_text = f'测试点数: {len(speeds)}\n粘滞系数: {viscous_coeff:.6f} N·m·s/rad\n平均库仑摩擦: {(T_coulomb_pos + T_coulomb_neg)/2:.5f} N·m'
            plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    verticalalignment='top', fontsize=10)
            
            plt.tight_layout()
            
            # 保存图像并发送到主线程
            self.update_plot.emit(fig, 'coulomb_friction')
            
            # 保存到文件
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            fig.savefig(f'friction_coulomb_{timestamp}.png', dpi=300, bbox_inches='tight')
            plt.close(fig)
        except Exception as e:
            self.log_message.emit(f"库仑摩擦绘图错误: {str(e)}")
    
    def _plot_static_test(self, time_data, torque_data, velocity_data, pos_data, direction):
        """绘制静摩擦测试过程图"""
        try:
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

            # 力矩随时间变化
            ax1.plot(time_data, torque_data, 'r-', linewidth=1.5, label='施加力矩')
            ax1.set_ylabel('力矩 [N·m]', fontsize=12)
            ax1.set_title(f'{direction}静摩擦测试过程', fontsize=14, fontweight='bold')
            ax1.grid(True, alpha=0.3)
            ax1.legend(fontsize=10)

            # 速度随时间变化
            ax2.plot(time_data, velocity_data, 'g-', linewidth=1.5, label='角速度')
            ax2.axhline(y=0.05, color='orange', linestyle='--', alpha=0.7, label='运动阈值')
            ax2.axhline(y=-0.05, color='orange', linestyle='--', alpha=0.7)
            ax2.set_ylabel('速度 [rad/s]', fontsize=12)
            ax2.grid(True, alpha=0.3)
            ax2.legend(fontsize=10)

            # 位置随时间变化
            ax3.plot(time_data, pos_data, 'b-', linewidth=1.5, label='角位置')
            ax3.set_xlabel('时间 [s]', fontsize=12)
            ax3.set_ylabel('位置 [rad]', fontsize=12)
            ax3.grid(True, alpha=0.3)
            ax3.legend(fontsize=10)

            # 找出开始移动的点
            movement_threshold = 0.05
            move_indices = [i for i, v in enumerate(velocity_data) if abs(v) > movement_threshold]
            if move_indices:
                break_idx = move_indices[0]
                break_time = time_data[break_idx]
                break_torque = torque_data[break_idx]

                for ax in [ax1, ax2, ax3]:
                    ax.axvline(x=break_time, color='red', linestyle='--', linewidth=2, alpha=0.8)

                ax1.plot(break_time, break_torque, 'ro', markersize=8,
                         label=f'脱离点: {break_torque:.5f} N·m')
                ax1.legend(fontsize=10)

            plt.tight_layout()

            self.update_plot.emit(fig, f'static_friction_{direction}')

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            fig.savefig(f'friction_static_{direction}_{timestamp}.png', dpi=300, bbox_inches='tight')
            plt.close(fig)
        except Exception as e:
            self.log_message.emit(f"静摩擦绘图错误: {str(e)}")


# 电机状态显示组件
class MotorStatusWidget(QGroupBox):
    def __init__(self):
        super().__init__("电机实时状态")
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # 状态图标和文字
        status_layout = QHBoxLayout()
        self.status_icon_label = QLabel("❓")
        self.status_icon_label.setStyleSheet("font-size: 16px;")
        self.status_text_label = QLabel("未连接")
        self.status_text_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        
        status_layout.addWidget(self.status_icon_label)
        status_layout.addWidget(self.status_text_label)
        status_layout.addStretch()
        
        # 创建网格布局显示参数
        params_layout = QGridLayout()
        
        # 位置
        self.pos_label = QLabel("位置:")
        self.pos_value = QLabel("0.000 rad")
        self.pos_value.setStyleSheet("font-weight: bold; color: blue;")
        params_layout.addWidget(self.pos_label, 0, 0)
        params_layout.addWidget(self.pos_value, 0, 1)
        
        # 速度
        self.vel_label = QLabel("速度:")
        self.vel_value = QLabel("0.000 rad/s")
        self.vel_value.setStyleSheet("font-weight: bold; color: green;")
        params_layout.addWidget(self.vel_label, 0, 2)
        params_layout.addWidget(self.vel_value, 0, 3)
        
        # 扭矩
        self.torque_label = QLabel("扭矩:")
        self.torque_value = QLabel("0.000 N·m")
        self.torque_value.setStyleSheet("font-weight: bold; color: red;")
        params_layout.addWidget(self.torque_label, 1, 0)
        params_layout.addWidget(self.torque_value, 1, 1)
        
        # MOS温度
        self.mos_temp_label = QLabel("MOS温度:")
        self.mos_temp_value = QLabel("0.0 ℃")
        self.mos_temp_value.setStyleSheet("font-weight: bold; color: orange;")
        params_layout.addWidget(self.mos_temp_label, 1, 2)
        params_layout.addWidget(self.mos_temp_value, 1, 3)
        
        # 电机温度
        self.rotor_temp_label = QLabel("线圈温度:")
        self.rotor_temp_value = QLabel("0.0 ℃")
        self.rotor_temp_value.setStyleSheet("font-weight: bold; color: purple;")
        params_layout.addWidget(self.rotor_temp_label, 2, 0)
        params_layout.addWidget(self.rotor_temp_value, 2, 1)
        
        # 健康状态
        self.health_label = QLabel("健康状态:")
        self.health_value = QLabel("未知")
        params_layout.addWidget(self.health_label, 2, 2)
        params_layout.addWidget(self.health_value, 2, 3)
        
        layout.addLayout(status_layout)
        layout.addLayout(params_layout)
        self.setLayout(layout)
    
    def update_status(self, status_data):
        """更新状态显示"""
        self.status_icon_label.setText(status_data['status_icon'])
        self.status_text_label.setText(status_data['status_text'])
        
        self.pos_value.setText(f"{status_data['position']:.3f} rad")
        self.vel_value.setText(f"{status_data['velocity']:.3f} rad/s")
        self.torque_value.setText(f"{status_data['torque']:.3f} N·m")
        self.mos_temp_value.setText(f"{status_data['t_mos']:.1f} ℃")
        self.rotor_temp_value.setText(f"{status_data['t_rotor']:.1f} ℃")
        
        # 更新健康状态
        if status_data['has_error']:
            self.health_value.setText("异常")
            self.health_value.setStyleSheet("font-weight: bold; color: red;")
        elif status_data['is_healthy']:
            self.health_value.setText("正常")
            self.health_value.setStyleSheet("font-weight: bold; color: green;")
        else:
            self.health_value.setText("未知")
            self.health_value.setStyleSheet("font-weight: bold; color: gray;")
        
        # 根据温度改变颜色
        if status_data['t_mos'] > 60:
            self.mos_temp_value.setStyleSheet("font-weight: bold; color: red;")
        elif status_data['t_mos'] > 40:
            self.mos_temp_value.setStyleSheet("font-weight: bold; color: orange;")
        else:
            self.mos_temp_value.setStyleSheet("font-weight: bold; color: green;")
            
        if status_data['t_rotor'] > 80:
            self.rotor_temp_value.setStyleSheet("font-weight: bold; color: red;")
        elif status_data['t_rotor'] > 60:
            self.rotor_temp_value.setStyleSheet("font-weight: bold; color: orange;")
        else:
            self.rotor_temp_value.setStyleSheet("font-weight: bold; color: green;")


# 主窗口
class FrictionIdentifierApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("电机摩擦力参数自动识别工具 v2.0")
        self.setMinimumSize(1300, 700)
        
        self.results = {}
        self.identifier_thread = None
        self.status_thread = None
        
        # 设置默认参数
        self.default_params = {
            'motor_type': "DM4310",
            'node_id': 0x01,
            'master_id': 0x11,
            'com_port': 'COM5',
            'baud_rate': 921600,
            'viscous_coeff': 0.0001330964,
            'inertia': 1.976204E-05,
            'test_speeds': [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 1.5, 2.0, -0.001, -0.005, -0.01, -0.05, -0.5, -1.0, -1.5, -2.0],
            'duration': 5,
            'settling_time': 0.5,
            'torque_increment': 0.0001,
            'max_torque': 0.5
        }
        
        self.setup_ui()
        
        # 设置初始状态显示
        self.motor_status_widget.update_status({
            'position': 0.0,
            'velocity': 0.0,
            'torque': 0.0,
            't_mos': 0.0,
            't_rotor': 0.0,
            'status': 0,
            'status_text': "未连接",
            'status_icon': "❓",
            'is_healthy': True,
            'has_error': False
        })
    
    def setup_ui(self):
        # 中央小部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)

        # 创建主标签页
        self.main_tabs = QTabWidget()
        main_layout.addWidget(self.main_tabs)
        
        # 第一个标签页：参数设置和控制
        self.setup_control_tab()
        
        # 第二个标签页：库仑摩擦图表
        self.setup_coulomb_tab()
        
        # 第三个标签页：静摩擦图表
        self.setup_static_tab()
        
        # 连接信号
        self.check_motor_btn.clicked.connect(self.check_motor_status)
        self.read_dyn_btn.clicked.connect(self.read_dynamics_from_motor)
        self.start_coulomb_btn.clicked.connect(lambda: self.start_identification('coulomb'))
        self.start_static_btn.clicked.connect(lambda: self.start_identification('static'))
        self.start_comprehensive_btn.clicked.connect(lambda: self.start_identification('comprehensive'))
        self.stop_btn.clicked.connect(self.stop_identification)
        self.save_results_btn.clicked.connect(self.save_results)
        self.load_results_btn.clicked.connect(self.load_results)
        
        # 初始日志
        self.log("电机摩擦力参数自动识别工具 v2.0 已启动")
        self.log("请先点击'检查电机状态'进行连接测试")
    
    def setup_control_tab(self):
        """设置第一个标签页：参数设置和控制"""
        control_tab = QWidget()
        self.main_tabs.addTab(control_tab, "参数设置与控制")
        
        control_layout = QVBoxLayout(control_tab)
        control_layout.setContentsMargins(5, 5, 5, 5)
        control_layout.setSpacing(5)
        
        # 创建分割器
        splitter = QSplitter(Qt.Vertical)
        control_layout.addWidget(splitter)
        
        # 上半部分 - 参数设置和状态显示
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(2, 2, 2, 2)
        top_layout.setSpacing(5)
        
        # 左侧 - 参数设置区域
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        param_group = QGroupBox("参数设置")
        param_layout = QGridLayout()
        
        # 电机参数
        motor_group = QGroupBox("电机参数")
        motor_layout = QFormLayout()
        
        self.motor_type_combo = QComboBox()
        motor_types = ["DM4310", "DM4340", "DM6006", "DM8006", "DM8009", "DM10010L", "DM10010"]
        self.motor_type_combo.addItems(motor_types)
        
        self.node_id_edit = QLineEdit(hex(self.default_params['node_id']))
        self.master_id_edit = QLineEdit(hex(self.default_params['master_id']))
        self.com_port_edit = QLineEdit(self.default_params['com_port'])
        self.baud_rate_edit = QLineEdit(str(self.default_params['baud_rate']))
        
        motor_layout.addRow("电机型号:", self.motor_type_combo)
        motor_layout.addRow("节点ID(hex):", self.node_id_edit)
        motor_layout.addRow("主控ID(hex):", self.master_id_edit)
        motor_layout.addRow("串口:", self.com_port_edit)
        motor_layout.addRow("波特率:", self.baud_rate_edit)
        motor_group.setLayout(motor_layout)
        
        # 动力学参数
        dynamics_group = QGroupBox("动力学参数")
        dynamics_layout = QFormLayout()
        
        self.viscous_coeff_edit = QLineEdit(str(self.default_params['viscous_coeff']))
        self.inertia_edit = QLineEdit(str(self.default_params['inertia']))

        dynamics_layout.addRow("粘滞摩擦系数 [N·m·s/rad]:", self.viscous_coeff_edit)
        dynamics_layout.addRow("转子惯量 [kg·m²]:", self.inertia_edit)
        dynamics_group.setLayout(dynamics_layout)

        # 自动读取按钮
        self.read_dyn_btn = QPushButton("自动读取")
        self.read_dyn_btn.setToolTip("从电机读取粘滞系数 (Damp) 与转动惯量 (Inertia)")
        self.read_dyn_btn.setMinimumHeight(24)
        dynamics_layout.addRow(self.read_dyn_btn)

        dynamics_group.setLayout(dynamics_layout)

        # 测试参数
        test_group = QGroupBox("测试参数")
        test_layout = QFormLayout()
        
        self.test_speeds_edit = QLineEdit(str(self.default_params['test_speeds']).replace('[', '').replace(']', ''))
        self.duration_edit = QLineEdit(str(self.default_params['duration']))
        self.settling_time_edit = QLineEdit(str(self.default_params['settling_time']))
        self.torque_increment_edit = QLineEdit(str(self.default_params['torque_increment']))
        self.max_torque_edit = QLineEdit(str(self.default_params['max_torque']))
        
        test_layout.addRow("测试速度 [rad/s] (逗号分隔):", self.test_speeds_edit)
        test_layout.addRow("数据采集时间 [s]:", self.duration_edit)
        test_layout.addRow("速度稳定时间 [s]:", self.settling_time_edit)
        test_layout.addRow("力矩增量 [N·m]:", self.torque_increment_edit)
        test_layout.addRow("最大测试力矩 [N·m]:", self.max_torque_edit)
        test_group.setLayout(test_layout)
        
        # 添加参数组到参数布局
        param_layout.addWidget(motor_group, 0, 0)
        param_layout.addWidget(dynamics_group, 0, 1)
        param_layout.addWidget(test_group, 1, 0, 1, 2)
        
        param_group.setLayout(param_layout)
        left_layout.addWidget(param_group)
        
        # 右侧 - 电机状态和控制
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # 电机状态显示
        self.motor_status_widget = MotorStatusWidget()
        right_layout.addWidget(self.motor_status_widget)
        
        # 控制按钮区域（两列）
        control_group = QGroupBox("控制面板，提示：让电机空载，并固定电机")
        control_btn_layout = QGridLayout()
        control_btn_layout.setContentsMargins(6, 6, 6, 6)
        control_btn_layout.setHorizontalSpacing(8)
        control_btn_layout.setVerticalSpacing(6)

        # 电机检查按钮（整行占两列）
        self.check_motor_btn = QPushButton("检查电机状态")
        self.check_motor_btn.setMinimumHeight(30)
        self.check_motor_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; font-size: 10px; }"
            "QPushButton:hover { background-color: #45a049; }"
        )
        control_btn_layout.addWidget(self.check_motor_btn, 0, 0, 1, 2)

        # 分隔线（整行占两列）
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setFrameShadow(QFrame.Sunken)
        control_btn_layout.addWidget(line1, 1, 0, 1, 2)

        # 功能按钮（两列排布）
        self.start_coulomb_btn = QPushButton("识别库仑摩擦")
        self.start_static_btn = QPushButton("识别静摩擦")
        self.start_comprehensive_btn = QPushButton("全面识别")
        self.stop_btn = QPushButton("停止")
        self.save_results_btn = QPushButton("保存结果")
        self.load_results_btn = QPushButton("加载结果")

        for btn in [self.start_coulomb_btn, self.start_static_btn, self.start_comprehensive_btn,
                    self.stop_btn, self.save_results_btn, self.load_results_btn]:
            btn.setMinimumHeight(25)

        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; font-weight: bold; }"
            "QPushButton:disabled { background-color: #cccccc; }"
        )

        # 网格位置：
        # 第2行：库仑/静摩擦
        control_btn_layout.addWidget(self.start_coulomb_btn,     2, 0)
        control_btn_layout.addWidget(self.start_static_btn,      2, 1)
        # 第3行：全面识别/停止
        control_btn_layout.addWidget(self.start_comprehensive_btn, 3, 0)
        control_btn_layout.addWidget(self.stop_btn,                3, 1)
        # 第4行：保存/加载
        control_btn_layout.addWidget(self.save_results_btn,      4, 0)
        control_btn_layout.addWidget(self.load_results_btn,      4, 1)

        # 进度与状态（右侧进度条，左侧标签）
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_label = QLabel("就绪")
        self.progress_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        control_btn_layout.addWidget(QLabel("进度:"), 5, 0, alignment=Qt.AlignRight | Qt.AlignVCenter)
        control_btn_layout.addWidget(self.progress_bar, 5, 1)
        control_btn_layout.addWidget(self.progress_label, 6, 0, 1, 2)

        # 拉伸留白
        control_btn_layout.setRowStretch(7, 1)

        control_group.setLayout(control_btn_layout)
        right_layout.addWidget(control_group)
        
        # 添加左右部分到顶部布局
        top_layout.addWidget(left_widget, 7)
        top_layout.addWidget(right_widget, 3)
        
        # 下半部分 - 子标签页（日志和结果）
        bottom_widget = QWidget()
        bottom_layout = QVBoxLayout(bottom_widget)
       
        self.sub_tabs = QTabWidget()
        
        # 日志标签页
        self.log_tab = QWidget()
        log_layout = QVBoxLayout(self.log_tab)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)  # 限制高度
        log_layout.addWidget(self.log_text)
        
        # 结果标签页
        self.results_tab = QWidget()
        results_layout = QVBoxLayout(self.results_tab)
        self.results_table = QTableWidget(0, 2)
        self.results_table.setHorizontalHeaderLabels(["参数 / Parameter", "值 / Value"])
        self.results_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.results_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        results_layout.addWidget(self.results_table)
        
        # 添加子标签页
        self.sub_tabs.addTab(self.log_tab, "日志")
        self.sub_tabs.addTab(self.results_tab, "结果")
        
        bottom_layout.addWidget(self.sub_tabs)
        
        # 添加上下部分到分割器
        splitter.addWidget(top_widget)
        splitter.addWidget(bottom_widget)
        splitter.setSizes([600, 300])
    
    def setup_coulomb_tab(self):
        """设置第二个标签页：库仑摩擦图表"""
        coulomb_tab = QWidget()
        self.main_tabs.addTab(coulomb_tab, "库仑摩擦图表")
        
        coulomb_layout = QVBoxLayout(coulomb_tab)
        coulomb_layout.setContentsMargins(5, 5, 5, 5)
        
        # 添加标题和说明
        title_label = QLabel("库仑摩擦力矩识别结果")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #2c3e50; padding: 10px;")
        title_label.setAlignment(Qt.AlignCenter)
        coulomb_layout.addWidget(title_label)
        
        desc_label = QLabel("库仑摩擦是物体在相对运动时产生的摩擦阻力，与运动方向相反，大小基本恒定。\n本测试通过不同速度下的力矩测量来识别正负方向的库仑摩擦力矩。")
        desc_label.setStyleSheet("color: #7f8c8d; padding: 5px; font-size: 11px;")
        desc_label.setAlignment(Qt.AlignCenter)
        coulomb_layout.addWidget(desc_label)
        
        # 创建滚动区域用于图表
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        self.coulomb_canvas_widget = QWidget()
        self.coulomb_layout = QVBoxLayout(self.coulomb_canvas_widget)
        scroll_area.setWidget(self.coulomb_canvas_widget)
        coulomb_layout.addWidget(scroll_area)
        
        # 添加提示标签
        info_label = QLabel("请先运行库仑摩擦识别测试以显示图表")
        info_label.setStyleSheet("color: #95a5a6; font-style: italic; padding: 20px;")
        info_label.setAlignment(Qt.AlignCenter)
        self.coulomb_layout.addWidget(info_label)
    
    def setup_static_tab(self):
        """设置第三个标签页：静摩擦图表"""
        static_tab = QWidget()
        self.main_tabs.addTab(static_tab, "静摩擦图表")
        
        static_layout = QVBoxLayout(static_tab)
        static_layout.setContentsMargins(5, 5, 5, 5)
        
        # 添加标题和说明
        title_label = QLabel("静摩擦力矩识别结果")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #2c3e50; padding: 10px;")
        title_label.setAlignment(Qt.AlignCenter)
        static_layout.addWidget(title_label)
        
        desc_label = QLabel("静摩擦是物体由静止状态开始运动时需要克服的最大摩擦阻力。\n本测试通过逐渐增加力矩直到电机开始转动来识别静摩擦的脱离力矩。")
        desc_label.setStyleSheet("color: #7f8c8d; padding: 5px; font-size: 11px;")
        desc_label.setAlignment(Qt.AlignCenter)
        static_layout.addWidget(desc_label)
        
        # 创建滚动区域用于图表
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        self.static_canvas_widget = QWidget()
        self.static_layout = QVBoxLayout(self.static_canvas_widget)
        scroll_area.setWidget(self.static_canvas_widget)
        static_layout.addWidget(scroll_area)
        
        # 添加提示标签
        info_label = QLabel("请先运行静摩擦识别测试以显示图表")
        info_label.setStyleSheet("color: #95a5a6; font-style: italic; padding: 20px;")
        info_label.setAlignment(Qt.AlignCenter)
        self.static_layout.addWidget(info_label)
    
    def check_motor_status(self):
        """检查电机状态"""
        if self.status_thread and self.status_thread.isRunning():
            self.log("停止实时状态更新...")
            self.status_thread.stop()
            self.check_motor_btn.setText("检查电机状态")
            self.check_motor_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; font-size: 12px; } QPushButton:hover { background-color: #45a049; }")
            return
            
        params = self.get_params_from_ui()
        if params is None:
            return
            
        self.log("\n开始检查电机状态...")
        
        try:
            # 临时连接以检查电机
            motor_type_dict = {
                "DM4310": DM_Motor_Type.DM4310,
                "DM4340": DM_Motor_Type.DM4340,
                "DM6006": DM_Motor_Type.DM6006,
                "DM8006": DM_Motor_Type.DM8006,
                "DM8009": DM_Motor_Type.DM8009,
                "DM10010L": DM_Motor_Type.DM10010L,
                "DM10010": DM_Motor_Type.DM10010,
            }
            
            motor_type = motor_type_dict.get(params['motor_type'], DM_Motor_Type.DM4310)
            
            motor = Motor(motor_type, params['node_id'], params['master_id'])
            serial_device = serial.Serial(params['com_port'], params['baud_rate'], timeout=0.5)
            motor_control = MotorControl(serial_device)
            motor_control.addMotor(motor)
            
            self.log(f"连接成功: {params['com_port']} @ {params['baud_rate']} bps")
            
            # 读取电机信息
            self.log("\n读取电机参数...")
            motor_info = {
                '软件版本': motor_control.read_motor_param(motor, DM_variable.sub_ver),
                '控制模式': motor_control.read_motor_param(motor, DM_variable.CTRL_MODE),
                '电机ID': motor_control.read_motor_param(motor, DM_variable.ESC_ID),
                '主控ID': motor_control.read_motor_param(motor, DM_variable.MST_ID),
                '减速比': motor_control.read_motor_param(motor, DM_variable.Gr),
                '最大位置': motor_control.read_motor_param(motor, DM_variable.PMAX),
                '最大速度': motor_control.read_motor_param(motor, DM_variable.VMAX),
                '最大扭矩': motor_control.read_motor_param(motor, DM_variable.TMAX),
            }
            
            # 控制模式映射
            control_mode_map = {
                1: "MIT模式",
                2: "位置速度模式",
                3: "速度模式",
                4: "力位混合模式"
            }
            
            self.log("\n=== 电机信息 ===")
            for key, value in motor_info.items():
                if key == '控制模式':
                    mode_text = control_mode_map.get(value, f"未知模式({value})")
                    self.log(f"{key}: {mode_text}")
                else:
                    self.log(f"{key}: {value}")
            
            # 获取实时状态
            motor_control.refresh_motor_status(motor)
            current_pos = motor.getPosition()
            current_vel = motor.getVelocity()
            current_torque = motor.getTorque()
            t_mos = motor.getT_MOS() if hasattr(motor, 'getT_MOS') else 0.0
            t_rotor = motor.getT_Rotor() if hasattr(motor, 'getT_Rotor') else 0.0
            
            self.log("\n=== 实时状态 ===")
            self.log(f"位置: {current_pos:.3f} rad")
            self.log(f"速度: {current_vel:.3f} rad/s")
            self.log(f"扭矩: {current_torque:.3f} N·m")
            self.log(f"MOS温度: {t_mos:.1f} ℃")
            self.log(f"线圈温度: {t_rotor:.1f} ℃")
            
            # 判断电机状态
            is_enabled = abs(current_vel) > 0.001 or abs(current_torque) > 0.001
            has_temp_warning = t_mos > 60 or t_rotor > 80
            
            if has_temp_warning:
                self.log("电机状态: ⚠️ 温度警告")
            elif is_enabled:
                self.log("电机状态: ✅ 运行中")
            else:
                self.log("电机状态: ⭕ 待机")
            
            # 更新初始状态显示
            initial_status = {
                'position': current_pos,
                'velocity': current_vel,
                'torque': current_torque,
                't_mos': t_mos,
                't_rotor': t_rotor,
                'status': 1 if is_enabled else 0,
                'status_text': "运行中" if is_enabled else "待机",
                'status_icon': "✅" if is_enabled else "⭕",
                'is_healthy': not has_temp_warning,
                'has_error': False
            }
            self.motor_status_widget.update_status(initial_status)
            
            # 关闭临时连接
            serial_device.close()
            
            # 启动实时状态监控线程
            self.status_thread = MotorStatusThread(params)
            self.status_thread.status_updated.connect(self.motor_status_widget.update_status)
            self.status_thread.log_message.connect(self.log)
            self.status_thread.start()
            
            self.check_motor_btn.setText("停止状态监控")
            self.check_motor_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; font-size: 12px; } QPushButton:hover { background-color: #da190b; }")
            
            self.log("\n电机检查完成，已启动实时状态监控")
            
        except Exception as e:
            self.log(f"电机检查失败: {str(e)}")
            self.log("请检查连接参数是否正确")
    
    def get_params_from_ui(self):
        """从UI控件获取参数"""
        try:
            # 解析测试速度
            test_speeds_str = self.test_speeds_edit.text().strip()
            test_speeds = [float(x.strip()) for x in test_speeds_str.split(',')]
            
            # 解析电机ID (十六进制)
            node_id_str = self.node_id_edit.text().strip()
            if node_id_str.startswith('0x'):
                node_id = int(node_id_str, 16)
            else:
                node_id = int(node_id_str, 16) if all(c in '0123456789abcdefABCDEF' for c in node_id_str) else int(node_id_str)
                
            master_id_str = self.master_id_edit.text().strip()
            if master_id_str.startswith('0x'):
                master_id = int(master_id_str, 16)
            else:
                master_id = int(master_id_str, 16) if all(c in '0123456789abcdefABCDEF' for c in master_id_str) else int(master_id_str)
            
            params = {
                'motor_type': self.motor_type_combo.currentText(),
                'node_id': node_id,
                'master_id': master_id,
                'com_port': self.com_port_edit.text().strip(),
                'baud_rate': int(self.baud_rate_edit.text().strip()),
                'viscous_coeff': float(self.viscous_coeff_edit.text().strip()),
                'inertia': float(self.inertia_edit.text().strip()),
                'test_speeds': test_speeds,
                'duration': float(self.duration_edit.text().strip()),
                'settling_time': float(self.settling_time_edit.text().strip()),
                'torque_increment': float(self.torque_increment_edit.text().strip()),
                'max_torque': float(self.max_torque_edit.text().strip())
            }
            
            return params
        except Exception as e:
            self.log(f"参数解析错误: {str(e)}")
            return None
    def read_dynamics_from_motor(self):
        """从电机读取粘滞系数(Damp)与转动惯量(Inertia)，并回填到动力学参数。
           若实时状态监控占用串口，则临时暂停监控，读取完成后自动恢复。"""
        ser = None
        was_monitoring = False
        try:
            # 如果正在监控，占用着串口 => 暂停
            if self.status_thread and self.status_thread.isRunning():
                was_monitoring = True
                self.log("检测到实时状态监控占用串口，正在暂时暂停监控以读取参数...")
                self.status_thread.stop()
                self.check_motor_btn.setText("检查电机状态")
                self.check_motor_btn.setStyleSheet(
                    "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; font-size: 12px; } "
                    "QPushButton:hover { background-color: #45a049; }"
                )

            # 解析必要连接参数（只解析连接必需项，避免其它字段出错）
            motor_type_text = self.motor_type_combo.currentText().strip()
            node_id_str = self.node_id_edit.text().strip()
            master_id_str = self.master_id_edit.text().strip()
            com_port = self.com_port_edit.text().strip()
            baud_rate = int(self.baud_rate_edit.text().strip())

            def _parse_id(s: str) -> int:
                if s.startswith(("0x", "0X")):
                    return int(s, 16)
            # 仅包含十六进制字符也按16进制解析，否则按10进制
                return int(s, 16) if all(c in "0123456789abcdefABCDEF" for c in s) else int(s)

            node_id = _parse_id(node_id_str)
            master_id = _parse_id(master_id_str)

            motor_type_map = {
                "DM4310": DM_Motor_Type.DM4310,
                "DM4340": DM_Motor_Type.DM4340,
                "DM6006": DM_Motor_Type.DM6006,
                "DM8006": DM_Motor_Type.DM8006,
                "DM8009": DM_Motor_Type.DM8009,
                "DM10010L": DM_Motor_Type.DM10010L,
                "DM10010": DM_Motor_Type.DM10010,
            }
            motor_type = motor_type_map.get(motor_type_text, DM_Motor_Type.DM4310)

            self.log("正在读取电机动力学参数 (Damp / Inertia)...")

            # 独立短连接，读取后立刻关闭，避免长期占用
            ser = serial.Serial(com_port, baud_rate, timeout=0.5)
            mc = MotorControl(ser)
            motor = Motor(motor_type, node_id, master_id)
            mc.addMotor(motor)

            # 直接读寄存器
            damp = mc.read_motor_param(motor, DM_variable.Damp)
            inertia = mc.read_motor_param(motor, DM_variable.Inertia)

            # 回填界面（存在才写）
            if damp is not None:
                self.viscous_coeff_edit.setText(f"{float(damp):.10g}")
            if inertia is not None:
                self.inertia_edit.setText(f"{float(inertia):.10g}")

            self.log(f"读取完成：Damp={damp if damp is not None else 'None'}，Inertia={inertia if inertia is not None else 'None'}")

        except Exception as e:
            self.log(f"读取动力学参数失败：{e}")
            QMessageBox.warning(self, "读取失败", f"读取电机动力学参数失败：\n{e}")
        finally:
            # 确保串口关闭
            try:
                if ser and ser.is_open:
                    ser.close()
            except:
                pass

            # 如之前在监控，自动恢复监控
            if was_monitoring:
                try:
                    params = self.get_params_from_ui()
                    if params:
                        self.status_thread = MotorStatusThread(params)
                        self.status_thread.status_updated.connect(self.motor_status_widget.update_status)
                        self.status_thread.log_message.connect(self.log)
                        self.status_thread.start()
                        self.check_motor_btn.setText("停止状态监控")
                        self.check_motor_btn.setStyleSheet(
                            "QPushButton { background-color: #f44336; color: white; font-weight: bold; font-size: 12px; } "
                            "QPushButton:hover { background-color: #da190b; }"
                        )
                        self.log("已恢复实时状态监控。")
                    else:
                        self.log("未能恢复状态监控：参数解析失败。")
                except Exception as re:
                    self.log(f"恢复状态监控失败：{re}")
    
    def start_identification(self, test_type):
        """开始识别过程"""
        # 停止状态监控
        if self.status_thread and self.status_thread.isRunning():
            self.status_thread.stop()
            self.check_motor_btn.setText("检查电机状态")
            self.check_motor_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; font-size: 12px; } QPushButton:hover { background-color: #45a049; }")
            
        if self.identifier_thread is not None and self.identifier_thread.isRunning():
            self.log("已有识别进程在运行，请等待完成或停止")
            return
        
        # 获取参数
        params = self.get_params_from_ui()
        if params is None:
            return
        
        # 创建工作线程
        self.identifier_thread = FrictionIdentifierThread(params, test_type)
        
        # 连接信号
        self.identifier_thread.update_progress.connect(self.update_progress)
        self.identifier_thread.update_plot.connect(self.update_plot)
        self.identifier_thread.update_results.connect(self.update_results)
        self.identifier_thread.log_message.connect(self.log)
        self.identifier_thread.test_completed.connect(self.on_test_completed)
        
        # 更新UI状态
        self.check_motor_btn.setEnabled(False)
        self.start_coulomb_btn.setEnabled(False)
        self.start_static_btn.setEnabled(False)
        self.start_comprehensive_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_results_btn.setEnabled(False)
        self.load_results_btn.setEnabled(False)
        
        # 清除之前的图表
        self.clear_plots()
        
        # 开始线程
        self.log(f"开始 {test_type} 摩擦识别过程...")
        self.identifier_thread.start()
    
    def stop_identification(self):
        """停止识别过程"""
        if self.identifier_thread is not None and self.identifier_thread.isRunning():
            self.log("正在停止识别过程...")
            self.identifier_thread.running = False
            self.identifier_thread.wait(2000)  # 等待线程结束
            self.on_test_completed()
    
    def on_test_completed(self):
        """测试完成处理"""
        # 更新UI状态
        self.check_motor_btn.setEnabled(True)
        self.start_coulomb_btn.setEnabled(True)
        self.start_static_btn.setEnabled(True)
        self.start_comprehensive_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.save_results_btn.setEnabled(True)
        self.load_results_btn.setEnabled(True)
    
    def update_progress(self, value, status):
        """更新进度条"""
        self.progress_bar.setValue(value)
        self.progress_label.setText(status)
    
    def update_plot(self, fig, plot_type):
        """更新图表"""
        # 创建一个 matplotlib 画布
        canvas = FigureCanvas(fig)
        toolbar = NavigationToolbar(canvas, self)
        
        # 清除之前的内容并添加新画布
        if plot_type == 'coulomb_friction':
            # 清除之前的子控件
            while self.coulomb_layout.count():
                item = self.coulomb_layout.takeAt(0)
                widget = item.widget()
                if widget:
                    widget.deleteLater()
            
            self.coulomb_layout.addWidget(toolbar)
            self.coulomb_layout.addWidget(canvas)
            self.main_tabs.setCurrentIndex(1)  # 切换到库仑摩擦标签页
        
        elif plot_type.startswith('static_friction'):
            # 清除之前的子控件
            while self.static_layout.count():
                item = self.static_layout.takeAt(0)
                widget = item.widget()
                if widget:
                    widget.deleteLater()
            
            self.static_layout.addWidget(toolbar)
            self.static_layout.addWidget(canvas)
            self.main_tabs.setCurrentIndex(2)  # 切换到静摩擦标签页
    
    def clear_plots(self):
        """清除图表"""
        # 清除库仑摩擦图表
        while self.coulomb_layout.count():
            item = self.coulomb_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        
        # 添加提示标签
        info_label = QLabel("请先运行库仑摩擦识别测试以显示图表")
        info_label.setStyleSheet("color: #95a5a6; font-style: italic; padding: 20px;")
        info_label.setAlignment(Qt.AlignCenter)
        self.coulomb_layout.addWidget(info_label)
        
        # 清除静摩擦图表
        while self.static_layout.count():
            item = self.static_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        
        # 添加提示标签
        info_label2 = QLabel("请先运行静摩擦识别测试以显示图表")
        info_label2.setStyleSheet("color: #95a5a6; font-style: italic; padding: 20px;")
        info_label2.setAlignment(Qt.AlignCenter)
        self.static_layout.addWidget(info_label2)
    
    def update_results(self, results):
        """更新结果表格"""
        self.results = results
        self.results_table.setRowCount(0)  # 清空表格
        
        # 填充结果表格
        row = 0
        
        # 添加库仑摩擦结果
        if 'coulomb_friction' in results:
            self.add_result_row("库仑摩擦力矩 / Coulomb Friction Torque [N·m]", f"{results['coulomb_friction']:.6f}")
            if 'coulomb_friction_pos' in results:
                self.add_result_row("正向库仑摩擦 / Positive Coulomb Friction [N·m]", f"{results['coulomb_friction_pos']:.6f}")
            if 'coulomb_friction_neg' in results:
                self.add_result_row("负向库仑摩擦 / Negative Coulomb Friction [N·m]", f"{results['coulomb_friction_neg']:.6f}")
        
        # 添加静摩擦结果
        if 'static_friction' in results and results['static_friction'] is not None:
            self.add_result_row("静摩擦力矩 / Static Friction Torque [N·m]", f"{results['static_friction']:.6f}")
            if 'static_friction_pos' in results and results['static_friction_pos'] is not None:
                self.add_result_row("正向静摩擦 / Positive Static Friction [N·m]", f"{results['static_friction_pos']:.6f}")
            if 'static_friction_neg' in results and results['static_friction_neg'] is not None:
                self.add_result_row("负向静摩擦 / Negative Static Friction [N·m]", f"{results['static_friction_neg']:.6f}")
        
        # 添加已知参数
        self.add_result_row("粘滞摩擦系数 / Viscous Friction Coefficient [N·m·s/rad]", f"{results['viscous_friction']:.10f}")
        self.add_result_row("转子惯量 / Rotor Inertia [kg·m²]", f"{results['inertia']:.10f}")
        
        # 切换到结果标签页
        self.sub_tabs.setCurrentWidget(self.results_tab)
    
    def add_result_row(self, name, value):
        """向结果表格添加一行"""
        row = self.results_table.rowCount()
        self.results_table.insertRow(row)
        self.results_table.setItem(row, 0, QTableWidgetItem(name))
        self.results_table.setItem(row, 1, QTableWidgetItem(value))
    
    def save_results(self):
        """保存结果到文件"""
        if not self.results:
            self.log("没有可保存的结果")
            return
        
        try:
            # 确保目录存在
            if not os.path.exists('friction_results'):
                os.makedirs('friction_results')
            
            # 使用文件对话框选择保存类型
            options = QFileDialog.Options()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 提供多种保存格式选择
            filename, file_type = QFileDialog.getSaveFileName(
                self, "保存识别结果", 
                os.path.join('friction_results', f"friction_params_{timestamp}"),
                "文本文件 (*.txt);;JSON文件 (*.json);;所有文件 (*)", 
                options=options)
            
            if not filename:
                return
            
            # 更新时间戳
            self.results['timestamp'] = datetime.now().isoformat()
            
            if filename.endswith('.txt') or file_type.startswith("文本文件"):
                # 保存为可读的txt格式
                self.save_results_as_txt(filename)
            else:
                # 保存为JSON格式
                self.save_results_as_json(filename)
            
            self.log(f"结果已保存到: {filename}")
            
        except Exception as e:
            self.log(f"保存结果失败: {str(e)}")
    
    def save_results_as_txt(self, filename):
        """保存结果为txt格式（中英文对照）"""
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("电机摩擦力参数识别结果 / Motor Friction Parameter Identification Results\n")
            f.write("=" * 80 + "\n\n")
            
            # 测试信息
            f.write("测试信息 / Test Information:\n")
            f.write("-" * 50 + "\n")
            f.write(f"测试时间 / Test Time: {self.results.get('timestamp', 'Unknown')}\n")
            if 'motor_info' in self.results:
                motor_info = self.results['motor_info']
                f.write(f"电机版本 / Motor Version: {motor_info.get('sub_ver', 'Unknown')}\n")
                f.write(f"最大扭矩 / Max Torque: {motor_info.get('max_torque', 'Unknown')} N·m\n")
                f.write(f"减速比 / Gear Ratio: {motor_info.get('gear_ratio', 'Unknown')}\n")
            f.write("\n")
            
            # 摩擦参数结果
            f.write("摩擦参数结果 / Friction Parameter Results:\n")
            f.write("-" * 50 + "\n")
            
            # 库仑摩擦
            if 'coulomb_friction' in self.results:
                f.write(f"平均库仑摩擦力矩 / Average Coulomb Friction Torque: {self.results['coulomb_friction']:.6f} N·m\n")
                if 'coulomb_friction_pos' in self.results:
                    f.write(f"  正向库仑摩擦 / Positive Direction: {self.results['coulomb_friction_pos']:.6f} N·m\n")
                if 'coulomb_friction_neg' in self.results:
                    f.write(f"  负向库仑摩擦 / Negative Direction: {self.results['coulomb_friction_neg']:.6f} N·m\n")
                f.write("\n")
            
            # 静摩擦
            if 'static_friction' in self.results and self.results['static_friction'] is not None:
                f.write(f"平均静摩擦力矩 / Average Static Friction Torque: {self.results['static_friction']:.6f} N·m\n")
                if 'static_friction_pos' in self.results and self.results['static_friction_pos'] is not None:
                    f.write(f"  正向静摩擦 / Positive Direction: {self.results['static_friction_pos']:.6f} N·m\n")
                if 'static_friction_neg' in self.results and self.results['static_friction_neg'] is not None:
                    f.write(f"  负向静摩擦 / Negative Direction: {self.results['static_friction_neg']:.6f} N·m\n")
                f.write("\n")
            
            # 已知参数
            f.write("已知参数 / Known Parameters:\n")
            f.write("-" * 50 + "\n")
            f.write(f"粘滞摩擦系数 / Viscous Friction Coefficient: {self.results['viscous_friction']:.10f} N·m·s/rad\n")
            f.write(f"转子惯量 / Rotor Inertia: {self.results['inertia']:.10f} kg·m²\n")
            f.write("\n")
            
            # 测试数据统计（如果有的话）
            if 'coulomb_raw_data' in self.results:
                speeds = self.results['coulomb_raw_data']['speeds']
                torques = self.results['coulomb_raw_data']['torques']
                f.write("库仑摩擦测试数据统计 / Coulomb Friction Test Data Statistics:\n")
                f.write("-" * 50 + "\n")
                f.write(f"测试点数 / Number of Test Points: {len(speeds)}\n")
                f.write(f"速度范围 / Velocity Range: {min(speeds):.3f} ~ {max(speeds):.3f} rad/s\n")
                f.write(f"力矩范围 / Torque Range: {min(torques):.6f} ~ {max(torques):.6f} N·m\n")
                f.write("\n")
            
            # 文件说明
            f.write("文件说明 / File Description:\n")
            f.write("-" * 50 + "\n")
            f.write("本文件包含了电机摩擦力参数自动识别的完整结果。\n")
            f.write("This file contains the complete results of automatic motor friction parameter identification.\n\n")
            f.write("库仑摩擦 (Coulomb Friction): 运动过程中的干摩擦力矩，与运动方向相反\n")
            f.write("静摩擦 (Static Friction): 物体从静止开始运动时需要克服的最大摩擦力矩\n")
            f.write("粘滞摩擦 (Viscous Friction): 与速度成正比的摩擦阻力\n")
            f.write("\n")
            f.write("=" * 80 + "\n")
    
    def save_results_as_json(self, filename):
        """保存结果为JSON格式"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=4, ensure_ascii=False)
        
        # 同时保存一个最新结果的副本
        latest_path = os.path.join('friction_results', 'latest_friction_params.json')
        with open(latest_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=4, ensure_ascii=False)
    
    def load_results(self):
        """从文件加载结果"""
        try:
            # 使用文件对话框
            options = QFileDialog.Options()
            filepath, _ = QFileDialog.getOpenFileName(
                self, "加载识别结果", 
                'friction_results',
                "JSON文件 (*.json);;所有文件 (*)", options=options)
            
            if not filepath:
                return
            
            with open(filepath, 'r', encoding='utf-8') as f:
                loaded_results = json.load(f)
            
            # 更新结果
            self.results = loaded_results
            self.update_results(loaded_results)
            
            # 显示日志
            self.log(f"成功加载结果: {filepath}")
            
            # 根据内容重建可能的图表
            if 'coulomb_raw_data' in loaded_results:
                try:
                    speeds = np.array(loaded_results['coulomb_raw_data']['speeds'])
                    torques = np.array(loaded_results['coulomb_raw_data']['torques'])
                    T_coulomb_pos = loaded_results.get('coulomb_friction_pos', 0)
                    T_coulomb_neg = loaded_results.get('coulomb_friction_neg', 0)
                    viscous_coeff = loaded_results.get('viscous_friction', 0)
                    
                    # 创建图表并显示
                    fig = plt.figure(figsize=(12, 8))
                    plt.scatter(speeds, torques, color='blue', s=50, alpha=0.7, label='实测数据点', zorder=5)
                    
                    # 理论模型线
                    x_model = np.linspace(min(speeds) - 0.5, max(speeds) + 0.5, 200)
                    y_model_pos = np.where(x_model > 0, viscous_coeff * x_model + T_coulomb_pos, 0)
                    y_model_neg = np.where(x_model < 0, viscous_coeff * x_model - T_coulomb_neg, 0)
                    y_model = y_model_pos + y_model_neg
                    plt.plot(x_model, y_model, 'r-', linewidth=2.5, label='摩擦模型', zorder=4)
                    
                    # 标记库仑摩擦力矩
                    plt.axhline(y=T_coulomb_pos, color='g', linestyle='--', linewidth=2, alpha=0.8,
                              label=f'正向库仑摩擦: {T_coulomb_pos:.5f} N·m', zorder=3)
                    plt.axhline(y=-T_coulomb_neg, color='m', linestyle='--', linewidth=2, alpha=0.8,
                              label=f'负向库仑摩擦: {T_coulomb_neg:.5f} N·m', zorder=3)
                    
                    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3, zorder=1)
                    plt.axvline(x=0, color='k', linestyle='-', alpha=0.3, zorder=1)
                    
                    plt.xlabel('角速度 [rad/s]', fontsize=14)
                    plt.ylabel('力矩 [N·m]', fontsize=14)
                    plt.title('电机库仑摩擦力矩识别结果 (已加载)', fontsize=16, fontweight='bold')
                    plt.legend(fontsize=12, loc='best')
                    plt.grid(True, alpha=0.3)
                    
                    plt.tight_layout()
                    
                    # 创建一个 matplotlib 画布
                    canvas = FigureCanvas(fig)
                    toolbar = NavigationToolbar(canvas, self)
                    
                    # 清除之前的子控件
                    while self.coulomb_layout.count():
                        item = self.coulomb_layout.takeAt(0)
                        widget = item.widget()
                        if widget:
                            widget.deleteLater()
                    
                    self.coulomb_layout.addWidget(toolbar)
                    self.coulomb_layout.addWidget(canvas)
                except Exception as e:
                    self.log(f"重建库仑摩擦图表失败: {str(e)}")
            
        except Exception as e:
            self.log(f"加载结果失败: {str(e)}")
    
    def log(self, message):
        """向日志窗口添加消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # 自动滚动到底部
        cursor = self.log_text.textCursor()
        cursor.movePosition(cursor.End)
        self.log_text.setTextCursor(cursor)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止状态监控线程
        if self.status_thread and self.status_thread.isRunning():
            self.status_thread.stop()
            
        # 检查是否有线程在运行
        if self.identifier_thread is not None and self.identifier_thread.isRunning():
            reply = QMessageBox.question(self, '确认退出', 
                "识别过程正在进行中，确定要退出吗？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.stop_identification()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle('Fusion')
    
    # 设置应用程序样式表
    app.setStyleSheet("""
        QMainWindow {
            background-color: #f5f5f5;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #cccccc;
            border-radius: 5px;
            margin-top: 1ex;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
        }
        QTabWidget::pane {
            border: 1px solid #cccccc;
            background-color: white;
        }
        QTabBar::tab {
            background-color: #e1e1e1;
            border: 1px solid #cccccc;
            padding: 8px 12px;
            margin-right: 2px;
        }
        QTabBar::tab:selected {
            background-color: white;
            border-bottom-color: white;
        }
        QPushButton {
            background-color: #2980b9;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #3498db;
        }
        QPushButton:pressed {
            background-color: #21618c;
        }
        QPushButton:disabled {
            background-color: #bdc3c7;
            color: #7f8c8d;
        }
    """)
    
    # 创建并显示主窗口
    window = FrictionIdentifierApp()
    window.show()
    
    sys.exit(app.exec_())

