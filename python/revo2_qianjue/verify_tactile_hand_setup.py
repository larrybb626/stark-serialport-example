#!/usr/bin/env python3
"""
触觉仿生手集成环境验证脚本

该脚本用于验证系统环境、依赖包、硬件设备是否正常配置
可用于快速诊断问题

使用方法：
    python verify_tactile_hand_setup.py

输出：
    ✓ 表示正常
    ✗ 表示失败
    ⚠ 表示警告
"""

import sys
import os
import asyncio
import time
from pathlib import Path

# 颜色输出
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_success(msg):
    print(f"{Colors.GREEN}✓{Colors.RESET} {msg}")

def print_error(msg):
    print(f"{Colors.RED}✗{Colors.RESET} {msg}")

def print_warning(msg):
    print(f"{Colors.YELLOW}⚠{Colors.RESET} {msg}")

def print_info(msg):
    print(f"{Colors.BLUE}ℹ{Colors.RESET} {msg}")

def print_header(msg):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{msg:^60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.RESET}\n")


# ==================== 验证步骤 ====================

def verify_python_version():
    """验证 Python 版本"""
    print_header("1. Python 版本检查")
    
    version = sys.version_info
    if version.major >= 3 and version.minor >= 7:
        print_success(f"Python {version.major}.{version.minor}.{version.micro} (满足要求)")
        return True
    else:
        print_error(f"Python {version.major}.{version.minor} (需要 3.7+)")
        return False


def verify_dependencies():
    """验证核心依赖包"""
    print_header("2. 依赖包检查")
    
    dependencies = {
        'numpy': 'NumPy',
        'cv2': 'OpenCV',
        'asyncio': 'AsyncIO',
        'xensesdk': 'Xense SDK (触觉传感器)',
    }
    
    all_ok = True
    for module, name in dependencies.items():
        try:
            __import__(module)
            print_success(f"{name} 已安装")
        except ImportError:
            print_error(f"{name} 未安装 - 运行: pip install {module}")
            if module == 'xensesdk':
                print_warning("  触觉模块: pip install xensesdk")
            all_ok = False
    
    return all_ok


def verify_project_structure():
    """验证项目文件结构"""
    print_header("3. 项目文件结构检查")
    
    required_files = {
        'revo2_ctrl.py': '手部控制主文件',
        'revo2_utils.py': '手部工具函数',
        'revo2_tactile_grasp_demo.py': '触觉集成演示',
        'common_init.py': '通用初始化',
    }
    
    script_dir = Path(__file__).parent
    all_ok = True
    
    for filename, description in required_files.items():
        filepath = script_dir / filename
        if filepath.exists():
            print_success(f"{filename}: {description}")
        else:
            print_error(f"{filename} 缺失: {description}")
            all_ok = False
    
    return all_ok


async def verify_hand_device():
    """验证手部设备连接"""
    print_header("4. 手部设备连接检查")
    
    try:
        from revo2_utils import open_modbus_revo2
        
        print_info("尝试自动检测手部设备...")
        
        # 设置超时以防止长时间等待
        try:
            client, slave_id = await asyncio.wait_for(
                open_modbus_revo2(port_name=None, quick=True),
                timeout=10.0
            )
            print_success(f"手部设备已连接 (Slave ID: 0x{slave_id:02X})")
            
            # 快速测试
            try:
                device_info = client.get_device_info(slave_id)
                print_success(f"设备信息: {device_info.description[:60]}...")
                
                # 关闭连接
                from common_imports import libstark
                libstark.modbus_close(client)
                return True
            except Exception as e:
                print_error(f"无法读取设备信息: {e}")
                return False
                
        except asyncio.TimeoutError:
            print_error("设备检测超时（10秒）- 检查连接")
            return False
            
    except ImportError as e:
        print_error(f"无法导入手部模块: {e}")
        return False
    except Exception as e:
        print_error(f"设备检测异常: {e}")
        return False


async def verify_tactile_sensor():
    """验证触觉传感器连接"""
    print_header("5. 触觉传感器连接检查")
    
    try:
        from xensesdk import Sensor
        from xensesdk.xenseInterface.sensorEnum import CameraSource
        
        print_info("尝试自动检测触觉传感器...")
        
        # 尝试创建传感器实例（使用 None 表示自动检测）
        sensor = Sensor.create(None, api=CameraSource.AV_V4L2)
        
        if sensor:
            print_success("触觉传感器已连接")
            
            # 尝试采集一帧数据
            try:
                print_info("采集测试帧...")
                data = sensor.selectSensorInfo(Sensor.OutputType.Force3DTuned)[0]
                
                if data is not None:
                    import numpy as np
                    print_success(f"数据采集成功 - 数据形状: {data.shape}")
                    
                    # 计算统计信息
                    force_norm = np.linalg.norm(data, axis=2)
                    print_info(f"  - 力值范围: {np.min(force_norm):.2f} - {np.max(force_norm):.2f} N")
                    print_info(f"  - 平均力值: {np.mean(force_norm):.2f} N")
                    
                    sensor.release()
                    return True
                else:
                    print_error("无法读取传感器数据")
                    sensor.release()
                    return False
                    
            except Exception as e:
                print_error(f"数据采集失败: {e}")
                sensor.release()
                return False
        else:
            print_error("无法检测到触觉传感器")
            print_warning("  - 检查 USB 连接")
            print_warning("  - 检查序列号")
            print_warning("  - 检查设备驱动")
            return False
            
    except ImportError:
        print_error("Xense SDK 未安装 - 运行: pip install xensesdk")
        return False
    except Exception as e:
        print_error(f"传感器检测异常: {e}")
        return False


def verify_serial_ports():
    """列出可用的串行端口"""
    print_header("6. 可用串行端口")
    
    try:
        import serial.tools.list_ports as list_ports
        
        ports = list(list_ports.comports())
        
        if ports:
            for port in ports:
                print_info(f"{port.device}: {port.description}")
            return True
        else:
            print_warning("未检测到可用的串行端口")
            return False
            
    except ImportError:
        print_warning("pyserial 未安装")
        return False
    except Exception as e:
        print_error(f"检查串行端口异常: {e}")
        return False


def verify_configuration_files():
    """检查配置文件"""
    print_header("7. 配置文件检查")
    
    script_dir = Path(__file__).parent
    config_files = {
        'tactile_grasp_config_template.py': '配置模板',
        'TACTILE_GRASP_DEMO.md': '完整文档',
        'TACTILE_QUICK_REFERENCE.md': '快速参考',
    }
    
    all_ok = True
    for filename, description in config_files.items():
        filepath = script_dir / filename
        if filepath.exists():
            size = filepath.stat().st_size / 1024  # KB
            print_success(f"{filename}: {description} ({size:.1f} KB)")
        else:
            print_warning(f"{filename} 未找到: {description}")
    
    return all_ok


# ==================== 主函数 ====================

async def main():
    """运行所有验证"""
    print(f"\n{Colors.BOLD}触觉仿生手集成系统 - 环境验证{Colors.RESET}\n")
    
    results = {
        'Python版本': verify_python_version(),
        '依赖包': verify_dependencies(),
        '项目文件': verify_project_structure(),
        '串行端口': verify_serial_ports(),
        '配置文件': verify_configuration_files(),
        '手部设备': await verify_hand_device(),
        '触觉传感器': await verify_tactile_sensor(),
    }
    
    # 打印总结
    print_header("验证总结")
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    for check, result in results.items():
        status = f"{Colors.GREEN}✓ 通过{Colors.RESET}" if result else f"{Colors.RED}✗ 失败{Colors.RESET}"
        print(f"{check:20} {status}")
    
    print()
    print_info(f"通过: {passed}/{total} 项检查")
    
    if passed == total:
        print_success("所有检查通过！系统已就绪")
        print()
        print(f"{Colors.BOLD}可以运行以下命令：{Colors.RESET}")
        print("  python revo2_tactile_grasp_demo.py")
        print("  python revo2_tactile_grasp_demo.py --mode monitor")
        return 0
    elif passed >= total - 2:
        print_warning("大部分检查通过，可能需要调整")
        print()
        print(f"{Colors.BOLD}建议：{Colors.RESET}")
        for check, result in results.items():
            if not result:
                print(f"  1. 检查 {check}")
        return 1
    else:
        print_error("检查失败过多，请先解决问题")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
