import rospy
import serial
import yaml
import os

def load_serial_config():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 从scripts目录向上找到camera_ctrl目录，然后找到config目录
    config_path = os.path.join(current_dir, '..', 'config', 'serial_config.yaml')
    config_path = os.path.normpath(config_path)  # 规范化路径
    """加载串口配置文件"""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            rospy.loginfo(f"加载串口配置: {config}")
            return config
    except Exception as e:
        rospy.logerr(f"加载配置文件失败: {e}")
        # 返回默认配置
        return {
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 9600,
            'timeout': 1
        }
    
def send_to_serial(serialized_str, config=None):
    """
    将序列化数据发送到指定串口
    
    参数:
        serialized_str (bytes): 要发送的序列化数据
        config (dict): 串口配置，包含port, baudrate, timeout
        
    返回:
        bool: 发送成功返回True，失败返回False
    """
    if config is None:
        config = load_serial_config()
        
    try:
        port = config['serial_port']
        baudrate = config.get('baudrate', 9600)
        timeout = config.get('timeout', 1)
        
        # 打开串口
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            # 等待串口初始化
            rospy.sleep(0.1)
            
            # 写入数据
            bytes_written = ser.write(serialized_str)
            rospy.loginfo(f"成功发送 {bytes_written} 字节到 {port}")
            
            # 可选：读取响应
            if ser.in_waiting:
                response = ser.read(ser.in_waiting)
                rospy.loginfo(f"收到响应: {response}")
                
            return True
            
    # except serial.SerialException as e:
    #     rospy.logerr(f"串口通信错误 ({port}): {e}")
    #     return False
    except Exception as e:
        rospy.logerr(f"发送数据时发生未知错误: {e}")
        return False