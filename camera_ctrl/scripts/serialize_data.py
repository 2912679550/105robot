def calculate_crc(data):
    """
    计算CRC-8/DVB-S2校验值

    参数:
        data (bytes、列表或十六进制字符串): 待计算CRC的数据

    返回:
        int: 计算得到的CRC校验值
    """
    # 处理十六进制字符串输入
    if isinstance(data, str):
        data = bytes.fromhex(data)

    crc = 0x00  # 初始CRC值

    for byte in data:
        crc ^= byte  # 与当前字节异或

        # 处理每一位
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5  # 多项式为0x1D5，但左移后相当于0xD5
            else:
                crc <<= 1

        crc &= 0xFF  # 确保结果是8位

    return crc


def signed_decimal_to_hex(decimal_num):
    """将有符号十进制数转换为16位十六进制字符串（高位在前）"""
    # 应用反向精度（乘以100）
    value = decimal_num * 100

    # 四舍五入到最接近的整数
    int_value = round(value)

    # 检查是否超出16位有符号整数范围
    if int_value < -32768 or int_value > 32767:
        raise ValueError(f"值 {decimal_num} 超出16位有符号整数范围(-327.68 到 327.67)")

    # 处理有符号数（转换为无符号表示）
    if int_value < 0:
        int_value += 0x10000  # 加上65536得到补码表示

    # 转换为16位十六进制字符串
    hex_str = format(int_value, '04X').lower()  # 格式化为4位十六进制，小写字母

    # 分解为高字节和低字节
    high_byte = hex_str[:2]
    low_byte = hex_str[2:]

    return high_byte + " "+ low_byte



def serializeData(param1, param2, param3):
    """
    序列化数据为特定格式的十六进制字符串

    参数:
        param1 (float): 参数1
        param2 (float): 参数2
        param3 (float): 参数3

    返回:
        str: 序列化后的十六进制字符串
    """
    # 转换参数为十六进制
    hex_param1 = signed_decimal_to_hex(param1)
    hex_param2 = signed_decimal_to_hex(param2)
    hex_param3 = signed_decimal_to_hex(param3)

    # 构建数据帧
    prefix = "aa 0c 05 05"
    suffix = "01"
    data_frame = f"{prefix} {hex_param1} {hex_param2} {hex_param3} {suffix}"

    # 计算CRC校验值
    crc = calculate_crc(data_frame.replace(" ", ""))

    # 格式化CRC校验值
    crc_hex = f"{crc:02x}"

    # 最终数据帧
    final_data_frame = f"{data_frame} {crc_hex}"

    return final_data_frame
