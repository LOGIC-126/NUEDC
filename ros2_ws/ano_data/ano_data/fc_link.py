def build_pos_frame(x, y):
    """
    构建当前位置(POS)数据帧
    :param x: X坐标值 (整数)
    :param y: Y坐标值 (整数)
    :return: 字节数组格式的数据帧
    """
    # 帧头 | 长度 | 类型
    frame = bytearray([0xAA, 0x04, 0x01])
    
    # 添加坐标数据 (小端字节序)
    frame.append(x & 0xFF)        # x低字节
    frame.append((x >> 8) & 0xFF)  # x高字节
    frame.append(y & 0xFF)        # y低字节
    frame.append((y >> 8) & 0xFF)  # y高字节
    
    # 计算校验和 (长度 + 类型 + 所有数据)
    checksum = frame[1] + frame[2]  # 长度 + 类型
    for byte in frame[3:]:
        checksum += byte
    frame.append(checksum & 0xFF)  # 取低8位
    
    return frame

def build_tar_frame(x, y, z):
    """
    构建目标位置(TAR)数据帧
    :param x: X坐标值 (整数)
    :param y: Y坐标值 (整数)
    :param z: Z坐标值 (整数)
    :return: 字节数组格式的数据帧
    """
    # 帧头 | 长度 | 类型
    frame = bytearray([0xAA, 0x06, 0x02])
    
    # 添加坐标数据 (小端字节序)
    frame.append(x & 0xFF)        # x低字节
    frame.append((x >> 8) & 0xFF)  # x高字节
    frame.append(y & 0xFF)        # y低字节
    frame.append((y >> 8) & 0xFF)  # y高字节
    frame.append(z & 0xFF)        # z低字节
    frame.append((z >> 8) & 0xFF)  # z高字节
    
    # 计算校验和 (长度 + 类型 + 所有数据)
    checksum = frame[1] + frame[2]  # 长度 + 类型
    for byte in frame[3:]:
        checksum += byte
    frame.append(checksum & 0xFF)  # 取低8位
    
    return frame

def frame_to_hex(frame):
    """将字节数组转换为十六进制字符串表示"""
    return ' '.join([f'0x{byte:02X}' for byte in frame])
