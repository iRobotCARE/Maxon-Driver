"""maxon电机驱动文件
"libEposCmd.so.6.8.1.0" 已经放在了系统路经下

# EPOS4 初始化步骤:
1. 确定设备名称、协议栈名称、接口名称、端口名称
2. 打开设备、设置协议栈、清除故障、使能设备
3. 设置操作模式、设置运动模式参数
4. ......
5. 关闭设备

# 控制单位转换：
- 位置单位: Encoder; * 4 (Quadrature Decoding) * 1024 (resolution) * 35 (Speed Ratio) = 1 (round)

edited by hsy at 2025-04-29
"""
import ctypes
import time
import os
import numpy as np


# 定义操作模式值到描述性名称的映射字典
# 数据来源于 EPOS 文档中的 Table 5-19
OPERATION_MODE_MAP = {
    1: "Profile Position Mode (PPM)",
    3: "Profile Velocity Mode (PVM)",
    6: "Homing Mode (HM)",
    7: "Interpolated Position Mode (IPM)",
   -1: "Position Mode (PM, CSP)",    # 注意值为负数
   -2: "Velocity Mode (VM, CSV)",    # 注意值为负数
   -3: "Current Mode (CM, CST)",     # 注意值为负数
   -5: "Master Encoder Mode",
   -6: "Step Direction Mode"
}

# 定义传感器类型值到描述的映射字典
# 数据来源于 Table 4-7
SENSOR_TYPE_DESCRIPTION_MAP = {
    0: "Unknown / No sensor",
    1: "Incremental encoder 1 with index (3-channel)",
    2: "Incremental encoder 1 without index (2-channel)",
    3: "Hall Sensors",
    4: "SSI encoder binary coded",
    5: "SSI encoder Grey coded",
    6: "Incremental encoder 2 with index (3-channel)",
    7: "Incremental encoder 2 without index (2-channel)",
    8: "Analog incremental encoder with index (3-channel)",
    9: "Analog incremental encoder without index (2-channel)"
    # 如果有更多类型，可以继续添加
}

def get_operation_mode_description(mode_value):
  """根据传入的整数模式值，返回其描述性名称。
  Args:
      mode_value (int): 操作模式的整数值 (例如 1, 3, -1 等)。
  Returns:
      str: 操作模式的描述性名称，如果未找到则返回包含原值的未知提示。
  """
  # 使用字典的 .get() 方法，如果找不到键，则返回第二个参数作为默认值
  return OPERATION_MODE_MAP.get(mode_value, f"Unknown Mode (Value: {mode_value})")


def get_position_sensor_description(sensor_type_value):
  """
  根据传入的位置传感器类型整数值，返回其对应的描述字符串。
  Args:
      sensor_type_value (int): 代表位置传感器类型的整数值 (来自 Table 4-7 的 Value 列)。
  Returns:
      str: 传感器类型的描述字符串。如果传入的值未知，则返回提示信息。
  """
  # 如果不存在，返回 .get() 方法的第二个参数作为默认值。
  return SENSOR_TYPE_DESCRIPTION_MAP.get(sensor_type_value, f"Unknown Sensor Type (Value: {sensor_type_value})")


def print_center(string, width=60, fill_char='='):
    """打印居中对齐的字符串
    """
    # 计算填充字符的数量
    fill_count = (width - len(string)) // 2
    # 打印居中对齐的字符串
    print(f"{fill_char * fill_count} {string} {fill_char * fill_count}")
    # 如果总宽度是奇数，补一个填充字符
    if (width - len(string)) % 2 != 0:
        print(fill_char)


class EPOS:
    def __init__(self, device_name:str="EPOS4", protocol_name:str="MAXON SERIAL V2", interface_name:str="USB"):
        try:
            self.epos = ctypes.CDLL("libEposCmd.so")
        except OSError as e:
            print(f"Error loading libEposCmd.so from system path: {e}")

        self.device_name = ctypes.c_char_p(device_name.encode('utf-8'))
        self.protocol_name = ctypes.c_char_p(protocol_name.encode('utf-8'))
        self.interface_name = ctypes.c_char_p(interface_name.encode('utf-8'))
        candidate_ports = self.candidate_device()
        # 需要确定电机是哪个端口，一个电机对应一个端口
        port_name = candidate_ports[0] if candidate_ports else None
        self.port_name = ctypes.c_char_p(port_name.encode('utf-8'))

        self.candidate_baudrate()
        # print("===================================================")

        self.node_id:int = 1
        self.key_handle:int = 0

    
    def candidate_device(self):
        """输出所有可用的设备的信息
        也可以在通过 "./HelloEposCmd  -r" 命令行来查看所有的候选设备信息
        """
        # 获取设备名称列表 -----------------------
        device_name_list   = []
        max_str_size       = ctypes.c_ushort(100)
        device_name_buffer = ctypes.create_string_buffer(max_str_size.value)
        end_of_selection   = ctypes.c_int(True)
        p_error_code       = ctypes.c_uint()
        _ = self.epos.VCS_GetDeviceNameSelection(
            True, device_name_buffer, max_str_size, ctypes.byref(end_of_selection), ctypes.byref(p_error_code))
        # print(f"Available devices:{device_name_buffer.value.decode('utf-8')}")
        device_name_list.append(device_name_buffer.value.decode('utf-8'))

        while end_of_selection.value==False:
            # 继续调用VCS_GetDeviceNameSelection，直到end_of_selection为True
            _ = self.epos.VCS_GetDeviceNameSelection(
                False, device_name_buffer, max_str_size, ctypes.byref(end_of_selection), ctypes.byref(p_error_code))
            # print(f"Available devices:{device_name_buffer.value.decode('utf-8')}")
            device_name_list.append(device_name_buffer.value.decode('utf-8'))
        print(f"Available devices: {device_name_list}")

        device_name = self.device_name          # 指定设备名称, 依据epos型号

        # 获取协议栈名称列表 -----------------------
        protocol_name_list     = []
        max_str_size_proto     = ctypes.c_ushort(100)
        protocol_name_buffer   = ctypes.create_string_buffer(max_str_size_proto.value) # Output buffer
        end_of_selection_proto = ctypes.c_int()         # Variable for C int* output
        p_error_code_proto     = ctypes.c_uint()        # Variable for C unsigned int* output

        _ = self.epos.VCS_GetProtocolStackNameSelection(
            device_name, True, protocol_name_buffer, max_str_size_proto, ctypes.byref(end_of_selection_proto), ctypes.byref(p_error_code_proto))
        protocol_name_list.append(protocol_name_buffer.value.decode('utf-8'))

        while end_of_selection_proto.value==False:
            _ = self.epos.VCS_GetProtocolStackNameSelection(
                device_name, False, protocol_name_buffer, max_str_size_proto, ctypes.byref(end_of_selection_proto), ctypes.byref(p_error_code_proto))
            protocol_name_list.append(protocol_name_buffer.value.decode('utf-8'))
        print(f"Available protocol stacks: {protocol_name_list}")

        protocol_name = self.protocol_name      # 指定协议栈名称

        # 获取接口名称列表 -----------------------
        interface_name_list   = []
        interface_name_buffer = ctypes.create_string_buffer(max_str_size.value)
        end_of_selection_inf  = ctypes.c_int()
        p_error_code_inf      = ctypes.c_uint()

        _ = self.epos.VCS_GetInterfaceNameSelection(
            device_name, protocol_name, True, interface_name_buffer, max_str_size, ctypes.byref(end_of_selection_inf), ctypes.byref(p_error_code_inf))
        interface_name_list.append(interface_name_buffer.value.decode('utf-8'))

        while end_of_selection_inf.value==False:
            _ = self.epos.VCS_GetInterfaceNameSelection(
                device_name, protocol_name, False, interface_name_buffer, max_str_size, ctypes.byref(end_of_selection_inf), ctypes.byref(p_error_code_inf))
            interface_name_list.append(interface_name_buffer.value.decode('utf-8'))
        print(f"Available interfaces: {interface_name_list}")

        interface_name = self.interface_name    # 用USB连接
        
        # 获取端口名称列表 -----------------------
        port_name_list   = []
        port_name_buffer      = ctypes.create_string_buffer(max_str_size.value)
        end_of_selection_port = ctypes.c_int()
        p_error_code_port     = ctypes.c_uint()

        _ = self.epos.VCS_GetPortNameSelection(
            device_name, protocol_name, interface_name, True, port_name_buffer, max_str_size, ctypes.byref(end_of_selection_port), ctypes.byref(p_error_code_port))
        port_name_list.append(port_name_buffer.value.decode('utf-8'))

        while end_of_selection_port.value==False:
            _ = self.epos.VCS_GetPortNameSelection(
                device_name, protocol_name, interface_name, False, port_name_buffer, max_str_size, ctypes.byref(end_of_selection_port), ctypes.byref(p_error_code_port))
            port_name_list.append(port_name_buffer.value.decode('utf-8'))
        print(f"Available ports: {port_name_list}")

        print(f"Device name: {device_name.value.decode('utf-8')}; " + 
              f"Protocol name: {protocol_name.value.decode('utf-8')}; " + 
              f"Interface name: {interface_name.value.decode('utf-8')}; ")

        return port_name_list


    def initialize_device(self):
        """初始化设备"""
        self.open_device()
        self.set_protocal_stack()
        self.clear_fault()


    def open_device(self):
        p_error_code = ctypes.c_uint()
        self.key_handle = self.epos.VCS_OpenDevice(
            self.device_name, self.protocol_name, self.interface_name, self.port_name, ctypes.byref(p_error_code))
        
        if self.key_handle != 0:
            print("Open Device, and key handle is: %8d" % self.key_handle)
            return True

        else:
            print("Could not open Device!")
            self.print_error_info(p_error_code)
            return False
        
    
    def close_device(self):
        p_error_code = ctypes.c_uint()
        ret = self.epos.VCS_CloseDevice(self.key_handle, ctypes.byref(p_error_code))

        if ret == 0:
            self.print_error_info(p_error_code)
            return False
        else:
            print("Device closed successfully!")
            return True
        
    def clear_fault(self):
        """清除电机故障, 切换到"Disable"状态
        """
        error_code = ctypes.c_uint()
        ret = self.epos.VCS_ClearFault(
            self.key_handle, self.node_id, ctypes.byref(error_code))
        if ret == 0:
            print("Error during clearing Fault")
            self.print_error_info(error_code)
            return False
        else:
            print("Fault cleared successfully!")
            return True
        
    
    def enable_state(self):
        pls_enabled = ctypes.c_bool()
        error_code = ctypes.c_uint()

        if self.device_error_check():           # 检查设备是否有错误
            ret = self.epos.VCS_SetEnableState(
                self.key_handle, self.node_id, ctypes.byref(error_code))
            
            if ret == 0:
                print("Error during enabling Device")
                self.print_error_info(error_code)
                return False
            else:
                self.epos.VCS_GetEnableState(
                    self.key_handle, self.node_id, ctypes.byref(pls_enabled), ctypes.byref(error_code))
                if pls_enabled.value == 1:      # 验证设备是否使能
                    print("Device Enabled")
                    return True
                else:
                    print("Device Not Enabled!")
                    return False
        else:
            print("Device is in Error State!")
            return False


    def disable_state(self):
        pls_disabled = ctypes.c_bool()
        p_error_code = ctypes.c_uint()

        ret = self.epos.VCS_SetDisableState(
            self.key_handle, self.node_id, ctypes.byref(p_error_code))

        if ret == 0:
            print("Error Set Disable State")
            return False
        else:
            self.epos.VCS_GetDisableState(
                self.key_handle, self.node_id, ctypes.byref(pls_disabled), ctypes.byref(p_error_code))

            if pls_disabled.value == 1:
                print("Device Disabled")
                return True
            else:
                print("Device could not be disabled")
                return False
            

    def set_operation_mode(self, mode:int):
        mode_c = ctypes.c_int8(mode)
        error_code = ctypes.c_uint()

        ret = self.epos.VCS_SetOperationMode(
            self.key_handle, self.node_id, mode_c, ctypes.byref(error_code))
        
        if ret: # 返回 BOOL，非零为成功
            print(f"Successfully set Operation Mode to {get_operation_mode_description(mode)}.")
            return True
        else:
            print(f"VCS_SetOperationMode failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        
    
    def get_operation_mode(self):
        mode_c = ctypes.c_int8()
        error_code = ctypes.c_uint()

        ret = self.epos.VCS_GetOperationMode(
            self.key_handle, self.node_id, ctypes.byref(mode_c), ctypes.byref(error_code))
        
        if ret:
            print(f"Operation Mode: {get_operation_mode_description(mode_c.value)}")
            return mode_c.value
        else:
            print(f"VCS_GetOperationMode failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        
    
    def set_ppm_parameter(self, velocity:int, acceleration:int, deceleration:int):
        error_code = ctypes.c_uint()
        ret = self.epos.VCS_SetPositionProfile(
            self.key_handle, self.node_id, velocity, acceleration, deceleration, ctypes.byref(error_code))

        if ret:
            print(f"Set Position Profile: Velocity={velocity}, Acceleration={acceleration}, Deceleration={deceleration}")
            return True
        else:
            print(f"VCS_SetPositionProfile failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        
    
    def get_ppm_parameter(self):
        velocity_c = ctypes.c_uint()
        acceleration_c = ctypes.c_uint()
        deceleration_c = ctypes.c_uint()
        error_code = ctypes.c_uint()
        ret = self.epos.VCS_GetPositionProfile(
            self.key_handle, self.node_id, ctypes.byref(velocity_c), ctypes.byref(acceleration_c), ctypes.byref(deceleration_c), ctypes.byref(error_code))
        if ret:
            print(f"Position Profile: Velocity={velocity_c.value}, Acceleration={acceleration_c.value}, Deceleration={deceleration_c.value}")
            return [velocity_c.value, acceleration_c.value, deceleration_c.value]
        else:
            print(f"VCS_GetPositionProfile failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        

    def ppm_move_position_encoder(self, target_position:int, abosolute:bool=True, immediate:bool=True, blocking_time:int=1000):
        """电机移动到指定位置(Encoder单位)"""
        error_code = ctypes.c_uint()
        ret = self.epos.VCS_MoveToPosition(
            self.key_handle, self.node_id, target_position, abosolute, immediate, ctypes.byref(error_code))
        
        if ret:
            # 只能判断成功发送了指令, 并不代表电机已经到达目标位置
            self.epos.VCS_WaitForTargetReached(
                self.key_handle, self.node_id, blocking_time, ctypes.byref(error_code))
            return True
        else:
            raise RuntimeError(f"VCS_MoveToPosition failed!\n" + 
                               f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        

    def ppm_move_position(self, target_angle:float, abosolute:bool=True, immediate:bool=True, block_time:int=1000):
        """电机移动到指定位置(Angle弧度单位)
        """
        # 1. 角度转换为编码器单位
        target_position = int(4 * 1024 * 35 * target_angle / 2 / np.pi)
        ret = self.ppm_move_position_encoder(
            target_position, abosolute=abosolute, immediate=immediate, blocking_time=block_time)
        
    
    def get_position_encoder(self):
        position_c = ctypes.c_int()
        error_code = ctypes.c_uint()
        ret = self.epos.VCS_GetPositionIs(
            self.key_handle, self.node_id, ctypes.byref(position_c), ctypes.byref(error_code))
        
        if ret:
            print(f"Current Position: {position_c.value}")
            return position_c.value
        else:
            print(f"VCS_GetPositionIs failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        
    
    def get_sensor_type(self):
        sensor_type = ctypes.c_ushort()
        error_code = ctypes.c_uint()

        ret = self.epos.VCS_GetSensorType(
            self.key_handle, self.node_id, ctypes.byref(sensor_type), ctypes.byref(error_code))
        if ret:
            print(f"Sensor Type: {get_position_sensor_description(sensor_type.value)}")
            return sensor_type.value
        else:
            print(f"VCS_GetSensorType failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False


    def get_incencoder_parameter(self):
        encoder_resolution = ctypes.c_uint()
        inverted_ploarity = ctypes.c_bool()
        error_code = ctypes.c_uint()

        ret = self.epos.VCS_GetIncEncoderParameter(
            self.key_handle, self.node_id, ctypes.byref(encoder_resolution), ctypes.byref(inverted_ploarity), ctypes.byref(error_code))
        
        if ret:
            # resolution表示
            print(f"Encoder Resolution: {encoder_resolution.value}; Inverted Polarity: {inverted_ploarity.value}")
            return [encoder_resolution.value, inverted_ploarity.value]
        else:
            print(f"VCS_GetIncEncoderParameter failed!")
            print(f"Error Code: {error_code.value:#010x}; Description: {self.epos.VCS_GetErrorInfo(error_code.value)}")
            return False
        
    
    def device_error_check(self):
        nb_device_error = ctypes.c_uint()
        error_code = ctypes.c_uint()
    
        ret = self.epos.VCS_GetNbOfDeviceError(
            self.key_handle, self.node_id, ctypes.byref(nb_device_error), ctypes.byref(error_code))

        if nb_device_error.value >= 1:
            print(f"Device Error: {nb_device_error.value}")
            return False
        else:
            self.print_error_info(error_code)
            return True
        
    
    def candidate_baudrate(self):
        """返回所有可用的波特率
        Returns:
            List: _description_
        """
        baudrate_list = []
        baudrate_sel = ctypes.c_uint()      # For unsigned int* pBaudrateSel
        end_of_selection = ctypes.c_int()   # For int* pEndOfSelection
        p_error_code = ctypes.c_uint()        # For unsigned int* pErrorCode
        ret = self.epos.VCS_GetBaudrateSelection(
            self.device_name, self.protocol_name, self.interface_name, self.port_name, True, ctypes.byref(baudrate_sel), ctypes.byref(end_of_selection), ctypes.byref(p_error_code))
        baudrate_list.append(baudrate_sel.value)

        while end_of_selection.value == False:
            self.epos.VCS_GetBaudrateSelection(
                self.device_name, self.protocol_name, self.interface_name, self.port_name, False, ctypes.byref(baudrate_sel), ctypes.byref(end_of_selection), ctypes.byref(p_error_code))
            baudrate_list.append(baudrate_sel.value)
        print(f"Available boundrates: {baudrate_list}")

        return baudrate_list


    def set_protocal_stack(self):
        p_error_code = ctypes.c_uint()
        self.epos.VCS_SetProtocolStackSettings(
            self.key_handle, 1000000, 500, ctypes.byref(p_error_code))
        

    def print_error_info(self, p_error_code):
        buffer_size = 40
        error_info_buffer = ctypes.create_string_buffer(buffer_size)

        if p_error_code.value != 0x0:       
            self.epos.VCS_GetErrorInfo(p_error_code.value, error_info_buffer, buffer_size)
            print(f"Error Code: {p_error_code.value:#010x}; Description: {error_info_buffer.value.decode('utf-8', errors='ignore')}")
            return False

        else:               # 表示没有错误
            return True



if __name__ == "__main__":
    motor = EPOS(device_name="EPOS4", protocol_name="MAXON SERIAL V2", interface_name="USB")
    try:
        print_center("EPOS4 Initialization")
        motor.initialize_device()
        motor.enable_state()

        motor.get_sensor_type()
        motor.get_incencoder_parameter()
        motor.get_ppm_parameter()

        print_center("Movement")
        motor.set_operation_mode(1)                 # 1: Profile Position mode
        motor.set_ppm_parameter(800, 10000, 10000)  # 设置速度、加速度、减速度

        motor.get_position_encoder()
        motor.ppm_move_position_encoder(0)
        motor.get_position_encoder()

    finally:
        print_center("Disabling Device")
        motor.disable_state()
        motor.close_device()
