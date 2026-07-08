# pyright: reportMissingImports=false
try:
    from maix import uart
except ImportError:
    uart = None

"""
MaixCam 串口帧协议，共 9 字节：
    帧头: 0xAA 0x55
    状态: uint8_t，0=无目标，1=有目标
    X误差: int16，小端，目标点 x - 屏幕中心 x
    Y误差: int16，小端，目标点 y - 屏幕中心 y
    帧尾: 0x0D 0x0A
"""

FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"

STATUS_NO_TARGET = 0
STATUS_HAS_TARGET = 1

INT16_MIN = -32768
INT16_MAX = 32767
FRAME_LEN = 9


def _int16_to_le_bytes(value):
    """将有符号 int16 转成小端 2 字节。"""
    value = int(value)
    if value < INT16_MIN or value > INT16_MAX:
        raise ValueError("int16 out of range: %d" % value)

    if value < 0:
        value += 0x10000

    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def build_frame(has_target, err_x=0, err_y=0):
    """
    构建一帧串口数据。

    has_target: 是否识别到目标
    err_x: 目标点 x - 屏幕中心 x，向右为正
    err_y: 目标点 y - 屏幕中心 y，向下为正
    """
    status = STATUS_HAS_TARGET if has_target else STATUS_NO_TARGET

    # 无目标时误差统一发 0，单片机只需要根据 status 判断是否跟踪。
    if status == STATUS_NO_TARGET:
        err_x = 0
        err_y = 0

    frame = bytearray(FRAME_HEAD)
    frame.append(status)
    frame.extend(_int16_to_le_bytes(err_x))
    frame.extend(_int16_to_le_bytes(err_y))
    frame.extend(FRAME_TAIL)

    if len(frame) != FRAME_LEN:
        raise ValueError("invalid frame length: %d" % len(frame))

    return bytes(frame)


def build_error_frame(err_x, err_y):
    """构建有目标状态帧。"""
    return build_frame(True, err_x, err_y)


def build_no_target_frame():
    """构建无目标状态帧。"""
    return build_frame(False, 0, 0)


class UartCom:
    def __init__(self, baudrate=115200, timeout=500, device=None):
        self.serial = None
        self.device = None
        self.baudrate = baudrate
        self.timeout = timeout

        self.open(device)

    def open(self, device=None):
        """打开串口；未指定 device 时默认使用 MaixPy 返回的第一个串口。"""
        if uart is None:
            print("串口打开失败: 当前环境没有 maix.uart")
            return False

        devices = uart.list_devices()
        if not devices:
            print("串口打开失败: 未找到可用串口")
            return False

        port = device if device else devices[0]

        try:
            self.serial = uart.UART(port, self.baudrate)
        except Exception as exc:
            self.serial = None
            print("串口打开失败:%s" % exc)
            return False

        self.device = port
        print("串口已经打开:%s, baudrate:%d" % (port, self.baudrate))
        return True

    def is_open(self):
        """返回串口是否已经成功打开。"""
        return self.serial is not None

    def send(self, data):
        """发送原始字节数据。"""
        if not self.serial:
            return False

        try:
            self.serial.write(data)
        except Exception as exc:
            print("串口发送失败:%s" % exc)
            return False

        return True

    def send_packet(self, has_target, err_x=0, err_y=0):
        """按协议打包并发送一帧数据。"""
        try:
            frame = build_frame(has_target, err_x, err_y)
        except ValueError as exc:
            print("串口打包失败:%s" % exc)
            return False

        return self.send(frame)

    def send_error(self, err_x, err_y):
        """识别到目标时，发送目标点相对屏幕中心的误差。"""
        return self.send_packet(True, err_x, err_y)

    def send_no_target(self):
        """未识别到目标时，发送无目标状态帧。"""
        return self.send_packet(False, 0, 0)

    def send_offset(self, dx, dy):
        """兼容旧调用名；dx、dy 等价于 err_x、err_y。"""
        return self.send_error(dx, dy)

    def send_center(self, x, y):
        """兼容旧调用名；此处的 x、y 表示误差，不是绝对坐标。"""
        return self.send_error(x, y)
