import struct
import zlib

import cv2 as cv
import serial

sht75_crc_table = [
    0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
    67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
    134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
    197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
    61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
    126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
    187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
    248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
    122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
    57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
    252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
    191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
    71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
    4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
    193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
    130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172]


def crc_8(input_str):
    crc = 0
    for a in range(len(input_str)):
        crc = sht75_crc_table[(input_str[a]) ^ crc]
    return crc


class SerialProtocolParser:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate)

    def open_serial(self):
        self.ser.open()

    def close_serial(self):
        self.ser.close()

    def _find_header(self):
        while True:
            byte = self.ser.read(1)
            if byte == b'\xA5':  # 找到SOF 0xA5
                return byte + self.ser.read(3)  # 返回整个帧头

    def _parse_header(self, header):
        if len(header) != 4 or header[0] != 0xA5:
            raise ValueError("Invalid header format")

        # 从帧头提取数据长度和帧头CRC校验值
        data_length, header_crc = struct.unpack("<HB", header[1:])
        calculated_crc = crc_8(header)

        if header_crc != calculated_crc:
            raise ValueError("Header CRC check failed")

        return data_length, header_crc

    def _calculate_header_crc8(self, header):
        # 使用 CRC8 算法计算校验和
        data = header[:3]  # Exclude the crc_check byte
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
        return crc & 0xFF  # 返回低 8 位

    def _calculate_frame_crc(self, frame):
        # 使用 zlib 计算 CRC16 校验和 (保持不变)
        crc = zlib.crc32(frame) & 0xFFFF
        return crc

    def read_frame(self):
        header = self._find_header()
        data_length, _ = self._parse_header(header)

        # 读取控制位
        cmd_id = self.ser.read(2)

        # 读取数据位
        data = self.ser.read(2 + 4 * data_length)

        frame_crc = struct.unpack("<H", self.ser.read(2))[0]
        calculated_crc = self._calculate_frame_crc(header + cmd_id + data)

        if frame_crc != calculated_crc:
            raise ValueError("Frame CRC check failed")

        # 解析数据段
        flags_register = struct.unpack("<H", data[:2])[0]
        float_data = struct.unpack("<{}f".format(data_length), data[2:])

        return cmd_id, flags_register, float_data

    def send_frame(self, cmd_id, flags_register, data):
        data_length = len(data) // 4
        header = struct.pack("<BH", 0xA5, data_length)
        header_crc = self._calculate_header_crc8(header)
        header = struct.pack("<BHB", 0xA5, data_length, header_crc)

        cmd_id = struct.pack("<H", cmd_id)

        flags_register = struct.pack("<H", flags_register)

        data = struct.pack("<{}f".format(data_length), *data)

        frame = header + cmd_id + flags_register + data
        frame_crc = self._calculate_frame_crc(frame)
        frame += struct.pack("<H", frame_crc)

        self.ser.write(frame)

    def __del__(self):
        self.close_serial()


def pre_process(image, threshold=120, color="RED"):
    image_channels = cv.split(image)
    if color == "RED":
        image = cv.subtract(image_channels[2], image_channels[0])
    elif color == "BLUE":
        image = cv.subtract(image_channels[0], image_channels[2])
    _, binary = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)

    element = cv.getStructuringElement(cv.MORPH_RECT, [5, 5], [3, 3])
    binary = cv.morphologyEx(binary, cv.MORPH_OPEN, element)

    element = cv.getStructuringElement(cv.MORPH_RECT, [5, 5], [3, 3])
    binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, element)

    return binary


def get_armor_light_angle(rotated_rect):
    width, height = rotated_rect[1][0], rotated_rect[1][1]
    angle = rotated_rect[2]

    if width > height:
        angle += 90

    if angle < 0:
        angle += 180
    elif angle >= 180:
        angle -= 180

    return angle


def get_all_armor_light(image):
    contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    armor_light_contours = []
    for i in contours:
        rect = cv.minAreaRect(i)
        angle = get_armor_light_angle(rect)

        if angle < 20 or angle > 160:
            height = max(rect[1][0], rect[1][1])
            width = min(rect[1][0], rect[1][1])

            if height / width > 2:
                armor_light_contours.append(i)

    return armor_light_contours


def draw_contours(image, contours, color=(0, 255, 0), thickness=2):
    image = cv.drawContours(image, contours, -1, color, thickness)
    return image


def get_armor(contours):
    if len(contours) < 2:
        return []
    contours.sort(key=lambda x: cv.minAreaRect(x)[0][0])
    candidate_armor = []
    for i in range(len(contours) - 1):
        candidate_armor.append([contours[i], contours[i + 1]])

    armor = []

    for i in candidate_armor:
        armor_left_rect = cv.minAreaRect(i[0])
        armor_right_rect = cv.minAreaRect(i[1])

        angle_left = get_armor_light_angle(armor_left_rect)
        angle_right = get_armor_light_angle(armor_right_rect)

        center_left = armor_left_rect[0]
        center_right = armor_right_rect[0]

        height_left = max(armor_left_rect[0][0], armor_right_rect[0][1])
        height_right = max(armor_right_rect[1][0], armor_left_rect[1][1])
        avg_height = (height_right + height_left) / 2

        if abs(angle_left - angle_right) > 10:
            continue

        if abs(center_left[1] - center_right[1]) > avg_height * 0.1:
            continue

        armor.append(i)
    return armor


def get_hit_point(armor):
    if len(armor) != 2:
        return ()

    left_rect = cv.minAreaRect(armor[0])
    right_rect = cv.minAreaRect(armor[1])

    hit_point = (int((left_rect[0][0] + right_rect[0][0]) / 2), int((left_rect[0][1] + right_rect[0][1]) / 2))

    return hit_point


def draw_armor(image, armors, color=(255, 0, 0), thickness=1, radius=5, hit_point_color=(0, 0, 255)):
    for armor in armors:
        if len(armor) != 2:
            continue

        centers = []
        for armor_light in armor:
            rect = cv.minAreaRect(armor_light)
            centers.append((int(rect[0][0]), int(rect[0][1])))

        image = cv.line(image, centers[0], centers[1], color, thickness)
        hit_point = get_hit_point(armor)
        image = cv.circle(image, hit_point, radius, hit_point_color, thickness)

    return image


"""
TARGET_COLOR = "RED"

video = cv.VideoCapture("1.avi")

while video.isOpened():
    ret, frame = video.read()
    if ret:
        binary_frame = pre_process(frame, 120, TARGET_COLOR)
        # cv.imshow("binary", binary_frame)

        armor_lights = get_all_armor_light(binary_frame)
        contours_frame = draw_contours(frame, armor_lights)

        armor = get_armor(armor_lights)
        target_frame = draw_armor(frame, armor)

        cv.imshow("Frame", target_frame)
    else:
        video.release()
        cv.waitKey(0)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.destroyAllWindows()
"""

parser = SerialProtocolParser("/dev/ttyACM0")
parser.close_serial()
parser.open_serial()

while True:
    try:
        cmd_id, flags_register, float_data = parser.read_frame()
        print(cmd_id, flags_register, float_data)
    except ValueError:
        continue
