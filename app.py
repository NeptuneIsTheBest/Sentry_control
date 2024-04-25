import struct
import zlib

import cv2 as cv
import serial


class SerialProtocolParser:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate)

    def _find_header(self):
        while True:
            byte = self.ser.read(1)
            if byte == b'\xA5':  # 找到帧头
                return byte + self.ser.read(3)  # 读取剩余的头部字节

    def _parse_header(self, header):
        if len(header) != 4 or header[0] != 0xA5:
            raise ValueError("Invalid header format")
        data_length = struct.unpack("<H", header[1:3])[0]
        header_crc = header[3]
        calculated_crc = self._calculate_header_crc8(header)
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

        # 读取数据段和帧尾
        data = self.ser.read(data_length + 2)

        frame_crc = struct.unpack("<H", data[-2:])[0]
        calculated_crc = self._calculate_frame_crc(header + data[:-2])

        if frame_crc != calculated_crc:
            raise ValueError("Frame CRC check failed")

        # 解析数据段
        flags_register = struct.unpack("<H", data[:2])[0]
        float_data = struct.unpack("<" + "f" * (data_length // 4), data[2:-2])

        return flags_register, float_data

    def send_frame(self, cmd_id, data):
        data_length = len(data) // 4  # 假设 data 是 float 类型的数据
        header = struct.pack("<BHHB", 0xA5, data_length, 0, 0)
        header_crc = self._calculate_header_crc8(header)
        header = struct.pack("<BHHB", 0xA5, data_length, header_crc, cmd_id)

        frame = header + data
        frame_crc = self._calculate_frame_crc(frame)
        frame += struct.pack("<H", frame_crc)

        self.ser.write(frame)


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

s = LowerComputerComm()
