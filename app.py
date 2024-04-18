import struct

import cv2 as cv
import serial

UPPER_PC_COMM_RECORD_ARRAY_LENGTH = 16
UPPER_PC_COMM_SEND_LENGTH = 20
UPPER_PC_COMM_REC_LENGTH = 16
UPPER_PC_COMM_REC_SOF = 0x66
UPPER_PC_COMM_REC_EOF = 0x88

COMM_SET_UP_FRAME = 0xCCCC
RECORD_ANGLE_FRAME = 0xFFFF
REQUEST_TRANSMIT_FRAME = 0xBBBB
SHOOT_ALLOW_FRAME = 0x0202
SHOOT_AUTO_FRAME = 0x0101
BLUE_TEAM_FRAME = 0xDDDD
RED_TEAM_FRAME = 0xEEEE


class LowerComputerComm:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate)

    def open(self):
        self.ser.open()

    def close(self):
        self.ser.close()

    def send(self, frame_type, target_pitch_angle_d, target_yaw_angle_d, target_speed_on_rail, target_locked):
        frame = struct.pack("<BBHffHBB",
                            UPPER_PC_COMM_REC_SOF,
                            frame_type, target_pitch_angle_d, target_yaw_angle_d, target_speed_on_rail, target_locked,
                            UPPER_PC_COMM_REC_EOF)
        print(frame)
        self.ser.write(frame)

    def recv(self):
        self.ser.read_until(bytes(UPPER_PC_COMM_REC_EOF))

    def __del__(self):
        self.ser.close()


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

s = LowerComputerComm("/dev/")
