import enum
import pickle
import platform
import struct
import subprocess
from pathlib import Path
from typing import List, Tuple, Union, Optional

import cv2
import numpy as np
import serial

import mvsdk

CRC_START_8 = 0x00

CRC_START_16 = 0xFFFF
CRC_POLY_16 = 0xA001

crc_tab8 = [0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46, 67, 114, 33, 16, 135, 182, 229, 212,
            250, 203, 152, 169, 62, 15, 92, 109, 134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153,
            168, 197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235, 61, 12, 95, 110, 249, 200, 155,
            170, 132, 181, 230, 215, 64, 113, 34, 19, 126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50,
            97, 80, 187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149, 248, 201, 154, 171, 60, 13,
            94, 111, 65, 112, 35, 18, 133, 180, 231, 214, 122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7,
            54, 101, 84, 57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23, 252, 205, 158, 175,
            56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210, 191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85,
            194, 243, 160, 145, 71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105, 4, 53, 102,
            87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42, 193, 240, 163, 146, 5, 52, 103, 86, 120, 73,
            26, 43, 188, 141, 222, 239, 130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172]

crc_tab16 = [0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305, 1088, 52225,
             3264, 3456, 52545, 3840, 53185, 52865, 3648, 2560, 51905, 52097, 2880, 51457, 2496, 2176, 51265, 55297,
             6336, 6528, 55617, 6912, 56257, 55937, 6720, 7680, 57025, 57217, 8000, 56577, 7616, 7296, 56385, 5120,
             54465, 54657, 5440, 55041, 6080, 5760, 54849, 53761, 4800, 4992, 54081, 4352, 53697, 53377, 4160, 61441,
             12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529,
             15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617,
             14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865,
             60545, 11328, 58369, 9408, 9600, 58689, 9984, 59329, 59009, 9792, 8704, 58049, 58241, 9024, 57601, 8640,
             8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241,
             26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841,
             26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640,
             48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761,
             45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401,
             22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128,
             23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457,
             19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368,
             18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448]


def crc_8(input_bytes) -> int:
    crc = CRC_START_8
    for i in range(len(input_bytes)):
        crc = crc_tab8[(input_bytes[i]) ^ crc]
    return crc


def crc_16(input_bytes) -> int:
    crc = CRC_START_16

    for i in range(len(input_bytes)):
        crc = (crc >> 8) ^ crc_tab16[(crc ^ input_bytes[i]) & 0xFF]

    return crc


def set_flag_register(flags):
    flag_register = 0

    for i in range(min(len(flags), 4)):
        flag_register = (flag_register << 4) | (flags[i] & 0xFF)

    return flag_register


def get_flag_register(flag_register, flags_length):
    flags = [0] * flags_length

    for i in range(min(flags_length, 4)):
        shift_mount = (flags_length - i - 1) * 4
        flags[i] = (flag_register >> shift_mount) & 0xFF

    return flags


class SerialProtocolParser:
    def __init__(self, port, baudrate=115200, timeout=1):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            raise Exception(f"Failed to open serial port {port}: {e}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ser.close()

    def _find_header(self):
        while True:
            header = self.ser.read(1)
            if header == b'\xA5':  # 找到SOF 0xA5
                header += self.ser.read(3)  # 返回整个帧头
                break

        # 从帧头提取数据长度和帧头CRC校验值
        data_length, header_crc = struct.unpack("<HB", header[1:])
        calculated_crc = crc_8(header[:3])  # 排除CRC校验位

        if header_crc != calculated_crc:
            raise ValueError("Header CRC check failed")

        return header, data_length, header_crc

    def read_frame(self):
        header, data_length, _ = self._find_header()

        # 读取控制位
        cmd_id = self.ser.read(2)

        # 读取数据位
        data = self.ser.read(data_length)

        frame_crc = struct.unpack("<H", self.ser.read(2))[0]
        calculated_crc = crc_16(header + cmd_id + data)

        if frame_crc != calculated_crc:
            raise ValueError("Frame CRC check failed")

        # 解析数据段
        flags_register = struct.unpack("<H", data[:2])[0]
        float_data = struct.unpack("<{}f".format((data_length - 2) // 4), data[2:])

        return cmd_id, flags_register, float_data

    def send_frame(self, cmd_id, flags_register, data):
        data_length = 2 + 4 * len(data)
        header = struct.pack("<BH", 0xA5, data_length)
        header_crc = crc_8(header)
        header = struct.pack("<BHB", 0xA5, data_length, header_crc)

        cmd_id = struct.pack("<H", cmd_id)

        flags_register = struct.pack("<H", flags_register)

        data = struct.pack("<{}f".format(len(data)), *data)

        frame = header + cmd_id + flags_register + data
        frame_crc = crc_16(frame)
        frame += struct.pack("<H", frame_crc)

        self.ser.write(frame)

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()


class ArmorType(enum.Enum):
    INVALID = 0
    SMALL = 1
    LARGE = 2


class ArmorColor(enum.Enum):
    RED = "RED"
    BLUE = "BLUE"


class ArmorLight:
    def __init__(self, points):
        self.points = points
        self.rect = cv2.minAreaRect(points)
        self.center = self.rect[0]
        self.width, self.height = sorted([self.rect[1][0], self.rect[1][1]])
        self.angle = self.rect[2]

        self.tilt_angle = self.angle
        if self.angle > 90:
            self.tilt_angle = 180 - self.angle

        sorted_box = sorted(cv2.boxPoints(self.rect), key=lambda x: x[1])
        self.top = np.intp((sorted_box[0][0] + sorted_box[1][0]) / 2, (sorted_box[0][1] + sorted_box[1][1]) / 2)
        self.bottom = np.intp((sorted_box[2][0] + sorted_box[3][0]) / 2, (sorted_box[2][1] + sorted_box[3][1]) / 2)


class Armor:
    def __init__(self, left_light: ArmorLight, right_light: ArmorLight, armor_type: ArmorType = ArmorType.INVALID):
        self.left_light = left_light
        self.right_light = right_light

        self.center = (
            (left_light.center[0] + right_light.center[0]) / 2,
            (left_light.center[1] + right_light.center[1]) / 2
        )
        self.armor_type = armor_type
        self.number_image = None

        self.confidence = None
        self.number = ""
        self.classification_result = ""


class ArmorLightParams:
    def __init__(self, min_ratio: float = 0.1, max_ratio: float = 0.5, max_angle: int = 40):
        self.min_ratio = min_ratio
        self.max_ratio = max_ratio
        self.max_angle = max_angle


class ArmorParams:
    def __init__(self,
                 min_light_ratio: float = 0.7,
                 min_small_center_distance: float = 0.8,
                 max_small_center_distance: float = 3.2,
                 min_large_center_distance: float = 3.2,
                 max_large_center_distance: float = 5.5,
                 max_angle: float = 35.0,
                 small_armor_width: int = 135,
                 small_armor_height: int = 125,
                 large_armor_width: int = 230,
                 large_armor_height: int = 125):
        self.min_light_ratio = min_light_ratio
        self.min_small_center_distance = min_small_center_distance
        self.max_small_center_distance = max_small_center_distance
        self.min_large_center_distance = min_large_center_distance
        self.max_large_center_distance = max_large_center_distance
        self.max_angle = max_angle
        self.small_armor_width = small_armor_width
        self.small_armor_height = small_armor_height
        self.large_armor_width = large_armor_width
        self.large_armor_height = large_armor_height


class NumberClassifier:
    def __init__(self, model_path: Path, label_path: Path, threshold: float = 0.5, ignore_classes: Optional[List[str]] = None):
        self.model_path = model_path
        self.label_path = label_path
        self.threshold = threshold
        self.ignore_classes = ignore_classes or []

        self.net = cv2.dnn.readNetFromONNX(str(model_path))

        with open(self.label_path, "r") as label_file:
            self.class_names = [line.strip() for line in label_file]

    @staticmethod
    def extract_numbers(image, armors: List[Armor]):
        light_length = 12
        warp_height = 28
        small_armor_width = 32
        large_armor_width = 54
        roi_size = (20, 28)

        for armor in armors:
            lights_vertices = np.array([armor.left_light.bottom, armor.left_light.top, armor.right_light.top, armor.right_light.bottom], dtype=np.float32)

            top_light_y = (warp_height - light_length) // 2 - 1
            bottom_light_y = top_light_y + light_length
            warp_width = small_armor_width if armor.armor_type == ArmorType.SMALL else large_armor_width

            target_vertices = np.array([
                [0, bottom_light_y],
                [0, top_light_y],
                [warp_width - 1, top_light_y],
                [warp_width - 1, bottom_light_y]
            ], dtype=np.float32)

            rotation_matrix = cv2.getPerspectiveTransform(lights_vertices, target_vertices)
            number_image = cv2.warpPerspective(image, rotation_matrix, (warp_width, warp_height))

            start_x = (warp_width - roi_size[0]) // 2
            number_image = number_image[0:roi_size[1], start_x:start_x + roi_size[0]]

            number_image = cv2.cvtColor(number_image, cv2.COLOR_RGB2GRAY)
            _, number_image = cv2.threshold(number_image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            armor.number_image = number_image

    def classify(self, armors: List[Armor]):
        valid_armors = []

        for armor in armors:
            if not hasattr(armor, 'number_image') or armor.number_image is None:
                continue

            image = armor.number_image.copy() / 255.0
            blob = cv2.dnn.blobFromImage(image)

            self.net.setInput(blob)
            outputs = self.net.forward()

            max_prob = np.max(outputs)
            softmax_prob = np.exp(outputs - max_prob)
            softmax_prob /= np.sum(softmax_prob)

            confidence = np.max(softmax_prob)
            label_id = np.argmax(softmax_prob)

            armor.confidence = confidence
            armor.number = self.class_names[label_id]
            armor.classification_result = f"{armor.number}: {armor.confidence * 100.0:.1f}%"

            if self._validate_armor(armor):
                valid_armors.append(armor)

        armors[:] = valid_armors

    def _validate_armor(self, armor: Armor) -> bool:
        if armor.confidence < self.threshold:
            return False

        if armor.number in self.ignore_classes:
            return False

        mismatch_armor_type = False
        if armor.armor_type == ArmorType.LARGE:
            mismatch_armor_type = armor.number in ["outpost", "2", "guard"]
        elif armor.armor_type == ArmorType.SMALL:
            mismatch_armor_type = armor.number in ["1", "base"]

        return not mismatch_armor_type


class ArmorDetector:
    def __init__(self,
                 model_path: Path,
                 label_path: Path,
                 gray_threshold: int = 50,
                 threshold: int = 120,
                 morph_element: Union[cv2.Mat, np.ndarray] = cv2.getStructuringElement(cv2.MORPH_RECT, [5, 5], [3, 3]),
                 target_color: ArmorColor = ArmorColor.RED,
                 armor_light_params: ArmorLightParams = ArmorLightParams(),
                 armor_params: ArmorParams = ArmorParams(),
                 classifier_threshold: float = 0.5,
                 ignore_classes: List[str] = None):
        self.gray_threshold = gray_threshold
        self.threshold = threshold
        self.morph_element = morph_element
        self.target_color = target_color
        self.armor_light_params = armor_light_params
        self.armor_params = armor_params
        self.classifier = NumberClassifier(model_path, label_path, classifier_threshold, ignore_classes)

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, gray_binary = cv2.threshold(gray, self.gray_threshold, 255, cv2.THRESH_BINARY)

        b, g, r = cv2.split(image)
        if self.target_color == ArmorColor.RED:
            color_diff = cv2.subtract(r, b)
        else:
            color_diff = cv2.subtract(b, r)
        _, color_binary = cv2.threshold(color_diff, self.threshold, 255, cv2.THRESH_BINARY)

        binary = cv2.bitwise_and(color_binary, gray_binary)

        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.morph_element)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.morph_element)

        return binary

    def is_light(self, light: ArmorLight) -> bool:
        ratio = light.width / light.height
        return (self.armor_light_params.min_ratio < ratio < self.armor_light_params.max_ratio and
                light.tilt_angle < self.armor_light_params.max_angle)

    def is_armor(self, light_1: ArmorLight, light_2: ArmorLight) -> ArmorType:
        light_length_ratio = min(light_1.width, light_2.width) / max(light_1.width, light_2.width)
        if light_length_ratio <= self.armor_light_params.min_ratio:
            return ArmorType.INVALID

        light_centers_diff = np.array(light_1.center) - np.array(light_2.center)
        avg_light_length = (light_1.width + light_2.width) / 2
        center_distance = np.linalg.norm(light_centers_diff) / avg_light_length

        small_armor_distance = (self.armor_params.min_small_center_distance <= center_distance <= self.armor_params.max_small_center_distance)
        large_armor_distance = (self.armor_params.min_large_center_distance <= center_distance <= self.armor_params.max_large_center_distance)

        if not (small_armor_distance or large_armor_distance):
            return ArmorType.INVALID

        angle = np.abs(np.arctan2(light_centers_diff[1], light_centers_diff[0])) * 180 / np.pi
        if angle >= self.armor_params.max_angle:
            return ArmorType.INVALID

        return ArmorType.LARGE if center_distance > self.armor_params.min_large_center_distance else ArmorType.SMALL

    def find_lights(self, binary_image) -> List[ArmorLight]:
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lights = []
        for contour in contours:
            if len(contour) < 5:
                continue

            light = ArmorLight(contour)
            if self.is_light(light):
                lights.append(light)
        return lights

    @staticmethod
    def point_in_rect(point, rect):
        x, y = point
        rx, ry, rw, rh = rect
        return rx <= x <= rx + rw and ry <= y <= ry + rh

    def contain_light(self, light_1: ArmorLight, light_2: ArmorLight, lights: List[ArmorLight]) -> bool:
        points = np.array([light_1.top, light_1.bottom, light_2.top, light_2.bottom], dtype=np.float32)
        bounding_rect = cv2.boundingRect(points)

        light_centers = [light_1.center, light_2.center]
        for test_light in lights:
            if test_light.center in light_centers:
                continue

            if (
                    self.point_in_rect(test_light.top, bounding_rect) or
                    self.point_in_rect(test_light.bottom, bounding_rect) or
                    self.point_in_rect(test_light.center, bounding_rect)
            ):
                return True

        return False

    def match_lights(self, lights: List[ArmorLight]) -> List[Armor]:
        armors = []
        for i in range(len(lights)):
            light_1 = lights[i]
            for j in range(i + 1, len(lights)):
                light_2 = lights[j]

                if self.contain_light(light_1, light_2, lights):
                    continue

                armor_type = self.is_armor(light_1, light_2)
                if armor_type != ArmorType.INVALID:
                    armor = Armor(light_1, light_2, armor_type)
                    armors.append(armor)
        return armors

    def detect(self, image) -> Tuple[List[ArmorLight], List[Armor]]:
        binary_image = self.preprocess_image(image)

        lights = self.find_lights(binary_image)
        armors = self.match_lights(lights)

        if armors and self.classifier:
            self.classifier.extract_numbers(image, armors)
            self.classifier.classify(armors)

        return lights, armors

    def draw_result(self, image, lights: List[ArmorLight], armors: List[Armor]):
        for light in lights:
            top = tuple(map(int, light.top))
            bottom = tuple(map(int, light.bottom))

            cv2.circle(image, top, 3, (255, 255, 255), 1)
            cv2.circle(image, bottom, 3, (255, 255, 255), 1)

            line_color = (255, 255, 0) if self.target_color == ArmorColor.RED else (255, 0, 255)
            cv2.line(image, top, bottom, line_color, 1)

        for armor in armors:
            left_top = tuple(map(int, armor.left_light.top))
            left_bottom = tuple(map(int, armor.left_light.bottom))
            right_top = tuple(map(int, armor.right_light.top))
            right_bottom = tuple(map(int, armor.right_light.bottom))

            cv2.line(image, left_top, right_bottom, (0, 255, 0), 2)
            cv2.line(image, left_bottom, right_top, (0, 255, 0), 2)

            cv2.putText(image, armor.classification_result, left_top, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        return image


class PnPSolver:
    def __init__(self, camera_matrix, dist_coeffs, armor_params: ArmorParams = ArmorParams()):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.armor_params = armor_params

        small_half_y = self.armor_params.small_armor_width / 2000.0
        small_half_z = self.armor_params.small_armor_height / 2000.0
        large_half_y = self.armor_params.large_armor_width / 2000.0
        large_half_z = self.armor_params.large_armor_height / 2000.0

        self.small_armor_points = np.array([
            [0, small_half_y, -small_half_z],
            [0, small_half_y, small_half_z],
            [0, -small_half_y, small_half_z],
            [0, -small_half_y, -small_half_z]
        ], dtype=np.float32)

        self.large_armor_points = np.array([
            [0, large_half_y, -large_half_z],
            [0, large_half_y, large_half_z],
            [0, -large_half_y, large_half_z],
            [0, -large_half_y, -large_half_z]
        ], dtype=np.float32)

    def solve(self, armor: Armor):
        image_armor_points = np.array([
            armor.left_light.bottom,
            armor.left_light.top,
            armor.right_light.top,
            armor.right_light.bottom
        ], dtype=np.float32)

        object_points = self.small_armor_points if armor.armor_type == ArmorType.SMALL else self.large_armor_points

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_armor_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE
        )

        return success, rvec, tvec

    def calculate_distance_to_center(self, image_point):
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        return np.linalg.norm(np.array(image_point) - np.array([cx, cy]))


class MvCamera:
    def __init__(self, pipe=0, img_size=640):
        self.pipe = pipe
        self.img_size = img_size
        self.camera_matrix = None
        self.dist_coeffs = None

        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) == 0:
            raise RuntimeError("No camera devices found")

        if self.pipe >= len(dev_list):
            raise ValueError(f"Invalid camera pipe index: {self.pipe}. Available pipes are 0 to {len(dev_list) - 1}.")

        self.dev_info = dev_list[self.pipe]

        self.camera_handle = mvsdk.CameraInit(self.dev_info)
        self.camera_capability = mvsdk.CameraGetCapability(self.camera_handle)

        mono_camera = (self.camera_capability.sIspCapacity.bMonoSensor != 0)
        if mono_camera:
            mvsdk.CameraSetIspOutFormat(self.camera_handle, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.camera_handle, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(self.camera_handle, 0)

        mvsdk.CameraSetAeState(self.camera_handle, 0)
        mvsdk.CameraSetExposureTime(self.camera_handle, 10 * 1000)

        mvsdk.CameraPlay(self.camera_handle)

        self.frame_buffer_size = self.camera_capability.sResolutionRange.iWidthMax * self.camera_capability.sResolutionRange.iHeightMax * (1 if mono_camera else 3)
        self.frame_buffer = mvsdk.CameraAlignMalloc(self.frame_buffer_size, 16)

        try:
            self.load_camera_calibration_data("camera_calibration.pkl")
        except FileNotFoundError:
            self.camera_calibrate()

    def camera_calibrate(self, pattern_size=(6, 9), need_points=20):
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[1], 0:pattern_size[0]].T.reshape(-1, 2)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        obj_points = []
        img_points = []

        for frame in self:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            key = cv2.waitKey(1) & 0xFF

            if ret:
                sub_pix_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                if key == ord(' '):
                    obj_points.append(objp)
                    img_points.append(sub_pix_corners)

                frame = cv2.drawChessboardCorners(frame, pattern_size, sub_pix_corners, ret)

            if len(obj_points) == need_points:
                ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    obj_points, img_points, (self.img_size, self.img_size), self.camera_matrix, self.dist_coeffs
                )
                self.save_camera_calibration_data("camera_calibration.pkl")
                return ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs

            frame = cv2.putText(frame, "{}".format(len(img_points)), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 5, cv2.LINE_AA)
            cv2.imshow("Camera Calibration", frame)
            if key == ord("q"):
                break

        cv2.destroyWindow("Camera Calibration")

    def save_camera_calibration_data(self, filename):
        with open(filename, "wb") as f:
            pickle.dump((self.camera_matrix, self.dist_coeffs), f)

    def load_camera_calibration_data(self, filename):
        with open(filename, "rb") as f:
            self.camera_matrix, self.dist_coeffs = pickle.load(f)

    def close(self):
        mvsdk.CameraUnInit(self.camera_handle)
        mvsdk.CameraAlignFree(self.frame_buffer)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __iter__(self):
        return self

    def __del__(self):
        self.close()

    def __next__(self):
        try:
            raw_data, frame_header = mvsdk.CameraGetImageBuffer(self.camera_handle, 50)
            mvsdk.CameraImageProcess(self.camera_handle, raw_data, self.frame_buffer, frame_header)
            mvsdk.CameraReleaseImageBuffer(self.camera_handle, raw_data)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(self.frame_buffer, frame_header, 1)

            frame_data = (mvsdk.c_ubyte * frame_header.uBytes).from_address(self.frame_buffer)
            frame = np.frombuffer(frame_data, np.uint8)
            frame = frame.reshape(frame_header.iHeight, frame_header.iWidth,
                                  1 if frame_header.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3)

            frame = cv2.resize(frame, (self.img_size, self.img_size), interpolation=cv2.INTER_LINEAR)

            return frame
        except mvsdk.CameraException as e:
            print(e)


VCP_PORT = "/dev/ttyUSB0"

if __name__ == "__main__":
    armor_detector = ArmorDetector(model_path=Path("./model/mlp.onnx"), label_path=Path("./model/labels.txt"))
    if platform.system() == "Linux":
        subprocess.run("sudo chmod 666 {}".format(VCP_PORT).split())

    with SerialProtocolParser("{}".format(VCP_PORT)) as parser:
        with MvCamera() as camera:
            for frame in camera:
                lights, armors = armor_detector.detect(frame)
                frame = armor_detector.draw_result(frame, lights, armors)

                cv2.imshow("Target", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyAllWindows()

# TARGET_COLOR = "RED"
# VCP_PORT = "COM5"
#
# armor_obj_points = np.array([[0, 0, 0],  # 左上角 lt
#                              [0, 56.5, 0],  # 左下角 lb
#                              [140.5, 0, 0],  # 右上角 rt
#                              [140.5, 56.5, 0]  # 右下角 rb
#                              ], dtype=np.float32)
#
# if __name__ == '__main__':
#     if platform.system() == "Linux":
#         subprocess.run("sudo chmod 666 {}".format(VCP_PORT).split())
#
#     with SerialProtocolParser("{}".format(VCP_PORT)) as parser:
#         mv_camera = MvCamera()
#
#         for frame in mv_camera:
#             ec_data = parser.read_frame()[2]
#             yaw, pitch, roll = 0, 0, 0
#
#             binary_frame = pre_process(frame, 100, 120, TARGET_COLOR)
#             # cv.imshow("Binary", binary_frame)
#
#             armor_lights = get_all_armor_light(binary_frame)
#             armors = get_armor(armor_lights)
#
#             for armor in armors:
#                 left, right = get_armor_corners(armor)
#                 armor_img_points = np.array([[left[0][0], left[0][1]],  # 左上角 lt
#                                              [left[1][0], left[1][1]],  # 左下角 lb
#                                              [right[0][0], right[0][1]],  # 右上角 rt
#                                              [right[1][0], right[1][1]]  # 右下角 rb
#                                              ], dtype=np.float32)
#
#                 ret, rvecs, tvecs = cv.solvePnP(armor_obj_points, armor_img_points, mv_camera.camera_matrix,
#                                                 mv_camera.dist_coeffs)
#                 x = tvecs[0][0]
#                 y = tvecs[1][0]
#                 z = tvecs[2][0]
#                 distance = np.sqrt(x ** 2 + y ** 2 + z ** 2)
#
#                 hit_point = get_hit_point(armor)
#
#                 if not 300 < hit_point[0] < 330 or not 300 < hit_point[1] < 330:
#                     pass
#
#                 target_frame = draw_armor_info(frame, armor, distance)
#
#             target_frame = draw_armor(frame, armors)
#
#             cv.imshow("Target", target_frame)
#
#             # print(pitch, yaw)
#
#             parser.send_frame(0x01, set_flag_register([0, 0, 0]), (pitch, yaw, 0, 0, 0, 0))
#             if cv.waitKey(1) & 0xFF == ord("q"):
#                 break
#
#         cv.destroyAllWindows()
#         del mv_camera

# video = cv.VideoCapture("1.avi")

# while video.isOpened():
#     ret, frame = video.read()
#     if ret:
#         binary_frame = pre_process(frame, 120, TARGET_COLOR)
#         # cv.imshow("binary", binary_frame)
#
#         armor_lights = get_all_armor_light(binary_frame)
#         contours_frame = draw_contours(frame, armor_lights)
#
#         armor = get_armor(armor_lights)
#         target_frame = draw_armor(frame, armor)
#
#         cv.imshow("Frame", target_frame)
#     else:
#         video.release()
#         cv.waitKey(0)
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break
#
# cv.destroyAllWindows()
#
# cv.namedWindow('Blank Window')
# cv.imshow('Blank Window', np.zeros((512, 512, 3), np.uint8))
#
# keys_down = set()
#
# while True:
#     try:
#         key = cv.waitKey(1) & 0xFF
#
#         if key != 255:
#             keys_down.add(key)
#         else:
#             keys_down.clear()
#
#         pitch = 0
#         yaw = 0
#
#         vx = 0
#         vy = 0
#
#         if ord('q') in keys_down:
#             break
#
#         if ord('a') in keys_down:
#             yaw = -1
#         elif ord('d') in keys_down:
#             yaw = 1
#
#         if ord('w') in keys_down:
#             pitch = 1
#         elif ord('s') in keys_down:
#             pitch = -1
#
#         parser.send_frame(0x01, 0, (pitch, yaw, vx, vy, 0, 0))
#     except ValueError:
#         continue
#
# cv.destroyAllWindows()
