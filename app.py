import pickle
import platform
import struct
import subprocess

import cv2
import cv2 as cv
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
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate)

    def open_serial(self):
        self.ser.open()

    def close_serial(self):
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
        self.close_serial()


def pre_process(image, threshold=120, color="RED"):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, gray_binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    image_channels = cv.split(image)
    if color == "RED":
        image = cv.subtract(image_channels[2], image_channels[0])
    elif color == "BLUE":
        image = cv.subtract(image_channels[0], image_channels[2])
    _, color_binary = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)

    binary = cv.bitwise_and(color_binary, gray_binary)

    element = cv.getStructuringElement(cv.MORPH_RECT, [5, 5], [3, 3])
    binary = cv.morphologyEx(binary, cv.MORPH_OPEN, element)

    element = cv.getStructuringElement(cv.MORPH_RECT, [5, 5], [3, 3])
    binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, element)

    return binary


def get_armor_light_angle(rotated_rect):
    width, height = rotated_rect[1]
    angle = rotated_rect[2]

    if width > height:
        angle += 90

    angle = angle % 180

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


def get_armor_corners(armor):
    if len(armor) != 2:
        return ()

    left_box = np.intp(cv.boxPoints(cv.minAreaRect(armor[0])))
    right_box = np.intp(cv.boxPoints(cv.minAreaRect(armor[1])))

    left_box = sorted(left_box, key=lambda x: x[1])
    lt = (int((left_box[0][0] + left_box[1][0]) / 2), int((left_box[0][1] + left_box[1][1]) / 2))
    lb = (int((left_box[2][0] + left_box[3][0]) / 2), int((left_box[2][1] + left_box[3][1]) / 2))

    right_box = sorted(right_box, key=lambda x: x[1])
    rt = (int((right_box[0][0] + right_box[1][0]) / 2), int((right_box[0][1] + right_box[1][1]) / 2))
    rb = (int((right_box[2][0] + right_box[3][0]) / 2), int((right_box[2][1] + right_box[3][1]) / 2))

    return lt, lb, rt, rb


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

        hit_point = get_hit_point(armor)
        corners = get_armor_corners(armor)
        for corner in corners:
            image = cv.circle(image, corner, radius, hit_point_color, thickness)
        image = cv.circle(image, hit_point, radius, hit_point_color, thickness)

        image = cv.line(image, corners[0], corners[3], color, thickness)
        image = cv.line(image, corners[1], corners[2], color, thickness)

    return image


class MvCamera:
    def __init__(self, pipe=0, img_size=640):
        self.pipe = pipe
        self.img_size = img_size
        self.camera_matrix = None
        self.dist_coeffs = None

        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) == 0:
            raise RuntimeError("No camera devices found")

        assert self.pipe < len(dev_list)

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

        self.frame_buffer_size = self.camera_capability.sResolutionRange.iWidthMax * self.camera_capability.sResolutionRange.iHeightMax * (
            1 if mono_camera else 3)
        self.frame_buffer = mvsdk.CameraAlignMalloc(self.frame_buffer_size, 16)

        try:
            self.load_camera_calibration_data("camera_calibration.pkl")
        except FileNotFoundError:
            self.camera_calibrate()

    def camera_calibrate(self, pattern_size=(6, 9), need_points=20):
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[1], 0:pattern_size[0]].T.reshape(-1, 2)

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        obj_points = []
        img_points = []

        for frame in self:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, pattern_size, None)

            key = cv.waitKey(1) & 0xFF

            if ret:
                sub_pix_corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                if key == ord(' '):
                    obj_points.append(objp)
                    img_points.append(sub_pix_corners)

                frame = cv.drawChessboardCorners(frame, pattern_size, sub_pix_corners, ret)

            if len(obj_points) == need_points:
                ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, (
                    self.img_size, self.img_size), None, None)
                self.save_camera_calibration_data("camera_calibration.pkl")
                return ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs

            frame = cv.putText(frame, "{}".format(len(img_points)), (10, 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0),
                               5, cv.LINE_AA)
            cv.imshow("Camera Calibration", frame)
            if key == ord("q"):
                break

        cv.destroyWindow("Camera Calibration")

    def save_camera_calibration_data(self, filename):
        with open(filename, "wb") as f:
            pickle.dump((self.camera_matrix, self.dist_coeffs), f)

    def load_camera_calibration_data(self, filename):
        with open(filename, "rb") as f:
            self.camera_matrix, self.dist_coeffs = pickle.load(f)

    def __iter__(self):
        return self

    def __del__(self):
        mvsdk.CameraUnInit(self.camera_handle)
        mvsdk.CameraAlignFree(self.frame_buffer)

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

            frame = cv.resize(frame, (self.img_size, self.img_size), interpolation=cv.INTER_LINEAR)

            return frame
        except mvsdk.CameraException as e:
            print(e)


TARGET_COLOR = "RED"
VCP_PORT = "/dev/tty.usbmodem3157376B34391"

armor_obj_points = np.array([
    [0, 0, 0],  # 左上角 lt
    [0, 56.5, 0],  # 左下角 lb
    [140.5, 0, 0],  # 右上角 rt
    [140.5, 56.5, 0]  # 右下角 rb
], dtype=np.float32)

if __name__ == '__main__':
    if platform.system() == "Linux":
        subprocess.run("sudo chmod 666 {}".format(VCP_PORT).split())

    parser = SerialProtocolParser("{}".format(VCP_PORT))
    parser.close_serial()
    parser.open_serial()

    mv_camera = MvCamera()

    for frame in mv_camera:
        ec_data = parser.read_frame()[3]
        yaw, pitch, roll = ec_data[0], ec_data[1], ec_data[2]

        binary_frame = pre_process(frame, 120, TARGET_COLOR)

        armor_lights = get_all_armor_light(binary_frame)
        armors = get_armor(armor_lights)

        for armor in armors:
            lt, lb, rt, rb = get_armor_corners(armor)
            armor_img_points = np.array([
                [lt[0], lt[1]],  # 左上角 lt
                [lb[0], lb[1]],  # 左下角 lb
                [rt[0], rt[1]],  # 右上角 rt
                [rb[0], rb[1]]  # 右下角 rb
            ], dtype=np.float32)

            ret, rvecs, tvecs = cv.solvePnP(armor_obj_points, armor_img_points, mv_camera.camera_matrix,
                                            mv_camera.dist_coeffs)

        target_frame = draw_armor(frame, armors)

        cv.imshow("Target", target_frame)

        parser.send_frame(0x01, set_flag_register([0, 0, 0]), (pitch, yaw, 0, 0, 0, 0))
        if cv.waitKey(1) & 0xFF == ord("q"):
            break

    cv.destroyAllWindows()
    del mv_camera

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
