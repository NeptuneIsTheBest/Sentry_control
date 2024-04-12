import os
import struct
import subprocess
import sys
import threading
import time
from math import pi

import can
import psutil

CHASSIS_MOTOR_NUM = 4
LF = 0
RF = 1
RB = 2
LB = 3

PI = pi
WHEEL_RADIUS = 0.076
ROBOT_CENTER_TO_WHEEL_RADIUS = 0.209
M3508_REDUCTION_FACTOR = 19
WHEEL_RPM_TO_WHEEL_MPS = (2 * pi * WHEEL_RADIUS) / 60.0 / M3508_REDUCTION_FACTOR
WHEEL_MPS_TO_WHEEL_RPM = (1 / (WHEEL_RPM_TO_WHEEL_MPS * 1.0))
ROBOT_RPS_TO_WHEEL_MPS = (2 * pi * ROBOT_CENTER_TO_WHEEL_RADIUS)
WHEEL_MPS_TO_ROBOT_RPS = (1 / (ROBOT_RPS_TO_WHEEL_MPS * 1.0))

M3508_MAX_OUTPUT_CURRENT = 16000


class PID:
    def __init__(self, kp, ki, kd, max_i_out=None, max_out=None, mode="position"):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.max_i_out = max_i_out
        self.max_out = max_out
        self.mode = mode

        self.error = [0.0, 0.0, 0.0]
        self.d_buf = [0.0, 0.0, 0.0]
        self.p_out = self.i_out = self.d_out = 0.0
        self.out = 0.0

    @staticmethod
    def limit(value, max_value):
        if max_value is not None:
            return max(min(value, max_value), -max_value)
        return value

    def update(self, target_value, measured_value):
        self.error[2], self.error[1] = self.error[1], self.error[0]
        self.error[0] = target_value - measured_value

        if self.mode == "position":
            self.p_out = self.kp * self.error[0]
            self.i_out = self.ki * self.error[0]
            self.i_out = self.limit(self.i_out, self.max_i_out)

            self.d_buf[2], self.d_buf[1] = self.d_buf[1], self.d_buf[0]
            self.d_buf[0] = self.error[0] - self.error[1]
            self.d_out = self.kd * self.d_buf[0]

            self.out = self.p_out + self.i_out + self.d_out
            self.out = self.limit(self.out, self.max_out)
        elif self.mode == "delta":
            self.p_out = self.kp * (self.error[0] - self.error[1])
            self.i_out = self.ki * self.error[0]

            self.d_buf[2], self.d_buf[1] = self.d_buf[1], self.d_buf[0]
            self.d_buf[0] = self.error[0] - 2.0 * self.error[1] + self.error[2]
            self.d_out = self.kd * self.d_buf[0]

            self.out = self.p_out + self.i_out + self.d_out
            self.out = self.limit(self.out, self.max_out)

        return self.out

    def refresh_buffer(self):
        self.error = [0.0, 0.0, 0.0]
        self.d_buf = [0.0, 0.0, 0.0]
        self.p_out = self.i_out = self.d_out = 0.0
        self.out = 0.0


class MotorM3508:
    def __init__(self, motor_id,
                 pid=False, kp=0.0, ki=0.0, kd=0.0, max_i_out=None, max_out=M3508_MAX_OUTPUT_CURRENT, mode="delta"):
        self.motor_id = motor_id

        self.rotor_angle = 0
        self.rotor_speed = 0
        self.torque_current = 0
        self.motor_temperature = 0

        self.motor_target_speed = 0

        self.frame_counter = 0

        self.pid = None if not pid else PID(kp, ki, kd, max_i_out, max_out, mode)

    def update_motor_data(self, rotor_angle, rotor_speed, torque_current, motor_temperature):
        self.rotor_angle = rotor_angle
        self.rotor_speed = rotor_speed
        self.torque_current = torque_current
        self.motor_temperature = motor_temperature

        self.frame_counter += 1

    @property
    def motor_mechanical_speed(self):
        return self.rotor_speed * WHEEL_RPM_TO_WHEEL_MPS

    @property
    def need_torque_current(self):
        return self.pid.update(self.motor_target_speed, self.motor_mechanical_speed)

    def __str__(self):
        return ("Motor ID: {} Angle: {} Speed: {} Target Speed: {}"
                " Current: {} Target Current: {}").format(self.motor_id,
                                                          self.rotor_angle,
                                                          self.motor_mechanical_speed,
                                                          self.motor_target_speed,
                                                          self.torque_current,
                                                          self.need_torque_current)


Kp = 1000000.0
Ki = 0.0
Kd = 0.0
chassis_motors = [MotorM3508(i, True, Kp, Ki, Kd) for i in range(CHASSIS_MOTOR_NUM)]


class MotorM3508RxHandler(can.Listener):
    def __init__(self):
        super(MotorM3508RxHandler, self).__init__()

    def on_message_received(self, message: can.Message) -> None:
        if 0x200 < message.arbitration_id < 0x205:
            self.chassis_motor_data_received(message)

    @staticmethod
    def chassis_motor_data_received(message: can.Message) -> None:
        if message.dlc != 8:
            return
        chassis_motors[message.arbitration_id - 0x201].update_motor_data(*parse_c620_data(message.data))


def can_send_message(bus, message_id, data, timeout=None):
    bus.send(can.Message(timestamp=time.time(), arbitration_id=int(message_id), data=data, is_extended_id=False),
             timeout=timeout)


def can_receive_messages(bus, timeout=None):
    return bus.recv(timeout)


def parse_c620_data(data):
    rotor_angle = (data[0] << 8) + data[1]
    if rotor_angle & 0x8000:
        rotor_angle -= 0x10000

    rotor_speed = (data[2] << 8) + data[3]
    if rotor_speed & 0x8000:
        rotor_speed -= 0x10000

    torque_current = (data[4] << 8) + data[5]
    if torque_current & 0x8000:
        torque_current -= 0x10000

    motor_temperature = data[6]

    return rotor_angle, rotor_speed, torque_current, motor_temperature


def parse_c620_current(value):
    value = int(value)
    if not -32768 <= value <= 32767:
        raise ValueError("value is out of the range of a 16-bit signed integer")

    packed_value = struct.pack('>h', value)
    return packed_value


def can_transmit(bus, message_id, data_1, data_2, data_3, data_4, timeout=None):
    data = (parse_c620_current(data_1) +
            parse_c620_current(data_2) +
            parse_c620_current(data_3) +
            parse_c620_current(data_4))
    can_send_message(bus, message_id, data, timeout)


def chassis_task(bus):
    while True:
        can_transmit(bus, 0x200, *[chassis_motors[i].need_torque_current for i in range(CHASSIS_MOTOR_NUM)])


def print_chassis_motors():
    data = []
    for i in range(CHASSIS_MOTOR_NUM):
        data.append(str(chassis_motors[i]))
    print("\n".join(data))
    sys.stdout.flush()


if __name__ == "__main__":
    subprocess.run("sudo ip link set can0 type can bitrate 1000000".split())
    subprocess.run("sudo ifconfig can0 down".split())
    subprocess.run("sudo ifconfig can0 up".split())
    subprocess.run("sudo ifconfig can0 txqueuelen 1000".split())

    os.setpriority(os.PRIO_PROCESS, 0, -20)
    # current_process = psutil.Process()
    # current_process.cpu_affinity([1, 2])

    with can.interface.Bus(interface='socketcan', channel="can0", bitrate=1000000) as can0:
        m3508_rx_handler = MotorM3508RxHandler()
        can.Notifier(bus=can0, listeners=[m3508_rx_handler])

        thread_chassis_task = threading.Thread(target=chassis_task, args=(can0,), daemon=True)
        thread_chassis_task.start()

        thread_chassis_task.join()
