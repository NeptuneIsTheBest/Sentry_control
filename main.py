import subprocess

import can
from can import Message


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

    def limit(self, value, max_value):
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

    def clear(self):
        pass


class MotorM3508:
    def __init__(self, motor_id):
        self.motor_id = motor_id

        self.rotor_angle = 0
        self.rotor_speed = 0
        self.torque_current = 0
        self.motor_temperature = 0

        self.frame_counter = 0

    def update_motor_data(self, rotor_angle, rotor_speed, torque_current, motor_temperature):
        self.rotor_angle = rotor_angle
        self.rotor_speed = rotor_speed
        self.torque_current = torque_current
        self.motor_temperature = motor_temperature

        self.frame_counter += 1


class MotorM3508RxHandler(can.Listener):
    def __init__(self):
        super(MotorM3508RxHandler, self).__init__()

    def on_message_received(self, msg: Message) -> None:
        pass


def parse_c620_data(data):
    rotor_angle = (data[0] << 8) + data[1]
    rotor_speed = (data[2] << 8) + data[3]
    torque_current = (data[4] << 8) + data[5]
    motor_temperature = data[6]

    return rotor_angle, rotor_speed, torque_current, motor_temperature


def can_send_message(bus, message_id, data):
    msg = can.Message(arbitration_id=int(message_id), data=data)


def can_receive_messages(bus, timeout=None):
    msg = bus.recv(timeout)
    return msg


if __name__ == "__main__":
    subprocess.run("sudo ip link set can0 type can bitrate 1000000".split())
    subprocess.run("sudo ifconfig can0 up".split())
    with can.interface.Bus(interface='socketcan', channel="can0", bitrate=1000000) as can0:
        while True:
            msg = can_receive_messages(can0)
            print(hex(msg.arbitration_id), parse_c620_data(msg.data))
