#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CHASSIS_MOTOR_NUM 4
#define LF 0
#define RF 1
#define RB 2
#define LB 3

#define PI 3.14159265358979323846
#define WHEEL_RADIUS 0.076
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.209
#define M3508_REDUCTION_FACTOR 19
#define M3508_MAX_OUTPUT_CURRENT 16000

double WHEEL_RPM_TO_WHEEL_MPS = (2 * PI * WHEEL_RADIUS) / 60.0 / M3508_REDUCTION_FACTOR;
double WHEEL_MPS_TO_WHEEL_RPM = (1 / WHEEL_RPM_TO_WHEEL_MPS);
double ROBOT_RPS_TO_WHEEL_MPS = (2 * PI * ROBOT_CENTER_TO_WHEEL_RADIUS);
double WHEEL_MPS_TO_ROBOT_RPS = (1 / ROBOT_RPS_TO_WHEEL_MPS);

typedef struct {
    double kp, ki, kd;
    double max_i_out, max_out;
    double error[3];
    double d_buf[3];
    double p_out, i_out, d_out;
    double out;
} PID;

typedef struct {
    int motor_id;
    double rotor_angle;
    double rotor_speed;
    double torque_current;
    double motor_temperature;
    double motor_target_speed;
    int frame_counter;
    PID pid;
} MotorM3508;

MotorM3508 chassis_motors[CHASSIS_MOTOR_NUM];

void pid_init(PID *pid, double kp, double ki, double kd, double max_i_out, double max_out) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_i_out = max_i_out;
    pid->max_out = max_out;
    memset(pid->error, 0, sizeof(pid->error));
    memset(pid->d_buf, 0, sizeof(pid->d_buf));
    pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0;
}

double pid_update(PID *pid, double target_value, double measured_value) {
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target_value - measured_value;

    pid->p_out = pid->kp * pid->error[0];
    pid->i_out += pid->ki * pid->error[0];
    pid->i_out = fmax(fmin(pid->i_out, pid->max_i_out), -pid->max_i_out);

    pid->d_buf[2] = pid->d_buf[1];
    pid->d_buf[1] = pid->d_buf[0];
    pid->d_buf[0] = pid->error[0] - pid->error[1];
    pid->d_out = pid->kd * pid->d_buf[0];

    pid->out = pid->p_out + pid->i_out + pid->d_out;
    pid->out = fmax(fmin(pid->out, pid->max_out), -pid->max_out);

    return pid->out;
}

void motor_init(MotorM3508 *motor, int motor_id, double kp, double ki, double kd, double max_i_out, double max_out) {
    motor->motor_id = motor_id;
    motor->rotor_angle = motor->rotor_speed = motor->torque_current = motor->motor_temperature = 0;
    motor->frame_counter = 0;
    pid_init(&motor->pid, kp, ki, kd, max_i_out, max_out);
}

double get_mechanical_speed(MotorM3508 *motor) {
    return motor->rotor_speed * WHEEL_RPM_TO_WHEEL_MPS;
}

void update_motor_data(MotorM3508 *motor, double angle, double speed, double torque, double temperature) {
    motor->rotor_angle = angle;
    motor->rotor_speed = speed;
    motor->torque_current = torque;
    motor->motor_temperature = temperature;
    motor->frame_counter++;
}

void setup_socketcan(const char *interface) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, interface);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    // Set up the CAN interface here (bitrate, up/down, etc.)
    // Example: system("sudo ip link set can0 type can bitrate 1000000");
    // Example: system("sudo ifconfig can0 up");
}

void can_transmit(int socket, int message_id, double *currents, int count) {
    struct can_frame frame;
    frame.can_id = message_id;
    frame.can_dlc = count * 2;

    for (int i = 0; i < count; i++) {
        int16_t value = (int16_t)(currents[i]);
        frame.data[2*i] = (value >> 8) & 0xFF;
        frame.data[2*i + 1] = value & 0xFF;
    }

    write(socket, &frame, sizeof(struct can_frame));
}

void chassis_task(int socket) {
    double currents[CHASSIS_MOTOR_NUM];
    while (1) {
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            currents[i] = pid_update(&chassis_motors[i].pid, chassis_motors[i].motor_target_speed, get_mechanical_speed(&chassis_motors[i]));
        }
        can_transmit(socket, 0x200, currents, CHASSIS_MOTOR_NUM);
        usleep(100000); // 100 ms
    }
}

int main() {
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        motor_init(&chassis_motors[i], i, 1000000.0, 0.0, 0.0, 16000.0, 16000.0);
    }

    setup_socketcan("can0");

    // Continuously transmit motor currents
    chassis_task(0); // Placeholder socket value

    return 0;
}
