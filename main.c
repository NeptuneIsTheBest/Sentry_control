#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

#define CHASSIS_MOTOR_NUM 4
#define LF 0
#define RF 1
#define RB 2
#define LB 3

#define PI 3.14159265358979
#define WHEEL_RADIUS 0.076
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.209
#define M3508_REDUCTION_FACTOR 19
#define WHEEL_RPM_TO_WHEEL_MPS ((2 * PI * WHEEL_RADIUS) / 60.0 / M3508_REDUCTION_FACTOR)
#define WHEEL_MPS_TO_WHEEL_RPM (1 / (WHEEL_RPM_TO_WHEEL_MPS * 1.0))
#define ROBOT_RPS_TO_WHEEL_MPS (2 * PI * ROBOT_CENTER_TO_WHEEL_RADIUS)
#define WHEEL_MPS_TO_ROBOT_RPS (1 / (ROBOT_RPS_TO_WHEEL_MPS * 1.0))

#define M3508_MAX_OUTPUT_CURRENT 16000

typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t max_i_out;
    int16_t max_out;
    char mode[10];
    int16_t error[3];
    int16_t d_buf[3];
    int16_t p_out;
    int16_t i_out;
    int16_t d_out;
    int16_t out;
} PID;

typedef struct {
    int32_t motor_id;
    int16_t rotor_angle;
    int16_t rotor_speed;
    int16_t torque_current;
    int16_t need_torque_current;
    int8_t motor_temperature;
    int16_t motor_target_speed;
    int32_t frame_counter;
    PID pid;
} MotorM3508;

void PID_init(PID *pid, int16_t kp, int16_t ki, int16_t kd, int16_t max_i_out, int16_t max_out, char *mode) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_i_out = max_i_out;
    pid->max_out = max_out;
    strncpy(pid->mode, mode, 9);
    memset(pid->error, 0, sizeof(pid->error));
    memset(pid->d_buf, 0, sizeof(pid->d_buf));
    pid->p_out = pid->i_out = pid->d_out = pid->out = 0;
}

int16_t PID_limit(int16_t value, int16_t max_value) {
    if (max_value != 0) {
        if (value > max_value)
            return max_value;
        else if (value < -max_value)
            return -max_value;
    }
    return value;
}

int16_t PID_update(PID *pid, int16_t target_value, int16_t measured_value) {
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target_value - measured_value;

    if (strcmp(pid->mode, "position") == 0) {
        pid->p_out = pid->kp * pid->error[0];
        pid->i_out = pid->ki * pid->error[0];
        pid->i_out = PID_limit(pid->i_out, pid->max_i_out);

        pid->d_buf[2] = pid->d_buf[1];
        pid->d_buf[1] = pid->d_buf[0];
        pid->d_buf[0] = pid->error[0] - pid->error[1];
        pid->d_out = pid->kd * pid->d_buf[0];

        pid->out = pid->p_out + pid->i_out + pid->d_out;
        pid->out = PID_limit(pid->out, pid->max_out);
    } else if (strcmp(pid->mode, "delta") == 0) {
        pid->p_out = pid->kp * (pid->error[0] - pid->error[1]);
        pid->i_out = pid->ki * pid->error[0];

        pid->d_buf[2] = pid->d_buf[1];
        pid->d_buf[1] = pid->d_buf[0];
        pid->d_buf[0] = pid->error[0] - 2.0 * pid->error[1] + pid->error[2];
        pid->d_out = pid->kd * pid->d_buf[0];

        pid->out = pid->p_out + pid->i_out + pid->d_out;
        pid->out = PID_limit(pid->out, pid->max_out);
    }

    return pid->out;
}

void MotorM3508_init(MotorM3508 *motor, int32_t motor_id, int16_t kp, int16_t ki, int16_t kd, int16_t max_i_out, int16_t max_out, char *mode) {
    motor->motor_id = motor_id;
    motor->rotor_angle = 0;
    motor->rotor_speed = 0;
    motor->torque_current = 0;
    motor->need_torque_current = 0;
    motor->motor_temperature = 0;
    motor->motor_target_speed = 0;
    motor->frame_counter = 0;
    PID_init(&motor->pid, kp, ki, kd, max_i_out, max_out, mode);
}

void MotorM3508_update_motor_data(MotorM3508 *motor, int16_t rotor_angle, int16_t rotor_speed, int16_t torque_current, int8_t motor_temperature) {
    motor->rotor_angle = rotor_angle;
    motor->rotor_speed = rotor_speed;
    motor->torque_current = torque_current;
    motor->motor_temperature = motor_temperature;
    motor->frame_counter++;
}

int16_t MotorM3508_get_motor_mechanical_speed(MotorM3508 *motor) {
    return motor->rotor_speed * WHEEL_RPM_TO_WHEEL_MPS;
}

void MotorM3508_pid_update(MotorM3508 *motor) {
    motor->need_torque_current = PID_update(&motor->pid, motor->motor_target_speed, MotorM3508_get_motor_mechanical_speed(motor));
}

int16_t parse_c620_current(int16_t value) {
    if (value < -32768 || value > 32767)
        return 0;
    return value;
}

void can_transmit(int can_fd, int message_id, int16_t data_1, int16_t data_2, int16_t data_3, int16_t data_4) {
    struct can_frame frame;
    frame.can_id = message_id;
    frame.can_dlc = 8;
    frame.data[0] = parse_c620_current(data_1) >> 8;
    frame.data[1] = parse_c620_current(data_1) & 0xFF;
    frame.data[2] = parse_c620_current(data_2) >> 8;
    frame.data[3] = parse_c620_current(data_2) & 0xFF;
    frame.data[4] = parse_c620_current(data_3) >> 8;
    frame.data[5] = parse_c620_current(data_3) & 0xFF;
    frame.data[6] = parse_c620_current(data_4) >> 8;
    frame.data[7] = parse_c620_current(data_4) & 0xFF;
    write(can_fd, &frame, sizeof(struct can_frame));
}

void chassis_task(int can_fd) {
    while (1) {
        can_transmit(can_fd, 0x200, chassis_motors[LF].need_torque_current, chassis_motors[RF].need_torque_current, chassis_motors[RB].need_torque_current, chassis_motors[LB].need_torque_current);
    }
}

void chassis_update_task(MotorM3508 *motors) {
    while (1) {
        MotorM3508_pid_update(&motors[LF]);
        MotorM3508_pid_update(&motors[RF]);
        MotorM3508_pid_update(&motors[RB]);
        MotorM3508_pid_update(&motors[LB]);
    }
}

void print_chassis_motors(MotorM3508 *motors) {
    printf("Motor ID: %d Angle: %d Speed: %d Target Speed: %d Current: %d Target Current: %d\n",
           motors[LF].motor_id, motors[LF].rotor_angle, MotorM3508_get_motor_mechanical_speed(&motors[LF]), motors[LF].motor_target_speed, motors[LF].torque_current, motors[LF].need_torque_current);
    printf("Motor ID: %d Angle: %d Speed: %d Target Speed: %d Current: %d Target Current: %d\n",
           motors[RF].motor_id, motors[RF].rotor_angle, MotorM3508_get_motor_mechanical_speed(&motors[RF]), motors[RF].motor_target_speed, motors[RF].torque_current, motors[RF].need_torque_current);
    printf("Motor ID: %d Angle: %d Speed: %d Target Speed: %d Current: %d Target Current: %d\n",
           motors[RB].motor_id, motors[RB].rotor_angle, MotorM3508_get_motor_mechanical_speed(&motors[RB]), motors[RB].motor_target_speed, motors[RB].torque_current, motors[RB].need_torque_current);
    printf("Motor ID: %d Angle: %d Speed: %d Target Speed: %d Current: %d Target Current: %d\n",
           motors[LB].motor_id, motors[LB].rotor_angle, MotorM3508_get_motor_mechanical_speed(&motors[LB]), motors[LB].motor_target_speed, motors[LB].torque_current, motors[LB].need_torque_current);
    fflush(stdout);
}

int main() {
    MotorM3508 chassis_motors[CHASSIS_MOTOR_NUM];
    int16_t Kp = 500000, Ki = 0, Kd = 0;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        MotorM3508_init(&chassis_motors[i], i, Kp, Ki, Kd, 0, M3508_MAX_OUTPUT_CURRENT, "delta");
    }

    int can_fd = open("/dev/can0", O_RDWR);
    if (can_fd < 0) {
        printf("Failed to open CAN interface\n");
        return 1;
    }

    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ifconfig can0 down");
    system("sudo ifconfig can0 up");
    system("sudo ifconfig can0 txqueuelen 1000");

    setpriority(PRIO_PROCESS, 0, -20);
    pid_t pid = getpid();
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);
    CPU_SET(2, &cpu_set);
    sched_setaffinity(pid, sizeof(cpu_set_t), &cpu_set);

    pthread_t chassis_task_thread, chassis_update_thread;
    pthread_create(&chassis_task_thread, NULL, (void *(*)(void *))chassis_task, (void *)can_fd);
    pthread_create(&chassis_update_thread, NULL, (void *(*)(void *))chassis_update_task, (void *)chassis_motors);

    while (1) {
        print_chassis_motors(chassis_motors);
        usleep(100000);
    }

    close(can_fd);
    return 0;
}