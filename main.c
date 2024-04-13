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

double WHEEL_RPM_TO_WHEEL_MPS;
double WHEEL_MPS_TO_WHEEL_RPM;
double ROBOT_RPS_TO_WHEEL_MPS;
double WHEEL_MPS_TO_ROBOT_RPS;

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

// Initialization of PID and Motor structs omitted for brevity

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
    // Initialize the calculations for global variables
    WHEEL_RPM_TO_WHEEL_MPS = (2 * PI * WHEEL_RADIUS) / 60.0 / M3508_REDUCTION_FACTOR;
    WHEEL_MPS_TO_WHEEL_RPM = (1 / WHEEL_RPM_TO_WHEEL_MPS);
    ROBOT_RPS_TO_WHEEL_MPS = (2 * PI * ROBOT_CENTER_TO_WHEEL_RADIUS);
    WHEEL_MPS_TO_ROBOT_RPS = (1 / ROBOT_RPS_TO_WHEEL_MPS);

    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        motor_init(&chassis_motors[i], i, 1000000.0, 0.0, 0.0, 16000.0, 16000.0);
    }

    setup_socketcan("can0");

    // Continuously transmit motor currents
    chassis_task(0); // Placeholder socket value

    return 0;
}
