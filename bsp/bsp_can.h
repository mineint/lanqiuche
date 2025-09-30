#ifndef __BSP_CAN_H
#define __BSP_CAN_H


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2


extern void can_filter_init(void);
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    int8_t temper;
    int16_t iq;
    int16_t speed;
    uint16_t encoder;
    uint16_t last_encoder_value;
    int cumulative_angle;
}Motor;


void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_jieqiu(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void get_absolute_angle(Motor *motor);
void chassis_motor(float vx, float vy, int16_t vw, int16_t speed[4]);


#endif
