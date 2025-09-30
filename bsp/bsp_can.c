#include "main.h"
#include "gpio.h"
#include "bsp_can.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


Motor Motor_1,Motor_2,Motor_3,Motor_4,Motor_5,Motor_6,Motor_7,Motor_8;


static CAN_TxHeaderTypeDef  chassis_tx_message;
static CAN_TxHeaderTypeDef  TxHeader;
static uint8_t    chassis_can_send_data[8];

//
extern CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;   // 
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  // 
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; // 
    can_filter_st.FilterIdHigh = 0x0000;                //  
    can_filter_st.FilterIdLow = 0x0000;                // 
    can_filter_st.FilterMaskIdHigh = 0x0000;          // 
    can_filter_st.FilterMaskIdLow = 0x0000;           // 
    can_filter_st.FilterBank = 0;                     //  
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; // 
	
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);   //
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;   
    can_filter_st.FilterBank = 14;            
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
            case 0x201:
                Motor_1.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_1.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_1.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_1.temper = rx_data[6];
                break;
            case 0x202:
                Motor_2.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_2.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_2.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_2.temper = rx_data[6];
                break;
            case 0x203:
                Motor_3.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_3.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_3.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_3.temper = rx_data[6];
                break;
            case 0x204:
                Motor_4.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_4.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_4.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_4.temper = rx_data[6];
			case 0x205:
                Motor_5.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_5.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_5.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_5.temper = rx_data[6];
                break;
			case 0x207:
                Motor_7.encoder = ((uint16_t)rx_data[0] << 8) + rx_data[1];
                Motor_7.speed = ((uint16_t)rx_data[2] << 8) + rx_data[3];
                Motor_7.iq = ((uint16_t)rx_data[4] << 8) + rx_data[5];
                Motor_7.temper = rx_data[6];
                break;
		}
       
}

void motor5065(uint32_t ExtID, int32_t vec)
{
    uint8_t txData[4];
    uint32_t TxMailbox;
    TxHeader.ExtId = ExtID;
    TxHeader.IDE = CAN_ID_EXT; // 使用扩展帧 (29-bit ID)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4;
    TxHeader.TransmitGlobalTime = DISABLE;
    txData[0] = (vec >> 24) & 0xFF; // 高 8 位
    txData[1] = (vec >> 16) & 0xFF; // 次高 8 位
    txData[2] = (vec >> 8) & 0xFF;  // 次低 8 位
    txData[3] = vec & 0xFF;         // 低 8 位
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &TxHeader, txData, &TxMailbox);
}


void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_jieqiu(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor5 >> 8;
    chassis_can_send_data[1] = motor5;
    chassis_can_send_data[2] = motor6 >> 8;
    chassis_can_send_data[3] = motor6;
    chassis_can_send_data[4] = motor7 >> 8;
    chassis_can_send_data[5] = motor7;
    chassis_can_send_data[6] = motor8 >> 8;
    chassis_can_send_data[7] = motor8;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void get_absolute_angle(Motor *motor)
{
    uint16_t current_encoder_value = motor->encoder;
    int16_t delta = current_encoder_value - motor->last_encoder_value;
    if (delta > 4096) {
        delta -= 8192;
    } else if (delta < -4096) {
        delta += 8192;
    }
    motor->cumulative_angle += delta;
    motor->last_encoder_value = current_encoder_value;
}






void chassis_motor(float vx, float vy, int16_t vw, int16_t speed[4])  
{
	speed[0] =  vx*2 + vy*2 + vw;
	speed[1] = -vx*2 - vy*2 + vw;
	speed[2] = -vx*2 + vy*2 + vw;
	speed[3] =  vx*2 - vy*2 + vw;
}









