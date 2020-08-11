/* 
 * yhs_protocol.h
 * 
 * Created on: Aug 07, 2019 21:49
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef YHS_PROTOCOL_H
#define YHS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define YHS_CMD_BUF_LEN               32
#define YHS_STATUS_BUF_LEN            32
#define YHS_FRAME_SIZE                13



#define CAN_STEER_UNIT  0.043945
#define CAN_WHEEL_UNIT  0.04
#define CAN_BRAKE_UNIT  0.00390625

#define CAN_MSG_GEAR_CONTROL_CMD_ID        ((uint32_t)0x18C4D1D0)
#define CAN_MSG_STEER_CONTROL_CMD_ID       ((uint32_t)0x18C4D2D0)
#define CAN_MSG_WHEEL_CONTROL_CMD_ID       ((uint32_t)0x18C4D3D0)
#define CAN_MSG_BRAKE_CONTROL_CMD_ID       ((uint32_t)0x18C4D4D0)
#define CAN_MSG_PARKING_CONTROL_CMD_ID     ((uint32_t)0x18C4D5D0)
#define CAN_MSG_ODOM_CONTROL_CMD_ID        ((uint32_t)0x18C4D6D0)
#define CAN_MSG_LIGHT_CONTROL_CMD_ID       ((uint32_t)0x18C4D7D0)
#define CAN_MSG_SPEAKER_CONTROL_CMD_ID     ((uint32_t)0x18C4D8D0)


#define CAN_MSG_GEAR_FEEDBACK_ID            ((uint32_t)0x18C4D1EF)
#define CAN_MSG_STEER_FEEDBACK_ID           ((uint32_t)0x18C4D2EF)
#define CAN_MSG_WHEEL_FEEDBACK_ID           ((uint32_t)0x18C4D3EF)
#define CAN_MSG_BRAKE_FEEDBACK_ID           ((uint32_t)0x18C4D4EF)
#define CAN_MSG_PARKING_FEEDBACK_ID         ((uint32_t)0x18C4D5EF)
#define CAN_MSG_ODOM_FEEDBACK_ID            ((uint32_t)0x18C4D9EF)
#define CAN_MSG_LIGHT_FEEDBACK_ID           ((uint32_t)0x18C4DAEF)
#define CAN_MSG_SPEAKER_FEEDBACK_ID         ((uint32_t)0x18C4DBEF)



/*-------------------- Control/Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Note: id could be different for UART and CAN protocol


#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* YHS_PROTOCOL_H */
