/*
 * protocol.h - defines pertaining to the secondary MCU wire protocol
 *
 *  Created on: Nov 25, 2012
 *      Author: csdexter
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>


#define PROTOCOL_WRITE 0x80
#define PROTOCOL_DATA 0x40
#define PROTOCOL_READ 0x00
#define PROTOCOL_COMMAND 0x00

#define PROTOCOL_RCOMM (PROTOCOL_READ | PROTOCOL_COMMAND)
#define PROTOCOL_RDATA (PROTOCOL_READ | PROTOCOL_DATA)
#define PROTOCOL_WCOMM (PROTOCOL_WRITE | PROTOCOL_COMMAND)
#define PROTOCOL_WDATA (PROTOCOL_WRITE | PROTOCOL_DATA)

#define PROTOCOL_COMMAND_NOP 0x00
#define PROTOCOL_COMMAND_MAC 0x3C
#define PROTOCOL_COMMAND_WAY 0x3D
#define PROTOCOL_COMMAND_AYT 0x3E
#define PROTOCOL_COMMAND_HLT 0x3F

#define PROTOCOL_C_AYT_YIA 7
#define PROTOCOL_C_AYT_NYIA 6
#define PROTOCOL_C_AYT_ENVOK 5
#define PROTOCOL_C_AYT_COMOK 4

#define PROTOCOL_C_WAY_FI 0x00
#define PROTOCOL_C_WAY_FS 0x01
#define PROTOCOL_C_WAY_FD 0x02
#define PROTOCOL_C_WAY_FC 0x03
#define PROTOCOL_C_WAY_IS 0x00
#define PROTOCOL_C_WAY_IN 0x08
#define PROTOCOL_C_WAY_E2013 0x00
#define PROTOCOL_C_WAY_MA1 0x00
#define PROTOCOL_C_WAY_MA2 0x04
#define PROTOCOL_C_WAY_MA3 0x08
#define PROTOCOL_C_WAY_MA4 0x0C
#define PROTOCOL_C_WAY_MA5 0x10
#define PROTOCOL_C_WAY_MA6 0x14
#define PROTOCOL_C_WAY_MA7 0x18
#define PROTOCOL_C_WAY_MA8 0x1C
#define PROTOCOL_C_WAY_MA9 0x20
#define PROTOCOL_C_WAY_MA10 0x24
#define PROTOCOL_C_WAY_MA11 0x28
#define PROTOCOL_C_WAY_MA12 0x2C
#define PROTOCOL_C_WAY_MA13 0x30
#define PROTOCOL_C_WAY_MA14 0x34
#define PROTOCOL_C_WAY_MA15 0x38
#define PROTOCOL_C_WAY_MA16 0x3C
#define PROTOCOL_C_WAY_MI1 0x00
#define PROTOCOL_C_WAY_MI2 0x01
#define PROTOCOL_C_WAY_MI3 0x02
#define PROTOCOL_C_WAY_MI4 0x03
#define PROTOCOL_C_WAY_FWRa 0x00
#define PROTOCOL_C_WAY_FWRb 0x10
#define PROTOCOL_C_WAY_FWRc 0x20
#define PROTOCOL_C_WAY_FWRd 0x30
#define PROTOCOL_C_WAY_HWRa 0x00
#define PROTOCOL_C_WAY_HWRb 0x04
#define PROTOCOL_C_WAY_HWRc 0x08
#define PROTOCOL_C_WAY_HWRd 0x0C
#define PROTOCOL_C_WAY_PCRa 0x00
#define PROTOCOL_C_WAY_PCRb 0x01
#define PROTOCOL_C_WAY_PCRc 0x02
#define PROTOCOL_C_WAY_PCRd 0x03

#define PROTOCOL_TRANSACTION_SIZE 3

#endif /* PROTOCOL_H_ */
