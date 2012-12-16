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

#endif /* PROTOCOL_H_ */
