/*
 * msg.h
 *
 *  Created on: Jan 7, 2021
 *      Author: tom_h
 */

#ifndef INC_MSG_H_
#define INC_MSG_H_

#define MAX_MSG_LEN 128
#define MAX_MSG_ROW 16

enum msg_type {
	received_sms   = 1,
	send_sms       = 2,
	received_radio = 3,
	send_radio     = 4,
};


#endif /* INC_MSG_H_ */
