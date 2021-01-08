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
	send_radio     = 4
};

enum msg_status {
	msg_undef  = 0,
	msg_ok     = 1,
	msg_error  = 2,
	msg_full   = 3
};

typedef struct {
	uint8_t reserved;
	enum msg_type type;
	uint32_t stored_at;
	uint16_t bytes;
	uint8_t  value[MAX_MSG_LEN];
} one_msg_struct;

void msg_initialize(void);
uint8_t msg_free_rows(void);
enum msg_status msg_reserve(uint8_t *msg_indx);
one_msg_struct *msg_get_handle(uint8_t msg_indx);


#endif /* INC_MSG_H_ */


