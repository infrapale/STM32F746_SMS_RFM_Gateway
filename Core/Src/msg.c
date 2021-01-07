/*
 * msg.c
 *
 *  Created on: Jan 7, 2021
 *      Author: tom_h
 */

#include <main.h>
#include <msg.h>
#include <console.h>
#include <stdio.h>
#include <string.h>

struct one_msg_struct {
	uint8_t free;
	enum msg_type type;
	uint32_t stored_at;
	uint16_t bytes;
	uint8_t  value[MAX_MSG_LEN];
} msg_repo[MAX_MSG_ROW];

void msg_initialize(void){
	memset(msg_repo, 0x00, sizeof(msg_repo));
}
