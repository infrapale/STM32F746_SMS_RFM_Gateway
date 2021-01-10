/*
 * msg.c
 *
 *  Created on: Jan 7, 2021
 *      Author: tom_h
 */

#include <main.h>
#include <msg.h>
#include <stdio.h>
#include <string.h>


one_msg_struct msg_repo[MAX_MSG_ROW];

void msg_initialize(void){
	memset(msg_repo, 0x00, sizeof(msg_repo));
}

uint8_t msg_free_rows(void){
	uint8_t cnt = 0;
	for(uint8_t i=0; i < MAX_MSG_ROW; i++ ){
		if (msg_repo[i].reserved == 0){
			cnt++;
		}
	}
	return cnt;
}

enum msg_status msg_reserve(uint8_t *msg_indx){
    uint8_t indx = 0;
    enum msg_status status = msg_undef;

    while ((indx < MAX_MSG_ROW) && (status == msg_undef)){
    	if (msg_repo[indx].reserved == 0){
    		status = msg_ok;
    		*msg_indx = indx;
    		msg_repo[indx].reserved = 1;
    	} else {
    		if (++indx >= MAX_MSG_ROW){
    			status = msg_full;
    		}
    	}
    }
    return status;
}

one_msg_struct *msg_get_handle(uint8_t msg_indx){
	return &msg_repo[msg_indx];
}


