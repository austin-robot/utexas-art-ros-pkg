/*
 *  Description:  ART Throttle Controller command/protocol
 *
 *  Copyright (C) 2007 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _ART_COMMAND_PROTOCOL_H_
#define _ART_COMMAND_PROTOCOL_H_

/**  @file
   
     @brief ART throttle controller firmware command protocol.
 */

#undef CLI_ECHO

#define COM_OFFSET  0
#define LEN_OFFSET  0
#define SEQ_OFFSET  1
#define DATA_OFFSET 2

#define FLAG_ENABLE  1
#define FLAG_DISABLE 0

enum {
  ACK_CMD=0,
  NAK_CMD,
  STATUS_CMD,
  GOTO_CMD,
  PID_KP_CMD,
  PID_KI_CMD,
  PID_KD_CMD,
  PID_LIMITS_CMD,
  DEBUG_STATUS_CMD,
  CLI_SET_CMD,
  SET_IDLE_CMD,
  GET_REV_CMD,
  INVALID_CMD,
  MAX_CMD=INVALID_CMD
};

enum {
  NAK_BAD_CMD=0,
  NAK_BAD_LEN,
  NAK_BAD_CSUM,
  NAK_BAD_PARAM
};

typedef struct status_data_st_ {
  u08 position;
  u16 rpms;
  union {
    struct {
      u08  general:1;
      u08  throttle:1;
      u08  rpms:1;
      u08  estop:1;
      u08  diag:4;
    }flags;
    u08 flag_byte;
  };
  u08 direction;
  s16 dstate;
  s16 istate;
  u16 pwm_count;
} status_data_t;

typedef struct firmware_rev_st_ {
  u08 fw_rev[2];
  u08 fw_repos_status[1];
  u08 uc_lib_rev[2];
  u08 uc_lib_repos_status[1];
  u08 build_time[6];
} firmware_rev_t;

#endif /* _ART_COMMAND_PROTOCOL_H_ */
