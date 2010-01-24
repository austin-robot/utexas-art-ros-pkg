/*
 *  Copyright (C) 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#ifndef _AVR_CONTROLLER_H
#define _AVR_CONTROLLER_H

/**
  AVR Throttle Controller Command/Protocol Design
  ===============================================
  
  The Pilot as the master will send COMMAND packets to the slave
  throttle controller. The throttle controller will respond to each
  command packet with an ACK/NAK or STATUS (if requested) packet. All
  values are trasmitted as ASCII characters representing byte values in
  hexadecimal. Packets are terminated with a carriage return. The packet
  formats are as follows:
  
  ----------------------------------------------------------------------
  
  COMMAND packet
  
  ----------------------------------------------------------------------
  
  COM - (4 bit) Command code value. See COMMAND COM/DATA section below.
  LEN - (4 bit) Length of frame, including COM, LEN, SEQ, DATA, and
  CSUM.
  
  Note: This value reflects the "byte" packet length--not the ASCII
  string length. For example, a GOTO command will be 4 bytes in "byte"
  packet length , even though the ASCII string (excluding CR) is 8
  characters.
  
  SEQ - (1 byte) Sequence number. See Protocol Operation section below.
  DATA - (0-4 bytes) Command/response data in big-endian format.
  CSUM - (1 bytes) Checksum. See CSUM description below.
  
  ----------------------------------------------------------------------
  
  ACK/NAK packet
  
  ----------------------------------------------------------------------
  
  The ACK/NAK packet is the same format as the COMMAND packet with a COM
  code corresponding to an ACK or NAK and no DATA bytes. Note that the
  throttle controller will not generate it's own SEQ numbers, but will
  instead echo back the SEQ value of the COMMAND packet. See Protocol
  Operation below.
  
  ----------------------------------------------------------------------
  
  STATUS packet
  
  ----------------------------------------------------------------------
  
  The STATUS response packet is the same format as the COMMAND packet
  with a COM code of STATUS, and the corresponding DATA bytes. Note that
  the SEQ value provided by the throttle controller will be an echo of
  the value passed in the COMMAND request from the Pilot. The STATUS
  request packet from the Pilot will have a zero length DATA field.
  
  STATUS Response Fields: see avr_controller.h
  
  Protocol Operation: As noted above, the Pilot will send packets
  with unique SEQ values (presumably increasing until rollover) to
  the throttle controller. All responses from the throttle
  controller will echo back the SEQ value from the corresponding
  Pilot COMMAND packet. The throttle will keep track of the the last
  COMMAND/response exchange. If the Pilot times out waiting for a
  response from the throttle, the retry packets should re-use the
  original SEQ value for the COMMAND packet. This way, if the
  timeout was due to a lost ACK/NAK, the controller will see a
  duplicate request multiple times. If the re-tried command is a
  request for STATUS that was previously sent by the throttle, the
  latest status information is always returned for the retry
  (i.e. no cached status).
  
  ----------------------------------------------------------------------
  
  COMMAND COM/DATA
  
  ----------------------------------------------------------------------
  
  See <art/art_command_protocol.h>.  The main commands are:

  ACK (0)	Acknowledges COMMAND from Pilot.
  NAK (1)	Negative acknowledge of COMMAND from Pilot.
  STATUS (2)	STATUS command from Pilot or its response from the
		controller.  DATA bytes as in "avr_controller.h".  A
		STATUS request command from the PILOT will have a
		zero-length DATA field.
  GOTO (3)	Sets throttle position.  DATA byte: target postion.
  
  ----------------------------------------------------------------------
  
  CSUM
  
  ----------------------------------------------------------------------
  
  The "Simple Wikipedia" checksum algorithm is computed as follows:
  See
  
  Step 1: Adding all bytes together.
  Step 2: Drop the Carry Nibble.
  Step 3: Get the two's complement. This is the checksum byte.
  To Test the Checksum byte simply add it to the original group of
  bytes.
  Drop the carry nibble again giving 00h. Since it is 00h this means the
  checksum means the bytes were probably not changed.
  
  Note: The CSUM encompasses the COM, LEN, SEQ, and DATA fields.

  @author Jack O'Quin
*/

#include <netinet/in.h>			/* byte-reversal functions */

/* typedefs for fixed-size fields from standard C types */
#include <inttypes.h>
typedef int8_t   s08;
typedef int16_t  s16;
typedef uint8_t  u08;
typedef uint16_t u16;
typedef uint32_t u32;

#include "art_command_protocol.h"

#define AVR_POS_ABSURD 7		/* absurdly small position value */

/* warning! this is GCC-specific. */
#pragma pack(push,1)		/* ensure compiler packs this struct */

  struct avr_cmd {
    uint8_t	comlen;		/* command and length (4 bits each) */
    uint8_t	seq;		/* sequence number */
    union {
      uint8_t	data08;		/* 8-bit data field */
      uint16_t	data16;		/* 16-bit data field */
      uint32_t	data32;		/* 32-bit data field */
      struct {			/* status response data */
	uint8_t  pos;		/* throttle position */
	uint16_t rpms;		/* engine RPMs */
	uint8_t	 diagn;		/* diagnostic flag byte */
	uint8_t  direction;	/* motor direction: 1=fwd, 0=rev */
	int16_t  dstate;	/* derivative state */
	int16_t  istate;	/* integral state */
	uint16_t pwm_count;	/* motor output */
      } status;
    } data;
    /* checksum follows last data byte at offset (len-1) */
    uint8_t	csum;		/* reserve space for it */
  };

#pragma pack(pop)		/* restore default struct packing */

/* returns the offset of "field" in the AVR command structure */
#define avr_offset(field) \
  ((size_t) (((char *)(&(((struct avr_cmd*)NULL)->field))) - ((char *)NULL)))

/*
 * Accessor functions for packed bit fields.  Trusting the compiler to
 * pack them correctly does not work and is non-portable, anyway.
 */
static inline void avr_set_com_len(struct avr_cmd *cmd, int com, int len)
{cmd->comlen = (com << 4) | len;}

static inline uint8_t avr_get_com(struct avr_cmd *cmd)
{return (cmd->comlen >> 4) & 0x0F;}

static inline uint8_t avr_get_len(struct avr_cmd *cmd)
{return cmd->comlen & 0x0F;}

static inline uint8_t avr_get_pos(struct avr_cmd *status)
{return status->data.status.pos;}

static inline uint16_t avr_get_rpms(struct avr_cmd *status)
{return status->data.status.rpms;}

static inline bool avr_get_gen(struct avr_cmd *status)
{return status->data.status.diagn & 0x01;}

static inline bool avr_get_pvld(struct avr_cmd *status)
{return status->data.status.diagn>>1 & 0x01;}

static inline bool avr_get_rvld(struct avr_cmd *status)
{return status->data.status.diagn>>2 & 0x01;}

static inline bool avr_get_estop(struct avr_cmd *status)
{return status->data.status.diagn>>3 & 0x01;}

static inline uint8_t avr_get_diag(struct avr_cmd *status)
{return status->data.status.diagn>>4 & 0x0F;}

static inline uint8_t avr_get_direction(struct avr_cmd *status)
{return status->data.status.direction;}

static inline int16_t avr_get_dstate(struct avr_cmd *status)
{return ntohs(status->data.status.dstate);}

static inline int16_t avr_get_istate(struct avr_cmd *status)
{return ntohs(status->data.status.istate);}

static inline int16_t avr_get_pwm(struct avr_cmd *status)
{return ntohs(status->data.status.pwm_count);}


#endif // _AVR_CONTROLLER_H
