/*
 *  Copyright (C) 2005 Austin Robot Technology
 *    by Alberto Alonso, Jack O'Quin
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

  Interface to Applanix Position and Orientation System for Land Vehicles

  \author Jack O'Quin, Alberto Alonso

*/

#ifndef _APPLANIX_H_
#define _APPLANIX_H_ 1

#include <ros/ros.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pcap.h>

#define APPLANIX_DISPLAY_PORT 5600
#define APPLANIX_CONTROL_PORT 5601
#define APPLANIX_RTDATA_PORT 5602
#define APPLANIX_LOGDATA_PORT 5603

#define APPLANIX_MAXMSGSIZE 6000

#define APPLANIX_DEFAULT_IP "192.168.1.25"

#define APPLANIX_NMEA_DEG_PER_BIT 0.0054931640625

#pragma pack(1)
typedef struct GRPHDR_MSG_ {
  char grpstart[4];
  uint16_t groupnum;
  uint16_t bytecount;                   /* size includes footer, not header */
} GRPHDR_MSG;

#pragma pack(1)
typedef struct GRPFTR_MSG_ {
  uint16_t crc;
  char grpend[2];  
} GRPFTR_MSG;

typedef enum {
  ApplStatusFull =	0,
  ApplStatusFine =	1,
  ApplStatusGcChi2 =	2,
  ApplStatusPcChi2 =	3,
  ApplStatusGcChi1 =	4,
  ApplStatusPcChi1 =	5,
  ApplStatusCoarse =	6,
  ApplStatusInitial =	7,
  ApplStatusInvalid =	8
} appl_alignment_status_t;

#pragma pack(1)
typedef struct GRP1DATA_MSG_ {
  char timedist[26];
  double lat;
  double lon;
  double alt;
  float vel_north;
  float vel_east;
  float vel_down;
  double roll;
  double pitch;
  double heading;
  double wander;
  float track;
  float speed;
  float arate_lon;
  float arate_trans;
  float arate_down;
  float accel_lon;
  float accel_trans;
  float accel_down;
  char alignment;
  char padding;
} GRP1DATA_MSG;

#pragma pack(1)
typedef struct GRP1_MSG_ {
  GRPHDR_MSG hdr;
  GRP1DATA_MSG data;
  GRPFTR_MSG ftr;
} GRP1_MSG;

#pragma pack(1)
typedef struct GRP4DATA_MSG_ {
  char timedist[26];
  int32_t vel_x;
  int32_t vel_y;
  int32_t vel_z;
  int32_t ang_x;
  int32_t ang_y;
  int32_t ang_z;
  char datastatus;
  char imutype;
  char imurate;
  uint16_t imustatus;
  char padding;
} GRP4DATA_MSG;

#pragma pack(1)
typedef struct GRP4_MSG_ {
  GRPHDR_MSG hdr;
  GRP4DATA_MSG data;
  GRPFTR_MSG ftr;
} GRP4_MSG;

#pragma pack(1)
typedef struct GRP_MSG_ {
  union {
    GRPHDR_MSG hdr;
    GRP1_MSG grp1;
    GRP4_MSG grp4;
  };
} GRP_MSG;

typedef struct {
  GRP1DATA_MSG	grp1;
  GRP4DATA_MSG	grp4;
  ros::Time time;
} applanix_data_t;

class DevApplanix
{
 public:

  DevApplanix(void)
    {
      serverhost = APPLANIX_DEFAULT_IP;
      buffer_length = 0;
      have_DGPS = true;
    };
  virtual ~DevApplanix() {};

  virtual int connect_socket(void);
  virtual int get_packet(applanix_data_t *adata);

 protected:
  bool have_DGPS;			// have full DGPS nav solution

  // socket parameters
  int                     sockfd;
  struct sockaddr_in      serveraddr;
  const char                    *serverhost;
  struct hostent          *serverhostp;

  char packet_buffer[APPLANIX_MAXMSGSIZE];
  size_t buffer_length;

  virtual int  read_packet(ros::Time *time);

  // unpack specific message types
  virtual void unpack_grp1(applanix_data_t *adata, GRP1_MSG *msg);
  virtual void unpack_grp4(applanix_data_t *adata, GRP4_MSG *msg);
};

/** Applanix input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, Velodyne's DSR software,
 * ethereal, wireshark, or tcpdump.
 */
class DevApplanixPCAP: public DevApplanix
{
 public:

 DevApplanixPCAP(std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0): DevApplanix()
    {
      filename_ = filename;
      fp_ = NULL;  
      pcap_ = NULL;  
      got_data_ = false;
      empty_ = true;
      read_once_ = read_once;
      if (read_once_)
        ROS_INFO("Read input file only once.");
      read_fast_ = read_fast;
      if (read_fast_)
        ROS_INFO("Read input file as quickly as possible.");
      repeat_delay_ = repeat_delay;
      if (repeat_delay_ > 0.0)
        ROS_INFO("Delay %.3f seconds before repeating input file.",
                 repeat_delay_);
    }

  virtual int connect_socket(void);
  virtual int  read_packet(ros::Time *time);

 private:
  std::string filename_;
  FILE *fp_;
  pcap_t *pcap_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool got_data_;
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

/** Unit test derived class */
class DevApplanixTest: public DevApplanix
{
 public:

 DevApplanixTest(std::string testfile): DevApplanix()
    {
      fp_ = NULL;
      testfile_ = testfile;
      ROS_INFO("testing with GPS data from \"%s\"", testfile_.c_str());
    }

  virtual int connect_socket(void);
  virtual int get_packet(applanix_data_t *adata);

 private:
  std::string testfile_;
  FILE *fp_;
};

#endif // _APPLANIX_H_
