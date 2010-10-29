/*
 *  Copyright (C) 2005, 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

  Driver for Applanix Position and Orientation System for Land Vehicles

  \author Jack O'Quin
*/

#include <math.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include "applanix.h"

#define DEVICE "Applanix POS-LV"

void DevApplanix::unpack_grp1(applanix_data_t *adata, GRP1_MSG *msg) 
{
  ROS_DEBUG(DEVICE " Group1: Speed %.3f Lat %.8f Lon %.8f Alt "
            "%.2f Heading %.2f, Alignment %d",
            msg->data.speed, msg->data.lat, msg->data.lon,
            msg->data.alt, msg->data.heading, msg->data.alignment);
  ROS_DEBUG(DEVICE " Group1: roll %.3f pitch %.3f",
            msg->data.roll, msg->data.pitch);

  adata->grp1 = msg->data;

  // As long as Omnistar differential GPS is active, the POS-LV
  // returns NAD-83 coordinates.  Without DGPS, it gives WGS-84
  // instead, which is what DARPA uses for the Urban Challenge.  In
  // the area of Victorville, CA we measured a difference between the
  // two coordinate systems is about 1.5m (North) and -1.333m (West).
  // Add these offsets to the NAD-83 values to get equivalent WGS-84
  // coordinates.
  //
  // log a warning loudly, if DGPS is lost
  if (have_DGPS)
    {
      if (msg->data.alignment > ApplStatusFull
	  && msg->data.alignment < ApplStatusInitial)
	{
	  ROS_WARN("Differential GPS lost!  Alignment status %d",
		   msg->data.alignment);
	  have_DGPS = false;		// only warn once
	}
    }
  else					// DGPS was lost
    {
      if (msg->data.alignment == ApplStatusFull)
	{
	  ROS_WARN("Differential GPS regained!");
	  have_DGPS = true;
	}
    }
}

void DevApplanix::unpack_grp4(applanix_data_t *adata, GRP4_MSG *msg) 
{
  //ROS_DEBUG("Group 4 message %d b %d\n",sizeof(GRP4_MSG),sizeof(ushort));
  ROS_DEBUG("  Status Data 0x%x IMU 0x%x IMU type 0x%x",
            msg->data.datastatus, msg->data.imustatus, msg->data.imutype);
#if 0 // need to figure out how to interpret these fields
  ROS_DEBUG("  X, Y, Z velocities: (%d, %d, %d)",
            msg->data.vel_x, msg->data.vel_y, msg->data.vel_z);
  ROS_DEBUG("  roll, pitch and yaw angular velocities (raw): (%d, %d, %d)",
            msg->data.ang_x, msg->data.ang_y, msg->data.ang_z);
  ROS_DEBUG("  roll, pitch and yaw angular velocities (deg): (%.3f, %.3f, %.3f)",
            APPLANIX_NMEA_DEG_PER_BIT * msg->data.ang_x,
            APPLANIX_NMEA_DEG_PER_BIT * msg->data.ang_y,
            APPLANIX_NMEA_DEG_PER_BIT * msg->data.ang_z);
#endif
  adata->grp4 = msg->data;
}

int DevApplanix::connect_socket(void) 
{
  int rc;

  ROS_DEBUG(DEVICE " creating socket");
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      ROS_ERROR("Unable to create socket for " DEVICE " (%s)",
		strerror(errno));
      return errno;
    }
 
  memset(&serveraddr, 0, sizeof(struct sockaddr_in));
  serveraddr.sin_family = AF_INET;
 
  serverhostp = gethostbyname(serverhost);
  if (serverhostp == NULL)
    {
      ROS_ERROR("gethostbyname() failed for " DEVICE " (%s)",
		strerror(errno));
      return errno;
    }

  bcopy(serverhostp->h_addr,
	(char *)&(serveraddr.sin_addr.s_addr),
	serverhostp->h_length);
 
  serveraddr.sin_port = htons(APPLANIX_RTDATA_PORT);       
 
  // Now connect to the server
  ROS_DEBUG(DEVICE " connecting to socket");
  rc = connect(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr));
  if (rc < 0)
    {
      ROS_ERROR("Unable to connect to " DEVICE " socket (%s)",
		strerror(errno));
      return errno;
    }

  rc = fcntl(sockfd, F_SETFL, O_NONBLOCK);
  if (rc < 0)
    {
      ROS_ERROR("Unable to set O_NONBLOCK for " DEVICE " socket (%s)",
		strerror(errno));
      return errno;
    }

  return 0;
}
 
// read the Applanix input socket
//
// returns 0 if any data received;
//	   EAGAIN if no data received;
//	   errno value if error
int DevApplanix::read_packet(ros::Time *time)
{
  int nbytes;

  // try to complete the packet in our buffer
  do
    {
      ROS_DEBUG(DEVICE " reading socket");
      nbytes = recv(sockfd, packet_buffer+buffer_length,
		    sizeof(packet_buffer)-buffer_length, 0);
    }
  while ((nbytes < 0) && (errno == EINTR));

  if (nbytes == 0)			// nothing received?
    {
      ROS_DEBUG(DEVICE " nothing received");
      return EAGAIN;
    }
  else if (nbytes < 0)
    {
      if (errno != EAGAIN)
	ROS_WARN(DEVICE " socket recv() error (%s)", strerror(errno));
      return errno;
    }

  *time = ros::Time::now();
  buffer_length += nbytes;

  ROS_DEBUG("Got packet with %d bytes, time %.6f", nbytes, time->toSec());

  return 0;
}

// get next Applanix packet
//
// return the packet in the applanix data struct.  Update adata->time
// only when a new navigation solution (GRP1) packet arrives.
//
// returns 0 if full packet received;
//	   EAGAIN if no packet (or incomplete);
//	   errno value if error
int DevApplanix::get_packet(applanix_data_t *adata)
{
  GRPHDR_MSG *hdr = (GRPHDR_MSG *) packet_buffer;
  ros::Time packet_time = adata->time;

  // have we got a complete packet?
  while ((buffer_length < sizeof(GRPHDR_MSG))
	 || (buffer_length < hdr->bytecount + sizeof(GRPHDR_MSG)))
    {
      int rc = read_packet(&packet_time);
      if (rc != 0)			// no data received?
	{
          ROS_DEBUG_STREAM(DEVICE " partial packet received ("
                           << buffer_length << " bytes)");
	  return rc;
	}
    }

  // have a full packet in the buffer
  // \todo fix for 64-bit
  ROS_DEBUG(DEVICE " %*.*s %d packet, size %d",
            (int) sizeof(hdr->grpstart), (int) sizeof(hdr->grpstart),
            hdr->grpstart, hdr->groupnum, hdr->bytecount);

  // copy packet data to applanix data struct
  switch(hdr->groupnum)
  {
  case 1:
    unpack_grp1(adata, (GRP1_MSG *) packet_buffer);
    adata->time = packet_time;
    break;
  case 4:
    unpack_grp4(adata, (GRP4_MSG *) packet_buffer);
    break;
  default:
    break;
  }

  // shift packet_buffer to the left
  size_t packet_length = hdr->bytecount + sizeof(GRPHDR_MSG);
  buffer_length -= packet_length;
  if (buffer_length > 0)
    memmove(packet_buffer, packet_buffer+packet_length, buffer_length);

  return 0;
}


int DevApplanixPCAP::connect_socket(void)
{
  ROS_INFO("Opening input file \"%s\"", filename_.c_str());
  
  /* Open the capture file */
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
    {
      /// \todo print error message, if pcap open fails
      ROS_FATAL("Error opening Applanix socket dump file.");
      return -1;
    }
  return 0;
}

/** get packet from PCAP dump 
 *
 * \returns 0 if any data received;
 *	    EAGAIN if no data received;
 *	    errno value if error
 */
int DevApplanixPCAP::read_packet(ros::Time *time)
{
  struct pcap_pkthdr *header;
  const u_char *pkt_data;
  int res;

  // return EAGAIN after successful calls, so the driver does not blow
  // through the entire file
  if (got_data_)
    {
      got_data_ = false;
      return EAGAIN;
    }

  // read the next packet from the PCAP dump
  if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      if (header->len > sizeof(packet_buffer)-buffer_length)
        {
          // TODO Save this packet for next time?
          ROS_WARN_STREAM("PCAP packet (size " << header->len
                          << ") overflows buffer (" <<
                          sizeof(packet_buffer)-buffer_length <<" left)");
          return EAGAIN;                // buffer too full
        }

      size_t pkt_offset = ETH_HLEN;     // IP header offset in packet
      struct iphdr *ip = (struct iphdr *) (pkt_data + pkt_offset);
      size_t size_ip = ip->ihl * 4;
      if (size_ip < 20)
        {
          ROS_WARN("invalid IP header size: %d", ip->ihl);
          return EAGAIN;                // ignore this packet
        }
      pkt_offset += size_ip;            // next header offset in packet

      if (ip->protocol == IPPROTO_TCP)  // got a TCP packet?
        {
          struct tcphdr *tcp = (struct tcphdr *) (pkt_data + pkt_offset);
          size_t size_tcp = tcp->doff * 4;
          if (size_tcp < 20)
            {
              ROS_WARN("invalid TCP header offset: %d", tcp->doff);
              return EAGAIN;            // ignore this packet
            }
          uint16_t dport = ntohs(tcp->dest);
          if (dport != APPLANIX_RTDATA_PORT)
            {
              ROS_INFO("wrong TCP data port: %u", dport);
              return EAGAIN;            // ignore this packet
            }
          ROS_DEBUG_STREAM("have TCP packet, port " << dport);
          pkt_offset += size_tcp;           // message offset in packet
        }
      else if (ip->protocol == IPPROTO_UDP) // got a UDP packet?
        {
          // I have not figured out how to use tcpdump to dump a TCP
          // connection with the RTDATA port.  But, the low-bandwidth
          // UDP display port is easy.  So, use that, instead.
          struct udphdr *udp = (struct udphdr *) (pkt_data + pkt_offset);
          uint16_t dport = ntohs(udp->dest);
          if (dport != APPLANIX_DISPLAY_PORT)
            {
              ROS_DEBUG("other UDP data port: %u", dport);
              return EAGAIN;            // ignore this packet
            }
          ROS_DEBUG_STREAM("have UDP packet, port " << dport);
          pkt_offset += sizeof(struct udphdr); // message offset in packet
        }
      else
        {
          ROS_DEBUG_STREAM("non-Applanix packet, protocol " << ip->protocol);
          return EAGAIN;                // ignore other packets
        }


        // copy all packet data after headers to buffer
        size_t pkt_length = header->len - pkt_offset;
        memcpy(packet_buffer + buffer_length, pkt_data + pkt_offset, pkt_length);
        buffer_length += pkt_length;
        *time = ros::Time::now();
        got_data_ = true;
        empty_ = false;
    }
  else                                  // end of file or error?
    {
      if (empty_)                       // no data in file?
        {
          if (res == -1)
            {
              ROS_WARN("Error reading Applanix packet: %s",
                       pcap_geterr(pcap_));
              return EIO;
            }
          ROS_FATAL("No useable Applanix packets in PCAP file");
          ros::shutdown();              // terminate this node
          return EIO;
        }

      if (read_once_)
        {
          //ROS_INFO("end of file reached -- done reading.");
          // stop returning data from now on
          return EAGAIN;
        }

      if (repeat_delay_ > 0.0)
        {
          ROS_INFO("end of file reached -- delaying %.3f seconds.",
                   repeat_delay_);
          usleep(rint(repeat_delay_ * 1000000.0));
        }

      ROS_DEBUG("replaying Applanix dump file");

      // I can't figure out how to rewind the file, because it
      // starts with some kind of header.  So, close the file
      // and reopen it with pcap.
      pcap_close(pcap_);
      pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
      empty_ = true;                    // maybe the file disappeared?

      return EAGAIN;                    // provide nothing this call
    }

  return 0;
}

int DevApplanixTest::connect_socket(void)
{
  fp_ = fopen(testfile_.c_str(), "r");
  if (fp_ == NULL)
    {
      int rc = errno;
      ROS_ERROR("failed to open \"%s\" (%s)",
                testfile_.c_str(), strerror(rc));
      return rc;
    }

  ROS_INFO("unit test connection: running with fake data");
  return 0;
}

/** unit test mode derived class */
int DevApplanixTest::get_packet(applanix_data_t *adata)
{
  applanix_data_t test_data;
  memset(&test_data, 0, sizeof(test_data));

  int rc = fscanf(fp_, " %d %lf %lf %lf %f %f %f %lf %lf %lf %f",
                  (int *) &test_data.grp1.alignment,
                  &test_data.grp1.lat,
                  &test_data.grp1.lon,
                  &test_data.grp1.alt,
                  &test_data.grp1.vel_north,
                  &test_data.grp1.vel_east,
                  &test_data.grp1.vel_down,
                  &test_data.grp1.roll,
                  &test_data.grp1.pitch,
                  &test_data.grp1.heading,
                  &test_data.grp1.speed);

  if (feof(fp_))                        // EOF?
    {
      clearerr(fp_);                    // clear EOF
      rewind(fp_);                      // start over
      return EAGAIN;                    //   but, not this time
    }
  if (ferror(fp_))                      // I/O error?
    {
      clearerr(fp_);
      return EIO;                       // failure
    }
  else if (rc < 11 || test_data.grp1.alignment == ApplStatusInvalid)
    return EAGAIN;                      // empty or incomplete packet

  test_data.time = ros::Time::now();
  *adata = test_data;

  return 0;
}
