/*****************************************************************************
 *
 * \file
 *
 * \brief lwIP configuration for AVR32 UC3.
 *
 * Copyright (c) 2009-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *****************************************************************************/
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* Include user defined options first */
// #include "conf_eth.h"
#include "conf_lwip_threads.h"
#include "lwip/debug.h"
#include "tracedump.h"

#undef LWIP_PLATFORM_DIAG
#define LWIP_PLATFORM_DIAG(x)   NAKED_TRACE_COM2 x
#undef LWIP_PLATFORM_ASSERT
#define LWIP_PLATFORM_ASSERT(x)   NAKED_TRACE_COM2(x)


/* Define default values for unconfigured parameters. */
#define LWIP_NOASSERT 1 // To suppress some errors for now (no debug output)

/* Platform specific locking */

/*
 * enable SYS_LIGHTWEIGHT_PROT in lwipopts.h if you want inter-task protection
 * for certain critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT            1

/* ---------- Memory options ---------- */

/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT                   4

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE                        4 * 1024

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF                   0

/* Number of raw connection PCBs */
#define MEMP_NUM_RAW_PCB                0

#if (TFTP_USED == 1)
  /* ---------- UDP options ---------- */
  #define LWIP_UDP                1
  #define UDP_TTL                 255
  /* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
     per active UDP "connection". */
  #define MEMP_NUM_UDP_PCB        1
#else
  /* ---------- UDP options ---------- */
  #define LWIP_UDP                0
  #define UDP_TTL                 0
  /* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
     per active UDP "connection". */
  #define MEMP_NUM_UDP_PCB        0
#endif
/* MEMP_NUM_TCP_PCB: the number of simultaneously active TCP
   connections. */
#define MEMP_NUM_TCP_PCB                15
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
#define MEMP_NUM_TCP_PCB_LISTEN         1
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
#define MEMP_NUM_TCP_SEG                9
/* MEMP_NUM_SYS_TIMEOUT: the number of simultaneously active
   timeouts. */
#define MEMP_NUM_SYS_TIMEOUT            4

/* The following four are used only with the sequential API and can be
   set to 0 if the application only will use the raw API. */
/* MEMP_NUM_NETBUF: the number of struct netbufs. */
#define MEMP_NUM_NETBUF                 3
/* MEMP_NUM_NETCONN: the number of struct netconns. */
#define MEMP_NUM_NETCONN                6


/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */

#define PBUF_POOL_SIZE                  10

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */

#define PBUF_POOL_BUFSIZE               512

/** ETH_PAD_SIZE: number of bytes added before the Ethernet header to ensure
 * alignment of payload after that header. Since the header is 14 bytes long,
 * without this padding e.g. addresses in the IP header will not be aligned
 * on a 32-bit boundary, so setting this to 2 can speed up 32-bit-platforms.
 */
#define ETH_PAD_SIZE                    0

/* PBUF_LINK_HLEN: the number of bytes that should be allocated for a
   link level header. Defaults to 14 for Ethernet. */

#define PBUF_LINK_HLEN                  (14 + ETH_PAD_SIZE)


/**
 * DEFAULT_RAW_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_RAW. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_RAW_RECVMBOX_SIZE       6

/**
 * DEFAULT_UDP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_UDP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_UDP_RECVMBOX_SIZE       6

/**
 * DEFAULT_TCP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_TCP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_TCP_RECVMBOX_SIZE       6


/**
 * DEFAULT_ACCEPTMBOX_SIZE: The mailbox size for the incoming connections.
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when the acceptmbox is created.
 */
#define DEFAULT_ACCEPTMBOX_SIZE         6

/* ---------- ARP options ---------- */

/** Number of active hardware address, IP address pairs cached */
#define ARP_TABLE_SIZE                  5

/**
 * If enabled, outgoing packets are queued during hardware address
 * resolution.
 *
 * This feature has not stabilized yet. Single-packet queueing is
 * believed to be stable, multi-packet queueing is believed to
 * clash with the TCP segment queueing.
 *
 * As multi-packet-queueing is currently disabled, enabling this
 * _should_ work, but we need your testing feedback on lwip-users.
 *
 */
#undef  ARP_QUEUEING
// #define  ARP_QUEUEING   0


/* ---------- IP options ---------- */
/* Define IP_FORWARD to 1 if you wish to have the ability to forward
   IP packets across network interfaces. If you are going to run lwIP
   on a device with only one network interface, define this to 0. */
#define IP_FORWARD                      0

/* If defined to 1, IP options are allowed (but not parsed). If
   defined to 0, all packets with IP options are dropped. */
#define IP_OPTIONS                      1

/** IP reassembly and segmentation. Even if they both deal with IP
 *  fragments, note that these are orthogonal, one dealing with incoming
 *  packets, the other with outgoing packets
 */

/** Reassemble incoming fragmented IP packets */
#define IP_REASSEMBLY                   0


/* ---------- ICMP options ---------- */

#define ICMP_TTL                        255

/* ---------- RAW options ---------- */

#define LWIP_RAW                        0

//#define RAW_TTL                        (IP_DEFAULT_TTL)

/* ---------- DHCP options ---------- */

#define LWIP_DHCP                       0

/* 1 if you want to do an ARP check on the offered address
   (recommended). */
// #define DHCP_DOES_ARP_CHECK             1

/* ---------- SNMP options ---------- */


/* ---------- TCP options ---------- */
#define LWIP_TCP                        1

#define TCP_TTL                         255

#define TCP_WND                         2048


/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ                 1

/* TCP Maximum segment size. */
#define TCP_MSS                         1024

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF                     2048

/* TCP sender buffer space (pbufs). This must be at least = 2 *
   TCP_SND_BUF/TCP_MSS for things to work. */
#define TCP_SND_QUEUELEN                ((6 * (TCP_SND_BUF) + (TCP_MSS - 1))/(TCP_MSS))


/* Maximum number of retransmissions of data segments. */
#define TCP_MAXRTX                      6

/* Maximum number of retransmissions of SYN segments. */
#define TCP_SYNMAXRTX                   6


/*
   ------------------------------------
   ---------- Thread options ----------
   ------------------------------------
*/
/**
 * TCPIP_THREAD_NAME: The name assigned to the main tcpip thread.
 */
#define TCPIP_THREAD_NAME              "TCP/IP"

/**
 * TCPIP_THREAD_STACKSIZE: The stack size used by the main tcpip thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_STACKSIZE          lwipINTERFACE_STACK_SIZE

/**
 * TCPIP_THREAD_PRIO: The priority assigned to the main tcpip thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_PRIO               lwipINTERFACE_TASK_PRIORITY

/**
 * TCPIP_MBOX_SIZE: The mailbox size for the tcpip thread messages
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when tcpip_init is called.
 */
#define TCPIP_MBOX_SIZE                 6


/**
 * DEFAULT_THREAD_STACKSIZE: The stack size used by any other lwIP thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define DEFAULT_THREAD_STACKSIZE        configMINIMAL_STACK_SIZE



/* ---------- Socket Options ---------- */

#ifdef __GNUC__
// Do not use the lwip timeval structure.
#define LWIP_TIMEVAL_PRIVATE            0
// Use the gnu libc struct timeval instead.
#include <sys/time.h>
#endif


/* ---------- Statistics options ---------- */
#define LWIP_STATS                1

#define LWIP_STATS_DISPLAY        1

#if LWIP_STATS

#define LINK_STATS  1
#define IP_STATS    1
#define ICMP_STATS  1
#define UDP_STATS   1
#define TCP_STATS   1
#define MEM_STATS   1
#define MEMP_STATS  1
#define PBUF_STATS  1
#define SYS_STATS   1
#endif /* LWIP_STATS */

/* ---------- PPP options ---------- */


/* ---------- Lwip Debug options ---------- */



#define DBG_TYPES_ON                    0xff

#define ETHARP_DEBUG                    LWIP_DBG_OFF

#define NETIF_DEBUG                     LWIP_DBG_OFF

#define PBUF_DEBUG                      LWIP_DBG_ON

#define API_LIB_DEBUG                   LWIP_DBG_OFF

#define API_MSG_DEBUG                   LWIP_DBG_ON

#define SOCKETS_DEBUG                   LWIP_DBG_OFF

#define ICMP_DEBUG                      LWIP_DBG_OFF

#define INET_DEBUG                      LWIP_DBG_OFF

#define IP_DEBUG                        LWIP_DBG_OFF

#define IP_REASS_DEBUG                  LWIP_DBG_OFF

#define RAW_DEBUG                       LWIP_DBG_OFF

#define MEM_DEBUG                       LWIP_DBG_OFF

#define MEMP_DEBUG                      LWIP_DBG_OFF

#define SYS_DEBUG                       LWIP_DBG_OFF

#define TCP_DEBUG                       LWIP_DBG_ON

#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF

#define TCP_FR_DEBUG                    LWIP_DBG_OFF

#define TCP_RTO_DEBUG                   LWIP_DBG_OFF

#define TCP_CWND_DEBUG                  LWIP_DBG_OFF

#define TCP_WND_DEBUG                   LWIP_DBG_OFF

#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF

#define TCP_RST_DEBUG                   LWIP_DBG_OFF

#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF

#define UDP_DEBUG                       LWIP_DBG_OFF

#define TCPIP_DEBUG                     LWIP_DBG_OFF

#define DBG_MIN_LEVEL                   LWIP_DBG_LEVEL_SEVERE


// \note For a list of all possible lwIP configurations, check http://lwip.wikia.com/wiki/Lwipopts.h

#endif /* __LWIPOPTS_H__ */



