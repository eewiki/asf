#ifndef __NETIF_NRWLANIF_H__
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#define __NETIF_NRWLANIF_H__

#include "lwip/netif.h"
#include "lwip/err.h"

err_t wlif_init(struct netif *netif);
void wlif_poll(struct netif *netif);

#endif
