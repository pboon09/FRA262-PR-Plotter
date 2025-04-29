
#ifndef __RAK410_H_
#define __RAK410_H_ 1

#include <stdint.h>

/* RAK410 Setup */
void enable_rak410_setup(void);
void output_rak410_setup(uint8_t dhcp,uint32_t owner_ip,uint32_t gateway,uint8_t *status);
void disable_rak410_setup(void);

/* RAK410 UDP Send */
void enable_rak410_udpsend(void);
void output_rak410_udpsend(uint32_t ip, uint32_t port, uint32_t data1,uint32_t data2, uint8_t *status);
void disable_rak410_udpsend(void);
#endif //__RAK410_H_
