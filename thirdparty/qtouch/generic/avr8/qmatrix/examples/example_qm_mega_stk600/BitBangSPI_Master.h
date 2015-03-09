#ifndef BITBANGSPI_MASTER_H_INCLUDED
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#define BITBANGSPI_MASTER_H_INCLUDED

/*============================ PROTOTYPES ====================================*/
void BitBangSPI_Master_Init (void);
void BitBangSPI_Send_Message(void);

/*============================ MACROS ========================================*/


#define CPU_SPEED   8	// MHz

#define DELAYUS(DELAY) __builtin_avr_delay_cycles((DELAY)*CPU_SPEED);	



#endif
/* EOF */
