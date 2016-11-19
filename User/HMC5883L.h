#ifndef __HMC5883L_H
#define __HMC5883L_H
#include "stm32f4xx.h"

#define HMC58X3_ADDR 0x3C// 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0x0
#define HMC58X3_R_CONFB 0x1
#define HMC58X3_R_MODE 	0x2
#define HMC58X3_R_XH 		0x3
#define HMC58X3_R_XL 		0x4
 
#define HMC58X3_R_YH 		0x7 //!< Register address for YM.
#define HMC58X3_R_YL 		0x8  //!< Register address for YL.
#define HMC58X3_R_ZH 		0x5  //!< Register address for ZM.
#define HMC58X3_R_ZL 		0x6  //!< Register address for ZL.
 
#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

#define XOFFSET  123.5f
#define YOFFSET  170.5f

void initHMC5883(void);
void HMC5883_read(void);
float  forComplement(uint16_t temp);
void  SelfTest(void);
#endif
