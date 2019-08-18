#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/*
 * 8 Pin Power Connector
 * 
 * 1,2: GND
 * 3: Solar In
 * 4: Charge In
 * 5,6: UOut
 * 7,8: UBat
 */
#define TWI_ADDR    9
#define TWI_MEMSIZE 256

#define DEF_SHUTDOWNDELAY 120
#define DEF_POWERUPDELAY  5

#define TWI_MEM_LOOPCNT   (0x00) /* 32Bit */
                       // (0x02)
#define TWI_MEM_RTC       (0x04) /* 32Bit */
                       // (0x06)
#define TWI_MEM_PMSW      (0x08)
                       
#define TWI_MEM_SHDWNCNT  (0x10)
#define TWI_MEM_PWRUPCNT  (0x12)

#define TWI_MEM_U1        (0x20)
#define TWI_MEM_U2        (0x22)
#define TWI_MEM_U3        (0x24)
#define TWI_MEM_U4        (0x26)

#define TWI_MEM_I1        (0x30) /* ISolar  */
#define TWI_MEM_I2        (0x32) /* IOut    */
#define TWI_MEM_I3        (0x34) /* ICharge */

#define PIN_ON            8
#define PIN_FON          10
#define PIN_IN_PMSW      A7

//((270.0+47.0)/47.0)
//#define UMULTIPLYER   (1.32*1247.0/0x175)
#define UMULTIPLYER     (9.781)

#define PIN_IN_U1       A0  /* UBat    */
#define PIN_IN_U2       A1  /* USolar  */
#define PIN_IN_U3       A2  /* UOut    */
#define PIN_IN_U4       A3  /* UCharge */

#define ADC_CH_A0_A1_20x  9   /* ISolar  */
#define ADC_CH_A0_A3_20x  11  /* ICharge */
#define ADC_CH_A1_A2_20x  13  /* IOut    */

//#define PMSW_A  (0x1fa+((0x390-0x1fa)/2))
//#define PMSW_B  (0x390+((0x3ff-0x390)/2))
//#define PMSW_A  0x280
//#define PMSW_B  0x3ff
//#define PMSW_U1 (int)(0.3*1023.0/2.56)
//#define PMSW_U2 (int)(0.8*1023.0/2.56)
//#define PMSW_U3 (int)(0.97*1023.0/2.56)
#define PMSW_U1 0x111
#define PMSW_U2 0x300
#define PMSW_U3 0x399
//#define PMSW_A  (PMSW_U1+((PMSW_U2-PMSW_U1)/2))
//#define PMSW_B  (PMSW_U2+((PMSW_U3-PMSW_U2)/2))
#define PMSW_A  0x200
#define PMSW_B  0x350
