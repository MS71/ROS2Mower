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
#define TWI_MEMSIZE 256

#define DEF_SHUTDOWNDELAY    60        // shutdown after X seconds
#define DEF_POWERUPDELAY      (30*60)  // powerup after 30 minutes
#define DEF_POWERUPDELAY_INIT 3        // powerup after 30 minutes

//#define DEF_SHUTDOWNDELAY 1       // shutdown after X seconds
//#define DEF_POWERUPDELAY  1  // powerup after Y seconds

#define TWI_MEM_LOOPCNT   (0x00) /* 32Bit */
                       // (0x02)
#define TWI_MEM_RTC       (0x04) /* 32Bit */
                       // (0x06)
#define TWI_MEM_PMSW      (0x08)

#define TWI_MEM_WDT       (0x0A)

#define TWI_MEM_SHDWNCNT   (0x10)
#define TWI_MEM_PWRUPCNT   (0x12)
#define TWI_MEM_SHDWNREL   (0x14)
#define TWI_MEM_PWRUPREL   (0x16)
#define TWI_MEM_STAYONUBat (0x18)
#define TWI_MEM_REBOOT     (0x1A)

#define TWI_MEM_U1        (0x20)
#define TWI_MEM_U2        (0x22)
#define TWI_MEM_U3        (0x24)
#define TWI_MEM_U4        (0x26)
#define TWI_MEM_Ubat      (0x28)

#define TWI_MEM_I1        (0x30) /* ISolar  */
#define TWI_MEM_I2        (0x32) /* IOut    */
#define TWI_MEM_I3        (0x34) /* ICharge */

#define PIN_ON            8
#define PIN_FON          10
#define PIN_IN_PMSW       7

#define LP_N             100

//((270.0+47.0)/47.0)
//#define UMULTIPLYER   (1.32*1247.0/0x175)
//#define UMULTIPLYER     (9.781)
//#define UMULTIPLYER     ((270.0+47.0)/47.0)
//#define UMULTIPLYER     (122000.0/5486.0)
//#define UMULTIPLYER     (1)
#define UMULTIPLYER     (1047.0/47.0)

#define UBAT_ON          12400
#define UBAT_OFF         11000

#define PIN_IN_U1         3  /* UBat    */
#define PIN_IN_U2         1  /* USolar  */
#define PIN_IN_U3         2  /* UOut    */
#define PIN_IN_U4         0  /* UCharge */

#define ADC_ISOL_CH         (0b101111)  /* ISolar: 20* ADC3-ADC1 */
#define ADC_ISOL_OFFSET     0 
#define ADC_ISOL_MULT       (- 1.0/20.0)

#define ADC_ICHARGE_CH      (0b101011)  /* ICharge: 20* ADC3-ADC0 */
#define ADC_ICHARGE_OFFSET  0
#define ADC_ICHARGE_MULT    (- 1.0/20.0)

#define ADC_IOUT_CH         (0b110001)  /* IOut: 20* ADC3-ADC2 */
#define ADC_IOUT_OFFSET     0
#define ADC_IOUT_MULT       (1.0/20.0)

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
