#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define TWI_ADDR    9
#define TWI_MEMSIZE 256

#define DEF_SHUTDOWNDELAY 120

#define TWI_MEM_LOOPCNT   (0*2)
                       // (1*2)
#define TWI_MEM_SHDWNCNT  (2*2)
#define TWI_MEM_PWRUPCNT  (3*2)
#define TWI_MEM_PMSW      (4*2)
#define TWI_MEM_U1        (5*2)
#define TWI_MEM_U2        (6*2)
#define TWI_MEM_U3        (7*2)
#define TWI_MEM_U4        (8*2)

#define PIN_ON      8
#define PIN_FON     10
#define PIN_IN_PMSW A7

//((270.0+47.0)/47.0)
#define UMULTIPLYER   (1.32*1247.0/0x175)

#define PIN_IN_U1   A0
#define PIN_IN_U2   A1
#define PIN_IN_U3   A2
#define PIN_IN_U4   A3

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
#define PMSW_B  0x3f9

