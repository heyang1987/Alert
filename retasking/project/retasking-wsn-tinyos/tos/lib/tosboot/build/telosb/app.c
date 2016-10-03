#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 431 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 42 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memcpy(void *arg_0x2b161b30bbf0, const void *arg_0x2b161b309020, size_t arg_0x2b161b3092c8);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2b161b364290);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2b161b369300);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/opt/retasking-wsn-tinyos/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/opt/retasking-wsn-tinyos/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );





typedef unsigned int __istate_t;
#line 108
void __delay_cycles(unsigned long int delay);
# 155 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char IFG1 __asm ("__""IFG1");








extern volatile unsigned char ME1 __asm ("__""ME1");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 267
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");
#line 282
extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");
#line 303
extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");
#line 327
extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");






extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");
#line 382
extern volatile unsigned char U0CTL __asm ("__""U0CTL");

extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");

extern volatile unsigned char U0RCTL __asm ("__""U0RCTL");



extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");

extern volatile unsigned char U0TXBUF __asm ("__""U0TXBUF");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");
#line 928
extern volatile unsigned int FCTL1 __asm ("__""FCTL1");

extern volatile unsigned int FCTL2 __asm ("__""FCTL2");

extern volatile unsigned int FCTL3 __asm ("__""FCTL3");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");

extern volatile unsigned int ADC12IFG __asm ("__""ADC12IFG");
#line 1038
extern volatile unsigned int ADC12MEM0 __asm ("__""ADC12MEM0");
#line 1077
extern volatile unsigned char ADC12MCTL0 __asm ("__""ADC12MCTL0");
# 378 "/opt/retasking-wsn-tinyos/tos/chips/msp430/msp430hardware.h"
typedef uint8_t mcu_power_t  ;



enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 48 "telosb/hardware.h"
typedef uint16_t in_flash_addr_t;

typedef uint32_t ex_flash_addr_t;

static inline void wait(uint16_t t);






static inline void TOSH_SET_RED_LED_PIN()  ;
#line 59
static inline void TOSH_CLR_RED_LED_PIN()  ;
static inline void TOSH_SET_GREEN_LED_PIN()  ;
#line 60
static inline void TOSH_CLR_GREEN_LED_PIN()  ;
static inline void TOSH_SET_YELLOW_LED_PIN()  ;
#line 61
static inline void TOSH_CLR_YELLOW_LED_PIN()  ;









static inline uint8_t TOSH_READ_USERINT_PIN()  ;



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 75
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 75
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 76
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;

static void TOSH_SET_PIN_DIRECTIONS(void );
# 52 "../net/Deluge/Deluge.h"
#line 40
typedef nx_struct DelugeIdent {
  nx_uint32_t uidhash;
  nx_uint32_t size;
  nx_uint8_t numPgs;
  nx_uint8_t reserved;
  nx_uint16_t crc;
  nx_uint8_t appname[16];
  nx_uint8_t username[16];
  nx_uint8_t hostname[16];
  nx_uint8_t platform[16];
  nx_uint32_t timestamp;
  nx_uint32_t userhash;
} __attribute__((packed)) DelugeIdent;

enum __nesc_unnamed4251 {
  DELUGE_INVALID_UID = 0xffffffff, 
  DELUGE_NUM_VOLUMES = 4, 
  DELUGE_KEY = 0xDE00, 
  DELUGE_AM_FLASH_VOL_MANAGER = 0x53, 
  DELUGE_AM_DELUGE_MANAGER = 0x54, 
  DELUGE_AM_NODE_STATUS = 0x55, 
  AM_NODESTATUS = DELUGE_AM_NODE_STATUS, 
  DELUGE_COL_NODE_STATUS = 0x22
};

enum __nesc_unnamed4252 {
  DELUGE_CMD_STOP = 1, 
  DELUGE_CMD_LOCAL_STOP = 2, 
  DELUGE_CMD_ONLY_DISSEMINATE = 3, 
  DELUGE_CMD_DISSEMINATE_AND_REPROGRAM = 4, 
  DELUGE_CMD_REPROGRAM = 5, 
  DELUGE_CMD_REBOOT = 6, 
  DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_NODES = 7, 
  DELUGE_CMD_DISSEMINATE_AND_REPROGRAM_GROUP = 8, 
  DELUGE_CMD_UPDATE_GROUP = 9
};
#line 95
#line 88
typedef nx_struct NodeStatus {
  nx_uint16_t nodeId;
  nx_uint8_t groupId;
  nx_uint8_t state;
  nx_uint32_t appUid;
  nx_uint8_t appName[8];
  nx_uint32_t appTimeStamp;
} __attribute__((packed)) NodeStatus;
#line 108
#line 101
typedef nx_struct DelugeCmd {
  nx_uint8_t type;
  nx_uint32_t uidhash;
  nx_uint8_t imgNum;
  nx_uint32_t size;
  nx_uint32_t nodeIds;
  nx_uint8_t groupId;
} __attribute__((packed)) DelugeCmd;







#line 110
typedef struct BootArgs {
  uint16_t address;
  uint32_t imageAddr;
  uint8_t gestureCount;
  bool noReprogram;
  uint8_t groupId;
} BootArgs;
# 42 "../net/Deluge/extra/telosb/TOSBoot_platform.h"
enum __nesc_unnamed4253 {
  TOSBOOT_ARGS_ADDR = 0x70, 
  TOSBOOT_GESTURE_MAX_COUNT = 3, 
  TOSBOOT_GOLDEN_IMG_ADDR = 0xf0000L, 
  TOSBOOT_INT_PAGE_SIZE = 512L
};
# 39 "/opt/retasking-wsn-tinyos/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4254 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4255 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/retasking-wsn-tinyos/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4256 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4257 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/opt/retasking-wsn-tinyos/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4258 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4259 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4260 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/opt/retasking-wsn-tinyos/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/retasking-wsn-tinyos/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 63 "../net/Deluge/DelugePageTransfer.h"
typedef int32_t object_id_t;
typedef nx_int32_t nx_object_id_t;
typedef uint32_t object_size_t;
typedef nx_uint32_t nx_object_size_t;
typedef uint8_t page_num_t;
typedef nx_uint8_t nx_page_num_t;

enum __nesc_unnamed4261 {
  DELUGET2_PKT_PAYLOAD_SIZE = 28 - sizeof(nx_object_id_t ) - sizeof(nx_page_num_t ) - sizeof(nx_uint8_t ), 
  DELUGET2_BYTES_PER_PAGE = 1024, 
  DELUGET2_PKTS_PER_PAGE = (DELUGET2_BYTES_PER_PAGE - 1) / DELUGET2_PKT_PAYLOAD_SIZE + 1, 
  DELUGET2_PKT_BITVEC_SIZE = (DELUGET2_PKTS_PER_PAGE - 1) / 8 + 1, 

  DELUGE_PKT_PAYLOAD_SIZE = 23, 
  DELUGE_PKTS_PER_PAGE = 48, 
  DELUGE_BYTES_PER_PAGE = DELUGE_PKTS_PER_PAGE * DELUGE_PKT_PAYLOAD_SIZE, 

  DELUGE_VERSION = 2, 
  DELUGE_MAX_ADV_PERIOD_LOG2 = 22, 
  DELUGE_NUM_NEWDATA_ADVS_REQUIRED = 2, 
  DELUGE_NUM_MIN_ADV_PERIODS = 2, 
  DELUGE_MAX_NUM_REQ_TRIES = 1, 
  DELUGE_REBOOT_DELAY = 4, 
  DELUGE_FAILED_SEND_DELAY = 16, 
  DELUGE_MIN_DELAY = 16, 

  DELUGE_IDENT_SIZE = 128, 
  DELUGE_INVALID_ADDR = 0x7fffffffL, 
  DELUGE_MIN_ADV_PERIOD_LOG2 = 9, 
  DELUGE_MAX_REQ_DELAY = 0x1L << (DELUGE_MIN_ADV_PERIOD_LOG2 - 1), 
  DELUGE_NACK_TIMEOUT = DELUGE_MAX_REQ_DELAY >> 0x1, 
  DELUGE_MAX_IMAGE_SIZE = 128L * 1024L, 
  DELUGE_MAX_PAGES = 128, 
  DELUGE_CRC_SIZE = sizeof(uint16_t ), 
  DELUGE_CRC_BLOCK_SIZE = DELUGE_MAX_PAGES * DELUGE_CRC_SIZE, 
  DELUGE_GOLDEN_IMAGE_NUM = 0x0, 
  DELUGE_INVALID_OBJID = 0xff, 
  DELUGE_INVALID_PKTNUM = 0xff, 
  DELUGE_INVALID_PGNUM = 0xff, 
  DELUGE_QSIZE = 2
};






#line 105
typedef struct DelugeAdvTimer {
  uint32_t timer : 32;
  uint8_t periodLog2 : 8;
  bool overheard : 1;
  uint8_t newAdvs : 7;
} DelugeAdvTimer;







#line 112
typedef nx_struct DelugeObjDesc {
  nx_object_id_t objid;
  nx_page_num_t numPgs;
  nx_uint16_t crc;
  nx_page_num_t numPgsComplete;
  nx_uint8_t reserved;
} __attribute__((packed)) DelugeObjDesc;
# 53 "crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/opt/retasking-wsn-tinyos/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4262 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4263 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4264 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4265 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4266 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4267 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4268 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 169
#line 150
typedef enum __nesc_unnamed4269 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4270 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4271 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4272 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4273 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4274 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4275 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4276 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4277 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4278 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4279 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 3 "Exec.nc"
static void ExecC__Exec__exec(void );
# 43 "ExtFlash.nc"
static void ExtFlashP__ExtFlash__startRead(uint32_t addr);

static void ExtFlashP__ExtFlash__stopRead(void );
#line 44
static uint8_t ExtFlashP__ExtFlash__readByte(void );
# 62 "/opt/retasking-wsn-tinyos/tos/interfaces/Init.nc"
static error_t ExtFlashP__Init__init(void );
# 95 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
static error_t ExtFlashP__StdControl__start(void );









static error_t ExtFlashP__StdControl__stop(void );
# 46 "msp430/HplUsartControl.nc"
static void HplUsart0C__HplUsartControl__disableSPI(void );



static error_t HplUsart0C__HplUsartControl__isTxEmpty(void );
#line 47
static void HplUsart0C__HplUsartControl__setModeSPI(void );



static error_t HplUsart0C__HplUsartControl__isTxIntrPending(void );

static void HplUsart0C__HplUsartControl__tx(uint8_t data);
static uint8_t HplUsart0C__HplUsartControl__rx(void );
#line 52
static error_t HplUsart0C__HplUsartControl__isRxIntrPending(void );
# 43 "Hardware.nc"
static void HardwareC__Hardware__init(void );
static void HardwareC__Hardware__reboot(void );
# 56 "/opt/retasking-wsn-tinyos/tos/interfaces/InternalFlash.nc"
static error_t InternalFlashC__InternalFlash__read(void *addr, 
#line 50
void * buf, 





uint16_t size);
#line 68
static error_t InternalFlashC__InternalFlash__write(void *addr, 
#line 63
void * buf, 




uint16_t size);
# 45 "Leds.nc"
static void LedsC__Leds__glow(uint8_t a, uint8_t b);
#line 44
static void LedsC__Leds__flash(uint8_t a);
#line 43
static void LedsC__Leds__set(uint8_t ledsOn);
# 95 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
static error_t PowerOffC__StdControl__start(void );









static error_t PowerOffC__StdControl__stop(void );
# 43 "ProgFlash.nc"
static error_t ProgFlashC__ProgFlash__write(in_flash_addr_t addr, uint8_t *buf, in_flash_addr_t len);
# 43 "Voltage.nc"
static bool VoltageC__Voltage__okToProgram(void );
# 43 "Hardware.nc"
static void TosBootP__Hardware__init(void );
static void TosBootP__Hardware__reboot(void );
# 43 "ExtFlash.nc"
static void TosBootP__ExtFlash__startRead(uint32_t addr);

static void TosBootP__ExtFlash__stopRead(void );
#line 44
static uint8_t TosBootP__ExtFlash__readByte(void );
# 43 "ProgFlash.nc"
static error_t TosBootP__ProgFlash__write(in_flash_addr_t addr, uint8_t *buf, in_flash_addr_t len);
# 62 "/opt/retasking-wsn-tinyos/tos/interfaces/Init.nc"
static error_t TosBootP__SubInit__init(void );
# 95 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
static error_t TosBootP__SubControl__start(void );









static error_t TosBootP__SubControl__stop(void );
# 43 "Voltage.nc"
static bool TosBootP__Voltage__okToProgram(void );
# 45 "Leds.nc"
static void TosBootP__Leds__glow(uint8_t a, uint8_t b);
#line 44
static void TosBootP__Leds__flash(uint8_t a);
#line 43
static void TosBootP__Leds__set(uint8_t ledsOn);
# 3 "Exec.nc"
static void TosBootP__Exec__exec(void );
# 56 "/opt/retasking-wsn-tinyos/tos/interfaces/InternalFlash.nc"
static error_t TosBootP__IntFlash__read(void *addr, 
#line 50
void * buf, 





uint16_t size);
#line 68
static error_t TosBootP__IntFlash__write(void *addr, 
#line 63
void * buf, 




uint16_t size);
# 61 "TosBootP.nc"
enum TosBootP____nesc_unnamed4280 {
  TosBootP__LEDS_LOWBATT = 1, 
  TosBootP__LEDS_GESTURE = 7
};

enum TosBootP____nesc_unnamed4281 {
  TosBootP__R_SUCCESS, 
  TosBootP__R_INVALID_IMAGE_ERROR, 
  TosBootP__R_PROGRAMMING_ERROR
};

static void TosBootP__startupLeds(void );









static in_flash_addr_t TosBootP__extFlashReadAddr(void );







static bool TosBootP__verifyBlock(ex_flash_addr_t crcAddr, ex_flash_addr_t startAddr, uint16_t len);
#line 109
static inline bool TosBootP__verifyImage(ex_flash_addr_t startAddr);
#line 141
static error_t TosBootP__programImage(ex_flash_addr_t startAddr);
#line 215
static inline void TosBootP__runApp(void );




static inline void TosBootP__startupSequence(void );
#line 274
int main(void )   ;
# 47 "msp430/ExecC.nc"
static inline void ExecC__Exec__exec(void );
# 46 "msp430/HplUsartControl.nc"
static void ExtFlashP__UsartControl__disableSPI(void );



static error_t ExtFlashP__UsartControl__isTxEmpty(void );
#line 47
static void ExtFlashP__UsartControl__setModeSPI(void );



static error_t ExtFlashP__UsartControl__isTxIntrPending(void );

static void ExtFlashP__UsartControl__tx(uint8_t data);
static uint8_t ExtFlashP__UsartControl__rx(void );
#line 52
static error_t ExtFlashP__UsartControl__isRxIntrPending(void );
# 52 "stm25p/ExtFlashP.nc"
static inline error_t ExtFlashP__Init__init(void );







static inline error_t ExtFlashP__StdControl__start(void );



static error_t ExtFlashP__StdControl__stop(void );
#line 79
static inline void ExtFlashP__powerOnFlash(void );
#line 95
static void ExtFlashP__ExtFlash__startRead(uint32_t addr);
#line 114
static uint8_t ExtFlashP__ExtFlash__readByte(void );






static inline void ExtFlashP__ExtFlash__stopRead(void );
# 47 "msp430/HplUsart0C.nc"
static inline void HplUsart0C__HplUsartControl__disableSPI(void );










static inline void HplUsart0C__HplUsartControl__setModeSPI(void );
#line 108
static inline error_t HplUsart0C__HplUsartControl__isTxEmpty(void );






static error_t HplUsart0C__HplUsartControl__isTxIntrPending(void );







static inline error_t HplUsart0C__HplUsartControl__isRxIntrPending(void );







static inline void HplUsart0C__HplUsartControl__tx(uint8_t data);



static inline uint8_t HplUsart0C__HplUsartControl__rx(void );
# 50 "msp430/HardwareC.nc"
static inline void HardwareC__Hardware__init(void );





static inline void HardwareC__Hardware__reboot(void );
# 59 "msp430/InternalFlashC.nc"
enum InternalFlashC____nesc_unnamed4282 {
  InternalFlashC__IFLASH_OFFSET = 0x1000, 
  InternalFlashC__IFLASH_SIZE = 128, 
  InternalFlashC__IFLASH_SEG0_VNUM_ADDR = 0x107f, 
  InternalFlashC__IFLASH_SEG1_VNUM_ADDR = 0x10ff, 
  InternalFlashC__IFLASH_INVALID_VNUM = -1
};

static uint8_t InternalFlashC__chooseSegment(void );









static error_t InternalFlashC__InternalFlash__write(void *addr, void *buf, uint16_t size);
#line 119
static inline error_t InternalFlashC__InternalFlash__read(void *addr, void *buf, uint16_t size);
# 48 "lib/LedsC.nc"
enum LedsC____nesc_unnamed4283 {
  LedsC__RED_BIT = 1, 
  LedsC__GREEN_BIT = 2, 
  LedsC__YELLOW_BIT = 4
};

static void LedsC__Leds__set(uint8_t ledsOn);
#line 69
static void LedsC__Leds__flash(uint8_t a);
#line 81
static void LedsC__Leds__glow(uint8_t a, uint8_t b);
# 105 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
static error_t PowerOffC__SubControl__stop(void );
# 45 "Leds.nc"
static void PowerOffC__Leds__glow(uint8_t a, uint8_t b);
# 55 "msp430f1611/PowerOffC.nc"
static inline void PowerOffC__haltsystem(void );
#line 76
static inline error_t PowerOffC__StdControl__start(void );
#line 92
static inline error_t PowerOffC__StdControl__stop(void );
# 50 "msp430/ProgFlashC.nc"
enum ProgFlashC____nesc_unnamed4284 {
  ProgFlashC__RESET_ADDR = 0xfffe
};

static inline error_t ProgFlashC__ProgFlash__write(in_flash_addr_t addr, uint8_t *buf, uint16_t len);
# 50 "msp430/VoltageC.nc"
enum VoltageC____nesc_unnamed4285 {
  VoltageC__VTHRESH = 0xE66
};

static inline bool VoltageC__Voltage__okToProgram(void );
# 61 "/opt/retasking-wsn-tinyos/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4286 {

  SchedulerBasicP__NUM_TASKS = 0U, 
  SchedulerBasicP__NO_TASK = 255
};
# 391 "/opt/retasking-wsn-tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 50 "msp430/HardwareC.nc"
static inline void HardwareC__Hardware__init(void )
#line 50
{
  WDTCTL = 0x5A00 + 0x0080;
  BCSCTL1 = ((0x01 | 0x02) | 0x04) | 0x80;
  DCOCTL = (0x20 | 0x40) | 0x80;
}

# 43 "Hardware.nc"
inline static void TosBootP__Hardware__init(void ){
#line 43
  HardwareC__Hardware__init();
#line 43
}
#line 43
# 58 "msp430/HplUsart0C.nc"
static inline void HplUsart0C__HplUsartControl__setModeSPI(void )
#line 58
{




  U0CTL = ((0x01 | 0x10) | 0x04) | 0x02;


  U0TCTL |= 0x02 + 0x80 + 0x20;


  U0BR0 = 0x02;
  U0BR1 = 0;


  ME1 |= 0x40;

  U0CTL &= ~0x01;


  IFG1 = 0;
}

# 47 "msp430/HplUsartControl.nc"
inline static void ExtFlashP__UsartControl__setModeSPI(void ){
#line 47
  HplUsart0C__HplUsartControl__setModeSPI();
#line 47
}
#line 47
# 76 "telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x001D");

#line 76
  r |= 1 << 7;
}

#line 75
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 75
{
#line 75
  static volatile uint8_t r __asm ("0x001E");

#line 75
  r |= 1 << 4;
}

#line 76
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 76
{
#line 76
  static volatile uint8_t r __asm ("0x001E");

#line 76
  r |= 1 << 7;
}

# 52 "stm25p/ExtFlashP.nc"
static inline error_t ExtFlashP__Init__init(void )
#line 52
{
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  ExtFlashP__UsartControl__setModeSPI();
  return SUCCESS;
}

# 62 "/opt/retasking-wsn-tinyos/tos/interfaces/Init.nc"
inline static error_t TosBootP__SubInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = ExtFlashP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 69 "/opt/retasking-wsn-tinyos/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 45 "Leds.nc"
inline static void PowerOffC__Leds__glow(uint8_t a, uint8_t b){
#line 45
  LedsC__Leds__glow(a, b);
#line 45
}
#line 45
# 105 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
inline static error_t PowerOffC__SubControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = ExtFlashP__StdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 55 "msp430f1611/PowerOffC.nc"
static inline void PowerOffC__haltsystem(void )
#line 55
{

  uint16_t _lpmreg;

  TOSH_SET_PIN_DIRECTIONS();

  PowerOffC__SubControl__stop();

  PowerOffC__Leds__glow(0x7, 0x0);

  _lpmreg = 0x0080 + 0x0040 + 0x0020 + 0x0010;
  _lpmreg |= 0x0008;

   __asm volatile ("bis  %0, r2" :  : "m"((uint16_t )_lpmreg));}

# 71 "telosb/hardware.h"
static inline  uint8_t TOSH_READ_USERINT_PIN()
#line 71
{
#line 71
  static volatile uint8_t r __asm ("0x0028");

#line 71
  return r & (1 << 7);
}

#line 52
static inline void wait(uint16_t t)
#line 52
{
  for (; t > 0; t--) {
      __delay_cycles(0);
    }
}

# 76 "msp430f1611/PowerOffC.nc"
static inline error_t PowerOffC__StdControl__start(void )
#line 76
{

  int i;


  for (i = 0; i < 4; i++) 
    wait(0xffff);


  if (!TOSH_READ_USERINT_PIN()) {
    PowerOffC__haltsystem();
    }
  return SUCCESS;
}

# 60 "stm25p/ExtFlashP.nc"
static inline error_t ExtFlashP__StdControl__start(void )
#line 60
{
  return SUCCESS;
}

# 95 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
inline static error_t TosBootP__SubControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = ExtFlashP__StdControl__start();
#line 95
  __nesc_result = ecombine(__nesc_result, PowerOffC__StdControl__start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 108 "msp430/HplUsart0C.nc"
static inline error_t HplUsart0C__HplUsartControl__isTxEmpty(void )
#line 108
{
  if (U0TCTL & 0x01) {
      return SUCCESS;
    }
  return FAIL;
}

# 50 "msp430/HplUsartControl.nc"
inline static error_t ExtFlashP__UsartControl__isTxEmpty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = HplUsart0C__HplUsartControl__isTxEmpty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 47 "msp430/HplUsart0C.nc"
static inline void HplUsart0C__HplUsartControl__disableSPI(void )
#line 47
{




  ME1 = 0;
  U0CTL = 1;
  U0TCTL = 1;
  U0RCTL = 0;
}

# 46 "msp430/HplUsartControl.nc"
inline static void ExtFlashP__UsartControl__disableSPI(void ){
#line 46
  HplUsart0C__HplUsartControl__disableSPI();
#line 46
}
#line 46
# 60 "telosb/hardware.h"
static inline  void TOSH_CLR_GREEN_LED_PIN()
#line 60
{
#line 60
  static volatile uint8_t r __asm ("0x0031");

#line 60
  r &= ~(1 << 5);
}

#line 60
static inline  void TOSH_SET_GREEN_LED_PIN()
#line 60
{
#line 60
  static volatile uint8_t r __asm ("0x0031");

#line 60
  r |= 1 << 5;
}

#line 61
static inline  void TOSH_CLR_YELLOW_LED_PIN()
#line 61
{
#line 61
  static volatile uint8_t r __asm ("0x0031");

#line 61
  r &= ~(1 << 6);
}

#line 61
static inline  void TOSH_SET_YELLOW_LED_PIN()
#line 61
{
#line 61
  static volatile uint8_t r __asm ("0x0031");

#line 61
  r |= 1 << 6;
}

#line 59
static inline  void TOSH_CLR_RED_LED_PIN()
#line 59
{
#line 59
  static volatile uint8_t r __asm ("0x0031");

#line 59
  r &= ~(1 << 4);
}

#line 59
static inline  void TOSH_SET_RED_LED_PIN()
#line 59
{
#line 59
  static volatile uint8_t r __asm ("0x0031");

#line 59
  r |= 1 << 4;
}

# 47 "msp430/ExecC.nc"
static inline void ExecC__Exec__exec(void )
#line 47
{

  typedef void __attribute((noreturn)) (*tosboot_exec)(void );



  WDTCTL = 0x5A00 + 0x0008;
  (
  (tosboot_exec )0x4a00)();
}

# 3 "Exec.nc"
inline static void TosBootP__Exec__exec(void ){
#line 3
  ExecC__Exec__exec();
#line 3
}
#line 3
# 215 "TosBootP.nc"
static inline void TosBootP__runApp(void )
#line 215
{
  TosBootP__SubControl__stop();
  TosBootP__Exec__exec();
}

# 68 "/opt/retasking-wsn-tinyos/tos/interfaces/InternalFlash.nc"
inline static error_t TosBootP__IntFlash__write(void *addr, void * buf, uint16_t size){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = InternalFlashC__InternalFlash__write(addr, buf, size);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 56 "msp430/HardwareC.nc"
static inline void HardwareC__Hardware__reboot(void )
#line 56
{
  WDTCTL = 0;
}

# 44 "Hardware.nc"
inline static void TosBootP__Hardware__reboot(void ){
#line 44
  HardwareC__Hardware__reboot();
#line 44
}
#line 44
# 44 "Leds.nc"
inline static void TosBootP__Leds__flash(uint8_t a){
#line 44
  LedsC__Leds__flash(a);
#line 44
}
#line 44
# 119 "msp430/InternalFlashC.nc"
static inline error_t InternalFlashC__InternalFlash__read(void *addr, void *buf, uint16_t size)
#line 119
{

  addr += InternalFlashC__IFLASH_OFFSET;
  if (InternalFlashC__chooseSegment()) {
    addr += InternalFlashC__IFLASH_SIZE;
    }
  memcpy(buf, addr, size);

  return SUCCESS;
}

# 56 "/opt/retasking-wsn-tinyos/tos/interfaces/InternalFlash.nc"
inline static error_t TosBootP__IntFlash__read(void *addr, void * buf, uint16_t size){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = InternalFlashC__InternalFlash__read(addr, buf, size);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 54 "msp430/VoltageC.nc"
static inline bool VoltageC__Voltage__okToProgram(void )
#line 54
{

  int i;


  ADC12CTL0 = (0x010 | 0x0200) | 0x020;

  ADC12CTL1 = 0x0200;

  ADC12MCTL0 = (0x0080 | 0x0010) | 11;

  for (i = 0; i < 0x3600; i++) ;


  ADC12CTL0 |= 0x002;

  ADC12CTL0 |= 0x001;

  while ((ADC12IFG & 0x0001) == 0) ;


  ADC12CTL0 &= ~0x002;
  ADC12CTL0 = 0;


  return ADC12MEM0 > VoltageC__VTHRESH;
}

# 43 "Voltage.nc"
inline static bool TosBootP__Voltage__okToProgram(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = VoltageC__Voltage__okToProgram();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 220 "TosBootP.nc"
static inline void TosBootP__startupSequence(void )
#line 220
{

  BootArgs args;




  if (!TosBootP__Voltage__okToProgram()) {

      TosBootP__Leds__flash(TosBootP__LEDS_LOWBATT);
      TosBootP__startupLeds();
      TosBootP__runApp();
    }


  TosBootP__IntFlash__read((uint8_t *)TOSBOOT_ARGS_ADDR, &args, sizeof args);


  if (++ args.gestureCount >= TOSBOOT_GESTURE_MAX_COUNT - 1) {

      TosBootP__Leds__flash(TosBootP__LEDS_GESTURE);





      if (TosBootP__programImage(TOSBOOT_GOLDEN_IMG_ADDR) == TosBootP__R_PROGRAMMING_ERROR) {
          TosBootP__Hardware__reboot();
        }
    }
  else {

      TosBootP__IntFlash__write((uint8_t *)TOSBOOT_ARGS_ADDR, &args, sizeof args);
      if (! args.noReprogram) {


          if (TosBootP__programImage(args.imageAddr) == TosBootP__R_PROGRAMMING_ERROR) {
              TosBootP__Hardware__reboot();
            }
        }
    }


  TosBootP__startupLeds();


  args.gestureCount = 0xff;
  args.noReprogram = TRUE;
  TosBootP__IntFlash__write((uint8_t *)TOSBOOT_ARGS_ADDR, &args, sizeof args);

  TosBootP__runApp();
}

# 45 "Leds.nc"
inline static void TosBootP__Leds__glow(uint8_t a, uint8_t b){
#line 45
  LedsC__Leds__glow(a, b);
#line 45
}
#line 45
# 92 "msp430f1611/PowerOffC.nc"
static inline error_t PowerOffC__StdControl__stop(void )
#line 92
{
  return SUCCESS;
}

# 75 "telosb/hardware.h"
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 75
{
#line 75
  static volatile uint8_t r __asm ("0x001D");

#line 75
  r |= 1 << 4;
}

# 121 "stm25p/ExtFlashP.nc"
static inline void ExtFlashP__ExtFlash__stopRead(void )
#line 121
{
  TOSH_SET_FLASH_CS_PIN();
}

# 45 "ExtFlash.nc"
inline static void TosBootP__ExtFlash__stopRead(void ){
#line 45
  ExtFlashP__ExtFlash__stopRead();
#line 45
}
#line 45
#line 44
inline static uint8_t TosBootP__ExtFlash__readByte(void ){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = ExtFlashP__ExtFlash__readByte();
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
#line 43
inline static void TosBootP__ExtFlash__startRead(uint32_t addr){
#line 43
  ExtFlashP__ExtFlash__startRead(addr);
#line 43
}
#line 43
# 109 "TosBootP.nc"
static inline bool TosBootP__verifyImage(ex_flash_addr_t startAddr)
#line 109
{
  uint32_t addr;
  uint8_t numPgs;
  uint8_t i;


  if (!TosBootP__verifyBlock(startAddr + (unsigned short )& ((DelugeIdent *)0)->crc, 
  startAddr, (unsigned short )& ((DelugeIdent *)0)->crc)) {
    return FALSE;
    }

  TosBootP__ExtFlash__startRead(startAddr + (unsigned short )& ((DelugeIdent *)0)->numPgs);
  numPgs = TosBootP__ExtFlash__readByte();
  TosBootP__ExtFlash__stopRead();

  if (numPgs == 0 || numPgs == 0xff) {
    return FALSE;
    }
  startAddr += DELUGE_IDENT_SIZE;
  addr = DELUGE_CRC_BLOCK_SIZE;

  for (i = 0; i < numPgs; i++) {
      if (!TosBootP__verifyBlock(startAddr + i * sizeof(uint16_t ), 
      startAddr + addr, DELUGE_BYTES_PER_PAGE)) {
          return FALSE;
        }
      addr += DELUGE_BYTES_PER_PAGE;
    }

  return TRUE;
}

# 51 "msp430/HplUsartControl.nc"
inline static error_t ExtFlashP__UsartControl__isTxIntrPending(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplUsart0C__HplUsartControl__isTxIntrPending();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 131 "msp430/HplUsart0C.nc"
static inline void HplUsart0C__HplUsartControl__tx(uint8_t data)
#line 131
{
  U0TXBUF = data;
}

# 53 "msp430/HplUsartControl.nc"
inline static void ExtFlashP__UsartControl__tx(uint8_t data){
#line 53
  HplUsart0C__HplUsartControl__tx(data);
#line 53
}
#line 53
# 75 "telosb/hardware.h"
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 75
{
#line 75
  static volatile uint8_t r __asm ("0x001D");

#line 75
  r &= ~(1 << 4);
}

# 79 "stm25p/ExtFlashP.nc"
static inline void ExtFlashP__powerOnFlash(void )
#line 79
{

  uint8_t i;

  TOSH_CLR_FLASH_CS_PIN();


  for (i = 0; i < 5; i++) {
      ExtFlashP__UsartControl__tx(0xab);
      while (ExtFlashP__UsartControl__isTxIntrPending() != SUCCESS) ;
    }

  TOSH_SET_FLASH_CS_PIN();
}

# 135 "msp430/HplUsart0C.nc"
static inline uint8_t HplUsart0C__HplUsartControl__rx(void )
#line 135
{
  return U0RXBUF;
}

# 54 "msp430/HplUsartControl.nc"
inline static uint8_t ExtFlashP__UsartControl__rx(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HplUsart0C__HplUsartControl__rx();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 123 "msp430/HplUsart0C.nc"
static inline error_t HplUsart0C__HplUsartControl__isRxIntrPending(void )
#line 123
{
  if (IFG1 & 0x40) {
      IFG1 &= ~0x40;
      return SUCCESS;
    }
  return FAIL;
}

# 52 "msp430/HplUsartControl.nc"
inline static error_t ExtFlashP__UsartControl__isRxIntrPending(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = HplUsart0C__HplUsartControl__isRxIntrPending();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 53 "crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b)
{
  uint8_t i;

  crc = crc ^ (b << 8);
  i = 8;
  do 
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
      }
    else {
#line 63
      crc = crc << 1;
      }
  while (
#line 64
  --i);

  return crc;
}

# 43 "Leds.nc"
inline static void TosBootP__Leds__set(uint8_t ledsOn){
#line 43
  LedsC__Leds__set(ledsOn);
#line 43
}
#line 43
# 54 "msp430/ProgFlashC.nc"
static inline error_t ProgFlashC__ProgFlash__write(in_flash_addr_t addr, uint8_t *buf, uint16_t len)
#line 54
{

  volatile uint16_t *flashAddr = (uint16_t *)(uint16_t )addr;
  uint16_t *wordBuf = (uint16_t *)buf;
  uint16_t i = 0;



  if (addr < 0xffff - (len >> 1)) {
      FCTL2 = 0xA500 + 0x0080 + 0x0004;
      FCTL3 = 0xA500;
      FCTL1 = 0xA500 + 0x0002;
      *flashAddr = 0;
      FCTL1 = 0xA500 + 0x0040;
      for (i = 0; i < (len >> 1); i++, flashAddr++) {
          if ((uint16_t )flashAddr != ProgFlashC__RESET_ADDR) {
            *flashAddr = wordBuf[i];
            }
          else {
#line 72
            *flashAddr = 0x4000;
            }
        }
#line 74
      FCTL1 = 0xA500;
      FCTL3 = 0xA500 + 0x0010;
      return SUCCESS;
    }
  return FAIL;
}

# 43 "ProgFlash.nc"
inline static error_t TosBootP__ProgFlash__write(in_flash_addr_t addr, uint8_t *buf, in_flash_addr_t len){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = ProgFlashC__ProgFlash__write(addr, buf, len);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 397 "/opt/retasking-wsn-tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 274 "TosBootP.nc"
  int main(void )
#line 274
{

  __nesc_disable_interrupt();

  TOSH_SET_PIN_DIRECTIONS();
  TosBootP__Hardware__init();

  TosBootP__SubInit__init();
  TosBootP__SubControl__start();

  TosBootP__startupSequence();

  return 0;
}

# 78 "telosb/hardware.h"
static void TOSH_SET_PIN_DIRECTIONS(void )
{
  P3SEL = 0x0E;

  P1DIR = 0xe0;
  P1OUT = 0x00;

  P2DIR = 0x7b;
  P2OUT = 0x10;

  P3DIR = 0xf1;
  P3OUT = 0x00;

  P4DIR = 0xfd;
  P4OUT = 0xdd;

  P5DIR = 0xff;
  P5OUT = 0xff;

  P6DIR = 0xff;
  P6OUT = 0x00;
}

# 64 "stm25p/ExtFlashP.nc"
static error_t ExtFlashP__StdControl__stop(void )
#line 64
{

  TOSH_CLR_FLASH_CS_PIN();

  ExtFlashP__UsartControl__tx(0xb9);
  while (ExtFlashP__UsartControl__isTxEmpty() != SUCCESS) ;

  TOSH_SET_FLASH_CS_PIN();

  ExtFlashP__UsartControl__disableSPI();

  return SUCCESS;
}

# 81 "lib/LedsC.nc"
static void LedsC__Leds__glow(uint8_t a, uint8_t b)
#line 81
{
  int i;

#line 83
  for (i = 1536; i > 0; i -= 4) {
      LedsC__Leds__set(a);
      wait(i);
      LedsC__Leds__set(b);
      wait(1536 - i);
    }
}

#line 54
static void LedsC__Leds__set(uint8_t ledsOn)
#line 54
{
  if (ledsOn & LedsC__GREEN_BIT) {
    TOSH_CLR_GREEN_LED_PIN();
    }
  else {
#line 58
    TOSH_SET_GREEN_LED_PIN();
    }
#line 59
  if (ledsOn & LedsC__YELLOW_BIT) {
    TOSH_CLR_YELLOW_LED_PIN();
    }
  else {
#line 62
    TOSH_SET_YELLOW_LED_PIN();
    }
#line 63
  if (ledsOn & LedsC__RED_BIT) {
    TOSH_CLR_RED_LED_PIN();
    }
  else {
#line 66
    TOSH_SET_RED_LED_PIN();
    }
}

#line 69
static void LedsC__Leds__flash(uint8_t a)
#line 69
{
  uint8_t i;
#line 70
  uint8_t j;

#line 71
  for (i = 3; i; i--) {
      LedsC__Leds__set(a);
      for (j = 4; j; j--) 
        wait(0xffff);
      LedsC__Leds__set(0);
      for (j = 4; j; j--) 
        wait(0xffff);
    }
}

# 72 "TosBootP.nc"
static void TosBootP__startupLeds(void )
#line 72
{

  uint8_t output = 0x7;
  uint8_t i;

  for (i = 3; i; i--, output >>= 1) 
    TosBootP__Leds__glow(output, output >> 1);
}

# 105 "/opt/retasking-wsn-tinyos/tos/interfaces/StdControl.nc"
static error_t TosBootP__SubControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = ExtFlashP__StdControl__stop();
#line 105
  __nesc_result = ecombine(__nesc_result, PowerOffC__StdControl__stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 67 "msp430/InternalFlashC.nc"
static uint8_t InternalFlashC__chooseSegment(void )
#line 67
{
  int8_t vnum0 = * (int8_t *)InternalFlashC__IFLASH_SEG0_VNUM_ADDR;
  int8_t vnum1 = * (int8_t *)InternalFlashC__IFLASH_SEG1_VNUM_ADDR;

#line 70
  if (vnum0 == InternalFlashC__IFLASH_INVALID_VNUM) {
    return 1;
    }
  else {
#line 72
    if (vnum1 == InternalFlashC__IFLASH_INVALID_VNUM) {
      return 0;
      }
    }
#line 74
  return (int8_t )(vnum0 - vnum1) < 0;
}

# 141 "TosBootP.nc"
static error_t TosBootP__programImage(ex_flash_addr_t startAddr)
#line 141
{
  uint8_t buf[TOSBOOT_INT_PAGE_SIZE];
  uint32_t pageAddr;
#line 143
  uint32_t newPageAddr;
  in_flash_addr_t intAddr;
  in_flash_addr_t secLength;
  ex_flash_addr_t curAddr;

  if (!TosBootP__verifyImage(startAddr)) {
    return TosBootP__R_INVALID_IMAGE_ERROR;
    }
  curAddr = startAddr + DELUGE_IDENT_SIZE + DELUGE_CRC_BLOCK_SIZE;

  TosBootP__ExtFlash__startRead(curAddr);

  intAddr = TosBootP__extFlashReadAddr();
  secLength = TosBootP__extFlashReadAddr();
  curAddr = curAddr + 8;


  if (intAddr != 0x4a00) {







      TosBootP__ExtFlash__stopRead();
      return TosBootP__R_INVALID_IMAGE_ERROR;
    }

  TosBootP__ExtFlash__stopRead();

  while (secLength) {

      pageAddr = newPageAddr = intAddr / TOSBOOT_INT_PAGE_SIZE;

      TosBootP__ExtFlash__startRead(curAddr);

      do {


          if (secLength == 0xffffffff) {
              TosBootP__ExtFlash__stopRead();
              return FAIL;
            }

          buf[(uint16_t )intAddr % TOSBOOT_INT_PAGE_SIZE] = TosBootP__ExtFlash__readByte();
          intAddr++;
#line 189
          curAddr++;

          if (--secLength == 0) {
              intAddr = TosBootP__extFlashReadAddr();
              secLength = TosBootP__extFlashReadAddr();
              curAddr = curAddr + 8;
            }

          newPageAddr = intAddr / TOSBOOT_INT_PAGE_SIZE;
        }
      while (pageAddr == newPageAddr && secLength);
      TosBootP__ExtFlash__stopRead();

      TosBootP__Leds__set(pageAddr);



      if (
#line 205
      TosBootP__ProgFlash__write(pageAddr * TOSBOOT_INT_PAGE_SIZE, buf, 
      TOSBOOT_INT_PAGE_SIZE) == FAIL) {
          return TosBootP__R_PROGRAMMING_ERROR;
        }
    }

  return TosBootP__R_SUCCESS;
}

#line 90
static bool TosBootP__verifyBlock(ex_flash_addr_t crcAddr, ex_flash_addr_t startAddr, uint16_t len)
{
  uint16_t crcTarget;
#line 92
  uint16_t crcTmp;


  TosBootP__ExtFlash__startRead(crcAddr);
  crcTarget = (uint16_t )(TosBootP__ExtFlash__readByte() & 0xff) << 8;
  crcTarget |= (uint16_t )(TosBootP__ExtFlash__readByte() & 0xff);
  TosBootP__ExtFlash__stopRead();


  TosBootP__ExtFlash__startRead(startAddr);
  for (crcTmp = 0; len; len--) 
    crcTmp = crcByte(crcTmp, TosBootP__ExtFlash__readByte());
  TosBootP__ExtFlash__stopRead();

  return crcTarget == crcTmp;
}

# 95 "stm25p/ExtFlashP.nc"
static void ExtFlashP__ExtFlash__startRead(uint32_t addr)
#line 95
{

  uint8_t i;

  ExtFlashP__powerOnFlash();

  TOSH_CLR_FLASH_CS_PIN();


  addr |= (uint32_t )0x3 << 24;


  for (i = 4; i > 0; i--) {
      ExtFlashP__UsartControl__tx((addr >> (i - 1) * 8) & 0xff);
      while (ExtFlashP__UsartControl__isTxIntrPending() != SUCCESS) ;
    }
}

# 115 "msp430/HplUsart0C.nc"
static error_t HplUsart0C__HplUsartControl__isTxIntrPending(void )
#line 115
{
  if (IFG1 & 0x80) {
      IFG1 &= ~0x80;
      return SUCCESS;
    }
  return FAIL;
}

# 114 "stm25p/ExtFlashP.nc"
static uint8_t ExtFlashP__ExtFlash__readByte(void )
#line 114
{
  ExtFlashP__UsartControl__rx();
  ExtFlashP__UsartControl__tx(0);
  while (ExtFlashP__UsartControl__isRxIntrPending() != SUCCESS) ;
  return ExtFlashP__UsartControl__rx();
}

# 82 "TosBootP.nc"
static in_flash_addr_t TosBootP__extFlashReadAddr(void )
#line 82
{
  in_flash_addr_t result = 0;
  int8_t i;

#line 85
  for (i = 3; i >= 0; i--) 
    result |= ((in_flash_addr_t )TosBootP__ExtFlash__readByte() & 0xff) << i * 8;
  return result;
}

# 77 "msp430/InternalFlashC.nc"
static error_t InternalFlashC__InternalFlash__write(void *addr, void *buf, uint16_t size)
#line 77
{

  volatile int8_t *newPtr;
  int8_t *oldPtr;
  int8_t *bufPtr = (int8_t *)buf;
  int8_t version;
  uint16_t i;

  addr += InternalFlashC__IFLASH_OFFSET;
  newPtr = oldPtr = (int8_t *)InternalFlashC__IFLASH_OFFSET;
  if (InternalFlashC__chooseSegment()) {
      oldPtr += InternalFlashC__IFLASH_SIZE;
    }
  else {
      addr += InternalFlashC__IFLASH_SIZE;
      newPtr += InternalFlashC__IFLASH_SIZE;
    }

  FCTL2 = 0xA500 + 0x0080 + 0x0004;
  FCTL3 = 0xA500;
  FCTL1 = 0xA500 + 0x0002;
  *newPtr = 0;
  FCTL1 = 0xA500 + 0x0040;

  for (i = 0; i < InternalFlashC__IFLASH_SIZE - 1; i++, newPtr++, oldPtr++) {
      if ((uint16_t )newPtr < (uint16_t )addr || (uint16_t )addr + size <= (uint16_t )newPtr) {
        *newPtr = *oldPtr;
        }
      else {
#line 105
        *newPtr = * bufPtr++;
        }
    }
#line 107
  version = *oldPtr + 1;
  if (version == InternalFlashC__IFLASH_INVALID_VNUM) {
    version++;
    }
#line 110
  *newPtr = version;

  FCTL1 = 0xA500;
  FCTL3 = 0xA500 + 0x0010;

  return SUCCESS;
}

