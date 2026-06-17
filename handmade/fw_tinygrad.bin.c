#define 0x7 0x7
#define 2 0x2
#define 0x21 0x21
#define " RRE" 0x20525245

typedef unsigned char   undefined;

typedef unsigned char    bool;
typedef unsigned char    byte;
typedef unsigned int    dword;
typedef unsigned char    uchar;
typedef unsigned int    uint;
typedef unsigned char    undefined1;
typedef unsigned short    undefined2;
typedef unsigned int    undefined3;
typedef unsigned int    undefined4;
typedef unsigned short    ushort;
typedef unsigned short    word;
typedef struct CaseEntry CaseEntry, *PCaseEntry;

struct CaseEntry {
    byte bHandler_hi;
    byte bHandler_lo;
    byte bCase_val;
};

typedef struct CmCmdEntry CmCmdEntry, *PCmCmdEntry;

struct CmCmdEntry {
    byte bHandler_hi;
    byte bHandler_lo;
    char pKey[4];
};

typedef enum CmPcieLinkStep {
    STEP_05=5,
    STEP_0A=10,
    STEP_14=20,
    STEP_15=21,
    STEP_18=24,
    STEP_1C=28,
    STEP_1D=29
} CmPcieLinkStep;

typedef enum PdMsgType {
    PD_Source_Capabilities=1,
    PD_Request=2,
    PD_BIST=3,
    PD_Sink_Capabilities=4,
    PD_Battery_Status=5,
    PD_Enter_USB=8,
    PD_Vendor_Defined=15
} PdMsgType;

typedef enum Usb4RouterOpPath {
    ROUTEROP_PATH_E0=224,
    ROUTEROP_PATH_E1=225,
    ROUTEROP_CONFIG=226,
    ROUTEROP_PATH_E3=227,
    ROUTEROP_PATH_E4=228,
    ROUTEROP_PATH_E5=229,
    ROUTEROP_TUNNEL_RESET=232
} Usb4RouterOpPath;

typedef enum Usb4ModeEntryReqCode {
    STATE_01=1,
    STATE_03=3
} Usb4ModeEntryReqCode;

typedef unsigned short    wchar16;
typedef enum Usb4RouterCfgOp {
    ROUTER_CFG_READ=80,
    ROUTER_CFG_WRITE=81
} Usb4RouterCfgOp;

typedef enum Usb4LinkEventStatus {
    STATE_01=1,
    STATE_18=24,
    STATE_1E=30
} Usb4LinkEventStatus;

typedef enum CmPcieLinkState {
    STATE_00=0,
    STATE_01=1,
    STATE_02=2,
    STATE_03=3,
    STATE_04=4,
    STATE_05=5,
    STATE_0F=15,
    STATE_12=18,
    STATE_13=19
} CmPcieLinkState;

typedef enum Usb4RouterOpState {
    ROUTEROP_IDLE=0,
    ROUTEROP_MULTIPKT_1=1,
    ROUTEROP_MULTIPKT_2=2
} Usb4RouterOpState;

typedef enum Usb4ModeEntryState {
    STATE_01=1,
    STATE_05=5,
    STATE_0D=13
} Usb4ModeEntryState;

typedef struct CaseTableTail CaseTableTail, *PCaseTableTail;

struct CaseTableTail {
    word wSentinel;
    byte bDefault_hi;
    byte bDefault_lo;
};

typedef enum Usb4LinkState {
    STATE_01=1,
    STATE_05=5,
    STATE_10=16,
    STATE_11=17
} Usb4LinkState;

typedef uint uint32_t;



