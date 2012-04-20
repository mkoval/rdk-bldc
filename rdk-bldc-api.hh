#ifndef RDK_BLDC_API_HH_
#define RDK_BLDC_API_HH_

struct Command {
    typedef enum {
        kIdTarget        = 0x00,
        kUpgrade         = 0x01,
        kDiscoverTarget  = 0x02,
        kGetParams       = 0x10,
        kGetParamDesc    = 0x11,
        kGetParamValue   = 0x12,
        kSetParamValue   = 0x13,
        kLoadParams      = 0x14,
        kSaveParams      = 0x15,
        kGetDataItems    = 0x20,
        kEnableDataItem  = 0x21,
        kDisableDataItem = 0x22,
        kStartDataStream = 0x23,
        kStopDataStream  = 0x24,
        kRun             = 0x30,
        kStop            = 0x31,
        kEmergencyStop   = 0x32
    } Enum;
};

struct Param {
    typedef enum {
        kTargetSpeed = 0x04
    } Enum;
};

#endif
