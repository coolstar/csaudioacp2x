/*++

Copyright (c) Microsoft Corporation All Rights Reserved

Module Name:

    hw.h

Abstract:

    Declaration of Simple Audio Sample HW class. 
    Simple Audio Sample HW has an array for storing mixer and volume settings
    for the topology.
--*/

#ifndef _CSAUDIOACP2X_HW_H_
#define _CSAUDIOACP2X_HW_H_

typedef enum {
    CSAudioEndpointTypeDSP,
    CSAudioEndpointTypeSpeaker,
    CSAudioEndpointTypeHeadphone,
    CSAudioEndpointTypeMicArray,
    CSAudioEndpointTypeMicJack
} CSAudioEndpointType;

typedef enum {
    CSAudioEndpointRegister,
    CSAudioEndpointStart,
    CSAudioEndpointStop,
    CSAudioEndpointOverrideFormat
} CSAudioEndpointRequest;

typedef struct CSAUDIOFORMATOVERRIDE {
    UINT16 channels;
    UINT16 frequency;
    UINT16 bitsPerSample;
    UINT16 validBitsPerSample;
    BOOL force32BitOutputContainer;
} CsAudioFormatOverride;

typedef struct CSAUDIOARG {
    UINT32 argSz;
    CSAudioEndpointType endpointType;
    CSAudioEndpointRequest endpointRequest;
    union {
        CsAudioFormatOverride formatOverride;
    };
} CsAudioArg, * PCsAudioArg;

#define USEACPHW 1

#if USEACPHW
#include "acp2x.h"
#include "dw_i2s.h"
#define BIT(nr) (1UL << (nr))

union baseaddr {
    PVOID Base;
    UINT8* baseptr;
};

typedef struct _PCI_BAR {
    union baseaddr Base;
    ULONG Len;
} PCI_BAR, * PPCI_BAR;
#endif

//=============================================================================
// Defines
//=============================================================================
// BUGBUG we should dynamically allocate this...
#define MAX_TOPOLOGY_NODES      20

//=============================================================================
// Classes
//=============================================================================
///////////////////////////////////////////////////////////////////////////////
// CCsAudioAcp2xHW
// This class represents virtual Simple Audio Sample HW. An array representing volume
// registers and mute registers.

class CCsAudioAcp2xHW
{
public:
    int CsAudioArg2;
    void CSAudioAPICalled(CsAudioArg arg);

private:
    PCALLBACK_OBJECT CSAudioAPICallback;
    PVOID CSAudioAPICallbackObj;

    NTSTATUS CSAudioAPIInit();
    NTSTATUS CSAudioAPIDeinit();
    CSAudioEndpointType GetCSAudioEndpoint(eDeviceType deviceType);
    eDeviceType GetDeviceType(CSAudioEndpointType endpointType);

protected:
    LONG                        m_PeakMeterControls[MAX_TOPOLOGY_NODES];
    ULONG                       m_ulMux;            // Mux selection
    BOOL                        m_bDevSpecific;
    INT                         m_iDevSpecific;
    UINT                        m_uiDevSpecific;
#if USEACPHW
    PCI_BAR m_BAR6;
    UINT32 m_pme_en;

    SHORT bt_running_streams;
    SHORT sp_running_streams;

    UINT32 cgs_read32(UINT32 reg);
    void cgs_write32(UINT32 reg, UINT32 val);
#endif

private:
#if USEACPHW
    NTSTATUS acp2x_power_on();
    NTSTATUS acp2x_power_off();
#endif
public:
    CCsAudioAcp2xHW(_In_  PRESOURCELIST           ResourceList);
    ~CCsAudioAcp2xHW();

    bool                        ResourcesValidated();

    NTSTATUS acp2x_init();
    NTSTATUS acp2x_deinit();

    NTSTATUS acp3x_hw_params(eDeviceType deviceType);
    NTSTATUS acp3x_program_dma(eDeviceType deviceType, PMDL mdl, IPortWaveRTStream* stream);
    NTSTATUS acp3x_play(eDeviceType deviceType, UINT32 byteCount);
    NTSTATUS acp3x_stop(eDeviceType deviceType);
    NTSTATUS acp3x_current_position(eDeviceType deviceType, UINT32* linkPos, UINT64* linearPos);
    
    void                        MixerReset();
    BOOL                        bGetDevSpecific();
    void                        bSetDevSpecific
    (
        _In_  BOOL                bDevSpecific
    );
    INT                         iGetDevSpecific();
    void                        iSetDevSpecific
    (
        _In_  INT                 iDevSpecific
    );
    UINT                        uiGetDevSpecific();
    void                        uiSetDevSpecific
    (
        _In_  UINT                uiDevSpecific
    );
    ULONG                       GetMixerMux();
    void                        SetMixerMux
    (
        _In_  ULONG               ulNode
    );
    
    LONG                        GetMixerPeakMeter
    (   
        _In_  ULONG               ulNode,
        _In_  ULONG               ulChannel
    );

protected:
private:
};
typedef CCsAudioAcp2xHW    *PCCsAudioAcp2xHW;

#endif  // _CSAUDIOACP2X_HW_H_
