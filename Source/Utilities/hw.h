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
#include "acp.h"
#include "acp2x.h"
#include "dw_i2s.h"

#define BITS_PER_LONG sizeof(LONG) * 8
#define GENMASK(h, l) (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#define BIT(nr) (1UL << (nr))

union baseaddr {
    PVOID Base;
    UINT8* baseptr;
};

typedef struct _PCI_BAR {
    union baseaddr Base;
    ULONG Len;
} PCI_BAR, * PPCI_BAR;

struct acp_stream {
    UINT32 pte_offset;
    UINT16 ch1;
    UINT16 ch2;
    UINT32 sram_bank;
    UINT16 destination;
    UINT16 dma_dscr_idx_1;
    UINT16 dma_dscr_idx_2;
    UINT32 byte_cnt_high_reg_offset;
    UINT32 byte_cnt_low_reg_offset;
    UINT32 dma_curr_dscr;
};

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

    struct acp_stream btStreams[2];
    struct acp_stream i2sStreams[2];

    void udelay(ULONG usec);

    UINT32 cgs_read32(UINT32 reg);
    void cgs_write32(UINT32 reg, UINT32 val);

    UINT32 acp_read32(UINT32 reg);
    void acp_write32(UINT32 reg, UINT32 val);

    UINT32 i2s_read32(UINT32 i2s_base, UINT32 reg);
    void i2s_write32(UINT32 i2s_base, UINT32 reg, UINT32 val);

    NTSTATUS acp_readl_poll_timeout(UINT32 reg, UINT32 val, UINT32 mask, ULONG sleep_us, ULONG timeout_us);
#endif

private:
#if USEACPHW
    NTSTATUS acp2x_power_on();
    NTSTATUS acp2x_power_off();

    struct acp_stream* acp_get_stream(eDeviceType deviceType);

    void acp_set_sram_bank_state(UINT16 bank, bool power_on);

    void config_acp_dma_channel(UINT16 ch_num, UINT16 dscr_strt_idx,
        UINT16 num_dscrs, enum acp_dma_priority_level priority_level);
    void config_dma_descriptor_in_sram(UINT16 descr_idx,
        acp_dma_dscr_transfer_t* descr_info);
    void pre_config_reset(UINT16 ch_num);
    void set_acp_sysmem_dma_descriptors(UINT32 size, BOOL playback, UINT32 pte_offset, UINT16 ch,
        UINT32 sram_bank, UINT16 dma_dscr_idx);
    void set_acp_to_i2s_dma_descriptors(UINT32 size, BOOL playback, UINT32 sram_bank,
        UINT32 destination, UINT16 ch, UINT16 dma_dscr_idx);

    void acp_dma_start(UINT16 ch_num, BOOL isCircular);
    NTSTATUS acp_dma_stop(UINT16 ch_num);
#endif
public:
    CCsAudioAcp2xHW(_In_  PRESOURCELIST           ResourceList);
    ~CCsAudioAcp2xHW();

    bool                        ResourcesValidated();

    NTSTATUS acp2x_init();
    NTSTATUS acp2x_deinit();

    NTSTATUS acp2x_hw_params(eDeviceType deviceType);
    NTSTATUS acp2x_program_dma(eDeviceType deviceType, PMDL mdl, IPortWaveRTStream* stream);
    NTSTATUS acp2x_play(eDeviceType deviceType);
    NTSTATUS acp2x_stop(eDeviceType deviceType);
    NTSTATUS acp2x_current_position(eDeviceType deviceType, UINT32* linkPos, UINT64* linearPos);
    
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
