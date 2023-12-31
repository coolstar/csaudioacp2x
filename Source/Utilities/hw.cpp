/*++

Copyright (c)  Microsoft Corporation All Rights Reserved

Module Name:

    hw.cpp

Abstract:

    Implementation of Simple Audio Sample HW class. 
    Simple Audio Sample HW has an array for storing mixer and volume settings
    for the topology.
--*/
#include "definitions.h"
#include "hw.h"

UINT32 pciConfigReadDWord(UINT8 bus, UINT8 slot, UINT8 func, UINT8 offset) {
    UINT32 address;
    UINT32 lbus = (UINT32)bus;
    UINT32 lslot = (UINT32)slot;
    UINT32 lfunc = (UINT32)func;
    UINT32 tmp = 0;

    // Create configuration address as per Figure 1
    address = (UINT32)((lbus << 16) | (lslot << 11) |
        (lfunc << 8) | (offset & 0xFC) | ((UINT32)0x80000000));

    // Write out the address
    __outdword(PCI_IO_CONFIG_INDEX, address);
    // Read in the data
    tmp = __indword(PCI_IO_CONFIG_DATA + (offset & 3));
    return tmp;
}

//=============================================================================
// CCsAudioAcp2xHW
//=============================================================================

//=============================================================================
#pragma code_seg("PAGE")
CCsAudioAcp2xHW::CCsAudioAcp2xHW(_In_  PRESOURCELIST           ResourceList)
: m_ulMux(0),
    m_bDevSpecific(FALSE),
    m_iDevSpecific(0),
    m_uiDevSpecific(0)
/*++

Routine Description:

    Constructor for CsAudioAcp2xHW. 

Arguments:

Return Value:

    void

--*/
{
    PAGED_CODE();

#if USEACPHW
    PCM_PARTIAL_RESOURCE_DESCRIPTOR partialDescriptor = (ResourceList->FindTranslatedEntry(CmResourceTypeMemory, 0));
    if (partialDescriptor) {
        m_BAR5.Base.Base = MmMapIoSpace(partialDescriptor->u.Memory.Start, partialDescriptor->u.Memory.Length, MmNonCached);
        m_BAR5.Len = partialDescriptor->u.Memory.Length;
    }
    else {
        //No _CRS found. Grab IO Manually

        UINT32 BAR5Addr = pciConfigReadDWord(0, GFX_DEV, GFX_FUNC, BAR5);

        PHYSICAL_ADDRESS start;
        start.QuadPart = BAR5Addr & 0xFFFFFFF0;
        UINT32 Length = 0x40000;

        m_BAR5.Base.Base = MmMapIoSpace(start, Length, MmNonCached);
        m_BAR5.Len = Length;
    }

    PHYSICAL_ADDRESS miscAddr;
    miscAddr.QuadPart = ACPIMMIO_MISC_BASE;
    m_MISCBAR.Base.Base = MmMapIoSpace(miscAddr, 0x100, MmNonCached);
    m_MISCBAR.Len = 0x100;
#endif
    
    MixerReset();
} // CsAudioAcp2xHW
#pragma code_seg()

bool CCsAudioAcp2xHW::ResourcesValidated() {
    if (!m_BAR5.Base.Base)
        return false;
    if (!m_MISCBAR.Base.Base)
        return false;
    return true && NT_SUCCESS(this->CSAudioAPIInit());
}

CCsAudioAcp2xHW::~CCsAudioAcp2xHW() {
#if USEACPHW
    if (m_BAR5.Base.Base)
        MmUnmapIoSpace(m_BAR5.Base.Base, m_BAR5.Len);
    if (m_MISCBAR.Base.Base)
        MmUnmapIoSpace(m_MISCBAR.Base.Base, m_MISCBAR.Len);
#endif
    this->CSAudioAPIDeinit();
}

#if USEACPHW
static UINT32 read32(PVOID addr) {
    UINT32 ret = *(UINT32*)addr;
    //DbgPrint("Read from %p: 0x%x\n", addr, ret);
    return ret;
}

static void write32(PVOID addr, UINT32 data) {
    *(UINT32*)addr = data;
    //DbgPrint("Write to %p: 0x%x\n", addr, data);
}

void CCsAudioAcp2xHW::udelay(ULONG usec) {
    LARGE_INTEGER Interval;
    Interval.QuadPart = -10 * (LONGLONG)usec;
    KeDelayExecutionThread(KernelMode, false, &Interval);
}

UINT32 CCsAudioAcp2xHW::cgs_read32(UINT32 reg)
{
    if ((reg * 4) < m_BAR5.Len) {
        return read32(m_BAR5.Base.baseptr + (reg * 4));
    }
    DbgPrint("Invalid read from cgs register: 0x%x\n", reg);
    return 0;
}

void CCsAudioAcp2xHW::cgs_write32(UINT32 reg, UINT32 val)
{
    if ((reg * 4) < m_BAR5.Len) {
        write32(m_BAR5.Base.baseptr + (reg * 4), val);
        return;
    }
    DbgPrint("Invalid write to cgs register: 0x%x\n", reg);
}

UINT32 CCsAudioAcp2xHW::acp_read32(UINT32 reg)
{
    return cgs_read32(reg);
}

void CCsAudioAcp2xHW::acp_write32(UINT32 reg, UINT32 val)
{
    cgs_write32(reg, val);
}

UINT32 CCsAudioAcp2xHW::i2s_read32(UINT32 i2s_base, UINT32 reg)
{
    return read32(m_BAR5.Base.baseptr + i2s_base + reg);
}

void CCsAudioAcp2xHW::i2s_write32(UINT32 i2s_base, UINT32 reg, UINT32 val)
{
    write32(m_BAR5.Base.baseptr + i2s_base + reg, val);
}

NTSTATUS CCsAudioAcp2xHW::acp_readl_poll_timeout(UINT32 reg, UINT32 val, UINT32 mask, ULONG sleep_us, ULONG timeout_us) {
    UINT32 regval;
    LARGE_INTEGER StartTime;
    KeQuerySystemTimePrecise(&StartTime);
    for (;;) {
        regval = acp_read32(reg);
        LARGE_INTEGER CurrentTime;
        KeQuerySystemTimePrecise(&CurrentTime);
        if ((regval & mask) == val || ((CurrentTime.QuadPart - StartTime.QuadPart) / 10) > timeout_us)
            break;
        if (sleep_us)
            udelay(sleep_us);
    }
    return (regval & mask) == val ? STATUS_SUCCESS : STATUS_IO_TIMEOUT;
}

NTSTATUS CCsAudioAcp2xHW::acp2x_power_on() {
    UINT32 val;

    //Assert soft reset of ACP
    val = cgs_read32(mmACP_SOFT_RESET);
    val |= ACP_SOFT_RESET__SoftResetAud_MASK;
    cgs_write32(mmACP_SOFT_RESET, val);

    UINT32 count = ACP_SOFT_RESET_DONE_TIME_OUT_VALUE;
    while (true) {
        val = cgs_read32(mmACP_SOFT_RESET);
        if (ACP_SOFT_RESET__SoftResetAudDone_MASK ==
            (val & ACP_SOFT_RESET__SoftResetAudDone_MASK))
            break;

        if (--count == 0) {
            goto failure;
        }

        udelay(100);
    }

    //Enable clock to ACP and wait until the clock is enabled
    val = cgs_read32(mmACP_CONTROL);
    val |= ACP_CONTROL__ClkEn_MASK;
    cgs_write32(mmACP_CONTROL, val);

    count = ACP_CLOCK_EN_TIME_OUT_VALUE;
    while (true) {
        val = cgs_read32(mmACP_STATUS);
        if (val & (UINT32)0x1)
            break;

        if (--count == 0) {
            goto failure;
        }

        udelay(100);
    }

    //Deassert the SOFT RESET flags
    val = cgs_read32(mmACP_SOFT_RESET);
    val &= ~ACP_SOFT_RESET__SoftResetAud_MASK;
    cgs_write32(mmACP_SOFT_RESET, val);

    //For BT instance change pins from UART to BT
    val = cgs_read32(mmACP_BT_UART_PAD_SEL);
    val |= ACP_BT_UART_PAD_SELECT_MASK;
    cgs_write32(mmACP_BT_UART_PAD_SEL, val);

    /* initialize Onion control DAGB register */
    cgs_write32(mmACP_AXI2DAGB_ONION_CNTL, ACP_ONION_CNTL_DEFAULT);

    /* initialize Garlic control DAGB registers */
    cgs_write32(mmACP_AXI2DAGB_GARLIC_CNTL, ACP_GARLIC_CNTL_DEFAULT);

    UINT32 sram_pte_offset = ACP_DAGB_GRP_SRAM_BASE_ADDRESS |
        ACP_DAGB_BASE_ADDR_GRP_1__AXI2DAGBSnoopSel_MASK |
        ACP_DAGB_BASE_ADDR_GRP_1__AXI2DAGBTargetMemSel_MASK |
        ACP_DAGB_BASE_ADDR_GRP_1__AXI2DAGBGrpEnable_MASK;
    cgs_write32(mmACP_DAGB_BASE_ADDR_GRP_1, sram_pte_offset);
    cgs_write32(mmACP_DAGB_PAGE_SIZE_GRP_1, ACP_PAGE_SIZE_4K_ENABLE);

    cgs_write32(mmACP_DMA_DESC_BASE_ADDR, ACP_SRAM_BASE_ADDRESS);

    /* Num of descriptors in SRAM 0x4, means 256 descriptors;(64 * 4) */
    cgs_write32(mmACP_DMA_DESC_MAX_NUM_DSCR, 0x4);
    cgs_write32(mmACP_EXTERNAL_INTR_CNTL, ACP_EXTERNAL_INTR_CNTL__DMAIOCMask_MASK);

    return STATUS_SUCCESS;

failure:
    return STATUS_IO_TIMEOUT;
}

NTSTATUS CCsAudioAcp2xHW::acp2x_power_off() {
    UINT32 val;

    //Assert soft reset of ACP
    val = cgs_read32(mmACP_SOFT_RESET);
    val |= ACP_SOFT_RESET__SoftResetAud_MASK;
    cgs_write32(mmACP_SOFT_RESET, val);

    UINT32 count = ACP_SOFT_RESET_DONE_TIME_OUT_VALUE;
    while (true) {
        val = cgs_read32(mmACP_SOFT_RESET);
        if (ACP_SOFT_RESET__SoftResetAudDone_MASK ==
            (val & ACP_SOFT_RESET__SoftResetAudDone_MASK))
            break;

        if (--count == 0) {
            goto failure;
        }

        udelay(100);
    }

    //Disable clock to ACP
    val = cgs_read32(mmACP_CONTROL);
    val |= ~ACP_CONTROL__ClkEn_MASK;
    cgs_write32(mmACP_CONTROL, val);

    count = ACP_CLOCK_EN_TIME_OUT_VALUE;
    while (true) {
        val = cgs_read32(mmACP_STATUS);
        if (val & (UINT32)0x1)
            break;

        if (--count == 0) {
            goto failure;
        }

        udelay(100);
    }

    return STATUS_SUCCESS;
failure:
    return STATUS_IO_TIMEOUT;
}
#endif

NTSTATUS CCsAudioAcp2xHW::acp2x_init() {
    RtlZeroMemory(&this->btStreams, sizeof(this->btStreams));
    RtlZeroMemory(&this->i2sStreams, sizeof(this->i2sStreams));

#if USEACPHW
    //Set clock to 48 Mhz instead of 25 Mhz
    UINT32 val = read32(m_MISCBAR.Base.baseptr + MISCCLKCNTL1);
    val &= ~BIT(OSCCLKENB); //Disable 25 Mhz
    write32(m_MISCBAR.Base.baseptr + MISCCLKCNTL1, val);

    NTSTATUS status = acp2x_power_on();
    return status;
#else
    return STATUS_SUCCESS;
#endif
}

NTSTATUS CCsAudioAcp2xHW::acp2x_deinit() {
#if USEACPHW
    NTSTATUS status = acp2x_power_off();
    return status;
#else
    return STATUS_SUCCESS;
#endif
}

BOOL CCsAudioAcp2xHW::acp_is_playback(eDeviceType deviceType) {
    return (deviceType == eSpeakerDevice || deviceType == eHeadphoneDevice);
}

eDeviceType CCsAudioAcp2xHW::acp_get_companion(eDeviceType deviceType) {
    switch (deviceType) {
    case eSpeakerDevice:
        return eHeadphoneDevice;
    case eHeadphoneDevice:
        return eSpeakerDevice;
    case eMicArrayDevice1:
        return eMicJackDevice;
    case eMicJackDevice:
        return eMicArrayDevice1;
    default:
        DPF(D_ERROR, "Unknown device type");
        return eSpeakerDevice;
    }
}

UINT32 CCsAudioAcp2xHW::acp_get_i2s_regs(eDeviceType deviceType) {
    switch (deviceType) {
    case eSpeakerDevice:
        return ACP_BT_PLAY_REGS_START;
    case eHeadphoneDevice:
        return ACP_I2S_PLAY_REGS_START;
    case eMicArrayDevice1:
    case eMicJackDevice:
        return ACP_I2S_CAP_REGS_START;
    default:
        DPF(D_ERROR, "Unknown device type");
        return NULL;
    }
}

struct acp_stream* CCsAudioAcp2xHW::acp_get_stream(eDeviceType deviceType) {
    switch (deviceType) {
    case eSpeakerDevice:
        return &this->btStreams[0];
    case eHeadphoneDevice:
        return &this->i2sStreams[0];
    case eMicArrayDevice1:
        return &this->btStreams[1];
    case eMicJackDevice:
        return &this->i2sStreams[1];
    default:
        DPF(D_ERROR, "Unknown device type");
        return NULL;
    }
}

NTSTATUS CCsAudioAcp2xHW::acp2x_hw_params(eDeviceType deviceType) {
#if USEACPHW
    eDeviceType companionDevice = acp_get_companion(deviceType);
    struct acp_stream* acpCompanionStream = acp_get_stream(companionDevice);
    if (acpCompanionStream->isActive) {
        NTSTATUS status = acp2x_stop(companionDevice);
        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    UINT32 i2s_base = acp_get_i2s_regs(deviceType);
    {
        UINT32 comp1;
        UINT32 comp2;
        switch (deviceType) {
        case eSpeakerDevice:
            comp1 = i2s_read32(i2s_base, ACP_BT_COMP1_REG_OFFSET);
            comp2 = i2s_read32(i2s_base, ACP_BT_COMP2_REG_OFFSET);
            break;
        case eHeadphoneDevice:
            comp1 = i2s_read32(i2s_base, ACP_I2S_COMP1_PLAY_REG_OFFSET);
            comp2 = i2s_read32(i2s_base, ACP_I2S_COMP2_PLAY_REG_OFFSET);
            break;
        case eMicArrayDevice1:
        case eMicJackDevice:
            comp1 = i2s_read32(i2s_base, ACP_I2S_COMP1_CAP_REG_OFFSET);
            comp2 = i2s_read32(i2s_base, ACP_I2S_COMP2_CAP_REG_OFFSET);
            break;
        default:
            DPF(D_ERROR, "Unknown device type");
            return STATUS_INVALID_PARAMETER;
        }

        UINT32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));

        BOOL isPlayback = acp_is_playback(deviceType);

        //Stop DW I2S channels
        for (int i = 0; i < 4; i++) {
            i2s_write32(i2s_base, isPlayback ? TER(i) : RER(i), 0);
        }

        for (int ch_reg = 0; ch_reg < 1; ch_reg++) {
            i2s_write32(i2s_base, isPlayback ? TCR(ch_reg) : RCR(ch_reg), 0x02);
            i2s_write32(i2s_base, isPlayback ? TFCR(ch_reg) : RFCR(ch_reg), (fifo_depth / 2) - 1);
            i2s_write32(i2s_base, isPlayback ? TER(ch_reg) : RER(ch_reg), 1);
        }

        i2s_write32(i2s_base, CCR, 0);
    }

    //Set resolution (Only for stoney)
    UINT32 val = acp_read32(mmACP_I2S_16BIT_RESOLUTION_EN);
    switch (deviceType) {
    case eSpeakerDevice:
        val |= ACP_I2S_BT_16BIT_RESOLUTION_EN;
        break;
    case eHeadphoneDevice:
        val |= ACP_I2S_SP_16BIT_RESOLUTION_EN;
        break;
    case eMicArrayDevice1:
    case eMicJackDevice:
        val |= ACP_I2S_MIC_16BIT_RESOLUTION_EN;
        break;
    default:
        DPF(D_ERROR, "Unknown device type");
        return STATUS_INVALID_PARAMETER;
    }
    acp_write32(mmACP_I2S_16BIT_RESOLUTION_EN, val);

    struct acp_stream* stream = acp_get_stream(deviceType);
    if (!stream) {
        return STATUS_INVALID_PARAMETER;
    }

    switch (deviceType) {
    case eSpeakerDevice:
        stream->pte_offset = ACP_ST_BT_PLAYBACK_PTE_OFFSET;
        stream->ch1 = SYSRAM_TO_ACP_BT_INSTANCE_CH_NUM;
        stream->ch2 = ACP_TO_I2S_DMA_BT_INSTANCE_CH_NUM;
        stream->sram_bank = ACP_SRAM_BANK_1_ADDRESS;
        stream->destination = TO_BLUETOOTH;
        stream->dma_dscr_idx_1 = PLAYBACK_START_DMA_DESCR_CH8;
        stream->dma_dscr_idx_2 = PLAYBACK_START_DMA_DESCR_CH9;
        stream->byte_cnt_high_reg_offset =
            mmACP_I2S_BT_TRANSMIT_BYTE_CNT_HIGH;
        stream->byte_cnt_low_reg_offset =
            mmACP_I2S_BT_TRANSMIT_BYTE_CNT_LOW;
        break;
    case eHeadphoneDevice:
        stream->pte_offset = ACP_ST_PLAYBACK_PTE_OFFSET;
        stream->ch1 = SYSRAM_TO_ACP_CH_NUM;
        stream->ch2 = ACP_TO_I2S_DMA_CH_NUM;
        stream->sram_bank = ACP_SRAM_BANK_1_ADDRESS;
        stream->destination = TO_ACP_I2S_1;
        stream->dma_dscr_idx_1 = PLAYBACK_START_DMA_DESCR_CH12;
        stream->dma_dscr_idx_2 = PLAYBACK_START_DMA_DESCR_CH13;
        stream->byte_cnt_high_reg_offset =
            mmACP_I2S_TRANSMIT_BYTE_CNT_HIGH;
        stream->byte_cnt_low_reg_offset =
            mmACP_I2S_TRANSMIT_BYTE_CNT_LOW;
        break;
    case eMicArrayDevice1:
    case eMicJackDevice:
        stream->pte_offset = ACP_CAPTURE_PTE_OFFSET;
        stream->ch1 = I2S_TO_ACP_DMA_CH_NUM;
        stream->ch2 = ACP_TO_SYSRAM_CH_NUM;
        stream->pte_offset = ACP_ST_CAPTURE_PTE_OFFSET;
        stream->sram_bank = ACP_SRAM_BANK_3_ADDRESS;
        stream->destination = FROM_ACP_I2S_1;
        stream->dma_dscr_idx_1 = CAPTURE_START_DMA_DESCR_CH14;
        stream->dma_dscr_idx_2 = CAPTURE_START_DMA_DESCR_CH15;
        stream->byte_cnt_high_reg_offset =
            mmACP_I2S_RECEIVED_BYTE_CNT_HIGH;
        stream->byte_cnt_low_reg_offset =
            mmACP_I2S_RECEIVED_BYTE_CNT_LOW;
        stream->dma_curr_dscr = mmACP_DMA_CUR_DSCR_15;
        break;
    default:
        DPF(D_ERROR, "Unknown device type");
        return STATUS_INVALID_PARAMETER;
    }

#endif
    return STATUS_SUCCESS;
}

/*
 * Configure a given dma channel parameters - enable/disable,
 * number of descriptors, priority
 */
void CCsAudioAcp2xHW::config_acp_dma_channel(UINT16 ch_num, UINT16 dscr_strt_idx,
    UINT16 num_dscrs, enum acp_dma_priority_level priority_level)
{
    UINT32 dma_ctrl;

    /* disable the channel run field */
    dma_ctrl = acp_read32(mmACP_DMA_CNTL_0 + ch_num);
    dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChRun_MASK;
    acp_write32(mmACP_DMA_CNTL_0 + ch_num, dma_ctrl);

    /* program a DMA channel with first descriptor to be processed. */
    acp_write32(mmACP_DMA_DSCR_STRT_IDX_0 + ch_num, ACP_DMA_DSCR_STRT_IDX_0__DMAChDscrStrtIdx_MASK & dscr_strt_idx);

    /*
     * program a DMA channel with the number of descriptors to be
     * processed in the transfer
     */
    acp_write32(mmACP_DMA_DSCR_CNT_0 + ch_num, ACP_DMA_DSCR_CNT_0__DMAChDscrCnt_MASK & num_dscrs);

    /* set DMA channel priority */
    acp_write32(mmACP_DMA_PRIO_0 + ch_num, priority_level);
}

/* Initialize a dma descriptor in SRAM based on descriptor information passed */
void CCsAudioAcp2xHW::config_dma_descriptor_in_sram(UINT16 descr_idx,
    acp_dma_dscr_transfer_t* descr_info)
{
    UINT32 sram_offset;

    sram_offset = (descr_idx * sizeof(acp_dma_dscr_transfer_t));

    /* program the source base address. */
    acp_write32(mmACP_SRBM_Targ_Idx_Addr, sram_offset);
    acp_write32(mmACP_SRBM_Targ_Idx_Data, descr_info->src);
    /* program the destination base address. */
    acp_write32(mmACP_SRBM_Targ_Idx_Addr, sram_offset + 4);
    acp_write32(mmACP_SRBM_Targ_Idx_Data, descr_info->dest);

    /* program the number of bytes to be transferred for this descriptor. */
    acp_write32(mmACP_SRBM_Targ_Idx_Addr, sram_offset + 8);
    acp_write32(mmACP_SRBM_Targ_Idx_Data, descr_info->xfer_val);
}

void CCsAudioAcp2xHW::pre_config_reset(UINT16 ch_num)
{
    UINT32 dma_ctrl;
    NTSTATUS status;

    /* clear the reset bit */
    dma_ctrl = acp_read32(mmACP_DMA_CNTL_0 + ch_num);
    dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChRst_MASK;
    acp_write32(mmACP_DMA_CNTL_0 + ch_num, dma_ctrl);
    /* check the reset bit before programming configuration registers */
    status = acp_readl_poll_timeout(mmACP_DMA_CNTL_0 + ch_num, 0, ACP_DMA_CNTL_0__DMAChRst_MASK, 100, ACP_DMA_RESET_TIME);
}

void CCsAudioAcp2xHW::set_acp_sysmem_dma_descriptors(UINT32 size, BOOL playback, UINT32 pte_offset, UINT16 ch,
    UINT32 sram_bank, UINT16 dma_dscr_idx) {
    UINT16 i;
    acp_dma_dscr_transfer_t dmadscr[NUM_DSCRS_PER_CHANNEL] = { 0 };

    for (i = 0; i < NUM_DSCRS_PER_CHANNEL; i++) {
        dmadscr[i].xfer_val = 0;
        if (playback) {
            dma_dscr_idx = dma_dscr_idx + i;
            dmadscr[i].dest = sram_bank + (i * (size / 2));
            dmadscr[i].src = ACP_INTERNAL_APERTURE_WINDOW_0_ADDRESS
                + (pte_offset * SZ_4K) + (i * (size / 2));
            dmadscr[i].xfer_val |=
                (ACP_DMA_ATTR_DAGB_GARLIC_TO_SHAREDMEM << 16) |
                (size / 2);
        }
        else {
            dma_dscr_idx = dma_dscr_idx + i;
            dmadscr[i].src = sram_bank + (i * (size / 2));
            dmadscr[i].dest =
                ACP_INTERNAL_APERTURE_WINDOW_0_ADDRESS +
                (pte_offset * SZ_4K) + (i * (size / 2));
            dmadscr[i].xfer_val |=
                (ACP_DMA_ATTR_SHARED_MEM_TO_DAGB_GARLIC << 16) |
                (size / 2);
        }
        config_dma_descriptor_in_sram(dma_dscr_idx,
            &dmadscr[i]);
    }
    pre_config_reset(ch);
    config_acp_dma_channel(ch,
        dma_dscr_idx - 1,
        NUM_DSCRS_PER_CHANNEL,
        ACP_DMA_PRIORITY_LEVEL_NORMAL);
}

void CCsAudioAcp2xHW::set_acp_to_i2s_dma_descriptors(UINT32 size, BOOL playback, UINT32 sram_bank,
    UINT32 destination, UINT16 ch, UINT16 dma_dscr_idx) {
    UINT16 i;
    acp_dma_dscr_transfer_t dmadscr[NUM_DSCRS_PER_CHANNEL] = { 0 };

    for (i = 0; i < NUM_DSCRS_PER_CHANNEL; i++) {
        dmadscr[i].xfer_val = 0;
        if (playback) {
            dma_dscr_idx = dma_dscr_idx + i;
            dmadscr[i].src = sram_bank + (i * (size / 2));
            /* dmadscr[i].dest is unused by hardware. */
            dmadscr[i].dest = 0;
            dmadscr[i].xfer_val |= BIT(22) | (destination << 16) |
                (size / 2);
        }
        else {
            dma_dscr_idx = dma_dscr_idx + i;
            /* dmadscr[i].src is unused by hardware. */
            dmadscr[i].src = 0;
            dmadscr[i].dest =
                sram_bank + (i * (size / 2));
            dmadscr[i].xfer_val |= BIT(22) |
                (destination << 16) | (size / 2);
        }
        config_dma_descriptor_in_sram(dma_dscr_idx,
            &dmadscr[i]);
    }
    pre_config_reset(ch);
    /* Configure the DMA channel with the above descriptor */
    config_acp_dma_channel(ch, dma_dscr_idx - 1,
        NUM_DSCRS_PER_CHANNEL,
        ACP_DMA_PRIORITY_LEVEL_NORMAL);
}

void CCsAudioAcp2xHW::acp_set_sram_bank_state(UINT16 bank, bool power_on)
{
    UINT32 val, req_reg, sts_reg, sts_reg_mask;
    UINT32 loops = 1000;

    if (bank < 32) {
        req_reg = mmACP_MEM_SHUT_DOWN_REQ_LO;
        sts_reg = mmACP_MEM_SHUT_DOWN_STS_LO;
        sts_reg_mask = 0xFFFFFFFF;

    }
    else {
        bank -= 32;
        req_reg = mmACP_MEM_SHUT_DOWN_REQ_HI;
        sts_reg = mmACP_MEM_SHUT_DOWN_STS_HI;
        sts_reg_mask = 0x0000FFFF;
    }

    val = acp_read32(req_reg);
    if (val & (1 << bank)) {
        /* bank is in off state */
        if (power_on == true)
            /* request to on */
            val &= ~(1 << bank);
        else
            /* request to off */
            return;
    }
    else {
        /* bank is in on state */
        if (power_on == false)
            /* request to off */
            val |= 1 << bank;
        else
            /* request to on */
            return;
    }
    acp_write32(req_reg, val);

    while (acp_read32(sts_reg) != sts_reg_mask) {
        if (!loops--) {
            break;
        }
        udelay(10);
    }
}

NTSTATUS CCsAudioAcp2xHW::acp2x_program_dma(eDeviceType deviceType, PMDL mdl, IPortWaveRTStream *stream, UINT32 bytesCount) {
#if USEACPHW
    int pageCount = stream->GetPhysicalPagesCount(mdl);
    if (pageCount < 1) {
        return STATUS_NO_MEMORY;
    }

    acp_set_sram_bank_state(0, true);

    //Configure PTEs
    struct acp_stream* acpStream = acp_get_stream(deviceType);
    if (!acpStream) {
        return STATUS_INVALID_PARAMETER;
    }
    acpStream->bufferBytes = bytesCount;

    UINT32 pte_offset = acpStream->pte_offset;
    UINT32 offset = ACP_DAGB_GRP_SRBM_SRAM_BASE_OFFSET + (pte_offset * 8);
    for (int page_idx = 0; page_idx < pageCount; page_idx++) {
        PHYSICAL_ADDRESS address = stream->GetPhysicalPageAddress(mdl, page_idx);
        UINT32 low = address.LowPart;
        UINT32 high = address.HighPart;

        /* Load the low address of page int ACP SRAM through SRBM */
        acp_write32(mmACP_SRBM_Targ_Idx_Addr, offset + (page_idx * 8));

        acp_write32(mmACP_SRBM_Targ_Idx_Data, low);
        /* Load the High address of page int ACP SRAM through SRBM */
        acp_write32(mmACP_SRBM_Targ_Idx_Addr, offset + (page_idx * 8) + 4);

        /* page enable in ACP */
        high |= BIT(31);
        acp_write32(mmACP_SRBM_Targ_Idx_Data, high);
    }

    BOOL isPlaying = acp_is_playback(deviceType);
    UINT16 ch_acp_sysmem, ch_acp_i2s;
    if (isPlaying) {
        ch_acp_sysmem = acpStream->ch1;
        ch_acp_i2s = acpStream->ch2;
    }
    else {
        ch_acp_i2s = acpStream->ch1;
        ch_acp_sysmem = acpStream->ch2;
    }

    // Configure System memory <-> ACP SRAM DMA descriptors
    set_acp_sysmem_dma_descriptors(bytesCount,
        isPlaying, acpStream->pte_offset,
        ch_acp_sysmem, acpStream->sram_bank,
        acpStream->dma_dscr_idx_1);

    /* Configure ACP SRAM <-> I2S DMA descriptors */
    set_acp_to_i2s_dma_descriptors(bytesCount,
        isPlaying, acpStream->sram_bank,
        acpStream->destination, ch_acp_i2s,
        acpStream->dma_dscr_idx_2);

    config_acp_dma_channel(ch_acp_sysmem, acpStream->dma_dscr_idx_1, NUM_DSCRS_PER_CHANNEL, ACP_DMA_PRIORITY_LEVEL_NORMAL);
    config_acp_dma_channel(ch_acp_i2s, acpStream->dma_dscr_idx_2, NUM_DSCRS_PER_CHANNEL, ACP_DMA_PRIORITY_LEVEL_NORMAL);

    UINT32 i2s_base = acp_get_i2s_regs(deviceType);
    i2s_write32(i2s_base, isPlaying ? TXFFR : RXFFR, 1);

#endif
    return STATUS_SUCCESS;
}

void CCsAudioAcp2xHW::acp_dma_cap_channel_enable(UINT16 cap_channel)
{
    UINT32 val, ch_reg, imr_reg, res_reg;

    switch (cap_channel) {
    case CAP_CHANNEL1:
        ch_reg = mmACP_I2SMICSP_RER1;
        res_reg = mmACP_I2SMICSP_RCR1;
        imr_reg = mmACP_I2SMICSP_IMR1;
        break;
    case CAP_CHANNEL0:
    default:
        ch_reg = mmACP_I2SMICSP_RER0;
        res_reg = mmACP_I2SMICSP_RCR0;
        imr_reg = mmACP_I2SMICSP_IMR0;
        break;
    }
    val = acp_read32(mmACP_I2S_16BIT_RESOLUTION_EN);
    if (val & ACP_I2S_MIC_16BIT_RESOLUTION_EN) {
        acp_write32(ch_reg, 0x0);
        /* Set 16bit resolution on capture */
        acp_write32(res_reg, 0x2);
    }
    val = acp_read32(imr_reg);
    val &= ~ACP_I2SMICSP_IMR1__I2SMICSP_RXDAM_MASK;
    val &= ~ACP_I2SMICSP_IMR1__I2SMICSP_RXFOM_MASK;
    acp_write32(imr_reg, val);
    acp_write32(ch_reg, 0x1);
}

void CCsAudioAcp2xHW::acp_dma_cap_channel_disable(UINT16 cap_channel)
{
    UINT32 val, ch_reg, imr_reg;

    switch (cap_channel) {
    case CAP_CHANNEL1:
        imr_reg = mmACP_I2SMICSP_IMR1;
        ch_reg = mmACP_I2SMICSP_RER1;
        break;
    case CAP_CHANNEL0:
    default:
        imr_reg = mmACP_I2SMICSP_IMR0;
        ch_reg = mmACP_I2SMICSP_RER0;
        break;
    }
    val = acp_read32(imr_reg);
    val |= ACP_I2SMICSP_IMR1__I2SMICSP_RXDAM_MASK;
    val |= ACP_I2SMICSP_IMR1__I2SMICSP_RXFOM_MASK;
    acp_write32(imr_reg, val);
    acp_write32(ch_reg, 0x0);
}

void CCsAudioAcp2xHW::acp_dma_start(UINT16 ch_num, BOOL isCircular) {
    UINT32 dma_ctrl;

    /* read the dma control register and disable the channel run field */
    dma_ctrl = acp_read32(mmACP_DMA_CNTL_0 + ch_num);

    /* Invalidating the DAGB cache */
    acp_write32(mmACP_DAGB_ATU_CTRL, 1);

    /*
     * configure the DMA channel and start the DMA transfer
     * set dmachrun bit to start the transfer and enable the
     * interrupt on completion of the dma transfer
     */
    dma_ctrl |= ACP_DMA_CNTL_0__DMAChRun_MASK;

    switch (ch_num) {
    case ACP_TO_I2S_DMA_CH_NUM:
    case I2S_TO_ACP_DMA_CH_NUM:
    case ACP_TO_I2S_DMA_BT_INSTANCE_CH_NUM:
    case I2S_TO_ACP_DMA_BT_INSTANCE_CH_NUM:
    case ACP_TO_I2S_DMA_MICSP_INSTANCE_CH_NUM:
        dma_ctrl |= ACP_DMA_CNTL_0__DMAChIOCEn_MASK;
        break;
    default:
        dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChIOCEn_MASK;
        break;
    }

    /* enable for ACP to SRAM DMA channel */
    if (isCircular == TRUE)
        dma_ctrl |= ACP_DMA_CNTL_0__Circular_DMA_En_MASK;
    else
        dma_ctrl &= ~ACP_DMA_CNTL_0__Circular_DMA_En_MASK;

    acp_write32(mmACP_DMA_CNTL_0 + ch_num, dma_ctrl);
}

NTSTATUS CCsAudioAcp2xHW::acp_dma_stop(UINT16 ch_num)
{
    UINT32 dma_ctrl;
    UINT32 dma_ch_sts;
    UINT32 count = ACP_DMA_RESET_TIME;

    dma_ctrl = acp_read32(mmACP_DMA_CNTL_0 + ch_num);

    /*
     * clear the dma control register fields before writing zero
     * in reset bit
     */
    dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChRun_MASK;
    dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChIOCEn_MASK;

    acp_write32(mmACP_DMA_CNTL_0 + ch_num, dma_ctrl);
    dma_ch_sts = acp_read32(mmACP_DMA_CH_STS);

    if (dma_ch_sts & BIT(ch_num)) {
        /*
         * set the reset bit for this channel to stop the dma
         *  transfer
         */
        dma_ctrl |= ACP_DMA_CNTL_0__DMAChRst_MASK;
        acp_write32(mmACP_DMA_CNTL_0 + ch_num, dma_ctrl);
    }

    /* check the channel status bit for some time and return the status */
    while (true) {
        dma_ch_sts = acp_read32(mmACP_DMA_CH_STS);
        if (!(dma_ch_sts & BIT(ch_num))) {
            /*
             * clear the reset flag after successfully stopping
             * the dma transfer and break from the loop
             */
            dma_ctrl &= ~ACP_DMA_CNTL_0__DMAChRst_MASK;

            acp_write32(mmACP_DMA_CNTL_0
                + ch_num, dma_ctrl);
            break;
        }
        if (--count == 0) {
            return STATUS_IO_TIMEOUT;
        }
        udelay(10);
    }
    return STATUS_SUCCESS;
}

NTSTATUS CCsAudioAcp2xHW::acp2x_play(eDeviceType deviceType) {
#if USEACPHW
    struct acp_stream* acpStream = acp_get_stream(deviceType);
    if (!acpStream) {
        return STATUS_INVALID_PARAMETER;
    }

    if (acpStream->isActive) {
        return STATUS_SUCCESS;
    }


    BOOL isPlaying = acp_is_playback(deviceType);

    if (!isPlaying) {
        switch (deviceType) {
        case eMicArrayDevice1:
            acp_dma_cap_channel_disable(CAP_CHANNEL1);
            acp_dma_cap_channel_enable(CAP_CHANNEL0);
            break;
        case eMicJackDevice:
            acp_dma_cap_channel_disable(CAP_CHANNEL0);
            acp_dma_cap_channel_enable(CAP_CHANNEL1);
            break;
        default:
            break;
        }
    }

    acp_dma_start(acpStream->ch1, TRUE);
    acp_dma_start(acpStream->ch2, TRUE);

    UINT32 i2s_base = acp_get_i2s_regs(deviceType);

    i2s_write32(i2s_base, IER, 1);

    UINT32 irq = i2s_read32(i2s_base, IMR(0));
    if (isPlaying)
        i2s_write32(i2s_base, IMR(0), irq & ~0x30);
    else
        i2s_write32(i2s_base, IMR(0), irq & ~0x03);

    i2s_write32(i2s_base, isPlaying ? ITER : IRER, 1);
    i2s_write32(i2s_base, CER, 1);

    acpStream->isActive = TRUE;
#endif

    CsAudioArg arg;
    RtlZeroMemory(&arg, sizeof(CsAudioArg));
    arg.argSz = sizeof(CsAudioArg);
    arg.endpointType = GetCSAudioEndpoint(deviceType);
    arg.endpointRequest = CSAudioEndpointStart;
    ExNotifyCallback(this->CSAudioAPICallback, &arg, &CsAudioArg2);
return STATUS_SUCCESS;
}

NTSTATUS CCsAudioAcp2xHW::acp2x_stop(eDeviceType deviceType) {
    CsAudioArg arg;
    RtlZeroMemory(&arg, sizeof(CsAudioArg));
    arg.argSz = sizeof(CsAudioArg);
    arg.endpointType = GetCSAudioEndpoint(deviceType);
    arg.endpointRequest = CSAudioEndpointStop;
    ExNotifyCallback(this->CSAudioAPICallback, &arg, &CsAudioArg2);

#if USEACPHW
    struct acp_stream* acpStream = acp_get_stream(deviceType);
    if (!acpStream) {
        return STATUS_INVALID_PARAMETER;
    }

    if (!acpStream->isActive) {
        return STATUS_SUCCESS;
    }

    acp_dma_stop(acpStream->ch2);
    acp_dma_stop(acpStream->ch1);

    UINT32 i2s_base = acp_get_i2s_regs(deviceType);
    BOOL isPlaying = acp_is_playback(deviceType);

    for (int i = 0; i < 4; i++)
        i2s_read32(i2s_base, isPlaying ? TOR(i) : ROR(i));

    for (int i = 0; i < 4; i++) {
        UINT32 irq = i2s_read32(i2s_base, IMR(i));
        if (isPlaying)
            i2s_write32(i2s_base, IMR(i), irq | 0x30);
        else
            i2s_write32(i2s_base, IMR(i), irq | 0x03);
    }

    i2s_write32(i2s_base, isPlaying ? ITER : IRER, 0);

    //TODO: This may be shared
    i2s_write32(i2s_base, CER, 0);
    i2s_write32(i2s_base, IER, 0);

    acpStream->isActive = FALSE;
    acpStream->bufferBytes = 0;
#endif
    return STATUS_SUCCESS;
}

NTSTATUS CCsAudioAcp2xHW::acp2x_current_position(eDeviceType deviceType, UINT32 *linkPos, UINT64 *linearPos) {
#if USEACPHW
    struct acp_stream* acpStream = acp_get_stream(deviceType);
    if (!acpStream) {
        return STATUS_INVALID_PARAMETER;
    }

    UINT32 linearHigh = acp_read32(acpStream->byte_cnt_high_reg_offset);
    UINT32 linearLow = acp_read32(acpStream->byte_cnt_low_reg_offset);

    UINT64 rawPos = ((UINT64)linearHigh << 32) | (UINT64)linearLow;
    UINT64 rawPosMod = rawPos;
    if (acpStream->bufferBytes == 0) {
        rawPosMod = 0;
    }
    else {
        rawPosMod = rawPos % acpStream->bufferBytes;
    }

    if (linkPos)
        *linkPos = linearLow;
    if (linearPos)
        *linearPos = rawPosMod;
#endif
    return STATUS_SUCCESS;
}

//=============================================================================
BOOL
CCsAudioAcp2xHW::bGetDevSpecific()
/*++

Routine Description:

  Gets the HW (!) Device Specific info

Arguments:

  N/A

Return Value:

  True or False (in this example).

--*/
{
    return m_bDevSpecific;
} // bGetDevSpecific

//=============================================================================
void
CCsAudioAcp2xHW::bSetDevSpecific
(
    _In_  BOOL                bDevSpecific
)
/*++

Routine Description:

  Sets the HW (!) Device Specific info

Arguments:

  fDevSpecific - true or false for this example.

Return Value:

    void

--*/
{
    m_bDevSpecific = bDevSpecific;
} // bSetDevSpecific

//=============================================================================
INT
CCsAudioAcp2xHW::iGetDevSpecific()
/*++

Routine Description:

  Gets the HW (!) Device Specific info

Arguments:

  N/A

Return Value:

  int (in this example).

--*/
{
    return m_iDevSpecific;
} // iGetDevSpecific

//=============================================================================
void
CCsAudioAcp2xHW::iSetDevSpecific
(
    _In_  INT                 iDevSpecific
)
/*++

Routine Description:

  Sets the HW (!) Device Specific info

Arguments:

  fDevSpecific - true or false for this example.

Return Value:

    void

--*/
{
    m_iDevSpecific = iDevSpecific;
} // iSetDevSpecific

//=============================================================================
UINT
CCsAudioAcp2xHW::uiGetDevSpecific()
/*++

Routine Description:

  Gets the HW (!) Device Specific info

Arguments:

  N/A

Return Value:

  UINT (in this example).

--*/
{
    return m_uiDevSpecific;
} // uiGetDevSpecific

//=============================================================================
void
CCsAudioAcp2xHW::uiSetDevSpecific
(
    _In_  UINT                uiDevSpecific
)
/*++

Routine Description:

  Sets the HW (!) Device Specific info

Arguments:

  uiDevSpecific - int for this example.

Return Value:

    void

--*/
{
    m_uiDevSpecific = uiDevSpecific;
} // uiSetDevSpecific

//=============================================================================
ULONG                       
CCsAudioAcp2xHW::GetMixerMux()
/*++

Routine Description:

  Return the current mux selection

Arguments:

Return Value:

  ULONG

--*/
{
    return m_ulMux;
} // GetMixerMux

//=============================================================================
LONG
CCsAudioAcp2xHW::GetMixerPeakMeter
(   
    _In_  ULONG                   ulNode,
    _In_  ULONG                   ulChannel
)
/*++

Routine Description:

  Gets the HW (!) peak meter for Simple Audio Sample.

Arguments:

  ulNode - topology node id

  ulChannel - which channel are we reading?

Return Value:

  LONG - sample peak meter level

--*/
{
    UNREFERENCED_PARAMETER(ulChannel);

    if (ulNode < MAX_TOPOLOGY_NODES)
    {
        return m_PeakMeterControls[ulNode];
    }

    return 0;
} // GetMixerVolume

//=============================================================================
#pragma code_seg("PAGE")
void 
CCsAudioAcp2xHW::MixerReset()
/*++

Routine Description:

  Resets the mixer registers.

Arguments:

Return Value:

    void

--*/
{
    PAGED_CODE();

    for (ULONG i=0; i<MAX_TOPOLOGY_NODES; ++i)
    {
        m_PeakMeterControls[i] = PEAKMETER_SIGNED_MAXIMUM/2;
    }
    
    // BUGBUG change this depending on the topology
    m_ulMux = 2;
} // MixerReset
#pragma code_seg()

//=============================================================================
void                        
CCsAudioAcp2xHW::SetMixerMux
(
    _In_  ULONG                   ulNode
)
/*++

Routine Description:

  Sets the HW (!) mux selection

Arguments:

  ulNode - topology node id

Return Value:

    void

--*/
{
    m_ulMux = ulNode;
} // SetMixMux
