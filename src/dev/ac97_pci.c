/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National  Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2017, Panitan Wongse-ammat
 * Copyright (c) 2017, Peter Dinda
 * Copyright (c) 2017, The V3VEE Project  <http://www.v3vee.org>
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Authors: Panitan Wongse-ammat <Panitan.W@u.northwesttern.edu>
 *          Marc Warrior <warrior@u.northwestern.edu>
 *          Galen Lansbury <galenlansbury2017@u.northwestern.edu>
 *          Peter Dinda <pdinda@northwestern.edu>
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

// TODO: Why don't we get interrupted again after playing entry 0?
#include <nautilus/nautilus.h>
#include <nautilus/sounddev.h>
#include <nautilus/cpu.h>
#include <dev/pci.h>
#include <nautilus/mm.h> // malloc, free
#include <dev/ac97_pci.h>
#include <dev/e1000e_pci.h> // TODO: included so I can compile the file need to remove at a later date
#include <nautilus/irq.h>         // interrupt register
#include <nautilus/naut_string.h> // memset, memcpy
#include <nautilus/dev.h>         // NK_DEV_REQ_*
#include <nautilus/timer.h>       // nk_sleep(ns);
#include <nautilus/cpu.h>         // udelay
#include <math.h>
#include <nautilus/shell.h>

#ifndef NAUT_CONFIG_DEBUG_AC97_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("ac97_pci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ac97_pci: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("ac97_pci: " fmt, ##args)

#define READ_MEM(d, o) (*((volatile uint32_t *)(((d)->mem_start) + (o))))
#define WRITE_MEM(d, o, v) ((*((volatile uint32_t *)(((d)->mem_start) + (o)))) = (v))

#define READ_MEM64(d, o) (*((volatile uint64_t *)((d)->mem_start + (o))))
#define WRITE_MEM64(d, o, v) ((*((volatile uint64_t *)(((d)->mem_start) + (o)))) = (v))

/* Use these macros to easily debug data transfer between host and device */
#define INB(port) ({uint8_t _x = inb(port); DEBUG("INB found %x at port %x\n", _x, port); _x;})
#define INW(port) ({uint16_t _x = inw(port); DEBUG("INW found %x at port %x\n", _x, port); _x;})
#define INL(port) ({uint32_t _x = inl(port); DEBUG("INL found %x at port %x\n", _x, port); _x;})
#define OUTB(val, port) ({outb(val, port); DEBUG("OUTB placed %x at port %x\n", val, port);})
#define OUTW(val, port) ({outw(val, port); DEBUG("OUTW placed %x at port %x\n", val, port);})
#define OUTL(val, port) ({outl(val, port); DEBUG("OUTL placed %x at port %x\n", val, port);})

/* Use these macros to manage/access the AC97's BDL Descriptors and their entries. The driver will need
   to manage three BDLs for each of the three register boxes (PCM IN, PCM OUT, and Mic). */
#define BDL_MAX_SIZE 32                            // AC97 BDL Rings can have up to 32 entries
#define BDL_ENTRY_MAX_SIZE 0xFFFE                  // a single BDL entry can transfer up to this many samples
#define BDL_INC(a, b) (((a) + (b)) % BDL_MAX_SIZE) // modular increment bdl index

#define BDL_IN (state->bdl_in_desc)                               // access BDL Descriptor for PCM IN of an ac97_state struct
#define IN_HEAD (BDL_IN->head_pos)                                // pointer to current head of PCM IN ring
#define IN_TAIL (BDL_IN->tail_pos)                                // pointer to current tail of PCM IN ring
#define IN_SIZE (BDL_IN->size)                                    // effective size of PCM IN ring (its overall size is BDL_MAX_SIZE)
#define IN_RING (BDL_IN->bdl_ring)                                // pointer to the PCM IN ring itself
#define IN_ENTRY(i) (((ac97_bdl_entry *)IN_RING)[(i)])            // retrieve entire entry at index i
#define IN_ENTRY_ADDR(i) (((ac97_bdl_entry *)IN_RING)[(i)].addr)  // address to buffer of sound data for the entry at index i
#define IN_ENTRY_SIZE(i) (((ac97_bdl_entry *)IN_RING)[(i)].size)  // number of samples in the buffer of sound data for the entry at index i
#define IN_ENTRY_LAST(i) (((ac97_bdl_entry *)IN_RING)[(i)].last)  // flags to fire an interrupt signaling that this is the last entry in buffer
#define IN_ENTRY_IOC(i) (((ac97_bdl_entry *)IN_RING)[(i)].ioc)    // flags to fire an interrupt when this buffer is fully consumed

#define BDL_OUT (state->bdl_out_desc)                              // access BDL Descriptor for PCM OUT of an ac97_state struct
#define OUT_HEAD (BDL_OUT->head_pos)                               // pointer to current head of PCM OUT ring
#define OUT_TAIL (BDL_OUT->tail_pos)                               // pointer to current tail of PCM OUT ring
#define OUT_SIZE (BDL_OUT->size)                                   // effective size of PCM OUT ring (its overall size is BDL_MAX_SIZE)
#define OUT_RING (BDL_OUT->bdl_ring)                               // pointer to the PCM OUT ring itself
#define OUT_ENTRY(i) (((ac97_bdl_entry *)OUT_RING)[(i)])           // retrieve entire entry at index i
#define OUT_ENTRY_ADDR(i) (((ac97_bdl_entry *)OUT_RING)[(i)].addr) // address to buffer of sound data for the entry at index i
#define OUT_ENTRY_SIZE(i) (((ac97_bdl_entry *)OUT_RING)[(i)].size) // number of samples in the buffer of sound data for the entry at index i
#define OUT_ENTRY_LAST(i) (((ac97_bdl_entry *)OUT_RING)[(i)].last) // flags to fire an interrupt signaling that this is the last entry in buffer
#define OUT_ENTRY_IOC(i) (((ac97_bdl_entry *)OUT_RING)[(i)].ioc)   // flags to fire an interrupt when this buffer is fully consumed

#define BDL_MIC (state->bdl_mic_desc)                              // access BDL Descriptor for PCM MIC of an ac97_state struct
#define MIC_HEAD (BDL_MIC->head_pos)                               // pointer to current head of PCM MIC ring
#define MIC_TAIL (BDL_MIC->tail_pos)                               // pointer to current tail of PCM MIC ring
#define MIC_SIZE (BDL_MIC->size)                                   // effective size of PCM MIC ring (its overall size is BDL_MAX_SIZE)
#define MIC_RING (BDL_MIC->bdl_ring)                               // pointer to the PCM MIC ring itself
#define MIC_ENTRY(i) (((ac97_bdl_entry *)MIC_RING)[(i)])           // retrieve entire entry at index i
#define MIC_ENTRY_ADDR(i) (((ac97_bdl_entry *)MIC_RING)[(i)].addr) // address to buffer of sound data for the entry at index i
#define MIC_ENTRY_SIZE(i) (((ac97_bdl_entry *)MIC_RING)[(i)].size) // number of samples in the buffer of sound data for the entry at index i
#define MIC_ENTRY_LAST(i) (((ac97_bdl_entry *)MIC_RING)[(i)].last) // flags to fire an interrupt signaling that this is the last entry in buffer
#define MIC_ENTRY_IOC(i) (((ac97_bdl_entry *)MIC_RING)[(i)].ioc)   // flags to fire an interrupt when this buffer is fully consumed

/* Not sure what constant variables are necessary for the AC97... come back later and 
   re-examine the code between this comment and the next SECTION DONE line */

#define RESTART_DELAY 5

/*
// Constant variables
// These variables are configurable, and they will change the number of descriptors.
// The number of descriptors is always a multiple of eight.
// both tx_dsc_count and rx_dsc_count should be multiple of 16.
#define TX_DSC_COUNT 128
#define TX_BLOCKSIZE 256           // bytes available per DMA block
#define RX_DSC_COUNT 128           // equal to DMA block count
#define RX_BLOCKSIZE 256           // bytes available per DMA block
#define RESTART_DELAY 5            // usec = 5 us
#define RX_PACKET_BUFFER_SIZE 2048 // the size of the packet buffer
*/

/* SECTION DONE */

// After this line, PLEASE DO NOT CHANGE ANYTHING.

/*
// transmission unit
// Data sheet from page 36
// 16384 bytes is from the maximum packet buffer size that e1000 can receive.
// 16288 bytes is the maximum packet size that e1000 can send in theory.
// Ethernet standard MTU is 1500 bytes.
#define MAX_TU 16384 // maximum transmission unit
#define MIN_TU 48    // minimum transmission unit
#define MAC_LEN 6    // the length of the mac address
*/

// PCI CONFIG SPACE ************************************
#define INTEL_VENDOR_ID 0x8086
#define AC97_DEVICE_ID 0x2415
#define AC97_PCI_CMD_OFFSET 0x4    // Device Control - RW
#define AC97_PCI_STATUS_OFFSET 0x6 // Device Status - RO
#define AC97_PCI_BAR0_OFFSET 0x10  // Location of Native Audio Mixer Registers (may be I/O or MMIO)
#define AC97_PCI_BAR1_OFFSET 0x14 // Location of Native Audio Bus Master Registers (may be I/O or)

// PCI command register
#define AC97_PCI_CMD_IO_ACCESS_EN 1         // io access enable
#define AC97_PCI_CMD_MEM_ACCESS_EN (1 << 1) // memory access enable
#define AC97_PCI_CMD_LANRW_EN (1 << 2)      // enable mastering lan r/w
#define AC97_PCI_CMD_INT_DISABLE (1 << 10)  // legacy interrupt disable when set

// PCI status register
#define AC97_PCI_STATUS_INT (1 << 3) // interrupt status

// NATIVE AUDIO MIXER REGISTER OFFSETS ************************************
// These offsets are relative to the BAR0 base address. Reference: https://wiki.osdev.org/AC97
#define AC97_NAM_RESET_OFFSET 0x00   // writing anything here sets all NAM registers to defaults
#define AC97_NAM_MASTER_VOL 0x02     // sets master output volume
#define AC97_NAM_MIC_VOL 0x0E        // sets microphone volume
#define AC97_NAM_PCM_OUT_VOL 0x18    // sets output volume of PCM patterns
#define AC97_NAM_DEV_IN 0x1A         // use 0x0000 for built-in mic, or 0x0303 for AUX input
#define AC97_NAM_IN_GAIN 0x1C        // sets gain of input devices
#define AC97_NAM_MIC_GAIN 0x1E       // sets gain of microphone while recording
#define AC97_NAM_EXT_FUNC 0x28       // some cards support extra features (e.g. sampling rate != 48000 Hz)
#define AC97_NAM_EXT_FUNC_EN 0x2A    // enable the extra features on the card, if any
#define AC97_NAM_DAC_RATE 0x2C       // sets DAC sampling rate of device
#define AC97_NAM_ADC_RATE 0x32       // seta ADC sampling rate of device

// NATIVE AUDIO BUS MASTER REGISTER OFFSETS ************************************
// These offsets are relative to the BAR1 base address. Reference: https://wiki.osdev.org/AC97
#define AC97_NABM_IN_BOX 0x00        // register box for PCM IN
#define AC97_NABM_OUT_BOX 0x10       // register box for PCM OUT
#define AC97_NABM_MIC_BOX 0x20       // register box for microphone
#define AC97_NABM_CTRL 0x2C          // global control register
#define AC97_NABM_STATUS 0x30        // global status register

// NABM REGISTER BOX OFFSETS ************************************
// There are three register boxes: AC97_NABM_IN_BOX, AC97_NABM_OUT_BOX, and AC97_NABM_MIC_BOX.
// These offsets are relative to the base address of the NABM box. Reference: https://wiki.osdev.org/AC97
#define AC97_REG_BOX_ADDR 0x00       // physical address of buffer descriptor list
#define AC97_REG_BOX_APE 0x04        // number of the APE (buffer entry being currently processed)
#define AC97_REG_BOX_TOTAL 0x05      // number of total descriptor entries
#define AC97_REG_BOX_STATUS 0x06     // status of data transfer
#define AC97_REG_BOX_TRANS 0x08      // number of transferred samples in the APE
#define AC97_REG_BOX_NEXT 0x0A       // number of buffer entry to be processed next
#define AC97_REG_BOX_CTRL 0x0B       // transfer control

/* Now that I think about it, I think the box might need to be represented with a struct/bitfield. Macros might be used by convention within the code, too. */
struct ac97_nabm_box_desc     // describes a native audio bus register box
{
    // Should this be made into bitfields for each component, or is that overkill?
    uint32_t* addr;           // physical address of buffer descriptor list
    uint8_t ape;              // number of actual processed buffer descriptor entry
    uint8_t total;            // total number of entries in the buffer descriptor list
    uint16_t status;          // status of transferring data
    uint16_t trans_samples;   // number of transferred samples in ape
    uint8_t npe;              // number of next processed buffer entry
    uint8_t ctrl;             // transfer control 
};

struct ac97_nabm_desc                  // describes native audio bus registers 
{
    struct ac97_nabm_box_desc input;   // nabm register box for pcm input
    struct ac97_nabm_box_desc output;  // nabm register box for pcm output
    struct ac97_nabm_box_desc mic;     // nabm register box for microphone
    struct 
    {
        uint8_t gie         : 1;       // global interrupt enable
        uint8_t cr          : 1;       // cold reset
        uint8_t wr          : 1;       // warm reset
        uint8_t sd          : 1;       // shut down
        uint16_t rsvd       : 16;      // reserved
        uint8_t chnl        : 2;       // channels for pcm output 
        uint8_t out_mode    : 2;       // pcm output mode 
    } global_ctrl;                     // describes global control register
    struct 
    {
        uint32_t rsvd       : 20;      // reserved
        uint8_t chnl        : 2;       // channel capabilities
        uint8_t smpl        : 2;       // sample capabilities
    } global_status;                   // describes global status register
} __attribute__((packed));

typedef union {
    uint32_t val :24;
    struct
    {
        uint8_t gie         : 1;       // global interrupt enable
        uint8_t cr          : 1;       // cold reset
        uint8_t wr          : 1;       // warm reset
        uint8_t sd          : 1;       // shut down
        uint16_t rsvd       : 16;      // reserved
        uint8_t chnl        : 2;       // channels for pcm output
        uint8_t out_mode    : 2;       // pcm output mode
    }__attribute__((packed));          // describes global control register
}__attribute__((packed)) global_control_register_t;

typedef union 
{
    uint8_t val;
    struct{
        uint8_t last_entry : 5;
        uint8_t rsvd : 3;
    }__attribute__((packed));
}__attribute__((packed)) last_valid_entry_t;

typedef union
{

};

struct ac97_state // TODO: Shouldn't we have an ac97_dev class that contains an ac97_state? 
{
    // pointer to base class
    struct nk_sound_dev *sounddev;

    // pci interrupt and interrupt vector
    struct pci_dev *pci_dev;       
    uint8_t pci_intr;              // IRQ number on bus
    uint8_t intr_vec;              // IRQ we will see (found through a hack to be 0xe4)

    // our device list
    struct list_head ac97_node;

    // where registers are mapped into the I/0 address space
    uint16_t ioport_start_bar0;
    uint16_t ioport_end_bar0;
    uint16_t ioport_start_bar1;
    uint16_t ioport_end_bar1;

    // where registers are mapped into the physical memory space
    // TODO: The AC97 uses PMIO by default, so these are unused. Should we keep both sets
    //       of register mapping parameters in case the user wants to use MMIO?
    uint64_t mem_start;
    uint64_t mem_end; 

    // The following hide the details of the PCI BARs, since
    // we only have one block of registers
    enum { NONE, IO, MEMORY}  method;

    // Device name uponr registration in Nautilus
    char name[DEV_NAME_LEN];

    // The AC97 maintains three Buffer Descriptor Lists for PCM IN, PCM OUT, and Microphone
    struct ac97_bdl_desc *bdl_in_desc;
    struct ac97_bdl_desc *bdl_out_desc;
    struct ac97_bdl_desc *bdl_mic_desc;

    // All potential stream configuration options, to be filled out during the device's init
    struct list_head sample_rates_list;          // holds ac97_sample_rate objects
    uint8_t max_resolution; 
    uint8_t max_channels; 
    nk_sound_dev_scale_t allowed_scales;

    // The AC97 can support up to 1 input stream. The user application must support mixing. 
    struct nk_sound_dev_stream* stream;

    // TODO: What else do we need to add?
};

/* Used to create a list of all possible sample rates supported by the AC97 */
struct ac97_sample_rate
{
    uint16_t rate;        // the AC97 may support variable-rate frequencies between 7 and 48 kHz, which fit in a uint16_t
    struct list_head node;
};

/* ac97_desc_ring stores ac97_bdl_entry objects */
struct ac97_bdl_desc
{
    void *bdl_ring;   // pointer to physical ring buffer of ac97_bdl_entry objects
    uint8_t head_pos; // index to current head of ring
    uint8_t tail_pos; // index to current tail of ring
    uint8_t size;     // effective size of ring (number of ac97_bdl_entry objects)
};

typedef union // describes an ac97 bdl entry
{
    uint64_t val; // uint32_t data[2] <-- should this be like the mouse_packet struct in ps2.c?
    struct
    {
        uint32_t addr : 32;      // physical address to sound data in memory
        uint16_t size : 16;      // number of samples in the buffer this entry describes
        uint16_t rsvd : 14;      // reserved
        uint8_t last : 1;        // flags last entry of buffer, stop playing
        uint8_t ioc : 1;         // interrupt fired when data from this entry is transferred
    } __attribute__((packed));
} __attribute__((packed)) ac97_bdl_entry;

//master volume data structure
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t r_volume: 6; 
        uint8_t rsvd_1: 2;    
        uint8_t l_volume: 6; 
        uint8_t rsvd_2: 1;
        uint8_t mute: 1; 
    } __attribute__((packed)) ;
} __attribute__((packed)) m_volume_t;

// output PCM volume data structure
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t r_volume : 5;
        uint8_t rsvd_1 : 3;
        uint8_t l_volume : 5;
        uint8_t rsvd_2 : 2;
        uint8_t mute : 1;
    } __attribute__((packed));
} __attribute__((packed)) pcm_volume_t;

//global status struct
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t rsvd_1: 20;  
        uint8_t channel: 2;     // channel capabilities
        uint8_t sample: 2; // sample capabilities
        uint8_t rsvd_2: 8;
           
    } __attribute__((packed)) ;
} __attribute__((packed)) global_status_t;

//transfer control data structure
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t dma_control: 1;  // dma controller control
        uint8_t reset: 1;     // reset
        uint8_t lbe_interrupt: 1; // last buffer entry interrupt enabled
        uint8_t ioc_interrupt: 1; //ioc interrupt enaled
        uint8_t fifo_interrupt: 1;     // fifo interrupt enabled
        uint8_t rvsd: 3;
    } __attribute__((packed)) ;
} __attribute__((packed)) transfer_control_t;

// transfer status data structure
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t dma_status      : 1;  // dma transfer status
        uint8_t end_of_transfer : 1;  // is current entry equal to last valid entry?
        uint8_t lbe_interrupt   : 1;  // last buffer entry interrupt fired
        uint8_t ioc_interrupt   : 1;  // ioc interrupt fired
        uint8_t fifo_interrupt  : 1;  // fifo interrupt fired
        uint16_t rvsd : 11;           // reserved
    } __attribute__((packed));
} __attribute__((packed)) transfer_status_t;


double new_sin(double x){
    x = x * 180 / M_PI;

    while(x < 0){
        x += 360;
    }

    while(x > 180){
        x -= 360;
    }

    double neg = 1;

    if(x < 0)
    {
        x = -x;
        neg = -1;
    }

    double sin_val = neg * 4 * x * (180 - x) / (40500 - x * (180 - x));

    return sin_val;
}

static void create_sine_wave(uint16_t *buffer, uint64_t buffer_len, uint64_t tone_frequency, uint64_t sampling_frequency)
{
    /*
    According to https://alsa-project.org/files/pub/manuals/intel/29802801_801_AC97.pdf"
    The samples are stored two per DWord (16-bit samples). In the case of audio PCM, these 
    represent the left and right channels, respectively.
    */
    for (int i = 0; i < buffer_len; i+=2)
    {
        double x = (double) i * 2.0 * M_PI * (double) tone_frequency / (double) sampling_frequency;
        double sin_val = new_sin(x);
        buffer[i] = (uint16_t) (sin_val * 30000.0);
        buffer[i+1] = (uint16_t) (sin_val * 30000.0);
    }

    //for(uint16_t i = 0; i < 360; i++){
    //    uint16_t sample = buffer[i];
    //    DEBUG("Sin sample %d value: %hd\n",i,sample);
    //}
}

// // list of discovered devices
static struct list_head dev_list;
static struct ac97_state *dirty_state; 



static void ac97_set_sound_params(struct ac97_state *state, struct nk_sound_dev_params *params)
{
    /*
    Modifies the registers of the AC97 to configure the stream parameters found in 'params'
    */
    // bar2 global control register 21:20 PCM output channels, 21:22 sample resolution
    // TODO: set num channels, sample rate, sample resolution
    uint16_t gcr_port = state->ioport_start_bar1 + AC97_NABM_CTRL;
    global_control_register_t global_control_register;
    global_control_register.val = INL(gcr_port);
//    uint32_t test = INL(gcr_port);
//    global_control_register.val = test;
    switch(params->num_of_channels) {
        case 2:
            global_control_register.chnl = 0;
            break;
        case 4:
            global_control_register.chnl = 1;
            break;
        case 6:
            global_control_register.chnl = 2;
            break;
        default:
            ERROR("Number of channels is not a valid value\n");
    }

    switch(params->sample_resolution) {
        case NK_SOUND_DEV_SAMPLE_RESOLUTION_16:
            global_control_register.out_mode = 0;
            break;
        case NK_SOUND_DEV_SAMPLE_RESOLUTION_20:
            global_control_register.out_mode = 1;
            break;
        default:
            ERROR("Sample resolution is not a valid value\n");
    }

    // TODO: Write conditionals to handle sample resolution. Default = 48k
    uint16_t DAC_port = state->ioport_start_bar0 + AC97_NAM_DAC_RATE;
    uint16_t sample_rate = 0xBB80;
    OUTW(sample_rate, DAC_port);
    uint16_t returned_rate = INW(DAC_port);
    if(sample_rate != returned_rate){
        ERROR("Could not set provided sample rate");
    }
    return;
}

int ac97_get_available_modes(struct nk_sound_dev *dev, struct nk_sound_dev_params params[], uint32_t params_size)
{
    /*
    Given a pre-allocated array 'params' of size 'params_size', this function fills out the 'params' array
    with as many permutations of stream configuration options as possible. The supported configuration options
    are tracked in the 'state' object, which was presumably filled upon a call to ac97_pci_init() at boot.

    This function returns the TOTAL number of possible permutations. Thus, if 'params' is large enough to hold
    all possible configurations, the user can use the return value to traverse only elements that have been filled
    by this function. Or, if 'params' is too small, the user can learn how large it should be. 
    */

    struct ac97_state *ac_state = dev->dev.state; // get struct ac97_state* from device struct

    // AC97 may support 16 bit audio OR 16 and 20 bit audio. 
    int n_res_options = (ac_state->max_resolution == 16) ? 1 : 2;

    // AC97 may support audio for an even number of channels between 2 and 6
    int n_chnl_options = ac_state->max_channels / 2; 

    // After creating probe_variable_sample_rates, it turns out the AC97 seems to support ANY frequency between
    // 7000 and 48000 Hz. Not sure how to finish this function because we definitely don't want to return a list of 
    // 41000 integers. We need to talk to the other sound team to decide on how the abstraction layer will support this.

    /*
    for (int i = 16; i <= _state->max_resolution; i+=4) {
       resolutions[*num_options] = i;
       *num_options++;
    }
    */
    // TODO: I am bootstrapping this function for the time being while we wait on clarification on how to handle sample rates
    struct nk_sound_dev_params bootstrap_params;
    bootstrap_params.type = NK_SOUND_DEV_OUTPUT_STREAM;
    bootstrap_params.num_of_channels = 2;
    bootstrap_params.sample_rate = NK_SOUND_DEV_SAMPLE_RATE_48kHZ;
    bootstrap_params.sample_resolution = NK_SOUND_DEV_SAMPLE_RESOLUTION_16;
    bootstrap_params.scale = NK_SOUND_DEV_SCALE_LOGARITHMIC;
    params[0] = bootstrap_params;
    return 1;
    ERROR("This function has not been finished!\n");
    return -1; // TODO: return the number of filled options instead!
}

struct nk_sound_dev_stream *ac97_open_stream(void *state, struct nk_sound_dev_params *params)
{
    /*
    Opens a stream between the AC97 device and the caller. The AC97 can only support a single stream at
    a time.
    */

    struct ac97_state *ac_state = (struct ac97_state*) state;

    if(ac_state->stream) // 'stream' is initialized to NULL in ac97_pci_init() or upon a call to close_stream()
    {
        DEBUG("Stream value: %d\n", ac_state->stream);
        ERROR("Cannot open a second AC97 stream!\n");
        return -1;
    }

//     allocate the stream object, it will be freed by ac97_close_stream
    ac_state->stream = (struct nk_sound_dev_stream*) malloc(sizeof(struct nk_sound_dev_stream));
    if (!ac_state->stream)
    {
        ERROR("Could not allocate nk_sound_dev_stream object!\n");
        return -1;
    }
    ac_state->stream->stream_id = 0; // this field is not used by the AC97 since it can only support one stream
    ac_state->stream->params = *params; // TODO: Should the device assume the parameters are sound, or should it validate them?
    ac97_set_sound_params(ac_state, params); // only one stream, so configure its desired parameters
    // init output bdl
    if(ac97_init_output_bdl(ac_state) == -1){
        return -1;
    }
    return ac_state->stream;
}

// TODO: Write close_stream, and have it free the stream object then set it explicitly to NULL 

void ac97_close_stream(void* state, struct nk_sound_dev_stream *stream){
    /*
    Frees all the malloced resources from open stream and ac97_init_output_bdl

     CURRENTLY ERRORS WHEN CALLED (not in the function, calling the function itself causes the error)
     ERROR is invalid opcode
    */
    struct ac97_state *ac_state = (struct ac97_state*) state;

//     free stream and set to null

    DEBUG("Deallocating the stream struct in device state\n");
    free(ac_state->stream);
    ac_state->stream = NULL;

//     free bdl entries
    ac97_deinit_output_bdl(ac_state);
}

static void probe_variable_sample_rates(struct ac97_state *state)
{
    /* According to Page 13 of https://www.analog.com/media/en/technical-documentation/data-sheets/AD1887.pdf, 
       the DAC/ADC rate registers allow programming of the sampling frequency from 7 kHz to 48 kHz in 1 Hz increments. 
       Programming a value outside that range causes the codec to saturate. For valid inputs, if the value written is supported,
       the device will echo it back. Otherwise, the closest rate supported is returned. 

       This function probes the device for all possible inputs between 7 and 48 kHz, and places the supported inputs in the 
       sample_rates_list of the passed ac97_state object. 
    */
    uint16_t port = state->ioport_start_bar0 + AC97_NAM_ADC_RATE;
    uint16_t curr_rate = 0x1B58; // 7000
    uint16_t last_valid_rate = 0;
    uint16_t returned_rate;

    while (curr_rate <= 0xBB80) // 48000
    {
        OUTW(curr_rate, port);
        returned_rate = INW(port);

        /* This block was used to validate that all 41000 possible frequency values between 7000 and 48000 were accepted 
           by the QEMU AC97 via grep from the CLI. The string "was rejected!" was never found, so all rates were accepted.

        if (returned_rate != curr_rate)
        {
            DEBUG("Requested rate %d was rejected!", curr_rate);
        }
        */

        if (returned_rate != last_valid_rate)
        {
            struct ac97_sample_rate *rate = (struct ac97_sample_rate *)malloc(sizeof(struct ac97_sample_rate));
            if (!rate)
            {
                ERROR("Could not allocate AC97 Sample Rate struct!\n");
                return -1;
            }
            rate->rate = returned_rate;
            list_add(&rate->node, &state->sample_rates_list);

            last_valid_rate = returned_rate;
            curr_rate = returned_rate; // speeds up probing by moving up to the next valid rate
        }
        curr_rate += 1;
    }
}

int ac97_produce_out_buffer(struct ac97_state *state, void *buffer, uint16_t num_samples)
{
    /* Recieves a pointer to a chunk of data in memory, and the size of the buffer.
       This function assumes the buffer is compliant with the byte granularity decided upon in the ac97_state.

        NOTE: The inputted ac97_state must be named 'state' for macros to work properly
        NOTE: This function assumes the user is responsible for chopping up data into buffers with at most 0xFFFE samples
    */

    /* Our code maintains the buffer size; we don't want to overwrite unconsumed buffer entries */
    /* TODO: Should there be some queue of next buffer entries to process? If so, we should add
             this request to the queue instead of raising an error */
    if (OUT_SIZE == BDL_MAX_SIZE)
    {
        ERROR("Attempted to overwrite an unconsumed buffer entry!\n");
        return -1;
    }

    /* Check input buffer size for compliance with what the AC97 expects */
    if (num_samples > BDL_ENTRY_MAX_SIZE)
    {
        ERROR("Attempted to write a buffer with too many samples for an AC97!\n");
        return -1;
    }

    /* Write the buffer entry */
    uint8_t write_pos = OUT_TAIL;                 // tail will always point to the next writeable position
    
    DEBUG("Updating the PCM OUT BDL entry at position 0x%x\n", write_pos);
    OUT_ENTRY_ADDR(write_pos) = (uint32_t) buffer; // cast void* to a uint32_t (must be under 4GB memory)
    OUT_ENTRY_SIZE(write_pos) = num_samples;
    OUT_ENTRY_LAST(write_pos) = 1; // most recently written buffer is always last
    OUT_ENTRY_IOC(write_pos) = 1;  // turn on ioc transfer interrupt so the handler can manage the buffer size when data is transferred
    DEBUG("BDL entry now states: %016lx\n", OUT_ENTRY(write_pos).val);

    /* Tell the device what the new last valid entry is */
    last_valid_entry_t lve;
    lve.last_entry = write_pos; // BDL_INC(write_pos, 1)?
    lve.rsvd = 0;
    DEBUG("Reporting to device the new last valid entry...\n");
    OUTB(lve.val, state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TOTAL);

    /*
    Manage our BDL Ring's state variables
    */
    OUT_TAIL = BDL_INC(write_pos, 1); // increment the tail

    /* Turn off the last bit of the previous entry; we just wrote the new last */
    if (OUT_SIZE > 0)
    {
        // Is there a nice macro for BDL_DEC that wraps around?
        if (write_pos == 0)
            write_pos = 31;
        else
            write_pos -= 1;
        OUT_ENTRY_LAST(write_pos) = 0;
    }

    OUT_SIZE += 1; // update effective size of BDL

    return 0;
}

int ac97_write_to_stream(void *state, struct nk_sound_dev_stream *stream,
                        uint8_t *src, uint64_t len,
                        void (*callback)(nk_sound_dev_status_t status,
                                         void *context),
                        void *context) 
{
  DEBUG("Writing to stream\n");

  if (!state) {
    ERROR("The device state pointer is null\n");
    return -1;
  }
  //we don't care if stream param is null because there is only one stream in the AC97
  /* if (!stream) {
    ERROR("The stream pointer is null\n");
    return -1;
  } */

  struct ac97_state *ac_state = (struct ac97_state*) state;
  

  if (!ac_state->stream) {
    ERROR("Stream has not been opened yet\n");
    return -1;
  }

  // initialize BDL
  DEBUG("Writing data to BDL\n");
  if (ac97_write_to_bdl(ac_state, src, len, callback, context)) {
    ERROR("BDL write failed\n");
    return -1;
  }
  DEBUG("Completed BDL write\n");

  return 0;
}


//TODO: handle the usage of callbacks. Right now implementation is non blocking
int ac97_write_to_bdl(struct ac97_state *state, uint8_t *src, uint64_t len,
                        void (*callback)(nk_sound_dev_status_t status,
                                         void *context),
                        void *context )
{

    if(!src){
        ERROR("The src pointer is null\n");
        return -1;
    }

    if((*src & 1) != 0){
        ERROR("The src address must be even\n");
        return -1;
    }

    uint8_t n_available_buffers = BDL_MAX_SIZE - OUT_SIZE;

    //assuming use of 16 bit samples, and len is given in number of bytes (a byte might get cutoff, should I use ceil here too?)
    uint64_t n_samples = len / 2;
    DEBUG("Total number of samples %d\n", n_samples);

    //TODO: Fix,this will break when n_samples % bdl_entry_max_size = 0...ceil is like sin: returns itself.
    uint8_t buffers_required =(uint8_t)  ( ((double) n_samples/ (double) BDL_ENTRY_MAX_SIZE)+ 1);
    DEBUG("Buffers required: %d\n", buffers_required);

    if(buffers_required > n_available_buffers){
        ERROR("BDL does not have enough space for sound data, %d buffers required and %d open spots\n",buffers_required,n_available_buffers);
        return -1;
    }

    //chopping up samples into buffer entries and placing them in bdl
    for (int i = 0; i < buffers_required; i++) {
            DEBUG("Buffer %d will be at address %x\n", i, (uint32_t)(src + (i * 0xFFFE)));
            if (n_samples >= 0xFFFE){
                 ac97_produce_out_buffer(state, src + (i * 0xFFFE), 0xFFFE); // TODO: Is alignment okay here? Seemed sketchy during OH
                 n_samples -= 0xFFFE;
            }
            else{
                ac97_produce_out_buffer(state,src + ((i*0xFFFE)), (uint16_t) n_samples);
                n_samples = 0;
            }    
    }

    return 0;



}



static int write_to_bdl_test(char *buf, void *priv)
{
    /* Adds nbuf full-length sine wave sound buffers playing sound at frequency freq */
    uint64_t nBytes = 200000;
    uint64_t freq = 261;

    
        DEBUG("Adding %d sound buffers playing a sine wave at frequency %d\n", nBytes, freq);

        // Create one huge sine buffer
        uint16_t *sine_buf = (uint16_t *) malloc(nBytes); // each sample is 2 bytes
        if (!sine_buf) {
            nk_vc_printf("ERROR: Could not allocate full-length sound buffer\n");
            return 0;
        }
        create_sine_wave(sine_buf, nBytes / 2, freq, 44100); // TODO: pull sampling freq from device state
        DEBUG("Allocated a large sine buffer at address %x\n", (uint32_t) sine_buf);


        // produce using the write to bdl function
        ac97_write_to_bdl(dirty_state,(uint8_t*) sine_buf,nBytes,NULL,NULL);
        return 0;
   
    
}

static struct shell_cmd_impl write_to_bdl_impl = {
    .cmd = "write_to_bdl",
    .help_str = "write_to_bdl",
    .handler = write_to_bdl_test,
};
nk_register_shell_cmd(write_to_bdl_impl);


static int print_bdl_out(struct ac97_state *state)
{
    /* 
    Prints the contents of the PCM Output Box's Buffer Descriptor List to the Debug console.
    */
    DEBUG("Printing the contents of the PCM OUT BDL...\n");
    for (int i = 0; i < BDL_MAX_SIZE; i++)
    {
        ac97_bdl_entry entry = OUT_ENTRY(i);
        DEBUG("Entry %d: %016lx\n", i, entry.val);
    }
    return 0;
}



int ac97_consume_out_buffer(struct ac97_state *state)
{
    /* Frees the buffer most recently consumed by the AC97 device.

       NOTE: The inputted ac97_state must be named 'state' for macros to work properly
    */

    /* Our code maintains the buffer size; we can't consume if there are no entries */
    if (OUT_SIZE == 0)
    {
        ERROR("Attempted to consume a non-existent BDL entry!\n");
        return -1;
    }

    /* Ask the device what the current processed entry is. Error if we are misaligned. */
    // TODO: Ideally, this error check could be removed altogether, but I'd rather keep it for safety
    // NOTE: The device increments the APE as it fires the interrupt that the previous APE was consumed
    DEBUG("Asking the device for the value of the APE...\n");
    uint8_t curr_entry = INB(state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_APE);
    if (((last_valid_entry_t)curr_entry).last_entry != BDL_INC(OUT_HEAD, 1))
    {
        // The if check above is weird, but the last_valid_entry register has the same structure
        // as the actual_processed_entry register, so I'm just taking advantage of the existing struct
        ERROR("State variable is not consistent with device!\n");
        return -1;
    }

    /* Ask the device how many samples it has transferred from the current processed entry.
       Error if we are somehow consuming the buffer before all samples have been transferred.
    */
    /*
    TODO: This code always seems to raise an error. I wonder if the QEMU emulation doesn't properly update
          how many samples have been transferred from the APE, or if we're doing something incorrectly.

    uint16_t trans_samples = INW(state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TRANS);
    if (trans_samples != OUT_ENTRY_SIZE(OUT_HEAD))
    {
        ERROR("Attempting to consume a buffer that the device has not finished consuming!\n");
        return -1;
    }
    */

    /* Free the contents of the consumed buffer, then reset the buffer entry */
    // TODO: Perhaps the user space should be responsible for freeing the buffers
    uint8_t free_pos = OUT_HEAD;

    DEBUG("Clearing the buffer at position %d of the PCM OUT BDL\n", free_pos);
    // free((void*) OUT_ENTRY_ADDR(free_pos)); // TODO: Should the driver be responsible for freeing buffers, or should the application clean them up?
    OUT_ENTRY_ADDR(free_pos) = 0;
    OUT_ENTRY_SIZE(free_pos) = 0;
    OUT_ENTRY_LAST(free_pos) = 0;
    OUT_ENTRY_IOC(free_pos) = 0;

    /*
    Manage our BDL Ring's state variables
    */
    OUT_HEAD = BDL_INC(free_pos, 1); // increment the head
    OUT_SIZE -= 1;                   // update effective size of BDL

    return 0;
}

static int handler (excp_entry_t *excp, excp_vec_t vector, void *priv_data)
{
    DEBUG("Interrupt handler caught %x\n", vector);
    struct ac97_state *state = (struct ac97_state*) priv_data;

    /* Pause device 
    DEBUG("Pausing device while we handle the interrupt...\n");
    uint8_t tc_int_td = INB(state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    transfer_control_t tc_td = (transfer_control_t)tc_int_td;
    tc_td.dma_control = 0;
    OUTB(tc_td.val, state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL); */

    /* Track Transfer Status register in a struct so we can handle the interrupt the device raised */
    uint16_t trans_status = INW(state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);
    transfer_status_t tr_stat = (transfer_status_t)trans_status;
    DEBUG("Transfer status register contents: %x\n", tr_stat.val);

    /* Should the device halt when it sends an interrupt? (Apparently not)*/
    // if (tr_stat.dma_status == 0) ERROR("Device is not halted, but we're handling an interrupt!\n");
    
    /* Handle the interrupt internally based on the contents of the Transfer Status register. */
    // TODO: We may want to handle different interrupts differently. Implement this.
    // TODO: When multiple NABM registers are being used (e.g. sound is being played and recorded), 
    //       we'll need to clear the interrupt from the correct box. So far, this code assumes interrupts
    //       are raised from the output box only. 
    if (tr_stat.lbe_interrupt == 1) {
        DEBUG("Handling last buffer entry interrupt...\n");
        ac97_deinit_output_bdl(state);
        // OUTB(0x1C, state->ioport_end_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);
    }
    else if (tr_stat.ioc_interrupt == 1) {
        DEBUG("Handling ioc interrupt from a consumed buffer...\n");
        ac97_consume_out_buffer(state);
        // OUTB(0x1C, state->ioport_end_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);

        /*
        Test that the device properly manages the ring buffer state variables by playing
        continuous sound
        */
       /*  uint64_t buf_len = 0x1000;
        uint16_t *sine_buf = (uint16_t *) malloc(2 * buf_len); // each sample is 2 bytes
        if (!sine_buf)
        {
            nk_vc_printf("ERROR: Could not allocate half-length sound buffer\n");
            return 0;
        }
        uint64_t tone_freq = (OUT_HEAD * (500 - 200) / (BDL_MAX_SIZE - 1)) + 200; // stepped frequency between 200 and 500
        create_sine_wave(sine_buf, buf_len, tone_freq, 44100); // TODO: pull sampling freq from device state
        ac97_produce_out_buffer(state, sine_buf, 0x1000); */
    }
    else if (tr_stat.fifo_interrupt == 1) {
        DEBUG("Handling fifo error interrupt...\n");
        // OUTB(0x1C, state->ioport_end_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);
    }
    else {
        ERROR("Transfer status register does not indicate a known interrupt\n");
        return -1;  
    }

    DEBUG("Interrupt has been handled, clearing device register...\n");
    OUTW(0x001C, state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);

    /* After we handle the interrupt internally, tell the CPU and then clear the device */
    DEBUG("Telling APIC to move on from this interrupt...\n");
    IRQ_HANDLER_END(); // Lets APIC know interrupt is over

    /* For debugging purposes, print out the BDL contents after our processing */
    // print_bdl_out(state);

    /* Resume device 
    DEBUG("Resuming device since the interrupt is handled...\n");
    tc_int_td = INB(state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    tc_td = (transfer_control_t) tc_int_td;
    tc_td.dma_control = 1;
    OUTB(tc_td.val, state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL); */

    return 0;
}

static struct nk_sound_dev_int ops = {
        .get_available_modes = ac97_get_available_modes,
        .open_stream = ac97_open_stream,
        .close_stream = ac97_close_stream,
        .write_to_stream = NULL, //TODO: implement function. Ensure the agreed sampling frequency is set for both the DAC and ADC rates!
        .read_from_stream = NULL, //TODO implement function
        .get_stream_params = NULL, //TODO: implement function
        .play_stream = NULL, //TODO: implement function
        .close_stream = NULL
};

int ac97_pci_init(struct naut_info *naut)
{
    struct pci_info *pci = naut->sys.pci;
    struct list_head *curbus, *curdev;
    uint16_t num = 0;

    DEBUG("init\n");

    if (!pci)
    {
        ERROR("No PCI info\n");
        return -1;
    }

    INIT_LIST_HEAD(&dev_list);

    DEBUG("Finding AC97 devices\n");

    list_for_each(curbus, &(pci->bus_list))
    {
        struct pci_bus *bus = list_entry(curbus, struct pci_bus, bus_node);

        DEBUG("Searching PCI bus %u for AC97 devices\n", bus->num);

        list_for_each(curdev, &(bus->dev_list))
        {
            struct pci_dev *pdev = list_entry(curdev, struct pci_dev, dev_node);
            struct pci_cfg_space *cfg = &pdev->cfg;

            DEBUG("Device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
            // intel vendor id and ac97 device id
            if (cfg->vendor_id == INTEL_VENDOR_ID && cfg->device_id == AC97_DEVICE_ID)
            {
                int foundio = 0, foundmem = 0;

                DEBUG("Found AC97 Device\n");
                struct ac97_state *state = malloc(sizeof(struct ac97_state));
                
                if (!state)
                {
                    ERROR("Cannot allocate device\n");
                    return -1;
                }

                memset(state, 0, sizeof(*state));

                // We will only support MSI for now

                // find out the bar for AC97
                //TODO: edit from here
                // 6 -> # of BARs
                
		        for (int i = 0; i < 6; i++)
                {
                    uint32_t bar = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i * 4);
                    uint32_t size;
                    DEBUG("bar %d: 0x%0x\n", i, bar);
                    // go through until the last one, and get out of the loop
                    if (bar == 0)
                    {
                        break;
                    }
                    // get the last bit and if it is zero, it is the memory
                    // " -------------------------"  one, it is the io
                    if (!(bar & 0x1))
                    {
                        uint8_t mem_bar_type = (bar & 0x6) >> 1;
                        if (mem_bar_type != 0)
                        { // 64 bit address that we do not handle it
                            ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
                            return -1;
                        }
                    }

                    // determine size
                    // write all 1s, get back the size mask
                    pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i * 4, 0xffffffff);
                    // size mask comes back + info bits
                    // write all ones and read back. if we get 00 (negative size), size = 4.
                    size = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i * 4);

                    // mask all but size mask
                    if (bar & 0x1)
                    { // I/O
                        size &= 0xfffffffc;
                    }
                    else
                    { // memory
                        size &= 0xfffffff0;
                    }
                    // two complement, get back the positive size
                    size = ~size;
                    size++;

                    // now we have to put back the original bar
                    pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i * 4, bar);

                    if (!size)
                    { // size = 0 -> non-existent bar, skip to next one
                        continue;
                    }

                    uint32_t start = 0;
                    if (bar & 0x1 && (i==0))
                    { // IO
                        start = state->ioport_start_bar0 = bar & 0xffffffc0;
                        state->ioport_end_bar0 = state->ioport_start_bar0 + size;
                        foundio = 1;
                    }
                    if (bar & 0x1 && (i==1))
                    { // IO
                        start = state->ioport_start_bar1 = bar & 0xffffffc0;
                        state->ioport_end_bar1 = state->ioport_start_bar1 + size;
                        foundio = 1;
                    }

                    DEBUG("bar %d is %s address=0x%x size=0x%x\n", i,
                          bar & 0x1 ? "io port" : "memory", start, size);
                }

		

                INFO("Adding ac97 device: bus=%u dev=%u func=%u: ioport_start_bar0=%p ioport_end_bar0=%p ioport_start_bar1=%p ioport_end_bar1=%p\n",
                      bus->num, pdev->num, 0,
                      state->ioport_start_bar0, state->ioport_end_bar0,
                      state->ioport_start_bar1, state->ioport_end_bar1);

                /* REFERENCE: https://wiki.osdev.org/AC97
                "In initalization of sound card you must resume card from cold reset and set power for it. It can be done by write value 0x2 to 
                Global Control register if you do not want interrupts, or 0x3 if you want interrupts. After this, you should write any value to 
                NAM reset register to set all NAM register to their defaults. After this, you can read card capability info from Global Status 
                register what means how many channels it suport and if is 20 bit audio samples supported. Now is sound card ready to use."
                */
                // NOTE: Writing 0x2 disables interrupts, but still resets device
		        OUTL(0x00000003, state->ioport_start_bar1 + AC97_NABM_CTRL);

                // Write to NAM reset register to default the device
		        OUTW(0x0001, state->ioport_start_bar0);

                // Read card capabilities register
                uint32_t stat_reg = INL(state->ioport_start_bar1 + AC97_NABM_STATUS);

                /* Isolate 20th and 21st bits to see how many channels this AC97 can support */
                uint32_t channels_supported = (stat_reg & 0x00300000) >> 20; 
                
                // Translate bits to integer channel capabilities
                if (channels_supported == 0) {
                    state->max_channels = 2;
                }
                else if (channels_supported == 1) {
                    state->max_channels = 4;
                }
                else if (channels_supported == 2) {
                    state->max_channels = 6;
                }
                else DEBUG("AC97 channel bits are indicating reserved behavior.");

                /* Isolate 22nd and 23rd bits to see the maximum supported bit granularity of samples */
                uint32_t max_bit_samples = (stat_reg & 0x00C00000) >> 22;
                // Translate bits to integer channel capabilities
                if (max_bit_samples == 1) state->max_resolution = 20;
                else state->max_resolution = 16;

                DEBUG("This AC97 supports up to %d channels and %d-bit samples.\n", state->max_channels, state->max_resolution);

                // Check extended capabilities register, which may allow for variable sample rates
                DEBUG("Checking extended capabilities register...\n");
                uint16_t caps = INW(state->ioport_start_bar0 + AC97_NAM_EXT_FUNC);

                /* NOTE:
                As of 5/23/2023, the QEMU AC97 returns 0x0809 when we read the extended capabilities register. This simply
                indicates that the device's AC97 codec is compliant with Revision 2.3, and that variable rate audio is
                possible.
                Reference: https://www.analog.com/media/en/technical-documentation/data-sheets/AD1986A.pdf (Pages 31/32)

                We will leave an ERROR() check here in case the QEMU AC97 ever returns a different value, so that
                more code can be added to properly handle the extra configuration options.
                */
                if (caps != 0x0809)
                {
                    ERROR("Extended capabilities register indicates features that have not been implemented in software!\n");
                    return -1;
                }

                /* Initialize our sample rates list */
                INIT_LIST_HEAD(&state->sample_rates_list);

                // First bit indicates variable rate audio is supported; we must enable it then probe for all supported 
                // sample rates to fill out our sample rates list. If it isn't supported, just ask the device for the 
                // default rate and add that to the list. 
                if (caps & 0x0001) {
                    DEBUG("Enabling variable rate audio support...\n");
                    OUTW(0x0001, state->ioport_start_bar0 + AC97_NAM_EXT_FUNC_EN);
                    // probe_variable_sample_rates(state);
                } else {
                    DEBUG("Asking the AC97 for its default sample rate...\n");
                    uint16_t default_rate = INW(state->ioport_start_bar0 + AC97_NAM_DAC_RATE); // will be same as value from AC97_NAM_ADC_RATE register
                    struct ac97_sample_rate* rate = (struct ac97_sample_rate*) malloc(sizeof(struct ac97_sample_rate));
                    if (!rate) {
                        ERROR("Could not allocate AC97 Sample Rate struct!\n");
                        return -1;
                    }
                    rate->rate = default_rate;
                    list_add(&rate->node, &state->sample_rates_list);
                }
                /*
                // Print supported rates to the debug console
                struct list_head *currate;
                list_for_each(currate, &state->sample_rates_list) {
                    struct ac97_sample_rate *rate = list_entry(currate, struct ac97_sample_rate, node);
                    DEBUG("AC97 supports sampling at %d Hz", rate->rate);
                }
                */

                /* Allow device to use DMA */
                uint16_t old_cmd = pci_cfg_readw(bus->num, pdev->num, 0, 0x4);
                DEBUG("Old PCI CMD: 0x%04x\n", old_cmd);

                old_cmd |= 0x7; // make sure bus master is enabled
                // old_cmd &= ~0x40; not sure what this does 

                DEBUG("New PCI CMD: 0x%04x\n", old_cmd);

                pci_cfg_writew(bus->num, pdev->num, 0, 0x4, old_cmd);

                uint16_t stat = pci_cfg_readw(bus->num, pdev->num, 0, 0x6);
                DEBUG("PCI STATUS: 0x%04x\n", stat);

                /* Create interrupt vector for AC97 */
                // PCI Interrupt (A..D)
                // state->pci_intr = cfg->dev_cfg.intr_pin;

                // GRUESOME HACK
                /* Enable these wires to register possible interrupts */
                nk_unmask_irq(8);
                nk_unmask_irq(9);
                nk_unmask_irq(10);
                nk_unmask_irq(11);
                nk_unmask_irq(12);
                nk_unmask_irq(13);
                nk_unmask_irq(14);
                nk_unmask_irq(15);

                register_int_handler(0xe4, handler, state); // AC97 raises the 0xE4 interrupt vector (found via trial-and-error)

                /* Register this AC97 device */
                list_add(&dev_list, &state->ac97_node);
                sprintf(state->name, "ac97-%d", num);
                num++;

                state->pci_dev = pdev;

                state->sounddev = nk_sound_dev_register(state->name, 0, &ops, (void *)state);
               
                if (!state->sounddev)
                 {
                     ERROR("init fn: Cannot register the ac97 device \"%s\"", state->name);
                     return -1;
                 }

                // Initialize the AC97 PCM Output Ring Buffer BDL and its Description
                // ac97_init_output_bdl(state);

                /*
                Set master volume and PCM output volume to some defaults
                */
                DEBUG("Setting master and PCM output volumes...\n");
                uint16_t vol_int_m = INW(state->ioport_start_bar0 + AC97_NAM_MASTER_VOL);
                uint16_t vol_int_p = INW(state->ioport_start_bar0 + AC97_NAM_PCM_OUT_VOL);
                m_volume_t master_vol = (m_volume_t)vol_int_m;
                pcm_volume_t pcm_vol = (pcm_volume_t)vol_int_p;

                // Master volume is stepped from 0-63. Using 32 because it is somewhere in the middle (arbitrary)
                master_vol.l_volume = 32;
                master_vol.r_volume = 32;
                master_vol.mute = 0; // Ensure the master volume is not mute

                // PCM Output volume is stepped from 0-31. Using 16 because it is somewhere in the middle (arbitrary)
                pcm_vol.l_volume = 16;
                pcm_vol.r_volume = 16;
                pcm_vol.mute = 0; // Ensure the PCM volume is not mute

                OUTW(master_vol.val, state->ioport_start_bar0 + AC97_NAM_MASTER_VOL);
                OUTW(pcm_vol.val, state->ioport_start_bar0 + AC97_NAM_PCM_OUT_VOL);

                /* Initialize a stream */
                state->stream = NULL;

                /* TODO: Write default volume parameters to a stream so they can be configured? */


                /* TODO: Should the BDLs be initialized here? */
                /* TODO: Initialize a stream with some defualt parameters for playing sound. */

                dirty_state = state; // TODO: Remove this code when we can get sound working in a non-dirty way

                // if (!foundmem)
                //
                //     ERROR("init fn: ignoring device %s as it has no memory access method\n", state->name);
                //     continue;
                // }
                //DEBUG("AC97 device is now registered");


                // // now bring up the device / interrupts

            }
        }
    }

    return 0;
}

int ac97_dirty_sound()
{
    /* 
    The ac97_pci_init() function allocates a dirty static AC97 state struct in its last line.
    This function uses that state to write directly to the PCI bus to set up some sound to
    be played. 
    */
    DEBUG("Testing sound through dirty AC97 static state struct...\n");

    /*
    The code in this function follows the directions from https://wiki.osdev.org/AC97 under
    the section "Playing Sound"
    ---------------------------------------------------------------------------------------
    */

    /*
    Step 1) Set master volume and PCM output volume
    */
    DEBUG("Setting master and PCM output volumes...\n");
    uint16_t vol_int_m = INW(dirty_state->ioport_start_bar0 + AC97_NAM_MASTER_VOL);
    uint16_t vol_int_p = INW(dirty_state->ioport_start_bar0 + AC97_NAM_PCM_OUT_VOL);
    m_volume_t master_vol = (m_volume_t) vol_int_m;
    pcm_volume_t pcm_vol = (pcm_volume_t) vol_int_p;

    // Master volume is stepped from 0-63. Using 32 because it is somewhere in the middle (arbitrary)
    master_vol.l_volume = 32;
    master_vol.r_volume = 32;
    master_vol.mute = 0; // Ensure the master volume is not mute

    // PCM Output volume is stepped from 0-31. Using 16 because it is somewhere in the middle (arbitrary)
    pcm_vol.l_volume = 16;
    pcm_vol.r_volume = 16;
    pcm_vol.mute = 0; // Ensure the PCM volume is not mute

    OUTW(master_vol.val, dirty_state->ioport_start_bar0 + AC97_NAM_MASTER_VOL);
    OUTW(pcm_vol.val, dirty_state->ioport_start_bar0 + AC97_NAM_PCM_OUT_VOL);

    /*
    Step 2) Load sound data to memory and describe it in Buffer Descriptor List
    */
    // Create an audio buffer and fill it with sine wave samples
    uint64_t sampling_frequency = 44100;
    uint32_t duration = 1;
    uint64_t tone_frequency = 261; // middle C

    // With the parameters above, we'd actually need to create multiple BDL entries to store the
    // sound data. An entry can only transfer up to 0xFFFE samples!
    uint16_t buf_len = 0xFFFE; // uint64_t buf_len = sampling_frequency * duration * 4;

    uint16_t *sine_buf = (uint16_t *)malloc(2*buf_len); // Max had this as a uint8_t, I think it should be 16
    DEBUG("Creating and filling an audio buffer with %x sine wave samples %p...\n", buf_len, sine_buf);
    create_sine_wave(sine_buf, buf_len, tone_frequency, sampling_frequency);

    // Initialize the BDL
    // THEORY: For now, we might not have to mess with allocating a full ring buffer.
    //         I think we can just describe a single entry and let the device know that it's the only one.
    DEBUG("Initializing the sine wave BDL entry...\n");
    ac97_bdl_entry sine_entry;
    sine_entry.addr = (uint32_t) sine_buf; // NOTE: sine_buf must be less than 4GB
    sine_entry.last = buf_len; 
    sine_entry.last = 1; // Notes that this is the last entry in the list
    sine_entry.ioc = 1;

    /* 
    Step 3) Set reset bit of output channel and wait for card to clear it 
    */
    DEBUG("Setting reset bit of output box...\n");
    uint8_t tc_int = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    transfer_control_t tc = (transfer_control_t) tc_int;
    tc.reset = 1;
    OUTB(tc.val, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);

    // Wait for device to clear the reset bit
    while(true) 
    {
        uint8_t tc_int = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
        transfer_control_t tc = (transfer_control_t)tc_int;
        if (tc.reset == 0) break;
        else nk_sleep(100000000); // 100 ms
    }
    DEBUG("Device has cleared the reset bit...\n");

    /*
    Step 4) Write physical position of BDL to output NABM register
    */
    // Write phyiscal position of BDL to the output box
    DEBUG("Telling device that the BDL is located at %x...\n", &sine_entry);
    OUTL((uint32_t) &sine_entry, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_ADDR);

    // First bit is read-only, and 0. I think this is for the sake of alignment of addresses.
    // How do we enure a kmem_malloc'd address is always even? 
    DEBUG("SANITY CHECK: Asking device where the BDL is located...\n");
    uint32_t bdl_pmio_addr = INL(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_ADDR);
    DEBUG("BDL is at: %x\n", bdl_pmio_addr); // First bit is Read-Only... how do we write an address of 31 bits?

    /*
    According to this manual: https://www.intel.com/Assets/PDF/manual/252751.pdf
    Only the last bit of the buffer pointer must be 0. This is to ensure samples never straddle
    DWord boundaries. I think this means that we don't need to do any manipulation of the addresses
    as we write them, since won't they always be even to align the structs anyways? 
    */

    /*
    STEP 5) Write number of last valid buffer entry to NABM register 
    */
    // grabbing struct, setting relevent field, rewriting
    uint8_t lve_int = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TOTAL);
    last_valid_entry_t lve = (last_valid_entry_t) lve_int;
    lve.last_entry = 0; // Should this denote the last entry, or (last_entry + 1)? 
    DEBUG("Telling the device there is only one buffer entry...\n");
    OUTB(lve.val, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TOTAL);

    /*
    STEP 6) Set bit to activate transfer of data
    */
    // grabbing struct, setting relevant field, rewriting
    uint8_t tc_int_td = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    transfer_control_t tc_td = (transfer_control_t) tc_int_td;
    tc_td.dma_control = 1;
    // Turn on all interrupts for now
    tc_td.lbe_interrupt = 1;
    tc_td.ioc_interrupt = 1;
    tc_td.fifo_interrupt = 1;

    DEBUG("Setting bit to transfer data...\n");
    OUTB(tc_td.val, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);

    /* Check transfer status register to ensure DMA controller status is active */
    uint16_t trans_status = INW(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);
    transfer_status_t tr_stat = (transfer_status_t) trans_status;
    if (tr_stat.dma_status == 1) DEBUG("DMA is halted for some reason...\n");
    else DEBUG("DMA has begun transferring data...\n");

    /* 
    STEP 7) Play sound (the device DMA should handle this automatically).
            Include any necessary debugging statements here. 
    */
    uint8_t current_entry = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_APE);
    DEBUG("Currently processing entry %d\n", current_entry);
    uint8_t next_entry = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_NEXT);
    DEBUG("Next entry to process: %d\n", next_entry);

    /* Continuously check transfer status register and wait until the buffer is consumed */
    while(true) {
        // TODO: Why does the device think no samples have been transferred, even if sound is being played? 
        uint16_t trans_status = INW(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS);
        transfer_status_t tr_stat = (transfer_status_t) trans_status;

        //OUTW(0x1C, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_STATUS); // Clear interrupt from status reg

        if (tr_stat.end_of_transfer == 1) break;
        else {
            uint16_t read_pointer = INW(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TOTAL);
            DEBUG("%d samples have been transferred so far\n", read_pointer); 
            nk_sleep(100000000); // 100 ms
        }
    }

    /* Put Nautilus to sleep for 5s while the audio is hopefully played. Afterwards, free the buffer. */
    //DEBUG("Sleeping for 5s...\n");
    //nk_sleep(5000000000); 

    /* Free buffer entry that we just allocated */
    DEBUG("Freeing buffer entry from memory...\n");
    free(sine_buf);

    return 0;
}

static int handle_test_sound(char *buf, void *priv)
{
    return ac97_dirty_sound();
}

static struct shell_cmd_impl test_sound_impl = {
    .cmd = "test_dirty_sound",
    .help_str = "test_dirty_sound (no arguments)",
    .handler = handle_test_sound,
};
nk_register_shell_cmd(test_sound_impl);

// TODO: This should be done in a way such that the user doesn't have to call a command to callocate the BDL, 
//       but is still automatic after boot. We can't allocate the BDL during boot, otherwise, we can't call 
//       nk_sleep to make sure the device has cleared the reset bit on the NABM PCM OUT register box. 
//       This function should also be made less dirty (don't use dirty_state)
static int handle_create_bdl(char *buf, void *priv)
{
    return ac97_init_output_bdl(dirty_state);
}
static struct shell_cmd_impl create_bdl_impl = {
    .cmd = "create_bdl",
    .help_str = "create_bdl",
    .handler = handle_create_bdl,
};
nk_register_shell_cmd(create_bdl_impl);


int ac97_init_output_bdl(struct ac97_state* state) {
    /* Helper function to initialize and allocate the Buffer Descriptor List of an AC97 Device */
    // NOTE: the inputted ac97_state* must be named 'state' for macros to work properly

    /* Allocate the BDL Description object for the PCM OUT box */
    DEBUG("Allocating BDL description for PCM OUT box\n");
    void* bdl_pointer = malloc(sizeof(struct ac97_bdl_desc));
    if (!bdl_pointer) {
        ERROR("Cannot allocate description for the BDL of the PCM OUT box\n");
        return -1; 
    }
    BDL_OUT = (struct ac97_bdl_desc*) bdl_pointer; // the ac97_state expects a ac97_bdl_desc*

    /* Now allocate the BDL Ring, a contiguous space of size BDL_MAX_SIZE*sizeof(ac97_bdl_entry) */
    DEBUG("Allocating the BDL ring buffer for the PCM OUT box\n");
    OUT_RING = malloc(BDL_MAX_SIZE*sizeof(ac97_bdl_entry)); // the ac97_state expects a void* (macros cast it as needed)
    if (!OUT_RING) {
        ERROR("Cannot allocate the BDL ring buffer of the PCM OUT box\n");
        return -1;
    }
    memset(OUT_RING, 0, BDL_MAX_SIZE * sizeof(ac97_bdl_entry)); // initialize entries to 0s

    /* Set other state variables to indicate empty BDL */
    DEBUG("Describing an empty BDL in the PCM OUT box\n");
    OUT_HEAD = 0;
    OUT_TAIL = 0;
    OUT_SIZE = 0;

    /*
    Set reset bit of output channel and wait for card to clear it
    */
    DEBUG("Setting reset bit of output box...\n");
    uint8_t tc_int = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    transfer_control_t tc = (transfer_control_t)tc_int;
    tc.reset = 1;
    OUTB(tc.val, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);

    // Wait for device to clear the reset bit
    while (true)
    {
        uint8_t tc_int = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
        transfer_control_t tc = (transfer_control_t)tc_int;
        if (tc.reset == 0)
            break;
        else
            nk_sleep(100000000); // 100 ms
    }
    DEBUG("Device has cleared the reset bit...\n");

    /*
    Write physical position of BDL to output NABM register
    */
    // Write phyiscal position of BDL to the output box
    DEBUG("Telling device that the BDL is located at %x...\n", OUT_RING);
    OUTL((uint32_t)OUT_RING, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_ADDR);

    return 0; // indicate success
}

int ac97_deinit_output_bdl(struct ac97_state* state) {
    /* Helper function to deallocate and remove the Buffer Descripor List of an AC97 Device */
    // NOTE: the inputted ac97_state* must be named 'state' for macros to work properly
    DEBUG("Deallocating the BDL ring buffer for the PCM OUT box\n");

    for (int i=0; i < BDL_MAX_SIZE; i++) // Free all buffer entries pointed to by the BDL 
        {free((void*) OUT_ENTRY_ADDR(i));}
    
    free(OUT_RING); // Then free the pointer to the base address of the BDL

    DEBUG("Deallocating the BDL description for the PCM OUT box\n");
    free((void*) BDL_OUT); // Then free the BDL description object

    return 0; // indicate success
}

// TODO: Undirty this function by removing its use of the dirty_state static struct
static int handle_add_sound_buffers(char *buf, void *priv)
{
    /* Adds nbuf full-length sine wave sound buffers playing sound at frequency freq */
    uint8_t nbuf;
    uint64_t freq;

    if(sscanf(buf, "add_sound_buffers %d %d", &nbuf, &freq) == 2) {
        DEBUG("Adding %d sound buffers playing a sine wave at frequency %d\n", nbuf, freq);

        // Create one huge sine buffer
        uint64_t buf_len = nbuf * 0x1000; // total number of samples
        uint16_t *sine_buf = (uint16_t *) malloc(2*buf_len); // each sample is 2 bytes
        if (!sine_buf) {
            nk_vc_printf("ERROR: Could not allocate full-length sound buffer\n");
            return 0;
        }
        create_sine_wave(sine_buf, buf_len, freq, 44100); // TODO: pull sampling freq from device state
        DEBUG("Allocated a large sine buffer at address %x\n", (uint32_t) sine_buf);


        // Chop the samples and have the AC97 add them
        for (int i = 0; i < nbuf; i++) {
            DEBUG("Buffer %d will be at address %x\n", i, (uint32_t)(sine_buf + (i * 0x1000)));
            ac97_produce_out_buffer(dirty_state, sine_buf + (i * 0x1000), 0x1000); // TODO: Is alignment okay here? Seemed sketchy during OH
        }
        return 0;
    }
    nk_vc_printf("invalid add_sound_buffers command\n");
    return 0;
}

static struct shell_cmd_impl add_sound_buffers_impl = {
    .cmd = "add_sound_buffers",
    .help_str = "add_sound_buffers nbuf freq",
    .handler = handle_add_sound_buffers,
};
nk_register_shell_cmd(add_sound_buffers_impl);

// TODO: Undirty this function by removing its use of the dirty_state static struct
static int handle_consume_sound_buffers(char *buf, void *priv)
{
    /*
    Activates the DMA of the AC97 so it can start playing sound data
    */
    // print the BDL before we consume it
    print_bdl_out(dirty_state);

    // grabbing struct, setting relevant field, rewriting
    uint8_t tc_int_td = INB(dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    transfer_control_t tc_td = (transfer_control_t) tc_int_td;
    tc_td.dma_control = 1;
    // Turn on all interrupts for now
    tc_td.lbe_interrupt = 1;
    tc_td.ioc_interrupt = 1;
    tc_td.fifo_interrupt = 1;

    DEBUG("Setting bit to transfer data...\n");
    OUTB(tc_td.val, dirty_state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_CTRL);
    return 0;
}

static struct shell_cmd_impl consume_sound_buffers_impl = {
    .cmd = "consume_sound_buffers",
    .help_str = "consume_sound_buffers",
    .handler = handle_consume_sound_buffers,
};
nk_register_shell_cmd(consume_sound_buffers_impl);

int ac97_pci_deinit()
{
    /* TODO: Call ac97_deinit_output_bdl */
    INFO("deinited and leaking\n");
    return 0;
}


int test_ac97_abs()
{
    DEBUG("\n\n\nTESTING AC97 DEVICE THROUGH ABSTRACTION LAYER\n\n\n");
    // Find ac97 device
    struct nk_sound_dev* ac97_device = nk_sound_dev_find("ac97-0");
    if (ac97_device == 0)
    {
        DEBUG("No device found\n");
        return -1;
    }else{
        DEBUG("Found Device: %s\n", ac97_device->dev.name);
    }

    //Query stream params
    struct nk_sound_dev_params params[2];
    uint32_t params_size = 2;
    int num_entries = nk_sound_dev_get_available_modes(ac97_device, params, params_size);
    if(num_entries == -1){
        DEBUG("Could not retrieve stream params\n");
        return -1;
    }else{
        for(int i=0; i<num_entries; i++) {
            DEBUG("AC97 stream params\ntype: %d\nnum_channels: %d\nsample_rate: %d\nsample_resolution: %d\n",
                  params[i].type, params[i].num_of_channels, params[i].sample_rate, params[i].sample_resolution);
        }
    }
    // Open stream
    struct nk_sound_dev_stream * ac97_stream = nk_sound_dev_open_stream(ac97_device, params);
    if (ac97_stream == -1){
        DEBUG("Could not open stream\n");
        return -1;
    }else{
        struct ac97_state *state = ac97_device->dev.state;
        uint16_t gcr_port = state->ioport_start_bar1 + AC97_NABM_CTRL;
        global_control_register_t global_control_register;
        global_control_register.val = INL(gcr_port);
        uint16_t DAC_port = state->ioport_start_bar0 + AC97_NAM_DAC_RATE;
        uint16_t sample_rate = INW(DAC_port);
        DEBUG("Opened stream with the following parameters:\nnum_of_channels: %d\nsample_rate: %d\nsample_resolution: %d\n",
              (global_control_register.chnl + 1) * 2,  sample_rate, global_control_register.out_mode ? 20 : 16);
    }
//    nk_sound_dev_close_stream(ac97_device, ac97_stream);
}

static struct shell_cmd_impl test_ac97_abs_impl = {
        .cmd = "test_ac97_abs",
        .help_str = "Test interacting with the ac97 device through the abstraction layer",
        .handler = test_ac97_abs,
};
nk_register_shell_cmd(test_ac97_abs_impl);

