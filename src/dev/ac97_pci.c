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

#define BDL_IN (state->bdl_in_desc)                                      // access BDL Descriptor for PCM IN of an ac97_state struct
#define IN_HEAD (BDL_IN->head_pos)                                       // pointer to current head of PCM IN ring
#define IN_TAIL (BDL_IN->tail_pos)                                        // pointer to current tail of PCM IN ring
#define IN_SIZE (BDL_IN->size)                                            // effective size of PCM IN ring (its overall size is BDL_MAX_SIZE)
#define IN_RING (BDL_IN->bdl_ring)                                       // pointer to the PCM IN ring itself
#define IN_ENTRY(i) (((ac97_bdl_entry *)IN_RING)[(i)])            // retrieve entire entry at index i
#define IN_ENTRY_ADDR(i) (((ac97_bdl_entry *)IN_RING)[(i)].addr)  // address to buffer of sound data for the entry at index i
#define IN_ENTRY_SIZE(i) (((ac97_bdl_entry *)IN_RING)[(i)].size)  // number of samples in the buffer of sound data for the entry at index i
#define IN_ENTRY_LAST(i) (((ac97_bdl_entry *)IN_RING)[(i)].last)  // flags to fire an interrupt signaling that this is the last entry in buffer
#define IN_ENTRY_IOC(i) (((ac97_bdl_entry *)IN_RING)[(i)].ioc)    // flags to fire an interrupt when this buffer is fully consumed

#define BDL_OUT (state->bdl_out_desc)                                     // access BDL Descriptor for PCM OUT of an ac97_state struct
#define OUT_HEAD (BDL_OUT->head_pos)                                      // pointer to current head of PCM OUT ring
#define OUT_TAIL (BDL_OUT->tail_pos)                                       // pointer to current tail of PCM OUT ring
#define OUT_SIZE (BDL_OUT->size)                                           // effective size of PCM OUT ring (its overall size is BDL_MAX_SIZE)
#define OUT_RING (BDL_OUT->bdl_ring)                                      // pointer to the PCM OUT ring itself
#define OUT_ENTRY(i) (((ac97_bdl_entry *)OUT_RING)[(i)])           // retrieve entire entry at index i
#define OUT_ENTRY_ADDR(i) (((ac97_bdl_entry *)OUT_RING)[(i)].addr) // address to buffer of sound data for the entry at index i
#define OUT_ENTRY_SIZE(i) (((ac97_bdl_entry *)OUT_RING)[(i)].size) // number of samples in the buffer of sound data for the entry at index i
#define OUT_ENTRY_LAST(i) (((ac97_bdl_entry *)OUT_RING)[(i)].last) // flags to fire an interrupt signaling that this is the last entry in buffer
#define OUT_ENTRY_IOC(i) (((ac97_bdl_entry *)OUT_RING)[(i)].ioc)   // flags to fire an interrupt when this buffer is fully consumed

#define BDL_MIC (state->bdl_mic_desc)                                     // access BDL Descriptor for PCM MIC of an ac97_state struct
#define MIC_HEAD (BDL_MIC->head_pos)                                      // pointer to current head of PCM MIC ring
#define MIC_TAIL (BDL_MIC->tail_pos)                                       // pointer to current tail of PCM MIC ring
#define MIC_SIZE (BDL_MIC->size)                                           // effective size of PCM MIC ring (its overall size is BDL_MAX_SIZE)
#define MIC_RING (BDL_MIC->bdl_ring)                                      // pointer to the PCM MIC ring itself
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
#define AC97_NAM_SAMP_RATE 0x2C      // sets sampling rate of device

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

typedef union 
{
    uint8_t val;
    struct{
        uint8_t last_entry : 5;
        uint8_t rsvd : 3;
    }__attribute__((packed));
}__attribute__((packed)) last_valid_entry_t;

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

    // TODO: What else do we need to add?
    // We will need to track the parameters of the stream, once we get to that point
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



// accessor functions for device registers

// static INLine uint32_t hda_pci_read_regl(struct ac97_state *dev, uint32_t offset)
// {
//     uint32_t result;
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movl (%1), %0" : "=r"(result) : "r"(addr) : "memory");
//     }
//     else
//     {
//         result = INL(dev->ioport_start + offset);
//     }
//     DEBUG_REGS("readl %08x returns %08x\n", offset, result);
//     return result;
// }

// static INLine uint16_t hda_pci_read_regw(struct ac97_state *dev, uint32_t offset)
// {
//     uint16_t result;
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movw (%1), %0" : "=r"(result) : "r"(addr) : "memory");
//     }
//     else
//     {
//         result = INW(dev->ioport_start + offset);
//     }
//     DEBUG_REGS("readw %08x returns %04x\n", offset, result);
//     return result;
// }

// static INLine uint8_t hda_pci_read_regb(struct ac97_state *dev, uint32_t offset)
// {
//     uint8_t result;
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movb (%1), %0" : "=r"(result) : "r"(addr) : "memory");
//     }
//     else
//     {
//         result = INB(dev->ioport_start + offset);
//     }
//     DEBUG_REGS("readb %08x returns %02x\n", offset, result);
//     return result;
// }

// static INLine void hda_pci_write_regl(struct ac97_state *dev, uint32_t offset, uint32_t data)
// {
//     DEBUG_REGS("writel %08x with %08x\n", offset, data);
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movl %1, (%0)" : : "r"(addr), "r"(data) : "memory");
//     }
//     else
//     {
//         OUTL(data, dev->ioport_start + offset);
//     }
// }

// static INLine void hda_pci_write_regw(struct ac97_state *dev, uint32_t offset, uint16_t data)
// {
//     DEBUG_REGS("writew %08x with %04x\n", offset, data);
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movw %1, (%0)" : : "r"(addr), "r"(data) : "memory");
//     }
//     else
//     {
//         OUTW(data, dev->ioport_start + offset);
//     }
// }

// static INLine void hda_pci_write_regb(struct ac97_state *dev, uint32_t offset, uint8_t data)
// {
//     DEBUG_REGS("writeb %08x with %02x\n", offset, data);
//     if (dev->method == MEMORY)
//     {
//         uint64_t addr = dev->mem_start + offset;
//         __asm__ __volatile__ ("movb %1, (%0)" : : "r"(addr), "r"(data) : "memory");
//     }
//     else
//     {
//         OUTB(data, dev->ioport_start + offset);
//     }
// }

// //sets volume, what is the difference between PCM/master vol?
// static void set_volume(struct ac97_state *dev, struct volume_t *vol){
//     hda_pci_write_regw(dev,AC97_NAM_MASTER_VOL, vol);
//     hda_pci_write_regw(dev,AC97_NAM_PCM_OUT_VOL, vol);
// }
// TODO: rx and tx desc are the transmit and recieve buffers for e100o device, not needed for sound dev???
// struct e1000e_rx_desc
// {
//     uint64_t *addr;
//     uint16_t length;
//     uint16_t checksum;
//     struct
//     {
//         uint8_t dd : 1;
//         uint8_t eop : 1;
//         uint8_t ixsm : 1;
//         uint8_t vp : 1;
//         uint8_t rsv : 1;
//         uint8_t tcpcs : 1;
//         uint8_t ipcs : 1;
//         uint8_t pif : 1;
//     } status;
//     uint8_t errors;
//     uint16_t special;
// } __attribute__((packed));

// // legacy mode
// struct e1000e_tx_desc
// {
//     uint64_t *addr;
//     uint16_t length;
//     uint8_t cso;
//     union
//     {
//         struct
//         {
//             uint8_t eop : 1;
//             uint8_t ifcs : 1;
//             uint8_t ic : 1;
//             uint8_t rs : 1;
//             uint8_t rsvd : 1;
//             uint8_t dext : 1;
//             uint8_t vle : 1;
//             uint8_t ide : 1;
//         } bit;
//         uint8_t byte;
//     } cmd;
//     struct
//     {
//         uint8_t dd : 1;
//         uint8_t ec : 1;
//         uint8_t lc : 1;
//         uint8_t rsvtu : 1;
//         uint8_t rsvd2 : 4;
//     } status;
//     uint8_t css;
//     uint16_t special;
// } __attribute__((packed));

// // TODO change the type of context to void
// struct e1000e_fn_map
// {
//     void (*callback)(nk_net_dev_status_t, void *);
//     uint64_t *context;
// };

// struct e1000e_map_ring
// {
//     struct e1000e_fn_map *map_ring;
//     // head and tail positions of the fn_map ring queue
//     uint64_t head_pos;
//     uint64_t tail_pos;
//     // the number of elements in the map ring buffer
//     uint64_t ring_len;
// };

// // Timing
// struct tsc
// {
//     uint64_t start;
//     uint64_t end;
// };
// typedef struct tsc tsc_t;

// struct operation
// {
//     // should be a postx, but I did last week.
//     tsc_t postx_map;    // measure the time in post_tx, post_rx
//     tsc_t xpkt;         // send_packet, receive_packet
//     tsc_t irq;          // irq_handler
//     tsc_t irq_unmap;    // unmap callback in the irq function
//     tsc_t irq_callback; // invoke the callback function in the irq

//     tsc_t irq_macro; // IRQ_HANDLER_END() macro
//     tsc_t irq_reg;   // reading a value from the reg in the irq function
//     tsc_t dev_wait;
//     uint32_t dev_wait_count;
// };
// typedef struct operation op_t;

// struct iteration
// {
//     op_t tx;
//     op_t rx;
//     uint32_t irq_tx;
//     uint32_t irq_rx;
//     uint32_t irq_unknown; // count the unknown irq
// };
// typedef struct iteration iteration_t;

// // add timing code
// #define TIMING 0

// #if TIMING
// #define TIMING_GET_TSC(x) ((x) = rdtsc())
// #define TIMING_DIFF_TSC(r, s, e) ((r) = (e) - (s))
// #else
// #define TIMING_GET_TSC(x)
// #define TIMING_DIFF_TSC(r, s, e)
// #endif

// struct e1000e_state
// {
//     // a pointer to the base class
//     struct nk_net_dev *netdev;
//     // pci interrupt and interupt vector
//     struct pci_dev *pci_dev;

//     // our device list
//     struct list_head node;

//     // Where registers are mapped into the I/O address space
//     uint16_t ioport_start;
//     uint16_t ioport_end;
//     // Where registers are mapped into the physical memory address space
//     uint64_t mem_start;
//     uint64_t mem_end;

//     char name[DEV_NAME_LEN];
//     uint8_t mac_addr[6];

//     struct e1000e_desc_ring *tx_ring;
//     struct e1000e_desc_ring *rxd_ring;
//     // a circular queue mapping between callback function and tx descriptor
//     struct e1000e_map_ring *tx_map;
//     // a circular queue mapping between callback funtion and rx descriptor
//     struct e1000e_map_ring *rx_map;
//     // the size of receive buffers
//     uint64_t rx_buffer_size;
//     // interrupt mark set
//     uint32_t ims_reg;

// #if TIMING
//     volatile iteration_t measure;
// #endif
// };

// // list of discovered devices
static struct list_head dev_list;
static struct ac97_state *dirty_state; 

static struct nk_sound_dev_int ops = {
    .get_available_sample_rates = NULL, //TODO: implement function
    .get_available_sample_resolution = NULL, //TODO: implement function
    .get_available_num_of_channels = NULL, //TODO: implement function
    .get_available_scale = NULL, //TODO: implement function
    .open_stream = NULL, //TODO: implement function
    .close_stream = NULL, //TODO implement function
    .write_to_stream = NULL, //TODO: implement function
    .get_stream_params = NULL, //TODO: implement function
};

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

    /* Should the device halt when it sends an interrupt? */
    if (tr_stat.dma_status == 0) ERROR("Device is not halted, but we're handling an interrupt!\n");
    
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
                if (channels_supported == 0) channels_supported = 2;
                else if (channels_supported == 1) channels_supported = 4;
                else if (channels_supported == 2) channels_supported = 6;
                else DEBUG("AC97 channel bits were set to 11, indicating reserved behavior.");

                /* Isolate 22nd and 23rd bits to see the maximum supported bit granularity of samples */
                uint32_t max_bit_samples = (stat_reg & 0x00C00000) >> 22;
                // Translate bits to integer channel capabilities
                if (max_bit_samples == 1) max_bit_samples = 20;
                else max_bit_samples = 16;

                // TODO: The information from the card capabilities register should probably be stored in
                //       the device state so we know which configuration options are possible when someone
                //       calls an abstraction function like nk_sound_dev_get_available_sample_resolution()

                DEBUG("This AC97 supports up to %d channels and %d-bit samples.\n", channels_supported, max_bit_samples);

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

                /* TODO: Write default volume parameters to a stream so they can be configured? */

                dirty_state = state; // Remove this code when we can get sound working in a non-dirty way

                /* TODO: Should the BDLs be initialized here? */
                /* TODO: Initialize a stream with some defualt parameters for playing sound. */

                 //if (!foundmem)
                 //
                 //    ERROR("init fn: ignoring device %s as it has no memory access method\n", state->name);
                 //    continue;
                 //}
                //DEBUG("AC97 device is now registered");


                // // now bring up the device / interrupts

                // // disable interrupts
                // WRITE_MEM(state, E1000E_IMC_OFFSET, 0xffffffff);
                // DEBUG("init fn: device reset\n");
                // WRITE_MEM(state, E1000E_CTRL_OFFSET, E1000E_CTRL_RST);
                // // delay about 5 us (manual suggests 1us)
                // udelay(RESTART_DELAY);
                // // disable interrupts again after reset
                // WRITE_MEM(state, E1000E_IMC_OFFSET, 0xffffffff);

                // uint32_t mac_high = READ_MEM(state, E1000E_RAH_OFFSET);
                // uint32_t mac_low = READ_MEM(state, E1000E_RAL_OFFSET);
                // uint64_t mac_all = ((uint64_t)mac_low + ((uint64_t)mac_high << 32)) & 0xffffffffffff;
                // DEBUG("init fn: mac_all = 0x%lX\n", mac_all);
                // DEBUG("init fn: mac_high = 0x%x mac_low = 0x%x\n", mac_high, mac_low);

                // memcpy((void *)state->mac_addr, &mac_all, MAC_LEN);

                // // set the bit 22th of GCR register
                // uint32_t gcr_old = READ_MEM(state, E1000E_GCR_OFFSET);
                // WRITE_MEM(state, E1000E_GCR_OFFSET, gcr_old | E1000E_GCR_B22);
                // uint32_t gcr_new = READ_MEM(state, E1000E_GCR_OFFSET);
                // DEBUG("init fn: GCR = 0x%08x old gcr = 0x%08x tested %s\n",
                //       gcr_new, gcr_old,
                //       gcr_new & E1000E_GCR_B22 ? "pass" : "fail");
                // WRITE_MEM(state, E1000E_GCR_OFFSET, E1000E_GCR_B22);

                // WRITE_MEM(state, E1000E_FCAL_OFFSET, 0);
                // WRITE_MEM(state, E1000E_FCAH_OFFSET, 0);
                // WRITE_MEM(state, E1000E_FCT_OFFSET, 0);

                // // uint32_t ctrl_reg = (E1000E_CTRL_FD | E1000E_CTRL_FRCSPD | E1000E_CTRL_FRCDPLX | E1000E_CTRL_SLU | E1000E_CTRL_SPEED_1G) & ~E1000E_CTRL_ILOS;
                // // p50 manual
                // uint32_t ctrl_reg = E1000E_CTRL_SLU | E1000E_CTRL_RFCE | E1000E_CTRL_TFCE;

                // WRITE_MEM(state, E1000E_CTRL_OFFSET, ctrl_reg);

                // DEBUG("init fn: e1000e ctrl = 0x%08x expects 0x%08x\n",
                //       READ_MEM(state, E1000E_CTRL_OFFSET),
                //       ctrl_reg);

                // uint32_t status_reg = READ_MEM(state, E1000E_STATUS_OFFSET);
                // DEBUG("init fn: e1000e status = 0x%08x\n", status_reg);
                // DEBUG("init fn: does status.fd = 0x%01x? %s\n",
                //       E1000E_STATUS_FD,
                //       status_reg & E1000E_STATUS_FD ? "full deplex" : "halt deplex");
                // DEBUG("init fn: status.speed 0x%02x %s\n",
                //       e1000e_read_speed_bit(status_reg, E1000E_STATUS_SPEED_MASK,
                //                             E1000E_STATUS_SPEED_SHIFT),
                //       e1000e_read_speed_char(status_reg, E1000E_STATUS_SPEED_MASK,
                //                              E1000E_STATUS_SPEED_SHIFT));

                // DEBUG("init fn: status.asdv 0x%02x %s\n",
                //       e1000e_read_speed_bit(status_reg,
                //                             E1000E_STATUS_ASDV_MASK,
                //                             E1000E_STATUS_ASDV_SHIFT),
                //       e1000e_read_speed_char(status_reg,
                //                              E1000E_STATUS_ASDV_MASK,
                //                              E1000E_STATUS_ASDV_SHIFT));

                // if (e1000e_read_speed_bit(status_reg, E1000E_STATUS_SPEED_MASK, E1000E_STATUS_SPEED_SHIFT) != e1000e_read_speed_bit(status_reg, E1000E_STATUS_ASDV_MASK, E1000E_STATUS_ASDV_SHIFT))
                // {
                //     ERROR("init fn: setting speed and detecting speed do not match!!!\n");
                // }

                // DEBUG("init fn: status.lu 0x%01x %s\n",
                //       (status_reg & E1000E_STATUS_LU) >> 1,
                //       (status_reg & E1000E_STATUS_LU) ? "link is up." : "link is down.");
                // DEBUG("init fn: status.phyra %s Does the device require PHY initialization?\n",
                //       status_reg & E1000E_STATUS_PHYRA ? "1 Yes" : "0 No");

                // DEBUG("init fn: init receive ring\n");
                // e1000e_init_receive_ring(state);
                // DEBUG("init fn: init transmit ring\n");
                // e1000e_init_transmit_ring(state);

                // WRITE_MEM(state, E1000E_IMC_OFFSET, 0);
                // DEBUG("init fn: IMC = 0x%08x expects 0x%08x\n",
                //       READ_MEM(state, E1000E_IMC_OFFSET), 0);

                // DEBUG("init fn: interrupt driven\n");

                // if (pdev->msi.type == PCI_MSI_NONE)
                // {
                //     ERROR("Device %s does not support MSI - skipping\n", state->name);
                //     continue;
                // }

                // uint64_t num_vecs = pdev->msi.num_vectors_needed;
                // uint64_t base_vec = 0;

                // if (idt_find_and_reserve_range(num_vecs, 1, &base_vec))
                // {
                //     ERROR("Cannot find %d vectors for %s - skipping\n", num_vecs, state->name);
                //     continue;
                // }

                // DEBUG("%s vectors are %d..%d\n", state->name, base_vec, base_vec + num_vecs - 1);

                // if (pci_dev_enable_msi(pdev, base_vec, num_vecs, 0))
                // {
                //     ERROR("Failed to enable MSI for device %s - skipping\n", state->name);
                //     continue;
                // }

                // int i;
                // int failed = 0;

                // for (i = base_vec; i < (base_vec + num_vecs); i++)
                // {
                //     if (register_int_handler(i, e1000e_irq_handler, state))
                //     {
                //         ERROR("Failed to register handler for vector %d on device %s - skipping\n", i, state->name);
                //         failed = 1;
                //         break;
                //     }
                // }

                // if (!failed)
                // {
                //     for (i = base_vec; i < (base_vec + num_vecs); i++)
                //     {
                //         if (pci_dev_unmask_msi(pdev, i))
                //         {
                //             ERROR("Failed to unmask interrupt %d for device %s\n", i, state->name);
                //             failed = 1;
                //             break;
                //         }
                //     }
                // }

                // if (!failed)
                // {
                //     // interrupts should now be occuring

                //     // now configure device
                //     // interrupt delay value = 0 -> does not delay
                //     WRITE_MEM(state, E1000E_TIDV_OFFSET, 0);
                //     // receive interrupt delay timer = 0
                //     // -> interrupt when the device receives a package
                //     WRITE_MEM(state, E1000E_RDTR_OFFSET_NEW, E1000E_RDTR_FPD);
                //     DEBUG("init fn: RDTR new 0x%08x alias 0x%08x expect 0x%08x\n",
                //           READ_MEM(state, E1000E_RDTR_OFFSET_NEW),
                //           READ_MEM(state, E1000E_RDTR_OFFSET_ALIAS),
                //           E1000E_RDTR_FPD);
                //     WRITE_MEM(state, E1000E_RADV_OFFSET, 0);

                //     // enable only transmit descriptor written back, receive interrupt timer
                //     // rx queue 0
                //     uint32_t ims_reg = 0;
                //     ims_reg = E1000E_ICR_TXDW | E1000E_ICR_TXQ0 | E1000E_ICR_RXT0 | E1000E_ICR_RXQ0;
                //     WRITE_MEM(state, E1000E_IMS_OFFSET, ims_reg);
                //     state->ims_reg = ims_reg;
                //     e1000e_interpret_ims(state);
                //     // after the interrupt is turned on, the interrupt handler is called
                //     // due to the transmit descriptor queue empty.

                //     uint32_t icr_reg = READ_MEM(state, E1000E_ICR_OFFSET);
                //     // e1000e_interpret_ims(state);
                //     DEBUG("init fn: ICR before writting with 0xffffffff\n");
                //     // e1000e_interpret_icr(state);
                //     WRITE_MEM(state, E1000E_ICR_OFFSET, 0xffffffff);
                //     // e1000e_interpret_icr(state);
                //     DEBUG("init fn: finished writting ICR with 0xffffffff\n");

                //     // optimization
                //     WRITE_MEM(state, E1000E_AIT_OFFSET, 0);
                //     WRITE_MEM(state, E1000E_TADV_OFFSET, 0);
                //     DEBUG("init fn: end init fn --------------------\n");

                //     INFO("%s operational\n", state->name);
                //}
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

int ac97_produce_out_buffer(struct ac97_state* state, void* buffer, uint16_t num_samples)
{
    /* Recieves a pointer to a chunk of data in memory, and the size of the buffer. 
       This function assumes the buffer is compliant with the byte granularity decided upon in the ac97_state.

        NOTE: The inputted ac97_state must be named 'state' for macros to work properly
        NOTE: This function assumes the user is responsible for chopping up data into buffers with at most 0xFFFE samples
    */
    
    /* Our code maintains the buffer size; we don't want to overwrite unconsumed buffer entries */
    /* TODO: Should there be some queue of next buffer entries to process? If so, we should add
             this request to the queue instead of raising an error */
    if (OUT_SIZE == 32) {
        ERROR("Attempted to overwrite an unconsumed buffer entry!\n");
        return -1;
    }

    /* Check input buffer size for compliance with what the AC97 expects */
    if (num_samples > BDL_ENTRY_MAX_SIZE) {
        ERROR("Attempted to write a buffer with too many samples for an AC97!\n");
        return -1;
    }

    /* Write the buffer entry */
    uint8_t write_pos = OUT_TAIL; // tail will always point to the next writeable position
    OUT_ENTRY_ADDR(write_pos) = (uint32_t) buffer; // cast void* to a uint32_t (must be under 4GB memory)
    OUT_ENTRY_SIZE(write_pos) = num_samples;
    OUT_ENTRY_LAST(write_pos) = 1; // most recently written buffer is always last
    OUT_ENTRY_IOC(write_pos) = 1; // turn on ioc transfer interrupt so the handler can manage the buffer size when data is transferred
    
    /* Tell the device what the new last valid entry is */
    last_valid_entry_t lve;
    lve.last_entry = write_pos; // BDL_INC(write_pos, 1)? 
    lve.rsvd = 0;
    OUTB(lve.val, state->ioport_start_bar1 + AC97_NABM_OUT_BOX + AC97_REG_BOX_TOTAL);

    /*
    Manage our BDL Ring's state variables
    */
    OUT_TAIL = BDL_INC(write_pos, 1); // increment the tail

    /* Turn off the last bit of the previous entry; we just wrote the new last */
    if (OUT_SIZE > 0) { 
        // Is there a nice macro for BDL_DEC that wraps around? 
        if (write_pos == 0) write_pos = 31;
        else write_pos -= 1;
        OUT_ENTRY_LAST(write_pos) = 0;
    }

    OUT_SIZE += 1; // update effective size of BDL

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
    if (((last_valid_entry_t) curr_entry).last_entry != BDL_INC(OUT_HEAD, 1)) {
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
    // free((void*) OUT_ENTRY_ADDR(free_pos)); // TODO: Should the driver be responsible for freeing buffers, or should the application clean them up? 
    OUT_ENTRY_ADDR(free_pos) = 0;
    OUT_ENTRY_SIZE(free_pos) = 0;
    OUT_ENTRY_LAST(free_pos) = 0;
    OUT_ENTRY_IOC(free_pos) = 0; 

    /*
    Manage our BDL Ring's state variables
    */
    OUT_HEAD = BDL_INC(free_pos, 1); // increment the head
    OUT_SIZE -= 1; // update effective size of BDL

    return 0;
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
        uint64_t buf_len = nbuf * 0xFFFE; // total number of samples
        uint16_t *sine_buf = (uint16_t *) malloc(2*buf_len); // each sample is 2 bytes
        if (!sine_buf) {
            nk_vc_printf("ERROR: Could not allocate full-length sound buffer\n");
            return 0;
        }
        create_sine_wave(sine_buf, buf_len, freq, 44100); // TODO: pull sampling freq from device state
        DEBUG("Allocated a large sine buffer at address %x\n", (uint32_t) sine_buf);


        // Chop the samples and have the AC97 add them
        for (int i = 0; i < nbuf; i++) {
            DEBUG("Buffer %d will be at address %x\n", i, (uint32_t) (sine_buf + (i * 0xFFFE)));
            ac97_produce_out_buffer(dirty_state, sine_buf + (i * 0xFFFE), 0xFFFE); // TODO: Is alignment okay here? Seemed sketchy during OH
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

int print_bdl_out(struct ac97_state* state)
{
    DEBUG("Printing the contents of the PCM OUT BDL...\n");
    for (int i=0; i<BDL_MAX_SIZE; i++) 
    {
        ac97_bdl_entry entry = OUT_ENTRY(i);
        DEBUG("Entry %d: %016lx\n", i, entry.val);
    }
    return 0;
}


// // initialize the tx ring buffer to store transmit descriptors
// static int e1000e_init_transmit_ring(struct e1000e_state *state)
// {
//     TXMAP = malloc(sizeof(struct e1000e_map_ring));
//     if (!TXMAP)
//     {
//         ERROR("Cannot allocate txmap\n");
//         return -1;
//     }
//     memset(TXMAP, 0, sizeof(struct e1000e_map_ring));
//     TXMAP->ring_len = TX_DSC_COUNT;

//     TXMAP->map_ring = malloc(sizeof(struct e1000e_fn_map) * TX_DSC_COUNT);
//     if (!TXMAP->map_ring)
//     {
//         ERROR("Cannot allocate txmap->ring\n");
//         return -1;
//     }
//     memset(TXMAP->map_ring, 0, sizeof(struct e1000e_fn_map) * TX_DSC_COUNT);

//     TXD_RING = malloc(sizeof(struct e1000e_desc_ring));
//     if (!TXD_RING)
//     {
//         ERROR("Cannot allocate TXD_RING\n");
//         return -1;
//     }
//     memset(TXD_RING, 0, sizeof(struct e1000e_desc_ring));

//     // allocate TX_DESC_COUNT transmit descriptors in the ring buffer.
//     TXD_RING_BUFFER = malloc(sizeof(struct e1000e_tx_desc) * TX_DSC_COUNT);
//     if (!TXD_RING_BUFFER)
//     {
//         ERROR("Cannot allocate TXD_RING_BUFFER\n");
//         return -1;
//     }
//     memset(TXD_RING_BUFFER, 0, sizeof(struct e1000e_tx_desc) * TX_DSC_COUNT);
//     TXD_COUNT = TX_DSC_COUNT;

//     // store the address of the memory in TDBAL/TDBAH
//     WRITE_MEM(state, E1000E_TDBAL_OFFSET,
//               (uint32_t)(0x00000000ffffffff & (uint64_t)TXD_RING_BUFFER));
//     WRITE_MEM(state, E1000E_TDBAH_OFFSET,
//               (uint32_t)((0xffffffff00000000 & (uint64_t)TXD_RING_BUFFER) >> 32));
//     DEBUG("TXD_RING_BUFFER=0x%p, TDBAH=0x%08x, TDBAL=0x%08x\n",
//           TXD_RING_BUFFER,
//           READ_MEM(state, E1000E_TDBAH_OFFSET),
//           READ_MEM(state, E1000E_TDBAL_OFFSET));

//     // write tdlen: transmit descriptor length
//     WRITE_MEM(state, E1000E_TDLEN_OFFSET,
//               sizeof(struct e1000e_tx_desc) * TX_DSC_COUNT);

//     // write the tdh, tdt with 0
//     WRITE_MEM(state, E1000E_TDT_OFFSET, 0);
//     WRITE_MEM(state, E1000E_TDH_OFFSET, 0);
//     DEBUG("init tx fn: TDLEN = 0x%08x, TDH = 0x%08x, TDT = 0x%08x\n",
//           READ_MEM(state, E1000E_TDLEN_OFFSET),
//           READ_MEM(state, E1000E_TDH_OFFSET),
//           READ_MEM(state, E1000E_TDT_OFFSET));

//     TXD_PREV_HEAD = 0;
//     TXD_TAIL = 0;

//     // TCTL Reg: EN = 1b, PSP = 1b, CT = 0x0f (16d), COLD = 0x3F (63d)
//     uint32_t tctl_reg = E1000E_TCTL_EN | E1000E_TCTL_PSP | E1000E_TCTL_CT | E1000E_TCTL_COLD_FD;
//     WRITE_MEM(state, E1000E_TCTL_OFFSET, tctl_reg);
//     DEBUG("init tx fn: TCTL = 0x%08x expects 0x%08x\n",
//           READ_MEM(state, E1000E_TCTL_OFFSET),
//           tctl_reg);

//     // TXDCTL Reg: set WTHRESH = 1b, GRAN = 1b,
//     // other fields = 0b except bit 22th = 1b
//     WRITE_MEM(state, E1000E_TXDCTL_OFFSET,
//               E1000E_TXDCTL_GRAN | E1000E_TXDCTL_WTHRESH | (1 << 22));
//     // write tipg register
//     // 00,00 0000 0110,0000 0010 00,00 0000 1010 = 0x0060200a
//     // will be zero when emulating hardware
//     WRITE_MEM(state, E1000E_TIPG_OFFSET,
//               E1000E_TIPG_IPGT | E1000E_TIPG_IPGR1 | E1000E_TIPG_IPGR2);
//     DEBUG("init tx fn: TIPG = 0x%08x expects 0x%08x\n",
//           READ_MEM(state, E1000E_TIPG_OFFSET),
//           E1000E_TIPG_IPGT | E1000E_TIPG_IPGR1 | E1000E_TIPG_IPGR2);
//     return 0;
// }

// static int e1000e_set_receive_buffer_size(struct e1000e_state *state, uint32_t buffer_size)
// {
//     if (state->rx_buffer_size != buffer_size)
//     {
//         uint32_t rctl = READ_MEM(state, E1000E_RCTL_OFFSET) & E1000E_RCTL_BSIZE_MASK;
//         switch (buffer_size)
//         {
//         case E1000E_RECV_BSIZE_256:
//             rctl |= E1000E_RCTL_BSIZE_256;
//             break;
//         case E1000E_RECV_BSIZE_512:
//             rctl |= E1000E_RCTL_BSIZE_512;
//             break;
//         case E1000E_RECV_BSIZE_1024:
//             rctl |= E1000E_RCTL_BSIZE_1024;
//             break;
//         case E1000E_RECV_BSIZE_2048:
//             rctl |= E1000E_RCTL_BSIZE_2048;
//             break;
//         case E1000E_RECV_BSIZE_4096:
//             rctl |= E1000E_RCTL_BSIZE_4096;
//             break;
//         case E1000E_RECV_BSIZE_8192:
//             rctl |= E1000E_RCTL_BSIZE_8192;
//             break;
//         case E1000E_RECV_BSIZE_16384:
//             rctl |= E1000E_RCTL_BSIZE_16384;
//             break;
//         default:
//             ERROR("e1000e receive pkt fn: unmatch buffer size\n");
//             return -1;
//         }
//         WRITE_MEM(state, E1000E_RCTL_OFFSET, rctl);
//         state->rx_buffer_size = buffer_size;
//     }
//     return 0;
// }

// // initialize the rx ring buffer to store receive descriptors
// static int e1000e_init_receive_ring(struct e1000e_state *state)
// {
//     RXMAP = malloc(sizeof(struct e1000e_map_ring));
//     if (!RXMAP)
//     {
//         ERROR("Cannot allocate rxmap\n");
//         return -1;
//     }
//     memset(RXMAP, 0, sizeof(struct e1000e_map_ring));

//     RXMAP->map_ring = malloc(sizeof(struct e1000e_fn_map) * RX_DSC_COUNT);
//     if (!RXMAP->map_ring)
//     {
//         ERROR("Cannot allocate rxmap->ring\n");
//         return -1;
//     }
//     memset(RXMAP->map_ring, 0, sizeof(struct e1000e_fn_map) * RX_DSC_COUNT);
//     RXMAP->ring_len = RX_DSC_COUNT;

//     RXD_RING = malloc(sizeof(struct e1000e_desc_ring));
//     if (!RXD_RING)
//     {
//         ERROR("Cannot allocate rxd_ring buffer\n");
//         return -1;
//     }
//     memset(RXD_RING, 0, sizeof(struct e1000e_desc_ring));

//     // the number of the receive descriptor in the ring
//     RXD_COUNT = RX_DSC_COUNT;

//     // allocate the receive descriptor ring buffer
//     uint32_t rx_desc_size = sizeof(struct e1000e_rx_desc) * RX_DSC_COUNT;
//     RXD_RING_BUFFER = malloc(rx_desc_size);
//     if (!RXD_RING_BUFFER)
//     {
//         ERROR("Cannot allocate RXD_RING_BUFFER\n");
//         return -1;
//     }
//     memset(RXD_RING_BUFFER, 0, rx_desc_size);

//     // store the address of the memory in TDBAL/TDBAH
//     WRITE_MEM(state, E1000E_RDBAL_OFFSET,
//               (uint32_t)(0x00000000ffffffff & (uint64_t)RXD_RING_BUFFER));
//     WRITE_MEM(state, E1000E_RDBAH_OFFSET,
//               (uint32_t)((0xffffffff00000000 & (uint64_t)RXD_RING_BUFFER) >> 32));
//     DEBUG("init rx fn: RDBAH = 0x%08x, RDBAL = 0x%08x = rd_buffer\n",
//           READ_MEM(state, E1000E_RDBAH_OFFSET),
//           READ_MEM(state, E1000E_RDBAL_OFFSET));
//     DEBUG("init rx fn: rd_buffer = 0x%016lx\n", RXD_RING_BUFFER);

//     // write rdlen
//     WRITE_MEM(state, E1000E_RDLEN_OFFSET, rx_desc_size);
//     DEBUG("init rx fn: RDLEN=0x%08x should be 0x%08x\n",
//           READ_MEM(state, E1000E_RDLEN_OFFSET), rx_desc_size);

//     // write the rdh, rdt with 0
//     WRITE_MEM(state, E1000E_RDH_OFFSET, 0);
//     WRITE_MEM(state, E1000E_RDT_OFFSET, 0);
//     DEBUG("init rx fn: RDH=0x%08x, RDT=0x%08x expects 0\n",
//           READ_MEM(state, E1000E_RDH_OFFSET),
//           READ_MEM(state, E1000E_RDT_OFFSET));
//     RXD_PREV_HEAD = 0;
//     RXD_TAIL = 0;

//     WRITE_MEM(state, E1000E_RXDCTL_OFFSET,
//               E1000E_RXDCTL_GRAN | E1000E_RXDCTL_WTHRESH);
//     DEBUG("init rx fn: RXDCTL=0x%08x expects 0x%08x\n",
//           READ_MEM(state, E1000E_RXDCTL_OFFSET),
//           E1000E_RXDCTL_GRAN | E1000E_RXDCTL_WTHRESH);

//     // write rctl register specifing the receive mode
//     uint32_t rctl_reg = E1000E_RCTL_EN | E1000E_RCTL_SBP | E1000E_RCTL_UPE | E1000E_RCTL_LPE | E1000E_RCTL_DTYP_LEGACY | E1000E_RCTL_BAM | E1000E_RCTL_RDMTS_HALF | E1000E_RCTL_PMCF;

//     // receive buffer threshold and size
//     WRITE_MEM(state, E1000E_RCTL_OFFSET, rctl_reg);

//     // set the receive packet buffer size
//     e1000e_set_receive_buffer_size(state, RX_PACKET_BUFFER_SIZE);

//     DEBUG("init rx fn: RCTL=0x%08x expects  0x%08x\n",
//           READ_MEM(state, E1000E_RCTL_OFFSET),
//           rctl_reg);
//     return 0;
// }

// static int e1000e_send_packet(uint8_t *packet_addr,
//                               uint64_t packet_size,
//                               struct e1000e_state *state)
// {
//     DEBUG("send pkt fn: pkt_addr 0x%p pkt_size: %d\n", packet_addr, packet_size);

//     DEBUG("send pkt fn: before sending TDH = %d TDT = %d tail_pos = %d\n",
//           READ_MEM(state, E1000E_TDH_OFFSET),
//           READ_MEM(state, E1000E_TDT_OFFSET),
//           TXD_TAIL);
//     DEBUG("send pkt fn: tpt total packet transmit: %d\n",
//           READ_MEM(state, E1000E_TPT_OFFSET));

//     if (packet_size > MAX_TU)
//     {
//         ERROR("send pkt fn: packet is too large.\n");
//         return -1;
//     }

//     memset(((struct e1000e_tx_desc *)TXD_RING_BUFFER + TXD_TAIL),
//            0, sizeof(struct e1000e_tx_desc));
//     TXD_ADDR(TXD_TAIL) = (uint64_t *)packet_addr;
//     TXD_LENGTH(TXD_TAIL) = packet_size;
//     // set up send flags
//     // TXD_CMD(TXD_TAIL).bit.dext = 0;
//     // TXD_CMD(TXD_TAIL).bit.vle = 0;
//     // TXD_CMD(TXD_TAIL).bit.ifcs = 1;
//     // // set the end of packet flag if this is the last fragment
//     // TXD_CMD(TXD_TAIL).bit.eop = 1;
//     // // interrupt delay enable
//     // // if ide = 0 and rs = 1, the transmit interrupt will occur immediately
//     // // after the packet is sent.
//     // TXD_CMD(TXD_TAIL).bit.ide = 0;
//     // // report the status of the descriptor
//     // TXD_CMD(TXD_TAIL).bit.rs = 1;
//     TXD_CMD(TXD_TAIL).byte = E1000E_TXD_CMD_EOP | E1000E_TXD_CMD_IFCS | E1000E_TXD_CMD_RS;

//     // increment transmit descriptor list tail by 1
//     DEBUG("send pkt fn: moving the tail\n");
//     TXD_TAIL = TXD_INC(1, TXD_TAIL);
//     WRITE_MEM(state, E1000E_TDT_OFFSET, TXD_TAIL);
//     DEBUG("send pkt fn: after moving tail TDH = %d TDT = %d tail_pos = %d\n",
//           READ_MEM(state, E1000E_TDH_OFFSET),
//           READ_MEM(state, E1000E_TDT_OFFSET),
//           TXD_TAIL);

//     DEBUG("send pkt fn: transmit error %d\n",
//           TXD_STATUS(TXD_PREV_HEAD).ec | TXD_STATUS(TXD_PREV_HEAD).lc);

//     DEBUG("send pkt fn: txd cmd.rs = %d status.dd = %d\n",
//           TXD_CMD(TXD_PREV_HEAD).bit.rs,
//           TXD_STATUS(TXD_PREV_HEAD).dd);

//     DEBUG("send pkt fn: tpt total packet transmit: %d\n",
//           READ_MEM(state, E1000E_TPT_OFFSET));

//     // uint32_t status_pci = pci_cfg_readw(state->bus_num, state->pci_dev->num,
//     //                                     0, E1000E_PCI_STATUS_OFFSET);
//     // DEBUG("send pkt fn: status_pci 0x%04x int %d\n",
//     //       status_pci, status_pci & E1000E_PCI_STATUS_INT);
//     return 0;
// }

// static void e1000e_disable_receive(struct e1000e_state *state)
// {
//     uint32_t rctl_reg = READ_MEM(state, E1000E_RCTL_OFFSET);
//     rctl_reg &= ~E1000E_RCTL_EN;
//     WRITE_MEM(state, E1000E_RCTL_OFFSET, rctl_reg);
//     return;
// }

// static void e1000e_enable_receive(struct e1000e_state *state)
// {
//     uint32_t rctl_reg = READ_MEM(state, E1000E_RCTL_OFFSET);
//     rctl_reg |= E1000E_RCTL_EN;
//     WRITE_MEM(state, E1000E_RCTL_OFFSET, rctl_reg);
//     return;
// }

// static int e1000e_receive_packet(uint8_t *buffer,
//                                  uint64_t buffer_size,
//                                  struct e1000e_state *state)
// {
//     DEBUG("e1000e receive packet fn: buffer = 0x%p, len = %lu\n",
//           buffer, buffer_size);
//     DEBUG("e1000e receive pkt fn: before moving tail head: %d, tail: %d\n",
//           READ_MEM(state, E1000E_RDH_OFFSET),
//           READ_MEM(state, E1000E_RDT_OFFSET));

//     memset(((struct e1000e_rx_desc *)RXD_RING_BUFFER + RXD_TAIL),
//            0, sizeof(struct e1000e_rx_desc));

//     RXD_ADDR(RXD_TAIL) = (uint64_t *)buffer;

//     RXD_TAIL = RXD_INC(RXD_TAIL, 1);
//     WRITE_MEM(state, E1000E_RDT_OFFSET, RXD_TAIL);

//     DEBUG("e1000e receive pkt fn: after moving tail head: %d, prev_head: %d tail: %d\n",
//           READ_MEM(state, E1000E_RDH_OFFSET),
//           RXD_PREV_HEAD,
//           READ_MEM(state, E1000E_RDT_OFFSET));

//     return 0;
// }

// static uint64_t e1000e_packet_size_to_buffer_size(uint64_t sz)
// {
//     if (sz < E1000E_RECV_BSIZE_MIN)
//     {
//         // set the minimum packet size to 256 bytes
//         return E1000E_RECV_BSIZE_MIN;
//     }
//     // Round up the number to the buffer size with power of two
//     // In E1000E, the packet buffer is the power of two.
//     // citation: https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
//     sz--;
//     sz |= sz >> 1;
//     sz |= sz >> 2;
//     sz |= sz >> 4;
//     sz |= sz >> 8;
//     sz |= sz >> 16;
//     sz |= sz >> 32;
//     sz++;
//     // if the size is larger than the maximun buffer size, return the largest size.
//     if (sz >= E1000E_RECV_BSIZE_MAX)
//     {
//         return E1000E_RECV_BSIZE_MAX;
//     }
//     else
//     {
//         return sz;
//     }
// }

// static int e1000e_get_characteristics(void *vstate, struct nk_net_dev_characteristics *c)
// {
//     if (!vstate)
//     {
//         ERROR("The device state pointer is null\n");
//         return -1;
//     }

//     if (!c)
//     {
//         ERROR("The characteristics pointer is null\n");
//         return -1;
//     }

//     struct e1000e_state *state = (struct e1000e_state *)vstate;
//     memcpy(c->mac, (void *)state->mac_addr, MAC_LEN);
//     // minimum and the maximum transmission unit
//     c->min_tu = MIN_TU;
//     c->max_tu = MAX_TU;
//     c->packet_size_to_buffer_size = e1000e_packet_size_to_buffer_size;
//     return 0;
// }

// static int e1000e_unmap_callback(struct e1000e_map_ring *map,
//                                  uint64_t **callback,
//                                  void **context)
// {
//     // callback is a function pointer
//     DEBUG("unmap callback fn head_pos %d tail_pos %d\n",
//           map->head_pos, map->tail_pos);

//     if (map->head_pos == map->tail_pos)
//     {
//         // if there is an empty mapping ring buffer, do not unmap the callback
//         ERROR("Try to unmap an empty queue\n");
//         return -1;
//     }

//     uint64_t i = map->head_pos;
//     DEBUG("unmap callback fn: before unmap head_pos %d tail_pos %d\n",
//           map->head_pos, map->tail_pos);

//     *callback = (uint64_t *)map->map_ring[i].callback;
//     *context = map->map_ring[i].context;
//     map->map_ring[i].callback = NULL;
//     map->map_ring[i].context = NULL;
//     map->head_pos = (1 + map->head_pos) % map->ring_len;

//     DEBUG("unmap callback fn: callback 0x%p, context 0x%p\n",
//           *callback, *context);
//     DEBUG("end unmap callback fn: after unmap head_pos %d tail_pos %d\n",
//           map->head_pos, map->tail_pos);
//     return 0;
// }

// static int e1000e_map_callback(struct e1000e_map_ring *map,
//                                void (*callback)(nk_net_dev_status_t, void *),
//                                void *context)
// {
//     DEBUG("map callback fn: head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
//     if (map->head_pos == ((map->tail_pos + 1) % map->ring_len))
//     {
//         // when the mapping callback queue is full
//         ERROR("map callback fn: Callback mapping queue is full.\n");
//         return -1;
//     }

//     DEBUG("map callback fn: map head_pos: %d, tail_pos: %d\n",
//           map->head_pos, map->tail_pos);
//     uint64_t i = map->tail_pos;
//     struct e1000e_fn_map *fnmap = (map->map_ring + i);
//     fnmap->callback = callback;
//     fnmap->context = (uint64_t *)context;
//     map->tail_pos = (1 + map->tail_pos) % map->ring_len;
//     DEBUG("map callback fn: callback 0x%p, context 0x%p\n",
//           callback, context);
//     DEBUG("end map callback fn: after map head_pos: %d, tail_pos: %d\n",
//           map->head_pos, map->tail_pos);
//     return 0;
// }

// static void e1000e_interpret_int(struct e1000e_state *state, uint32_t int_reg)
// {
//     if (int_reg & E1000E_ICR_TXDW)
//         INFO("\t TXDW triggered\n");
//     if (int_reg & E1000E_ICR_TXQE)
//         INFO("\t TXQE triggered\n");
//     if (int_reg & E1000E_ICR_LSC)
//         INFO("\t LSC triggered\n");
//     if (int_reg & E1000E_ICR_RXO)
//         INFO("\t RX0 triggered\n");
//     if (int_reg & E1000E_ICR_RXT0)
//         INFO("\t RXT0 triggered\n");
//     if (int_reg & E1000E_ICR_TXD_LOW)
//         INFO("\t TXD_LOW triggered\n");
//     if (int_reg & E1000E_ICR_SRPD)
//     {
//         INFO("\t SRPD triggered\n");
//         INFO("\t RSRPD.size %u\n", READ_MEM(state, E1000E_RSRPD_OFFSET));
//     }
//     if (int_reg & E1000E_ICR_RXQ0)
//         INFO("\t RXQ0 triggered\n");
//     if (int_reg & E1000E_ICR_RXQ1)
//         INFO("\t RXQ1 triggered\n");
//     if (int_reg & E1000E_ICR_TXQ0)
//         INFO("\t TXQ0 triggered\n");
//     if (int_reg & E1000E_ICR_TXQ1)
//         INFO("\t TXQ1 triggered\n");
//     if (int_reg & E1000E_ICR_OTHER)
//         INFO("\t Other \n");
//     if (int_reg & E1000E_ICR_INT_ASSERTED)
//         INFO("\t INT_ASSERTED triggered\n");
//     INFO("interpret int: end --------------------\n");
// }

// static void e1000e_interpret_ims(struct e1000e_state *state)
// {
//     uint32_t ims_reg = READ_MEM(state, E1000E_IMS_OFFSET);
//     INFO("interpret ims: IMS 0x%08x\n", ims_reg);
//     e1000e_interpret_int(state, ims_reg);
// }

// static void e1000e_interpret_icr(struct e1000e_state *state)
// {
//     uint32_t icr_reg = READ_MEM(state, E1000E_ICR_OFFSET);
//     INFO("interpret icr: ICR 0x%08x\n", icr_reg);
//     e1000e_interpret_int(state, icr_reg);
// }

// static int e1000e_post_send(void *vstate,
//                             uint8_t *src,
//                             uint64_t len,
//                             void (*callback)(nk_net_dev_status_t, void *),
//                             void *context)
// {
//     // always map callback
//     struct e1000e_state *state = (struct e1000e_state *)vstate;
//     DEBUG("post tx fn: callback 0x%p context 0x%p\n", callback, context);

//     // #measure
//     TIMING_GET_TSC(state->measure.tx.postx_map.start);
//     int result = e1000e_map_callback(state->tx_map, callback, context);
//     TIMING_GET_TSC(state->measure.tx.postx_map.end);

//     // #measure
//     TIMING_GET_TSC(state->measure.tx.xpkt.start);
//     if (!result)
//     {
//         result = e1000e_send_packet(src, len, (struct e1000e_state *)state);
//     }
//     TIMING_GET_TSC(state->measure.tx.xpkt.end);

//     DEBUG("post tx fn: end\n");
//     return result;
// }

// static int e1000e_post_receive(void *vstate,
//                                uint8_t *src,
//                                uint64_t len,
//                                void (*callback)(nk_net_dev_status_t, void *),
//                                void *context)
// {
//     // mapping the callback always
//     // if result != -1 receive packet
//     struct e1000e_state *state = (struct e1000e_state *)vstate;
//     DEBUG("post rx fn: callback 0x%p, context 0x%p\n", callback, context);

//     // #measure
//     TIMING_GET_TSC(state->measure.rx.postx_map.start);
//     int result = e1000e_map_callback(state->rx_map, callback, context);
//     TIMING_GET_TSC(state->measure.rx.postx_map.end);

//     // #measure
//     TIMING_GET_TSC(state->measure.rx.xpkt.start);
//     if (!result)
//     {
//         result = e1000e_receive_packet(src, len, state);
//     }
//     TIMING_GET_TSC(state->measure.rx.xpkt.end);

//     DEBUG("post rx fn: end --------------------\n");
//     return result;
// }

// enum pkt_op
// {
//     op_unknown,
//     op_tx,
//     op_rx
// };

// static int e1000e_irq_handler(excp_entry_t *excp, excp_vec_t vec, void *s)
// {
//     DEBUG("irq_handler fn: vector: 0x%x rip: 0x%p s: 0x%p\n",
//           vec, excp->rip, s);

//     // #measure
//     uint64_t irq_start = 0;
//     uint64_t irq_end = 0;
//     uint64_t callback_start = 0;
//     uint64_t callback_end = 0;
//     enum pkt_op which_op = op_unknown;

//     TIMING_GET_TSC(irq_start);
//     struct e1000e_state *state = s;
//     uint32_t icr = READ_MEM(state, E1000E_ICR_OFFSET);
//     uint32_t mask_int = icr & state->ims_reg;
//     DEBUG("irq_handler fn: ICR: 0x%08x IMS: 0x%08x mask_int: 0x%08x\n",
//           icr, state->ims_reg, mask_int);

//     void (*callback)(nk_net_dev_status_t, void *) = NULL;
//     void *context = NULL;
//     nk_net_dev_status_t status = NK_NET_DEV_STATUS_SUCCESS;

//     if (mask_int & (E1000E_ICR_TXDW | E1000E_ICR_TXQ0))
//     {

//         which_op = op_tx;
//         // transmit interrupt
//         DEBUG("irq_handler fn: handle the txdw interrupt\n");
//         TIMING_GET_TSC(state->measure.tx.irq_unmap.start);
//         e1000e_unmap_callback(state->tx_map,
//                               (uint64_t **)&callback,
//                               (void **)&context);
//         TIMING_GET_TSC(state->measure.tx.irq_unmap.end);

//         // if there is an error while sending a packet, set the error status
//         if (TXD_STATUS(TXD_PREV_HEAD).ec || TXD_STATUS(TXD_PREV_HEAD).lc)
//         {
//             ERROR("irq_handler fn: transmit errors\n");
//             status = NK_NET_DEV_STATUS_ERROR;
//         }

//         // update the head of the ring buffer
//         TXD_PREV_HEAD = TXD_INC(1, TXD_PREV_HEAD);
//         DEBUG("irq_handler fn: total packet transmitted = %d\n",
//               READ_MEM(state, E1000E_TPT_OFFSET));
//     }

//     if (mask_int & (E1000E_ICR_RXT0 | E1000E_ICR_RXO | E1000E_ICR_RXQ0))
//     {
//         which_op = op_rx;
//         // receive interrupt
//         /* if (mask_int & E1000E_ICR_RXT0) { */
//         /*   DEBUG("irq_handler fn: handle the rxt0 interrupt\n"); */
//         /* } */

//         /* if (mask_int & E1000E_ICR_RXO) { */
//         /*   DEBUG("irq_handler fn: handle the rx0 interrupt\n"); */
//         /* } */

//         // INFO("rx length %d\n", RXD_LENGTH(RXD_PREV_HEAD));
//         TIMING_GET_TSC(state->measure.rx.irq_unmap.start);
//         e1000e_unmap_callback(state->rx_map,
//                               (uint64_t **)&callback,
//                               (void **)&context);
//         TIMING_GET_TSC(state->measure.rx.irq_unmap.end);

//         // e1000e_interpret_ims(state);
//         // TODO: check if we need this line
//         // WRITE_MEM(state, E1000E_IMC_OFFSET, E1000E_ICR_RXO);

//         // checking errors
//         if (RXD_ERRORS(RXD_PREV_HEAD))
//         {
//             ERROR("irq_handler fn: receive an error packet\n");
//             status = NK_NET_DEV_STATUS_ERROR;
//         }

//         // in the irq, update only the head of the buffer
//         RXD_PREV_HEAD = RXD_INC(1, RXD_PREV_HEAD);
//     }

//     TIMING_GET_TSC(callback_start);
//     if (callback)
//     {
//         DEBUG("irq_handler fn: invoke callback function callback: 0x%p\n", callback);
//         callback(status, context);
//     }
//     TIMING_GET_TSC(callback_end);

//     DEBUG("irq_handler fn: end irq\n\n\n");
//     // DO NOT DELETE THIS LINE.
//     // must have this line at the end of the handler
//     IRQ_HANDLER_END();

// #if TIMING
//     // #measure
//     irq_end = rdtsc();

//     if (which_op == op_tx)
//     {
//         state->measure.tx.irq_callback.start = callback_start;
//         state->measure.tx.irq_callback.end = callback_end;
//         state->measure.tx.irq.start = irq_start;
//         state->measure.tx.irq.end = irq_end;
//     }
//     else
//     {
//         state->measure.rx.irq_callback.start = callback_start;
//         state->measure.rx.irq_callback.end = callback_end;
//         state->measure.rx.irq.start = irq_start;
//         state->measure.rx.irq.end = irq_end;
//     }
// #endif

//     return 0;
// }

// uint32_t e1000e_read_speed_bit(uint32_t reg, uint32_t mask, uint32_t shift)
// {
//     uint32_t speed = (reg & mask) >> shift;
//     if (speed == E1000E_SPEED_ENCODING_1G_V1 || speed == E1000E_SPEED_ENCODING_1G_V2)
//     {
//         return E1000E_SPEED_ENCODING_1G_V1;
//     }
//     return speed;
// }

// char *e1000e_read_speed_char(uint32_t reg, uint32_t mask, uint32_t shift)
// {
//     uint32_t encoding = e1000e_read_speed_bit(reg, mask, shift);
//     char *res = NULL;
//     switch (encoding)
//     {
//     case E1000E_SPEED_ENCODING_10M:
//         res = "10M";
//         break;
//     case E1000E_SPEED_ENCODING_100M:
//         res = "100M";
//         break;
//     case E1000E_SPEED_ENCODING_1G_V1:
//     case E1000E_SPEED_ENCODING_1G_V2:
//         res = "1G";
//         break;
//     }
//     return res;
// }


// //
// // DEBUGGING AND TIMING ROUTINES FOLLOW
// //
// //

// #if TIMING
// static void e1000e_read_write(struct e1000e_state *state)
// {
//     INFO("Testing Read Write Delay\n");
//     volatile uint64_t mem_var = 0x55;
//     uint64_t tsc_sta = 0;
//     uint64_t tsc_end = 0;
//     uint64_t tsc_delay = 0;

//     INFO("Reading Delay from MEM\n");
//     TIMING_GET_TSC(tsc_sta);
//     uint64_t read_var = mem_var;
//     TIMING_GET_TSC(tsc_end);
//     TIMING_DIFF_TSC(tsc_delay, tsc_sta, tsc_end);
//     INFO("Reading Delay %lu\n\n", tsc_delay);

//     INFO("Writing Delay from MEM\n");
//     TIMING_GET_TSC(tsc_sta);
//     mem_var = 0x66;
//     TIMING_GET_TSC(tsc_end);
//     TIMING_DIFF_TSC(tsc_delay, tsc_sta, tsc_end);
//     INFO("Writing Delay %lu\n\n", tsc_delay);

//     INFO("Reading Delay from NIC\n");
//     TIMING_GET_TSC(tsc_sta);
//     uint32_t tctl_reg = READ_MEM(state, E1000E_TCTL_OFFSET);
//     TIMING_GET_TSC(tsc_end);
//     TIMING_DIFF_TSC(tsc_delay, tsc_sta, tsc_end);
//     INFO("Reading Delay %lu\n\n", tsc_delay);

//     INFO("Writing Delayi from NIC\n");
//     TIMING_GET_TSC(tsc_sta);
//     WRITE_MEM(state, E1000E_AIT_OFFSET, 0);
//     TIMING_GET_TSC(tsc_end);
//     TIMING_DIFF_TSC(tsc_delay, tsc_sta, tsc_end);
//     INFO("Writing Delay, %lu\n\n", tsc_delay);

//     INFO("Reading pci_cfg_readw Delay\n");
//     TIMING_GET_TSC(tsc_sta);
//     uint32_t status_pci = pci_dev_cfg_readw(state->pci_dev, E1000E_PCI_STATUS_OFFSET);
//     TIMING_GET_TSC(tsc_end);
//     TIMING_DIFF_TSC(tsc_delay, tsc_sta, tsc_end);
//     INFO("Reading pci_dev_cfg_readw Delay, %lu\n\n", tsc_delay);
// }
// #endif
