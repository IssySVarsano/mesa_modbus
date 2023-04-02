//    Copyright (C) 2023 Andy Pugh

//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/* A configurable component to use Mesa PktUART for modbus control */


#include "rtapi.h"
#include "rtapi_slab.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "hal.h"
// extra type not in hal.h
#define HAL_BLOCK 10

#include "../../../hal_priv.h"

#if !defined(__KERNEL__)
#include <stdio.h>
#include <stdlib.h>
#endif

#include "hostmot2-serial.h"

/* module information */
MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("Modbus control using Mesa PktUART");
MODULE_LICENSE("GPL");

#define MAX_CHAN 8
#define MAX_DATA 33

typedef struct {
    hal_u32_t *address;
    hal_data_u **pins;
    hal_float_t **scale;
    hal_float_t **offset;
} hm2_modbus_hal_t;

typedef struct {
    hal_type_t type;
    hal_pin_dir_t dir;
    unsigned char func;
    unsigned char addr0;
    unsigned char addr1;
    int count;
    char data[MAX_DATA]; // hex values as a string, to handle 2008 bits
} hm2_modbus_channel_t;

typedef struct {
    char port[HAL_NAME_LEN];
    int num_chans;
    int num_pins;
    hm2_modbus_hal_t hal;
    hm2_modbus_channel_t *chans;
} hm2_modbus_inst_t;

typedef struct {
    int num_insts;
    hm2_modbus_inst_t *insts;
} hm2_modbus_t;

typedef struct {
    hal_type_t type;
    hal_pin_dir_t dir;
    rtapi_u16 addr;
    unsigned char count;
    char name[HAL_NAME_LEN];
} hm2_modbus_chan_descriptor_t;

enum {
    START,
    WAIT_FOR_SEND_BEGIN,
    WAIT_FOR_SEND_COMPLETE,
    WAIT_FOR_DATA_FRAME,
};

// get the channel / register definitions
#include "mesa_modbus.h"

static int comp_id;
static hm2_modbus_t *m;
static void process(void *arg, long period);
static uint16_t RTU_CRC(unsigned char* buf, int len);
static int clocklow = 0;

char *ports[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(ports, MAX_CHAN, "PktUART names");

int rtapi_app_main(void){

    rtapi_set_msg_level(5); ////////////// Remove FIXME
    
    int retval;
    char hal_name[HAL_NAME_LEN];
    unsigned int rxmode;
    unsigned int txmode;
    unsigned int filter;
    
    if (!ports[0]) {
        rtapi_print_msg(RTAPI_MSG_ERR, "The "COMP_NAME" component requires at least"
                " one valid pktuart port, eg ports=\"hm2_5i25.0.pktuart.7\"\n");
        return -EINVAL;
    }

    comp_id = hal_init(COMP_NAME);
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, COMP_NAME": ERROR: hal_init() failed\n");
        return -1;
    }

    // allocate shared memory for the base struct
    m = (hm2_modbus_t*)rtapi_kmalloc(sizeof(hm2_modbus_t), RTAPI_GFP_KERNEL);
    //m = (hm2_modbus_t*)hal_malloc(sizeof(hm2_modbus_t));
    if (m == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                COMP_NAME": Out of Memory\n");
        hal_exit(comp_id);
        return -1;
    }

    // Count the instances.
    for (m->num_insts = 0; ports[m->num_insts];m->num_insts++) {}
    m->insts = (hm2_modbus_inst_t*)rtapi_kmalloc(m->num_insts * sizeof(hm2_modbus_inst_t), RTAPI_GFP_KERNEL);
    // Parse the config string
    for (int i = 0; i < m->num_insts; i++) {
        hm2_modbus_inst_t *inst = &m->insts[i];
        inst->num_chans = sizeof(channels)/sizeof(channels[0]);
        // there may be more pins than channels
        inst->num_pins = 0;
        for (int c = 0; c < inst->num_chans; c++){
            inst->num_pins += channels[c].count;
        }
        // Malloc structs and pins, some in main or kernel memory, some in HAL
        inst->chans = (hm2_modbus_channel_t *)rtapi_kmalloc(inst->num_chans * sizeof(hm2_modbus_channel_t), RTAPI_GFP_KERNEL);
        //inst->hal = (hm2_modbus_hal_t *)hal_malloc(sizeof(hm2_modbus_hal_t));
        inst->hal.address = (hal_u32_t*)hal_malloc(sizeof(hal_u32_t));
        inst->hal.pins = (hal_data_u **)hal_malloc(inst->num_pins * sizeof(hal_data_u));
        inst->hal.scale = (hal_float_t **)hal_malloc(inst->num_pins * sizeof(hal_float_t));
        inst->hal.offset = (hal_float_t **)hal_malloc(inst->num_pins * sizeof(hal_float_t));
        
        rtapi_strlcpy(inst->port, ports[i], HAL_NAME_LEN);
        retval = rtapi_snprintf(hal_name, HAL_NAME_LEN, COMP_NAME".%02i", i);
        if (retval >= HAL_NAME_LEN) {
            goto fail0;
        }
        retval = hal_export_funct(hal_name, process, inst, 1, 0, comp_id);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, COMP_NAME" ERROR: function export failed\n");
            goto fail0;
        }

        retval = hal_param_u32_newf(HAL_RW, inst->hal.address, comp_id, COMP_NAME".%02i.address", i);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, COMP_NAME" ERROR: failed to create address param\n");
            goto fail0;
        }
        *inst->hal.address = 0x01;

        switch (parity) {
            case 'N':
            case 'n':
                txmode = 0x20; //Drive enable auto, no parity
                rxmode = 0x0C; //RX Enable, Rx Mask, no parity
                break;
            case 'E':
            case 'e':
                txmode = 0x20020; //Drive enable auto, even parity
                rxmode = 0x2000C; //RX Enable, Rx Mask, even parity
                break;
            case 'O':
            case 'o':
                txmode = 0x60020; //Drive enable auto, odd parity
                rxmode = 0x6000C; //RX Enable, Rx Mask, odd parity
                break;
            default:
                rtapi_print_msg(RTAPI_MSG_ERR, "Invalid parity descriptor\n");
                goto fail0;
        }
        if (txdelay > 0xFF) txdelay = 0xFF;
        if (rxdelay > 0xFF) rxdelay = 0xFF;
        txmode |= txdelay << 8;
        rxmode |= rxdelay << 8;
        clocklow = hm2_pktuart_get_clock(inst->port);
        rtapi_print_msg(RTAPI_MSG_INFO, "Clocklow = %d\n", clocklow);
        filter = (clocklow * 0.5 / baudrate);
        if (filter > 0xFF) filter = 0xFF;
        rxmode |= (filter << 22);
        rtapi_print_msg(RTAPI_MSG_INFO, "txmode = %08X rxmode = %08X\n", txmode, rxmode);
        retval=hm2_pktuart_setup(inst->port, baudrate , txmode,  rxmode, 1, 1);
        if (retval<0)
        {
         rtapi_print_msg(1, COMP_NAME"PktUART setup problem: %d\n", retval);
         return -1;
        }

        int p = 0; // HAL pin index, not aligned to channel index
        for (int c = 0; c < inst->num_chans; c++){
            hm2_modbus_channel_t *ch = &(inst->chans[c]);
            rtapi_print_msg(RTAPI_MSG_INFO, "ch %i is at %p\n", c, ch);
            ch->type = channels[c].type;
            ch->dir = channels[c].dir;
            ch->addr0 = channels[c].addr & 0xFF;
            ch->addr1 = (channels[c].addr & 0xFF00) >> 8;
            ch->count = channels[c].count;
            switch (ch->type + ch->dir) {
                case HAL_BIT + HAL_OUT:
                    if (ch->count > 1){
                        for (int j = 0; j < ch->count; j++){
                            ch->func = 0x01;
                            retval = hal_pin_bit_newf(ch->dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                    comp_id, COMP_NAME".%02i.%s-%02i",
                                    i, channels[c].name, j);
                        }
                    } else {
                        ch->func = 0x01;
                        retval = hal_pin_bit_newf(ch->dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                            comp_id, COMP_NAME".%02i.%s",
                                            i, channels[c].name);
                    }
                    break;
                case HAL_BIT + HAL_IN:
                    if (ch->count > 8 * MAX_DATA){
                        rtapi_print_msg(RTAPI_MSG_ERR,"The Modbus protocol"
                        " enforces a hard limit of 2008 coils per transaction"
                        " but with the current setup file this component is"
                        " limited to %i coils per messsge\n", 8 * MAX_DATA);
                    } else if (ch->count > 1){
                        for (int j = 0; j < ch->count; j++){
                            ch->func = 0x15;
                            retval = hal_pin_bit_newf(ch->dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                      comp_id, COMP_NAME".%02i.%s-%02i",
                                        i, channels[c].name, j);
                        }
                    } else {
                        ch->func = 0x05;
                        retval = hal_pin_bit_newf(ch->dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                            comp_id, COMP_NAME".%02i.%s",
                                            i, channels[c].name);
                    }
                    break;
                default:
                    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unsupported HAL pin type in mesa_modbus definition file");
            }
        }
    }
    hal_ready(comp_id);
    return 0;

    fail0:
    hal_exit(comp_id);
    return -1;

}

int send_modbus_pkt(hm2_modbus_inst_t *inst, hm2_modbus_channel_t *ch){
    const uint hex[32] = {0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,10,11,12,13,14,15};
    rtapi_u16 checksum;
    rtapi_u16 max_frame_length = 20;
    rtapi_u16 fsizes[1];
    rtapi_u8  frames;
    unsigned char msg[MAX_MESSAGE_LENGTH];
    rtapi_u16 m = 0; // message byte pointer
    int p = 0; // data string pointer

    msg[m++] = *inst->hal.address;
    msg[m++] = ch->func;
    msg[m++] = ch->addr1;
    msg[m++] = ch->addr0;
    while (ch->data[p] != 0) {
        msg[m++] = 16 * hex[ch->data[p++] - '0'] + hex[ch->data[p++] - '0'];
    }
    checksum = RTU_CRC(msg, m);
    msg[m++] = checksum & 0xFF;
    msg[m++] = (checksum >> 8) & 0xFF;

    rtapi_print_msg(RTAPI_MSG_INFO, "Sending to %s bytes ", inst->port);
    for (int i = 0; i < m; i++) rtapi_print_msg(RTAPI_MSG_INFO, " %02X ", msg[i]);
    rtapi_print_msg(RTAPI_MSG_INFO, "\n");

    frames = 1;
    fsizes[0] = m;
    hm2_pktuart_send(inst->port, msg, &frames, fsizes);
    return 0;
}

void process(void *arg, long period) {
    static int state = START;
    static int index = -1;
    static int p = 0;
    
    hm2_modbus_inst_t *inst = arg;
    hm2_modbus_channel_t *ch;
    hm2_modbus_hal_t hal;
    char new_data[MAX_DATA] = {0};
    rtapi_u8 acc = 0;
    int byte_count;

    static int iter = 0; ///////////////////DEBUG

    switch (state) {
        case START:
            if (++index >= inst->num_chans) {
                index = 0; // channel counter
                p = 0; //  pins counter. Pins do not align to channels
            }
            ch = &(inst->chans[index]);
            hal = inst->hal;
            
            switch (ch->func){
                case 0x01: // Read coils
                case 0x02: // read discrete inputs
                    for (int i = 0; i < ch->count; i++){}
                    break;
                case 0x05: // Write single coil
                    if (hal.pins[p++]->b){
                        rtapi_snprintf(new_data, MAX_DATA, "FF00");
                    } else {
                        rtapi_snprintf(new_data, MAX_DATA, "0000");
                    }
                    if (strcmp(ch->data, new_data) != 0) {
                        rtapi_strlcpy(ch->data, new_data, MAX_DATA);
                        send_modbus_pkt(inst, ch);
                        state = WAIT_FOR_SEND_BEGIN;
                    }
                    break;
                case 0x15: // Write multiple coils
                    // first the data packet
                    byte_count = 0;
                    for (int i = 0; i < ch->count; i++){
                        if (hal.pins[p++]->b) acc += 1 << (i % 8);
                        if (i % 8 == 7 || i == (ch->count -1)) { // time for the next byte
                            rtapi_snprintf(new_data + strlen(new_data), MAX_DATA, "%02X", acc);
                            acc = 0;
                            byte_count++;
                        }
                    }
                    if (strcmp(ch->data + 2, new_data) != 0) { // +2 as new data has no bytecount
                        rtapi_snprintf(ch->data, MAX_DATA, "%02X%s", byte_count, new_data);
                        send_modbus_pkt(inst, ch);
                        state = WAIT_FOR_SEND_BEGIN;
                    }
                    break;
                default:
                    rtapi_print_msg(RTAPI_MSG_ERR, "Unknown Modbus instruction\n");
            }
            break;
        case WAIT_FOR_SEND_BEGIN:
            // single cycle delay to allow for queued data
            state = WAIT_FOR_SEND_COMPLETE;
            break;
        case WAIT_FOR_SEND_COMPLETE:
            rtapi_print_msg(RTAPI_MSG_ERR, "RX %X TX %X\n", hm2_pktuart_get_rx_status(inst->port),hm2_pktuart_get_tx_status(inst->port));
            
            if ( ! (hm2_pktuart_get_tx_status(inst->port) & 0x80)
                 || iter++ > 20) {
                state = START;
                iter = 0;
            }
            break;

        case WAIT_FOR_DATA_FRAME:
            if (hm2_pktuart_get_rx_status(inst->port) & 1 << 21) {
                // Get the data
                state = START;
            }
            break;
            
    }

}


void rtapi_app_exit(void){
int i;
int j;
for (i = 0; i < m->num_insts;i++){
    if (m->insts[i].chans != NULL) rtapi_kfree(m->insts[i].chans);
    // rtapi_kfree(m->insts[i].hal); Automatically freed as was hal_malloc-ed
}
if (m != NULL) rtapi_kfree(m);

hal_exit(comp_id);
}

uint16_t RTU_CRC(unsigned char* buf, int len)
{
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

