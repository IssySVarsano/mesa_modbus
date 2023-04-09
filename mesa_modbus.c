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
MODULE_DESCRIPTION("Modbus control using Mesa PktUART\n"
                    "halcompiled from mesa_modbus.c and renamed");
MODULE_LICENSE("GPL");

#define MAX_CHAN 8

char error_codes[11][30]={"NULL", "Illegal Function", "Illegal Data Address",
    "Illegal Data Value", "Server Device Failure", "Acknowledge",
    "Server Device Busy", "Negative Acknowledge", "Memory Parity Error",
    "Gateway Path Unavailable", "Gateway Failed to Respond"};

// This is needed by the header file
typedef struct {
    hal_type_t type;
    int func;
    rtapi_u16 addr;
    rtapi_u8 count;
    char name[HAL_NAME_LEN];
} hm2_modbus_chan_descriptor_t;
// extra type which handles wrapping
#define ENCODER 0xFF

// get the channel / register definitions
#include "mesa_modbus.h"

typedef struct {
    hal_u32_t *address;
    hal_data_u **pins;
    hal_float_t **scale;
    hal_float_t **offset;
} hm2_modbus_hal_t;

typedef struct {
    hal_type_t type;
    rtapi_u8 func;
    rtapi_u16 addr;
    int count;
    int start_pin;
    int ptr; // pointer to last byte
    rtapi_u8 data[MAX_MSG_LEN + 4]; // add enough space for header and checksum
} hm2_modbus_channel_t;

typedef struct{ 
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

enum {
    START,
    WAIT_FOR_SEND_BEGIN,
    WAIT_FOR_SEND_COMPLETE,
    WAIT_FOR_DATA_FRAME,
    WAIT_FOR_FRAME_SIZES,
    WAIT_FOR_DATA,
    FETCH_MORE_DATA,
    WAIT_A_BIT,
    WAIT_FOR_RX_CLEAR,
};

static int comp_id;
static hm2_modbus_t *m;
static void process(void *arg, long period);
int parse_data_frame(hm2_modbus_channel_t *ch, hm2_modbus_hal_t hal, rtapi_u32 *data, int bytes);
int build_data_frame(hm2_modbus_channel_t *ch, hm2_modbus_hal_t hal);
static uint16_t RTU_CRC(rtapi_u8* buf, int len);
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
        rtapi_print_msg(RTAPI_MSG_ERR, COMP_NAME": Out of Memory\n");
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
            hal_pin_dir_t dir;
            rtapi_print_msg(RTAPI_MSG_INFO, "ch %i is at %p\n", c, ch);
            ch->type = channels[c].type;
            ch->func = channels[c].func;
            ch->addr = channels[c].addr;
            ch->count = channels[c].count;
            ch->start_pin = p;
            dir = HAL_OUT; //default, is changed depending on function
            switch (ch->func) {
                case 5: // write coil
                    if (ch->count > 1) {
                        rtapi_print_msg(RTAPI_MSG_ERR, "Count > 1 invalid"
                        " for function 5, resetting\n");
                        ch->count = 1;
                    }
                    // deliberate fall-through
                case 15: // write multiple coils
                    if (ch->count > 8 * MAX_MSG_LEN){
                        ch-> count = 8 * MAX_MSG_LEN;
                        rtapi_print_msg(RTAPI_MSG_ERR,"The Modbus protocol"
                        " enforces a hard limit of 2008 coils per transaction"
                        " but with the current setup file this component is"
                        " limited to %i coils per message. resetting\n",
                        ch->count);
                    }
                    dir = HAL_IN;
                    // deliberate fall-through
                case 1: // read coils
                case 2: // read inputs
                    if (ch->count > 1){
                        for (int j = 0; j < ch->count; j++){
                            
                            retval = hal_pin_bit_newf(dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                    comp_id, COMP_NAME".%02i.%s-%02i",
                                    i, channels[c].name, j);
                        }
                    } else {
                        retval = hal_pin_bit_newf(dir,
                                    (hal_bit_t**)&(inst->hal.pins[p++]),
                                            comp_id, COMP_NAME".%02i.%s",
                                            i, channels[c].name);
                    }
                    break;
                case 6: // write single register
                    if (ch->count > 1) {
                        rtapi_print_msg(RTAPI_MSG_ERR, "Count > 1 invalid"
                        " for function %i, resetting\n", ch->func);
                        ch->count = 1;
                    }
                    // deliberate fall-through
                case 16: // write multiple registers
                    if (ch->count > MAX_MSG_LEN / 2){
                        ch-> count = MAX_MSG_LEN/2;
                        rtapi_print_msg(RTAPI_MSG_ERR,"The Modbus protocol"
                        " enforces a hard limit of 123 registers per transaction"
                        " but with the current setup file this component is"
                        " limited to %i coils per messsge. resetting\n",
                        ch->count);
                    }
                    dir = HAL_IN;
                    // deliberate fall-through
                case 3: // read holding registers
                case 4: // read input registers
                    for (int j = 0; j < ch->count; j++){
                        switch (ch->type){
                        case HAL_U32:
                            if (ch->count > 1) {
                                retval = hal_pin_u32_newf(dir,
                                    (hal_u32_t**)&(inst->hal.pins[p++]),
                                    comp_id, COMP_NAME".%02i.%s-%02i",
                                    i, channels[c].name, j);
                            } else {
                                retval = hal_pin_u32_newf(dir,
                                    (hal_u32_t**)&(inst->hal.pins[p++]),
                                    comp_id, COMP_NAME".%02i.%s",
                                    i, channels[c].name);
                            }
                            break;
                        case HAL_S32:
                            if (ch->count > 1) {
                                retval = hal_pin_s32_newf(dir,
                                    (hal_s32_t**)&(inst->hal.pins[p]),
                                    comp_id, COMP_NAME".%02i.%s-%02i",
                                    i, channels[c].name, j);
                                retval = hal_pin_float_newf(dir,
                                    &(inst->hal.offset[p++]),
                                    comp_id, COMP_NAME".%02i.%s-%02i-extnd",
                                    i, channels[c].name, j);
                            } else {
                                retval = hal_pin_s32_newf(dir,
                                    (hal_s32_t**)&(inst->hal.pins[p]),
                                    comp_id, COMP_NAME".%02i.%s",
                                    i, channels[c].name);
                                retval = hal_pin_float_newf(dir,
                                    &(inst->hal.offset[p++]),
                                    comp_id, COMP_NAME".%02i.%s-extnd",
                                    i, channels[c].name);
                            }
                            break;
                        case HAL_FLOAT:
                            if (ch->count > 1) {
                                retval = hal_pin_float_newf(dir,
                                    (hal_float_t**)&(inst->hal.pins[p]),
                                    comp_id, COMP_NAME".%02i.%s-%02i",
                                    i, channels[c].name, j);
                                retval = hal_pin_float_newf(HAL_IN,
                                    (hal_float_t**)&(inst->hal.scale[p]),
                                    comp_id, COMP_NAME".%02i.%s-%02i-scale",
                                    i, channels[c].name, j);
                                retval = hal_pin_float_newf(HAL_IN,
                                    (hal_float_t**)&(inst->hal.offset[p]),
                                    comp_id, COMP_NAME".%02i.%s-%02i-offset",
                                    i, channels[c].name, j);
                                *(inst->hal.scale[p]) = 1;
                                p++;
                            } else {
                                retval = hal_pin_float_newf(dir,
                                    (hal_float_t**)&(inst->hal.pins[p]),
                                    comp_id, COMP_NAME".%02i.%s",
                                    i, channels[c].name);
                                retval = hal_pin_float_newf(HAL_IN,
                                    (hal_float_t**)&(inst->hal.scale[p]),
                                    comp_id, COMP_NAME".%02i.%s-scale",
                                    i, channels[c].name, j);
                                retval = hal_pin_float_newf(HAL_IN,
                                    (hal_float_t**)&(inst->hal.offset[p]),
                                    comp_id, COMP_NAME".%02i.%s-offset",
                                    i, channels[c].name, j);
                                *(inst->hal.scale[p]) = 1;
                                p++;
                            }
                            break;
                        default:
                           rtapi_print_msg(RTAPI_MSG_ERR,
                           "Unsupported HAL pin type in mesa_modbus definition file\n",
                                         ch->type);
                            goto fail0;
                        } // type switch
                    } // pin count loop
                    break;
                default:
                    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unsupported modbus function %i in mesa_modbus definition file\n",
                                    ch->func);
                    goto fail0;
            } // func switch
        } // channel loop
    } // instance loop
    hal_ready(comp_id);
    return 0;

    fail0:
    hal_exit(comp_id);
    return -1;

}

int send_modbus_pkt(hm2_modbus_inst_t *inst, hm2_modbus_channel_t *ch){
    rtapi_u16 checksum;
    rtapi_u16 fsizes[1];
    rtapi_u8  frames;
    int p = 0; // data string pointer

    checksum = RTU_CRC(ch->data, ch->ptr + 1);
    ch->data[++(ch->ptr)] = checksum & 0xFF;
    ch->data[++(ch->ptr)] = (checksum >> 8) & 0xFF;

    rtapi_print_msg(RTAPI_MSG_INFO, "Sending to %s %i bytes ", inst->port, ch->ptr + 1);
    for (int i = 0; i <= ch->ptr; i++) rtapi_print_msg(RTAPI_MSG_INFO, " %02X ", ch->data[i]);
    rtapi_print_msg(RTAPI_MSG_INFO, "\n");

    frames = 1;
    fsizes[0] = ch->ptr + 1;
    hm2_pktuart_send(inst->port, ch->data, &frames, fsizes);
    return 0;
}

void do_timeout(char *port, int *state){
    static int iter = 0;
    static int old_state;
    if (*state != old_state){
        iter = 0;
        old_state = *state;
    }
    if (iter++ > 1000){
        rtapi_print_msg(RTAPI_MSG_INFO, "\n %i TIMEOUT_RESET %i\n", iter, *state);
        hm2_pktuart_setup(port, -1, -1, -1, 1, 1);
        *state = START;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "%i timeout %i\n", iter, *state);
}

void process(void *arg, long period) {
    static int state = START;
    static int index = -1;
    static int p = 0;
    
    hm2_modbus_inst_t *inst = arg;
    static hm2_modbus_channel_t *ch;
    static hm2_modbus_hal_t hal;
    rtapi_u32 txdata[16];
    static rtapi_u32 fsizes[16];
    static rtapi_u32 rxdata[257] = {55};
    int num_frames;
    static int frame_index;
    int r;

    rtapi_u32 rxstatus = hm2_pktuart_get_rx_status(inst->port);
    rtapi_u32 txstatus = hm2_pktuart_get_tx_status(inst->port);
    
    static int iter = 0; ///////////////////DEBUG

    switch (state) {
        case START:

            rtapi_print_msg(RTAPI_MSG_INFO, "START txstatus = %08X rxstatus = %08X\n", txstatus, rxstatus);

            // Check for received data
            if (rxstatus & 0x200000) {
                state = WAIT_FOR_DATA_FRAME;
                break;
            }
            
            // No incoming data, so service the outputs
            
            if (++index >= inst->num_chans) {
                index = 0; // channel counter
                p = 0; //  pins counter. Pins do not align to channels
            }
            ch = &(inst->chans[index]);
            hal = inst->hal;

            if (build_data_frame(ch, hal)){ // if data has changed
                r = send_modbus_pkt(inst, ch);
                state = WAIT_FOR_SEND_BEGIN;
            }

            break;
        case WAIT_FOR_SEND_BEGIN:
            // single cycle delay to allow for queued data
            rtapi_print_msg(RTAPI_MSG_INFO, "WAIT_FOR_SEND_BEGIN\n", txstatus, rxstatus);
            do_timeout(inst->port, &state); // just to reset the counter
            state = WAIT_FOR_SEND_COMPLETE;
            break;

        case WAIT_FOR_SEND_COMPLETE:
            rtapi_print_msg(RTAPI_MSG_INFO, "%i WAIT_FOR_SEND_COMPLETE RX %X TX %X\n", iter, rxstatus, txstatus);
            do_timeout(inst->port, &state);
            if ( ! (txstatus & 0x80)){
                state = WAIT_FOR_DATA_FRAME;
            }
            break;

        case WAIT_FOR_DATA_FRAME:
            do_timeout(inst->port, &state);
            rtapi_print_msg(RTAPI_MSG_INFO, "WAIT_FOR_DATA_FRAME - rxmode = %X\r", rxstatus);
            if ( ! ( rxstatus & 0x1F0000)) { // no data yet
                break;
            }
            num_frames = (rxstatus & 0x1F0000) >> 16;
            frame_index = 0;
            memset(fsizes, 0, sizeof(fsizes));
            // find the frame size
            hm2_pktuart_queue_get_frame_sizes(inst->port, fsizes);
            state = WAIT_FOR_FRAME_SIZES;
            break;
            
        case WAIT_FOR_FRAME_SIZES:
        case FETCH_MORE_DATA:
            rtapi_print_msg(RTAPI_MSG_INFO, "WAIT_FOR_FRAME_SIZES Index %i Frames %X %X %X %X\n", frame_index, fsizes[0], fsizes[1], fsizes[2], fsizes[3]);
            do_timeout(inst->port, &state);
            if (fsizes[frame_index] & 0xC000) { // indicates an overrun
                rtapi_print_msg(RTAPI_MSG_INFO, "RESET\n");
                state = START;
                hm2_pktuart_setup(inst->port, -1, -1, -1, 0, 1); // reset
                break;
            }
            r = hm2_pktuart_queue_read_data(inst->port, rxdata, fsizes[frame_index] & 0x3FF);
            rtapi_print_msg(RTAPI_MSG_INFO, "return value %i\n", r);
            state = WAIT_A_BIT;
            break;

        case WAIT_A_BIT:
            state = WAIT_FOR_DATA;
            break;
            
        case WAIT_FOR_DATA:
            rtapi_print_msg(RTAPI_MSG_INFO, "WAIT_FOR_DATA\n");
            parse_data_frame(ch, hal, rxdata, fsizes[frame_index] & 0x3FF);
            if ((fsizes[++frame_index] & 0x3FF) > 0){
                state = FETCH_MORE_DATA;
            } else {
                state = WAIT_FOR_RX_CLEAR;
            }
            break;

        case WAIT_FOR_RX_CLEAR:
            rtapi_print_msg(RTAPI_MSG_INFO, "%i WAIT_FOR_RX_CLEAR txdata = %08X\r", iter, rxstatus);
            do_timeout(inst->port, &state);
            rtapi_print_msg(RTAPI_MSG_INFO, "\r");
            if (rxstatus & 0x200000) break;
            state = START;
            break;
    }

}

int ch_append8(hm2_modbus_channel_t *ch, rtapi_u8 v){
    int r = 0;
    if (ch->ptr++ > MAX_MSG_LEN) return -MAX_MSG_LEN;
    if (ch->data[ch->ptr] != v) r = 1; // flag data changed
    ch->data[ch->ptr] = v;
    return r;
}

int ch_append16(hm2_modbus_channel_t *ch, rtapi_u16 v){
    int r = 0;
    r =  ch_append8(ch, (rtapi_u8)(v >> 8));
    r += ch_append8(ch, (rtapi_u8)(v & 0xFF));
    return r;
}
    
int ch_init(hm2_modbus_channel_t *ch, hm2_modbus_hal_t hal){
    int r;
    ch->ptr = -1;
    r = ch_append8(ch, *(hal.address));
    r = ch_append8(ch, ch->func);
    return r;
}

int build_data_frame(hm2_modbus_channel_t *ch, hm2_modbus_hal_t hal){
    
    char new_data[MAX_MSG_LEN] = {0};
    rtapi_u8 acc = 0;
    int byte_count;
    int r;
    int p = ch->start_pin;
    

    ch_init(ch, hal);
    
    switch (ch->func){
        case 1: // Read coils
        case 2: // read discrete inputs
        case 3: // read holding registers
        case 4: // read input registers
            r += ch_append16(ch, ch->addr);
            r += ch_append16(ch, ch->count);
            r += 1; // trigger a read packet every time
            break;
        case 5: // Write single coil
            r += ch_append16(ch, ch->addr);
            if (hal.pins[p]->b){
                r += ch_append16(ch, 0xFF00);
            } else {
                r += ch_append16(ch, 0x0000);
            }
            break;
        case 6: //Write single register
            r += ch_append16(ch, ch->addr);
            switch (ch->type){
                case HAL_U32:
                    r += ch_append16(ch, (rtapi_u16)hal.pins[p]->u);
                    break;
                case HAL_S32:
                    r += ch_append16(ch, (rtapi_s16)hal.pins[p]->s);
                    break;
                case HAL_FLOAT:
                    if (hal.scale != 0){
                            r += ch_append16(ch, (rtapi_s16)((hal.pins[p]->f
                                            - *hal.offset[p])
                                            / *hal.scale[p]));
                    }
                    break;
            }
            break;
        case 15: // Write multiple coils
            r += ch_append16(ch, ch->addr);
            r += ch_append16(ch, ch->count);
            r += ch_append8(ch, ( ch->count + 8 - 1) / 8);
            for (int i = 0; i < ch->count; i++){
                if (hal.pins[p++]->b) acc += 1 << (i % 8);
                if (i % 8 == 7 || i == (ch->count -1)) { // time for the next byte
                    r += ch_append8(ch, acc);
                    acc = 0;
                }
            }
            break;
        case 16: // write multiple holding registers
            r += ch_append16(ch, ch->addr);
            r += ch_append16(ch, ch->count);
            r += ch_append8(ch, ch->count * 2);
            for (int i = 0; i < ch->count; i++){
                switch (ch->type){
                case HAL_U32:
                    rtapi_snprintf(new_data + strlen(new_data), MAX_MSG_LEN, "%04X", (rtapi_u16)hal.pins[p]->u);
                    break;
                case HAL_S32:
                    rtapi_snprintf(new_data + strlen(new_data), MAX_MSG_LEN, "%04X", (rtapi_u16)hal.pins[p]->s);
                    break;
                case HAL_FLOAT:
                    if (hal.scale != 0){
                        rtapi_snprintf(new_data + strlen(new_data), MAX_MSG_LEN, "%04X",
                            (rtapi_u16)((hal.pins[p]->f - *hal.offset[p]) / *hal.scale[p]));
                    }
                    break;
                }
                p++ ; // increment pin pointer
            }
            if (strcmp(ch->data + 6, new_data) != 0) {
                rtapi_snprintf(ch->data, MAX_MSG_LEN, "%04X%02X%s", ch->count, ch->count * 2, new_data);
            }
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "Unknown Modbus instruction\n");
    }
    return r;
}

int parse_data_frame(hm2_modbus_channel_t *ch, hm2_modbus_hal_t hal, rtapi_u32 data[], int count){
    int i;
    int w = 0;
    int b = 0;
    int p;
    int tmp;
    rtapi_u8 bytes[MAX_MSG_LEN + 4];
    rtapi_u16 checksum;

    rtapi_print_msg(RTAPI_MSG_INFO, "Return packet is ");
    for (i = 0; i < count; i++){
        bytes[i] = (data[w] >> b) & 0xFF;
        rtapi_print_msg(RTAPI_MSG_INFO, "%02X ", bytes[i]);
        if ((b += 8) == 32) { b = 0; w++;}
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "\n");
    if ((bytes[1] & 0x7F ) != ch->func) {
        rtapi_print_msg(RTAPI_MSG_ERR, "call/response function number missmatch\n");
        return -1;
    }

    checksum = RTU_CRC(bytes, count - 2);
    if (bytes[count - 2] != (checksum & 0xFF) || bytes[count - 1] != (checksum >> 8)){
        rtapi_print_msg(RTAPI_MSG_ERR, "Modbus checksum error\n");
        rtapi_print_msg(RTAPI_MSG_ERR, "%2X%02X != %04X\n", bytes[count - 1], bytes[count - 2], checksum);
        return -1;
    }

    p = ch->start_pin;

    switch (bytes[1]){
        case 1: // read coils
        case 2: // read inputs
            w = 3;
            b = 0;
            for (i = 0; i < ch->count; i++){
                hal.pins[p++]->b = (bytes[w] & 1 << b);
                if (++b >= 8) { b = 0; w++;}
            }
            break;
        case 3: // read holding registers
        case 4: // read registers
            w = 3;
            for (i = 0; i < bytes[2] / 2; i++){
                switch (ch->type){
                case HAL_U32:
                    hal.pins[p]->u = 256 * bytes[w++] + bytes[w++];
                    p++;
                    break;
                case HAL_S32: // wrap the result into the (float) offset too
                    tmp = hal.pins[p]->u;
                    hal.pins[p]->u = bytes[w++] * 256 + bytes[w++];
                    tmp = hal.pins[p]->u - tmp;
                    if (tmp > 32768) tmp -= 65536;
                    if (tmp < -32768) tmp += 65536;
                    hal.offset[p] += tmp;
                    p++;
                    break;
                case HAL_FLOAT:
                    hal.pins[p]->f = *(hal.scale[p]) * (256 * bytes[w++] + bytes[w++])
                                     + *(hal.offset[p]);
                    p++;
                    break;
                }
            }
            break;
        // Nothing to do for write commands 5, 15, 16 ??
        case 5: // set single coil
        case 6: // write single register
        case 15: // write multiple coils echo
        case 16: // write multiple registers echo
            break;

        // The following are error codes    
        case 129: // 1 + error bit
        case 130: // 2 + error bit
        case 131: // 3 + error bit
        case 132: // 4 + error bit
        case 133: // 5 + error bit
        case 134: // 6 + error bit
        case 143: // 15 + error bit
        case 144: // 16 + error bit
            rtapi_print_msg(RTAPI_MSG_ERR, "Modbus error response function %i error %s\n",
                            bytes[1] & 0x8F, error_codes[bytes[2]]);
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "Unknown or unsupported Modbus function code\n");
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

uint16_t RTU_CRC(rtapi_u8* buf, int len)
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

