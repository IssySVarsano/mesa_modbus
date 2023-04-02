#define COMP_NAME "mesa_modbus"

unsigned int baudrate = 9600;
char parity = 'N';              // options N, O, E
unsigned int txdelay = 40;      // should generally be larger than Rx Delay
unsigned int rxdelay = 20;
unsigned int drive_delay = 0;   // delay between setting drive enable and sending data

#define MAX_MESSAGE_LENGTH 16   // may be increased if necessary to max 251
/*
The format of the channel descriptors is:

{TYPE, DIR, ADDR, COUNT, pin_name}

TYPE is one of HAL_BIT, HAL_FLOAT, HAL_S32, HAL_U32
DIR = HAL_IN, HAL_OUT, HAL_IO. HAL_IN = Write to Modbus register
COUNT = number of coils/registers to read
*/


static const hm2_modbus_chan_descriptor_t channels[] = {
/*  {TYPE,    DIR,     ADDR,   COUNT, pin_name} */
    {HAL_BIT, HAL_IN,  0x000, 1,     "relay-A"},
    {HAL_BIT, HAL_IN,  0x001, 1,     "relay-B"},
    {HAL_BIT, HAL_IN,  0x002, 1,     "relay-C"},
    {HAL_BIT, HAL_IN,  0x003, 1,     "relay-D"},
};
    //{HAL_BIT, HAL_OUT, 0x0000, 3,     "sense"  },
    //{HAL_BIT, HAL_OUT, 0x0003, 1,     "sense-A"}, 
//};
