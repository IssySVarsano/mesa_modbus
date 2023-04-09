#define COMP_NAME "mesa_modbus"

unsigned int baudrate = 9600;
char parity = 'N';              // options N, O, E
unsigned int txdelay = 20;      // should generally be larger than Rx Delay
unsigned int rxdelay = 15;
unsigned int drive_delay = 0;   // delay between setting drive enable and sending data

#define MAX_MSG_LEN 16   // may be increased if necessary to max 251
/*
The format of the channel descriptors is:

{TYPE, FUNC, ADDR, COUNT, pin_name}

TYPE is one of HAL_BIT, HAL_FLOAT, HAL_S32, HAL_U32, ENCODER
FUNC = 1, 2, 3, 4, 5, 6, 15, 16 - Modbus commands
COUNT = number of coils/registers to read
*/


static const hm2_modbus_chan_descriptor_t channels[] = {
/*  {TYPE,    FUNC, ADDR,   COUNT, pin_name} */
    {HAL_BIT, 1,  0x0000, 8,     "state"},
    {HAL_BIT, 2,  0x0000, 8,     "input"},
    {HAL_BIT, 5,  0x0000, 1,     "relay-0"},
    {HAL_BIT, 5,  0x0001, 1,     "relay-1"},
    {HAL_BIT, 5,  0x0002, 1,     "relay-2"},
    {HAL_BIT, 5,  0x0003, 1,     "relay-3"},
};
