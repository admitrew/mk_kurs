#include "ds18b20.h"

volatile uint32_t msTicks; 
uint8_t onewire_enum_fork_bit; 

void SysTick_Handler(void) {
  msTicks++;
}

void DelayMicro (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}


void ds18b20_PortInit(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// I/O Port B clock enabled, RCC->APB2ENR |= 1000b, IOPBEN = 1;
  GPIOB->CRH |= GPIO_CRH_MODE11;			// Output Mode, max speed 50MHz, GPIOB->CRH |=   11000000000000b, MODE11 = 11
  GPIOB->CRH |= GPIO_CRH_CNF11_0;			// General purpose output Open-drain, GPIOB->CRH |=  100000000000000b, CNF11 = 01
  GPIOB->CRH &= ~GPIO_CRH_CNF11_1;		// General purpose output Open-drain, GPIOB->CRH &= 0111111111111111b, CNF11 = 01
}
//--------------------------------------------------
uint8_t ds18b20_Reset(void)
{
  uint16_t status;
	GPIOB->BSRR = GPIO_BSRR_BR11;					// set 0 to PIN11 
  DelayMicro(480);											// delay 480 mcs
  GPIOB->BSRR = GPIO_BSRR_BS11;					// set 1 to PIN11 
  DelayMicro(60);												// delay 60 mcs
  status = GPIOB->IDR & GPIO_IDR_IDR11;	// check value on PIN11, if not zero then device not responds
  DelayMicro(480);											// delay 480 mcs
  return (status ? 1 : 0);
}
//----------------------------------------------------------
uint8_t ds18b20_ReadBit(void)
{
  uint8_t bit = 0;
  GPIOB->BSRR = GPIO_BSRR_BR11;							 	 // set 0 to PIN11
  DelayMicro(1);															 // delay 1 mcs
	GPIOB->BSRR = GPIO_BSRR_BS11;								 // set 1 to PIN11 
	DelayMicro(14);															 // delay 14 mcs
	bit = (GPIOB->IDR & GPIO_IDR_IDR11 ? 1 : 0); // check value on PIN11
	DelayMicro(45);															 // delay 45 mcs
  return bit;
}
//-----------------------------------------------
uint8_t ds18b20_ReadByte(void)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
		data |= ds18b20_ReadBit() << i;			// read bit by bit from sensor
  return data;
}
//-----------------------------------------------
void ds18b20_WriteBit(uint8_t bit)
{
  GPIOB->BSRR = GPIO_BSRR_BR11;					// set 0 to PIN11
  DelayMicro(bit ? 1 : 60);							// if bit is 1 delay 1mcs else 60mcs
  GPIOB->BSRR = GPIO_BSRR_BS11;					// set 1 to PIN11 
  DelayMicro(bit ? 60 : 1);							// if bit is 1 delay 60mcs else 1mcs
}
//-----------------------------------------------
void ds18b20_WriteByte(uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    ds18b20_WriteBit(data >> i & 1); 			// write bit by bit to sendor
    DelayMicro(5);												// delay for protection
  }
}
//-----------------------------------------------
void ds18b20_MatchRom(uint8_t* address)
{
		uint8_t i;
		ds18b20_Reset();
		ds18b20_WriteByte(MATCH_CODE);			// send match rom command to sensor
		for(i=0;i<8;i++)
		{
			ds18b20_WriteByte(address[i]); 		// send address to match with sensor address
		}
}



//-----------------------------------------------
static uint8_t ds18b20_ResolutionToConfig(uint8_t res_bits)
{
  if (res_bits == 9U)  return RESOLUTION_9BIT;
  if (res_bits == 10U) return RESOLUTION_10BIT;
  if (res_bits == 11U) return RESOLUTION_11BIT;
  if (res_bits == 12U) return RESOLUTION_12BIT;
  return 0U;
}

static uint8_t ds18b20_SelectDevice(uint8_t mode, uint8_t* address)
{
    uint8_t i;
    if (ds18b20_Reset()) {
        return 0U;
    }

    if (mode == 0U) {
        ds18b20_WriteByte(SKIP_ROM);
    } else {
        ds18b20_WriteByte(MATCH_CODE);
        for (i = 0U; i < 8U; i++) {
            ds18b20_WriteByte(address[i]);
        }
    }

    return 1U;
}

uint8_t ds18b20_WriteConfig(uint8_t mode, uint8_t* address, uint8_t res_bits, int8_t tl, int8_t th)
{
    uint8_t config_byte = ds18b20_ResolutionToConfig(res_bits);
    uint32_t wait_cnt;

    if (config_byte == 0U) {
        return 0U;
    }

    /* 1) Apply settings to the sensor scratchpad: TH, TL, configuration byte. */
    if (!ds18b20_SelectDevice(mode, address)) {
        return 0U;
    }
    ds18b20_WriteByte(WRITE_SCRATCHPAD);
    ds18b20_WriteByte((uint8_t)th);
    ds18b20_WriteByte((uint8_t)tl);
    ds18b20_WriteByte(config_byte);

    /* 2) Copy scratchpad to EEPROM so the configuration remains in the sensor. */
    if (!ds18b20_SelectDevice(mode, address)) {
        return 0U;
    }
    ds18b20_WriteByte(COPY_SCRATCHPAD);
    DelayMicro(20000U);

    /* 3) Recall EEPROM back to scratchpad. Reading after this checks the real
       configuration stored by the DS18B20, not only local variables. */
    if (!ds18b20_SelectDevice(mode, address)) {
        return 0U;
    }
    ds18b20_WriteByte(RECALL_E2);
    for (wait_cnt = 0U; wait_cnt < 1000U; wait_cnt++) {
        if (ds18b20_ReadBit()) {
            break;
        }
        DelayMicro(10U);
    }

    return 1U;
}

void ds18b20_Init(uint8_t mode, uint8_t* address, uint8_t res_bits)
{
    (void)ds18b20_WriteConfig(mode, address, res_bits, -30, 100);
}
//----------------------------------------------------------
void ds18b20_ConvertTemp(uint8_t mode, uint8_t* address)
{
  ds18b20_Reset();
  if(mode == 0) 													//if skip rom mode selected
  {
    ds18b20_WriteByte(SKIP_ROM); 					//send skip ROM command
  } 
	else 
	{
		ds18b20_MatchRom(address); 						//send match code command with address
	}
  ds18b20_WriteByte(CONVERT_TEMP); 				//send convert temp command
}
//----------------------------------------------------------
void ds18b20_ReadStratchpad(uint8_t mode, uint8_t *Data, uint8_t* address)
{
  uint8_t i;
  ds18b20_Reset();
  if(mode == 0)
  {
    ds18b20_WriteByte(SKIP_ROM); 					//if skip rom mode selected
  } 
	else 
	{
		ds18b20_MatchRom(address); 						//send match code command with address
	}
  ds18b20_WriteByte(READ_SCRATCHPAD); 		//send read scratchpad command
  for(i = 0;i < 9; i++)
  {
    Data[i] = ds18b20_ReadByte();					//read scratchpad byte by byte
  }
}

void ds18b20_ReadROM (uint8_t *Data) {
		uint8_t i;
		ds18b20_Reset();
		ds18b20_WriteByte(READ_ROM);					//send read rom command 		
		for(i = 0;i < 8; i++)
		{
			Data[i] = ds18b20_ReadByte(); 			//read rom byte by byte
		}
}

uint8_t Compute_CRC8 (uint8_t* data, uint8_t length) {
		uint8_t polynomial = 0x8C, crc = 0x0, i = 0, j = 0, lsb = 0, inbyte = 0;
		while (length--) {
				inbyte = data[j];
        for (i = 0; i < 8; i++) {
            lsb = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (lsb) 
							crc ^= polynomial;
            inbyte >>= 1;
        }
				j++;
		}
		return crc; 
}
// ROM Search Routine //

uint8_t Search_ROM(char command, Sensor *sensors) {
    uint8_t i = 0, sensor_num = 0;
		char DS1820_done_flag = 0;
    int DS1820_last_descrepancy = 0;
    char DS1820_search_ROM[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
    int descrepancy_marker, ROM_bit_index;
    char return_value, Bit_A, Bit_B;
    char byte_counter, bit_mask;
 
    return_value = 0;
    while (!DS1820_done_flag) {
        if (ds18b20_Reset()) {
            return 0;
        } else {
            ROM_bit_index=1;
            descrepancy_marker=0;
            char command_shift = command;
            for (int n=0; n<8; n++) {           // Search ROM command or Search Alarm command
								ds18b20_WriteBit(command_shift & 0x01);
                command_shift = command_shift >> 1; // now the next bit is in the least sig bit position.
            } 
            byte_counter = 0;
            bit_mask = 0x01;
            while (ROM_bit_index<=64) {
                Bit_A = ds18b20_ReadBit();
                Bit_B = ds18b20_ReadBit();
                if (Bit_A & Bit_B) {
                    descrepancy_marker = 0; // data read error, this should never happen
                    ROM_bit_index = 0xFF;
                } else {
                    if (Bit_A | Bit_B) {
                        // Set ROM bit to Bit_A
                        if (Bit_A) {
                            DS1820_search_ROM[byte_counter] = DS1820_search_ROM[byte_counter] | bit_mask; // Set ROM bit to one
                        } else {
                            DS1820_search_ROM[byte_counter] = DS1820_search_ROM[byte_counter] & ~bit_mask; // Set ROM bit to zero
                        }
                    } else {
                        // both bits A and B are low, so there are two or more devices present
                        if ( ROM_bit_index == DS1820_last_descrepancy ) {
                            DS1820_search_ROM[byte_counter] = DS1820_search_ROM[byte_counter] | bit_mask; // Set ROM bit to one
                        } else {
                            if ( ROM_bit_index > DS1820_last_descrepancy ) {
                                DS1820_search_ROM[byte_counter] = DS1820_search_ROM[byte_counter] & ~bit_mask; // Set ROM bit to zero
                                descrepancy_marker = ROM_bit_index;
                            } else {
                                if (( DS1820_search_ROM[byte_counter] & bit_mask) == 0x00 )
                                    descrepancy_marker = ROM_bit_index;
                            }
                        }
                    }
										ds18b20_WriteBit(DS1820_search_ROM[byte_counter] & bit_mask);
                    ROM_bit_index++;
                    if (bit_mask & 0x80) {
                        byte_counter++;
                        bit_mask = 0x01;
                    } else {
                        bit_mask = bit_mask << 1;
                    }
                }
            }
            DS1820_last_descrepancy = descrepancy_marker;
						for (i = 0; i < 8; i++) {
							 sensors[sensor_num].ROM_code[i] = DS1820_search_ROM[i];
						}
						sensor_num++;
        }
        if (DS1820_last_descrepancy == 0)
            DS1820_done_flag = 1;
    }
    return sensor_num;
}



