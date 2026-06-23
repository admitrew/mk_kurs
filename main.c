#include "stm32f10x.h"
#include "ds18b20.h"

#define MAX_SENSORS             2U
#define LM75B_ADDRESS           0x90U
#define LM75B_CONF_REG          0x01U
#define LM75B_TEMP_REG          0x00U
#define LM75B_THYST_REG         0x02U
#define LM75B_TOS_REG           0x03U
#define LM75B_OS_PORT_BIT       10U
#define I2C_REQUEST_WRITE       0x00U
#define I2C_REQUEST_READ        0x01U

Sensor sensors[MAX_SENSORS];

static uint8_t lm75b_ok = 0;
static uint8_t lm75b_os_config_ok = 0;

void Init_Sensors(void) {
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        sensors[i].raw_temp = 0x0;
        sensors[i].temp = 0.0f;
        sensors[i].crc8_rom = 0x0;
        sensors[i].crc8_data = 0x0;
        sensors[i].crc8_rom_error = 0x0;
        sensors[i].crc8_data_error = 0x0;
        sensors[i].resolution = 0;
        for (uint8_t j = 0; j < 8; j++) {
            sensors[i].ROM_code[j] = 0x00;
        }
        for (uint8_t j = 0; j < 9; j++) {
            sensors[i].scratchpad_data[j] = 0x00;
        }
    }
}

static uint32_t Get_APB1_FREQ(void) {
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;

    switch (ppre1) {
        case 0x04: return SystemCoreClock / 2U;
        case 0x05: return SystemCoreClock / 4U;
        case 0x06: return SystemCoreClock / 8U;
        case 0x07: return SystemCoreClock / 16U;
        default:   return SystemCoreClock;
    }
}

void USART2_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 - USART2_TX, alternate function push-pull, 2 MHz. */
    GPIOA->CRL &= ~(15U << (4U * 2U));
    GPIOA->CRL |=  (10U << (4U * 2U));

    /* PA3 - USART2_RX, floating input. */
    GPIOA->CRL &= ~(15U << (4U * 3U));
    GPIOA->CRL |=  (4U << (4U * 3U));

    USART2->BRR = Get_APB1_FREQ() / 9600U;
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

void USART2_Send(char c) {
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = (uint16_t)c;
}

void USART2_Print(const char *s) {
    while (*s) {
        USART2_Send(*s++);
    }
}

static void USART2_PrintUInt(uint32_t val) {
    char buf[11];
    uint8_t pos = 0;

    do {
        buf[pos++] = (char)('0' + (val % 10U));
        val /= 10U;
    } while (val != 0U);

    while (pos > 0U) {
        USART2_Send(buf[--pos]);
    }
}

static void USART2_PrintInt(int32_t val) {
    if (val < 0) {
        USART2_Send('-');
        val = -val;
    }
    USART2_PrintUInt((uint32_t)val);
}

void USART2_PrintFloat(float val, const char *suffix) {
    int32_t whole;
    int32_t fract;

    if (val < 0.0f) {
        USART2_Send('-');
        val = -val;
    }

    whole = (int32_t)val;
    fract = (int32_t)((val - (float)whole) * 100.0f + 0.5f);
    if (fract >= 100) {
        whole++;
        fract -= 100;
    }

    USART2_PrintUInt((uint32_t)whole);
    USART2_Send('.');
    USART2_Send((char)('0' + (fract / 10)));
    USART2_Send((char)('0' + (fract % 10)));
    USART2_Print(suffix);
}

static void LED_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(15UL << (4U * 5U));
    GPIOA->CRL |=  (1UL << (4U * 5U));
}

static void LED_On(void) {
    GPIOA->BSRR = 1UL << 5U;
}

static void LED_Off(void) {
    GPIOA->BSRR = 1UL << (5U + 16U);
}

static void LED_Blink(uint32_t delay_mcs) {
    LED_On();
    DelayMicro(delay_mcs);

    LED_Off();
    DelayMicro(delay_mcs);
}

/* LM75A OS output input: PB10 / Arduino D6. OS is open-drain and active LOW by default. */
static void LM75B_OS_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* PB10 - input with pull-up. */
    GPIOB->CRH &= ~(15UL << (4U * (LM75B_OS_PORT_BIT - 8U)));
    GPIOB->CRH |=  (8UL  << (4U * (LM75B_OS_PORT_BIT - 8U)));
    GPIOB->ODR |=  (1UL << LM75B_OS_PORT_BIT);
}

static uint8_t LM75B_OS_IsActive(void) {
    return ((GPIOB->IDR & (1UL << LM75B_OS_PORT_BIT)) == 0U) ? 1U : 0U;
}

/* I2C from laboratory work 4: PB6=SCL, PB7=SDA, LM75B. */
static uint8_t I2C_WaitSet(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
    while (((*reg) & mask) == 0U) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void I2C1_Init_LM75B(void) {
    uint32_t pclk1_mhz;
    uint32_t ccr;
    volatile uint32_t tmpreg;

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    tmpreg = RCC->APB2ENR;
    (void)tmpreg;

    /* PB6/PB7 - alternate function open-drain, 50 MHz. */
    GPIOB->CRL &= ~((15U << (4U * 6U)) | (15U << (4U * 7U)));
    GPIOB->CRL |=  ((15U << (4U * 6U)) | (15U << (4U * 7U)));

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    tmpreg = RCC->APB1ENR;
    (void)tmpreg;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    pclk1_mhz = Get_APB1_FREQ() / 1000000U;
    if (pclk1_mhz < 2U) {
        pclk1_mhz = 2U;
    }
    if (pclk1_mhz > 36U) {
        pclk1_mhz = 36U;
    }

    /* Standard mode 100 kHz: CCR = PCLK1 / (2 * 100 kHz). */
    ccr = Get_APB1_FREQ() / 200000U;
    if (ccr < 4U) {
        ccr = 4U;
    }

    I2C1->CR2 = pclk1_mhz;
    I2C1->CCR = ccr;
    I2C1->TRISE = pclk1_mhz + 1U;
    I2C1->OAR1 = 0x4000U;
    I2C1->CR1 = I2C_CR1_ACK | I2C_CR1_PE;

    lm75b_ok = 1U;
}

static void I2C1_StopAndClearErrors(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR);
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
}

static uint8_t I2C1_WriteData(uint8_t reg_addr, const uint8_t *buf, uint16_t bytes_count) {
    uint16_t i;

    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;

    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_SB, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    (void)I2C1->SR1;
    I2C1->DR = LM75B_ADDRESS | I2C_REQUEST_WRITE;

    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_ADDR, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = reg_addr;
    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_TXE, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }

    for (i = 0; i < bytes_count; i++) {
        I2C1->DR = buf[i];
        if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_TXE, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    }

    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_BTF, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

static uint8_t I2C1_SelectRegisterForRead(uint8_t reg_addr) {
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;

    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_SB, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    (void)I2C1->SR1;
    I2C1->DR = LM75B_ADDRESS | I2C_REQUEST_WRITE;

    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_ADDR, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = reg_addr;
    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_BTF, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }

    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_SB, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
    (void)I2C1->SR1;
    I2C1->DR = LM75B_ADDRESS | I2C_REQUEST_READ;
    return 1U;
}

static uint8_t I2C1_ReadData(uint8_t reg_addr, uint8_t *buf, uint16_t bytes_count) {
    volatile uint32_t tmp;

    if (bytes_count == 0U) {
        return 0U;
    }

    if (!I2C1_SelectRegisterForRead(reg_addr)) {
        return 0U;
    }

    if (bytes_count == 1U) {
        if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_ADDR, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
        I2C1->CR1 &= ~I2C_CR1_ACK;
        tmp = I2C1->SR1;
        tmp = I2C1->SR2;
        (void)tmp;
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_RXNE, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
        buf[0] = (uint8_t)I2C1->DR;
        I2C1->CR1 |= I2C_CR1_ACK;
        return 1U;
    }

    if (bytes_count == 2U) {
        if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_ADDR, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }

        /* Correct STM32F1 two-byte receive sequence: POS=1, ACK=0 before ADDR clear,
           then wait BTF, STOP, read DR twice. This avoids every-second-read errors. */
        I2C1->CR1 |= I2C_CR1_POS;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        tmp = I2C1->SR1;
        tmp = I2C1->SR2;
        (void)tmp;

        if (!I2C_WaitSet(&I2C1->SR1, I2C_SR1_BTF, 100000U)) { I2C1_StopAndClearErrors(); return 0U; }
        I2C1->CR1 |= I2C_CR1_STOP;
        buf[0] = (uint8_t)I2C1->DR;
        buf[1] = (uint8_t)I2C1->DR;

        I2C1->CR1 &= ~I2C_CR1_POS;
        I2C1->CR1 |= I2C_CR1_ACK;
        return 1U;
    }

    I2C1_StopAndClearErrors();
    return 0U;
}

static uint8_t LM75B_ConfigOS(void) {
    /* Configuration register: normal mode, OS comparator mode, OS active LOW, fault queue = 1. */
    const uint8_t conf[1] = {0x00U};

    /* LM75A Tos/Thyst registers use 9-bit signed data with 0.5 C resolution.
       29.0 C = 58 counts = 0x03A -> bytes 0x1D, 0x00.
       28.0 C = 56 counts = 0x038 -> bytes 0x1C, 0x00. */
    const uint8_t tos[2] = {0x1DU, 0x00U};
    const uint8_t thyst[2] = {0x1CU, 0x00U};

    if (!I2C1_WriteData(LM75B_CONF_REG, conf, 1U)) {
        return 0U;
    }
    if (!I2C1_WriteData(LM75B_TOS_REG, tos, 2U)) {
        return 0U;
    }
    if (!I2C1_WriteData(LM75B_THYST_REG, thyst, 2U)) {
        return 0U;
    }
    return 1U;
}

static uint8_t LM75B_ReadTemperature(float *temp) {
    uint8_t data[2] = {0x00U, 0x00U};
    int16_t raw;

    if (!I2C1_ReadData(LM75B_TEMP_REG, data, 2U)) {
        return 0U;
    }

    raw = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    *temp = (float)raw / 256.0f;
    return 1U;
}

int main(void) {
    uint8_t devCount = 0;
    uint8_t i = 0;
    uint8_t alarm = 0;
    uint8_t ds18b20_present = 0;

    SysTick_Config(SystemCoreClock / 1000000U);

    LED_Init();
    USART2_Init();
    I2C1_Init_LM75B();
    LM75B_OS_Init();
    if (lm75b_ok) {
        lm75b_os_config_ok = LM75B_ConfigOS();
    }

    ds18b20_PortInit();
    if (ds18b20_Reset() == 0U) {
        ds18b20_present = 1U;
    }

    Init_Sensors();

    if (ds18b20_present) {
        devCount = Search_ROM(0xF0, sensors);
        if (devCount > MAX_SENSORS) {
            devCount = MAX_SENSORS;
        }

        for (i = 0; i < devCount; i++) {
            uint8_t res = i + 10U;
            ds18b20_Init(1U, sensors[i].ROM_code, res);
            sensors[i].resolution = res;
        }
    }

    USART2_Print("Course project: DS18B20 + LM75B I2C + LM75A OS + UART\n\r");
    USART2_Print("UART: USART2 PA2/PA3, I2C1: PB6/PB7, LM75A OS: PB10/D6\n\r");
    if (!lm75b_os_config_ok) {
        USART2_Print("LM75A OS: threshold config warning; default Tos/Thyst may be used\n\r");
    }

    while (1) {
        float lm75b_temp = 0.0f;
        uint8_t lm75b_read_ok = 0U;
        uint8_t lm75b_os_active = LM75B_OS_IsActive();

        alarm = 0U;

        USART2_Print("=== NEW MEASUREMENTS ===\n\r");

        if (ds18b20_present && (devCount > 0U)) {
            for (i = 0; i < devCount; i++) {
                if (!sensors[i].crc8_data_error) {
                    ds18b20_ConvertTemp(1U, sensors[i].ROM_code);
                }
                DelayMicro(750000U);
                ds18b20_ReadStratchpad(1U, sensors[i].scratchpad_data, sensors[i].ROM_code);
                sensors[i].crc8_data = Compute_CRC8(sensors[i].scratchpad_data, 8U);
                sensors[i].crc8_data_error = (Compute_CRC8(sensors[i].scratchpad_data, 9U) == 0U) ? 0U : 1U;

                USART2_Print("DS18B20 ");
                USART2_Send((char)('0' + i + 1U));
                USART2_Print(": ");

                if (!sensors[i].crc8_data_error) {
                    sensors[i].raw_temp = (int16_t)(((uint16_t)sensors[i].scratchpad_data[1] << 8U) |
                                                     sensors[i].scratchpad_data[0]);
                    sensors[i].temp = (float)sensors[i].raw_temp * 0.0625f;

                    USART2_PrintFloat(sensors[i].temp, " C, resolution ");
                    USART2_PrintUInt(sensors[i].resolution);
                    USART2_Print(" bit\n\r");

                    if (sensors[i].temp > 29.0f) {
                        alarm = 1U;
                    }
                } else {
                    USART2_Print("CRC error\n\r");
                }

                DelayMicro(100000U);
            }
        } else {
            USART2_Print("DS18B20: not found\n\r");
        }

        if (lm75b_ok) {
            lm75b_read_ok = LM75B_ReadTemperature(&lm75b_temp);
            if (lm75b_read_ok) {
                USART2_Print("LM75B I2C: temp=");
                USART2_PrintFloat(lm75b_temp, " C\n\r");

                if (lm75b_temp > 29.0f) {
                    alarm = 1U;
                }
            } else {
                USART2_Print("LM75B I2C: read error\n\r");
            }

            if (lm75b_os_active) {
                alarm = 1U;
            }
        } else {
            USART2_Print("LM75B I2C: init error\n\r");
        }

        USART2_Print("\n\r");

        if (alarm) {
            LED_Blink(200000U);
        } else {
            LED_Off();
        }

        DelayMicro(1000000U);
    }
}
