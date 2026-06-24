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

static uint8_t ds18b20_resolution_bits[MAX_SENSORS] = {10U, 11U};
static int8_t ds18b20_tl_c[MAX_SENSORS] = {0, 0};
static int8_t ds18b20_th_c[MAX_SENSORS] = {40, 40};

static uint8_t lm75b_ok = 0;
static uint8_t lm75b_os_config_ok = 0;
static uint8_t lm75b_conf_value = 0x00U;
static float alarm_on_c = 29.0f;
static float alarm_off_c = 28.0f;
static uint8_t alarm_latched = 0U;

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

#define UART_RX_BUF_SIZE 128U

static volatile char uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t uart_rx_head = 0U;
static volatile uint8_t uart_rx_tail = 0U;

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        char c = (char)(USART2->DR & 0xFFU);
        uint8_t next = (uint8_t)((uart_rx_head + 1U) % UART_RX_BUF_SIZE);

        if (next != uart_rx_tail) {
            uart_rx_buf[uart_rx_head] = c;
            uart_rx_head = next;
        }
    }

    if (USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        volatile uint32_t tmp;

        tmp = USART2->SR;
        tmp = USART2->DR;
        (void)tmp;
    }
}

static uint8_t USART2_ReadChar(char *c)
{
    if (uart_rx_head == uart_rx_tail) {
        return 0U;
    }

    *c = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) % UART_RX_BUF_SIZE);

    return 1U;
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
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
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

static uint8_t USART2_CharAvailable(void) {
    return (USART2->SR & USART_SR_RXNE) ? 1U : 0U;
}

static char USART2_GetCharBlocking(void) {
    while (!USART2_CharAvailable()) {}
    return (char)(USART2->DR & 0xFFU);
}

static uint8_t USART2_ReadLine(char *buf, uint8_t max_len) {
    uint8_t pos = 0U;

    if (max_len == 0U) {
        return 0U;
    }

    while (1) {
        char c = USART2_GetCharBlocking();

        if (c == 27) {
            buf[0] = 0;
            USART2_Print("\n\r");
            return 0U;
        }
        if ((c == '\r') || (c == '\n')) {
            buf[pos] = 0;
            USART2_Print("\n\r");
            return pos;
        }
        if ((c == '\b') || (c == 127)) {
            if (pos > 0U) {
                pos--;
                USART2_Print("\b \b");
            }
            continue;
        }
        if ((c >= ' ') && (pos < (uint8_t)(max_len - 1U))) {
            buf[pos++] = c;
            USART2_Send(c);
        }
    }
}

static uint8_t ParseFloatSimple(const char *s, float *out) {
    uint8_t i = 0U;
    int8_t sign = 1;
    uint32_t whole = 0U;
    uint32_t frac = 0U;
    uint32_t div = 1U;
    uint8_t have_digit = 0U;

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if (s[i] == '-') { sign = -1; i++; }
    else if (s[i] == '+') { i++; }

    while ((s[i] >= '0') && (s[i] <= '9')) {
        whole = whole * 10U + (uint32_t)(s[i] - '0');
        have_digit = 1U;
        i++;
    }

    if (s[i] == '.') {
        i++;
        while ((s[i] >= '0') && (s[i] <= '9')) {
            if (div < 1000U) {
                frac = frac * 10U + (uint32_t)(s[i] - '0');
                div *= 10U;
            }
            have_digit = 1U;
            i++;
        }
    }

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if ((s[i] != 0) || !have_digit) {
        return 0U;
    }

    *out = (float)sign * ((float)whole + ((float)frac / (float)div));
    return 1U;
}

static uint8_t ParseIntSimple(const char *s, int32_t *out) {
    uint8_t i = 0U;
    int8_t sign = 1;
    int32_t val = 0;
    uint8_t have_digit = 0U;

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if (s[i] == '-') { sign = -1; i++; }
    else if (s[i] == '+') { i++; }

    while ((s[i] >= '0') && (s[i] <= '9')) {
        val = val * 10 + (int32_t)(s[i] - '0');
        have_digit = 1U;
        i++;
    }

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if ((s[i] != 0) || !have_digit) {
        return 0U;
    }

    *out = val * (int32_t)sign;
    return 1U;
}

static uint8_t ParseHexByteSimple(const char *s, uint8_t *out) {
    uint8_t i = 0U;
    uint16_t val = 0U;
    uint8_t have_digit = 0U;

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if ((s[i] == '0') && ((s[i + 1U] == 'x') || (s[i + 1U] == 'X'))) {
        i += 2U;
    }

    while (1) {
        uint8_t d;
        char c = s[i];
        if ((c >= '0') && (c <= '9')) { d = (uint8_t)(c - '0'); }
        else if ((c >= 'a') && (c <= 'f')) { d = (uint8_t)(c - 'a' + 10); }
        else if ((c >= 'A') && (c <= 'F')) { d = (uint8_t)(c - 'A' + 10); }
        else { break; }
        val = (uint16_t)((val << 4U) | d);
        if (val > 0xFFU) { return 0U; }
        have_digit = 1U;
        i++;
    }

    while ((s[i] == ' ') || (s[i] == '\t')) { i++; }
    if ((s[i] != 0) || !have_digit) {
        return 0U;
    }

    *out = (uint8_t)val;
    return 1U;
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

static void LM75B_TempToRegister(float temp, uint8_t data[2]) {
    int16_t counts;
    int16_t raw;

    if (temp >= 0.0f) {
        counts = (int16_t)(temp * 2.0f + 0.5f);
    } else {
        counts = (int16_t)(temp * 2.0f - 0.5f);
    }

    raw = (int16_t)(counts * 128);
    data[0] = (uint8_t)(((uint16_t)raw >> 8U) & 0xFFU);
    data[1] = (uint8_t)((uint16_t)raw & 0xFFU);
}

static float LM75B_RegisterToTemp(const uint8_t data[2]) {
    int16_t raw = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    int16_t counts = (int16_t)(raw >> 7U);
    return (float)counts * 0.5f;
}

static uint8_t LM75B_WriteConfig(uint8_t conf) {
    uint8_t data[1];
    data[0] = conf;
    if (!I2C1_WriteData(LM75B_CONF_REG, data, 1U)) {
        return 0U;
    }
    lm75b_conf_value = conf;
    return 1U;
}

static uint8_t LM75B_ReadConfig(uint8_t *conf) {
    uint8_t data[1] = {0x00U};
    if (!I2C1_ReadData(LM75B_CONF_REG, data, 1U)) {
        return 0U;
    }
    *conf = data[0];
    return 1U;
}

static uint8_t LM75B_WriteSetpoint(uint8_t reg_addr, float temp) {
    uint8_t data[2];
    LM75B_TempToRegister(temp, data);
    return I2C1_WriteData(reg_addr, data, 2U);
}

static uint8_t LM75B_ReadSetpoint(uint8_t reg_addr, float *temp) {
    uint8_t data[2] = {0x00U, 0x00U};
    if (!I2C1_ReadData(reg_addr, data, 2U)) {
        return 0U;
    }
    *temp = LM75B_RegisterToTemp(data);
    return 1U;
}

static uint8_t LM75B_ApplySettings(void) {
    if (alarm_on_c <= alarm_off_c) {
        return 0U;
    }
    if (!LM75B_WriteConfig(lm75b_conf_value)) {
        return 0U;
    }
    if (!LM75B_WriteSetpoint(LM75B_TOS_REG, alarm_on_c)) {
        return 0U;
    }
    if (!LM75B_WriteSetpoint(LM75B_THYST_REG, alarm_off_c)) {
        return 0U;
    }
    return 1U;
}

static uint8_t LM75B_ConfigOS(void) {
    lm75b_conf_value = 0x00U;
    alarm_on_c = 29.0f;
    alarm_off_c = 28.0f;
    return LM75B_ApplySettings();
}

static void LM75B_PrintCurrentSettings(void) {
    uint8_t conf = 0U;
    float tos = 0.0f;
    float thyst = 0.0f;

    USART2_Print("\n\rLM75A settings:\n\r");
    if (LM75B_ReadConfig(&conf)) {
        USART2_Print("CONF = 0x");
        USART2_Send((char)((conf >> 4U) < 10U ? ('0' + (conf >> 4U)) : ('A' + ((conf >> 4U) - 10U))));
        USART2_Send((char)(((conf & 0x0FU) < 10U) ? ('0' + (conf & 0x0FU)) : ('A' + ((conf & 0x0FU) - 10U))));
        USART2_Print("\n\r");
    } else {
        USART2_Print("CONF read error\n\r");
    }

    if (LM75B_ReadSetpoint(LM75B_TOS_REG, &tos)) {
        USART2_Print("Tos / LED ON  = ");
        USART2_PrintFloat(tos, " C\n\r");
    } else {
        USART2_Print("Tos read error\n\r");
    }

    if (LM75B_ReadSetpoint(LM75B_THYST_REG, &thyst)) {
        USART2_Print("Thyst / LED OFF = ");
        USART2_PrintFloat(thyst, " C\n\r");
    } else {
        USART2_Print("Thyst read error\n\r");
    }
}

static uint8_t LM75B_ReadFloatFromUart(const char *prompt, float *value) {
    char line[24];
    USART2_Print(prompt);
    if (USART2_ReadLine(line, sizeof(line)) == 0U) {
        return 0U;
    }
    if (!ParseFloatSimple(line, value)) {
        USART2_Print("Input error\n\r");
        return 0U;
    }
    if ((*value < -55.0f) || (*value > 125.0f)) {
        USART2_Print("Range error: use -55..125 C\n\r");
        return 0U;
    }
    return 1U;
}

static void LM75B_ConfigMenu(void) {
    char line[24];
    uint8_t menu = 1U;

    while (menu) {
        USART2_Print("\n\r==============================\n\r");
        USART2_Print("       LM75A CONFIG MENU\n\r");
        USART2_Print("==============================\n\r");
        USART2_Print("1 - Set LED ON temperature (Tos)\n\r");
        USART2_Print("2 - Set LED OFF temperature (Thyst)\n\r");
        USART2_Print("3 - Set LM75A config register\n\r");
        USART2_Print("4 - Show current LM75A registers\n\r");
        USART2_Print("ESC - Exit menu\n\r");
        USART2_Print("Current software thresholds: ON=");
        USART2_PrintFloat(alarm_on_c, " C, OFF=");
        USART2_PrintFloat(alarm_off_c, " C\n\r");
        USART2_Print("Enter choice: ");

        if (USART2_ReadLine(line, sizeof(line)) == 0U) {
            break;
        }

        if (line[0] == '1') {
            float v;
            if (LM75B_ReadFloatFromUart("Enter LED ON temperature/Tos, C: ", &v)) {
                if (v <= alarm_off_c) {
                    USART2_Print("Error: Tos must be higher than Thyst\n\r");
                } else {
                    alarm_on_c = v;
                    if (LM75B_ApplySettings()) {
                        USART2_Print("Tos written OK\n\r");
                    } else {
                        USART2_Print("Tos write error\n\r");
                    }
                }
            }
        } else if (line[0] == '2') {
            float v;
            if (LM75B_ReadFloatFromUart("Enter LED OFF temperature/Thyst, C: ", &v)) {
                if (v >= alarm_on_c) {
                    USART2_Print("Error: Thyst must be lower than Tos\n\r");
                } else {
                    alarm_off_c = v;
                    if (LM75B_ApplySettings()) {
                        USART2_Print("Thyst written OK\n\r");
                    } else {
                        USART2_Print("Thyst write error\n\r");
                    }
                }
            }
        } else if (line[0] == '3') {
            uint8_t conf;
            USART2_Print("Enter CONF byte in hex, e.g. 00: ");
            if (USART2_ReadLine(line, sizeof(line)) != 0U) {
                if (ParseHexByteSimple(line, &conf)) {
                    lm75b_conf_value = conf;
                    if (LM75B_ApplySettings()) {
                        USART2_Print("CONF written OK\n\r");
                    } else {
                        USART2_Print("CONF write error\n\r");
                    }
                } else {
                    USART2_Print("Input error\n\r");
                }
            }
        } else if (line[0] == '4') {
            LM75B_PrintCurrentSettings();
        } else {
            USART2_Print("Unknown choice\n\r");
        }
    }

    USART2_Print("Exit LM75A menu\n\r");
}


static uint8_t DS18B20_ConfigByteToResolution(uint8_t cfg) {
    switch (cfg & 0x60U) {
        case 0x00U: return 9U;
        case 0x20U: return 10U;
        case 0x40U: return 11U;
        default:    return 12U;
    }
}

static int16_t DS18B20_RawApplyResolution(int16_t raw, uint8_t resolution) {
    switch (resolution) {
        case 9U:
            return (int16_t)(raw & (int16_t)~0x0007);
        case 10U:
            return (int16_t)(raw & (int16_t)~0x0003);
        case 11U:
            return (int16_t)(raw & (int16_t)~0x0001);
        default:
            return raw;
    }
}

static void DS18B20_PrintROM(uint8_t *rom) {
    uint8_t i;
    for (i = 0U; i < 8U; i++) {
        uint8_t b = rom[i];
        USART2_Send((char)((b >> 4U) < 10U ? ('0' + (b >> 4U)) : ('A' + ((b >> 4U) - 10U))));
        USART2_Send((char)(((b & 0x0FU) < 10U) ? ('0' + (b & 0x0FU)) : ('A' + ((b & 0x0FU) - 10U))));
        if (i < 7U) {
            USART2_Send(' ');
        }
    }
}

static void DS18B20_ShowSensors(uint8_t devCount) {
    uint8_t i;
    USART2_Print("\n\rDS18B20 sensors:\n\r");
    if (devCount == 0U) {
        USART2_Print("No DS18B20 found\n\r");
        return;
    }

    for (i = 0U; i < devCount; i++) {
        uint8_t sp_ok;
        USART2_Print("Slot ");
        USART2_PrintUInt((uint32_t)(i + 1U));
        USART2_Print(": ROM=");
        DS18B20_PrintROM(sensors[i].ROM_code);
        USART2_Print("\n\r  saved config: resolution=");
        USART2_PrintUInt(ds18b20_resolution_bits[i]);
        USART2_Print(" bit, TL=");
        USART2_PrintInt(ds18b20_tl_c[i]);
        USART2_Print(" C, TH=");
        USART2_PrintInt(ds18b20_th_c[i]);
        USART2_Print(" C\n\r");

        ds18b20_ReadStratchpad(1U, sensors[i].scratchpad_data, sensors[i].ROM_code);
        sp_ok = (Compute_CRC8(sensors[i].scratchpad_data, 9U) == 0U) ? 1U : 0U;
        if (sp_ok) {
            USART2_Print("  sensor scratchpad: TL=");
            USART2_PrintInt((int8_t)sensors[i].scratchpad_data[3]);
            USART2_Print(" C, TH=");
            USART2_PrintInt((int8_t)sensors[i].scratchpad_data[2]);
            USART2_Print(" C, resolution=");
            USART2_PrintUInt(DS18B20_ConfigByteToResolution(sensors[i].scratchpad_data[4]));
            USART2_Print(" bit\n\r");
        } else {
            USART2_Print("  scratchpad read/CRC error\n\r");
        }
    }
}

static uint8_t DS18B20_ReadSlotFromUart(uint8_t devCount, uint8_t *slot, uint8_t allow_all) {
    char line[12];
    int32_t v;

    if (allow_all) {
        USART2_Print("Enter DS18B20 slot 1..");
        USART2_PrintUInt(devCount);
        USART2_Print(" or 0 for all: ");
    } else {
        USART2_Print("Enter DS18B20 slot 1..");
        USART2_PrintUInt(devCount);
        USART2_Print(": ");
    }

    if (USART2_ReadLine(line, sizeof(line)) == 0U) {
        return 0U;
    }
    if (!ParseIntSimple(line, &v)) {
        USART2_Print("Input error\n\r");
        return 0U;
    }
    if (allow_all && (v == 0)) {
        *slot = 0U;
        return 1U;
    }
    if ((v < 1) || (v > (int32_t)devCount)) {
        USART2_Print("Slot error\n\r");
        return 0U;
    }
    *slot = (uint8_t)v;
    return 1U;
}

static uint8_t DS18B20_ApplyConfigToSlotValues(uint8_t index, uint8_t resolution, int8_t tl, int8_t th) {
    uint8_t ok;
    uint8_t read_ok;
    uint8_t rb;
    int8_t rth;
    int8_t rtl;

    ok = ds18b20_WriteConfig(1U, sensors[index].ROM_code, resolution, tl, th);
    if (!ok) {
        return 0U;
    }

    ds18b20_ReadStratchpad(1U, sensors[index].scratchpad_data, sensors[index].ROM_code);
    read_ok = (Compute_CRC8(sensors[index].scratchpad_data, 9U) == 0U) ? 1U : 0U;
    if (!read_ok) {
        return 0U;
    }

    rb = DS18B20_ConfigByteToResolution(sensors[index].scratchpad_data[4]);
    rth = (int8_t)sensors[index].scratchpad_data[2];
    rtl = (int8_t)sensors[index].scratchpad_data[3];
    if ((rb != resolution) || (rth != th) || (rtl != tl)) {
        return 0U;
    }

    ds18b20_resolution_bits[index] = rb;
    ds18b20_tl_c[index] = rtl;
    ds18b20_th_c[index] = rth;
    sensors[index].resolution = rb;
    return 1U;
}

static uint8_t DS18B20_ApplyConfigToSlot(uint8_t index) {
    return DS18B20_ApplyConfigToSlotValues(index,
                                           ds18b20_resolution_bits[index],
                                           ds18b20_tl_c[index],
                                           ds18b20_th_c[index]);
}

static void DS18B20_ApplyConfigSelected(uint8_t slot, uint8_t devCount) {
    uint8_t i;
    uint8_t ok_all = 1U;

    if (slot == 0U) {
        for (i = 0U; i < devCount; i++) {
            if (!DS18B20_ApplyConfigToSlot(i)) {
                ok_all = 0U;
            }
        }
    } else {
        ok_all = DS18B20_ApplyConfigToSlot((uint8_t)(slot - 1U));
    }

    if (ok_all) {
        USART2_Print("DS18B20 config written and checked OK\n\r");
    } else {
        USART2_Print("DS18B20 config write/check error\n\r");
    }
}

static void DS18B20_ConfigMenu(uint8_t devCount) {
    char line[24];
    uint8_t menu = 1U;

    if (devCount == 0U) {
        USART2_Print("\n\rDS18B20 menu unavailable: no sensors found\n\r");
        return;
    }

    while (menu) {
        USART2_Print("\n\r==============================\n\r");
        USART2_Print("      DS18B20 CONFIG MENU\n\r");
        USART2_Print("==============================\n\r");
        USART2_Print("1 - Show DS18B20 sensors\n\r");
        USART2_Print("2 - Set resolution, 9..12 bit\n\r");
        USART2_Print("3 - Set TL alarm threshold\n\r");
        USART2_Print("4 - Set TH alarm threshold\n\r");
        USART2_Print("5 - Set TL and TH together\n\r");
        USART2_Print("ESC - Exit menu\n\r");
        USART2_Print("Enter choice: ");

        if (USART2_ReadLine(line, sizeof(line)) == 0U) {
            break;
        }

        if (line[0] == '1') {
            DS18B20_ShowSensors(devCount);
        } else if (line[0] == '2') {
            uint8_t slot;
            int32_t v;
            if (DS18B20_ReadSlotFromUart(devCount, &slot, 1U)) {
                USART2_Print("Enter resolution 9..12: ");
                if ((USART2_ReadLine(line, sizeof(line)) != 0U) && ParseIntSimple(line, &v) && (v >= 9) && (v <= 12)) {
                    if (slot == 0U) {
                        uint8_t i;
                        for (i = 0U; i < devCount; i++) { ds18b20_resolution_bits[i] = (uint8_t)v; }
                    } else {
                        ds18b20_resolution_bits[slot - 1U] = (uint8_t)v;
                    }
                    DS18B20_ApplyConfigSelected(slot, devCount);
                } else {
                    USART2_Print("Resolution error\n\r");
                }
            }
        } else if (line[0] == '3') {
            uint8_t slot;
            int32_t v;
            if (DS18B20_ReadSlotFromUart(devCount, &slot, 1U)) {
                USART2_Print("Enter TL, C (-55..125): ");
                if ((USART2_ReadLine(line, sizeof(line)) != 0U) && ParseIntSimple(line, &v) && (v >= -55) && (v <= 125)) {
                    if (slot == 0U) {
                        uint8_t i;
                        for (i = 0U; i < devCount; i++) { ds18b20_tl_c[i] = (int8_t)v; }
                    } else {
                        ds18b20_tl_c[slot - 1U] = (int8_t)v;
                    }
                    DS18B20_ApplyConfigSelected(slot, devCount);
                } else {
                    USART2_Print("TL error\n\r");
                }
            }
        } else if (line[0] == '4') {
            uint8_t slot;
            int32_t v;
            if (DS18B20_ReadSlotFromUart(devCount, &slot, 1U)) {
                USART2_Print("Enter TH, C (-55..125): ");
                if ((USART2_ReadLine(line, sizeof(line)) != 0U) && ParseIntSimple(line, &v) && (v >= -55) && (v <= 125)) {
                    if (slot == 0U) {
                        uint8_t i;
                        for (i = 0U; i < devCount; i++) { ds18b20_th_c[i] = (int8_t)v; }
                    } else {
                        ds18b20_th_c[slot - 1U] = (int8_t)v;
                    }
                    DS18B20_ApplyConfigSelected(slot, devCount);
                } else {
                    USART2_Print("TH error\n\r");
                }
            }
        } else if (line[0] == '5') {
            uint8_t slot;
            int32_t tl;
            int32_t th;
            if (DS18B20_ReadSlotFromUart(devCount, &slot, 1U)) {
                USART2_Print("Enter TL, C (-55..125): ");
                if ((USART2_ReadLine(line, sizeof(line)) != 0U) && ParseIntSimple(line, &tl) && (tl >= -55) && (tl <= 125)) {
                    USART2_Print("Enter TH, C (-55..125): ");
                    if ((USART2_ReadLine(line, sizeof(line)) != 0U) && ParseIntSimple(line, &th) && (th >= -55) && (th <= 125)) {
                        if (slot == 0U) {
                            uint8_t i;
                            for (i = 0U; i < devCount; i++) {
                                ds18b20_tl_c[i] = (int8_t)tl;
                                ds18b20_th_c[i] = (int8_t)th;
                            }
                        } else {
                            ds18b20_tl_c[slot - 1U] = (int8_t)tl;
                            ds18b20_th_c[slot - 1U] = (int8_t)th;
                        }
                        DS18B20_ApplyConfigSelected(slot, devCount);
                    } else {
                        USART2_Print("TH error\n\r");
                    }
                } else {
                    USART2_Print("TL error\n\r");
                }
            }
        } else {
            USART2_Print("Unknown choice\n\r");
        }
    }

    USART2_Print("Exit DS18B20 menu\n\r");
}

static char *NextField(char **p)
{
    char *start = *p;
    char *s = *p;

    while ((*s != 0) && (*s != ';')) {
        s++;
    }

    if (*s == ';') {
        *s = 0;
        *p = s + 1;
    } else {
        *p = s;
    }

    return start;
}

static void HandlePcCommand(char *line, uint8_t devCount)
{
    char *p = line;
    char *cmd = NextField(&p);

    if ((cmd[0] == 'L') || (cmd[0] == 'l')) {
        char *on_str = NextField(&p);
        char *off_str = NextField(&p);

        float on_value;
        float off_value;

        if (!ParseFloatSimple(on_str, &on_value) ||
            !ParseFloatSimple(off_str, &off_value)) {
            USART2_Print("ERR;LM75A;BAD_FORMAT\n\r");
            return;
        }

        if (on_value <= off_value) {
            USART2_Print("ERR;LM75A;ON_MUST_BE_HIGHER_THAN_OFF\n\r");
            return;
        }

        alarm_on_c = on_value;
        alarm_off_c = off_value;

        if (LM75B_ApplySettings()) {
            USART2_Print("ACK;LM75A;ON=");
            USART2_PrintFloat(alarm_on_c, "");
            USART2_Print(";OFF=");
            USART2_PrintFloat(alarm_off_c, "");
            USART2_Print("\n\r");
        } else {
            USART2_Print("ERR;LM75A;APPLY_FAILED\n\r");
        }

        return;
    }

    if ((cmd[0] == 'D') || (cmd[0] == 'd')) {
        char *slot_str = NextField(&p);
        char *res_str = NextField(&p);
        char *tl_str = NextField(&p);
        char *th_str = NextField(&p);

        int32_t slot;
        int32_t res;
        int32_t tl;
        int32_t th;

        if (!ParseIntSimple(slot_str, &slot) ||
            !ParseIntSimple(res_str, &res) ||
            !ParseIntSimple(tl_str, &tl) ||
            !ParseIntSimple(th_str, &th)) {
            USART2_Print("ERR;DS18B20;BAD_FORMAT\n\r");
            return;
        }

        if ((slot < 0) || (slot > devCount)) {
            USART2_Print("ERR;DS18B20;BAD_SLOT\n\r");
            return;
        }

        if ((res < 9) || (res > 12)) {
            USART2_Print("ERR;DS18B20;BAD_RESOLUTION\n\r");
            return;
        }

        if ((tl < -55) || (tl > 125) || (th < -55) || (th > 125) || (tl >= th)) {
            USART2_Print("ERR;DS18B20;BAD_THRESHOLDS\n\r");
            return;
        }

        if (slot == 0) {
            uint8_t i;

            for (i = 0U; i < devCount; i++) {
                ds18b20_resolution_bits[i] = (uint8_t)res;
                ds18b20_tl_c[i] = (int8_t)tl;
                ds18b20_th_c[i] = (int8_t)th;
            }
        } else {
            ds18b20_resolution_bits[slot - 1] = (uint8_t)res;
            ds18b20_tl_c[slot - 1] = (int8_t)tl;
            ds18b20_th_c[slot - 1] = (int8_t)th;
        }

        DS18B20_ApplyConfigSelected((uint8_t)slot, devCount);

        USART2_Print("ACK;DS18B20;SLOT=");
        USART2_PrintInt(slot);
        USART2_Print(";RES=");
        USART2_PrintInt(res);
        USART2_Print(";TL=");
        USART2_PrintInt(tl);
        USART2_Print(";TH=");
        USART2_PrintInt(th);
        USART2_Print("\n\r");

        return;
    }

    USART2_Print("ERR;UNKNOWN_COMMAND\n\r");
}

static void ProcessUartCommands(uint8_t devCount)
{
    static char pc_cmd_buf[64];
    static uint8_t pc_cmd_pos = 0U;
    static uint8_t pc_cmd_active = 0U;

    char c;

    while (USART2_ReadChar(&c)) {
        if (!pc_cmd_active) {
            if (c == '@') {
                pc_cmd_active = 1U;
                pc_cmd_pos = 0U;
            } else if ((c == 'm') || (c == 'M')) {
                LM75B_ConfigMenu();
            } else if ((c == 'd') || (c == 'D')) {
                DS18B20_ConfigMenu(devCount);
            }
        } else {
            if ((c == '\r') || (c == '\n')) {
                pc_cmd_buf[pc_cmd_pos] = 0;
                pc_cmd_active = 0U;

                if (pc_cmd_pos > 0U) {
                    HandlePcCommand(pc_cmd_buf, devCount);
                }
            } else {
                if (pc_cmd_pos < (sizeof(pc_cmd_buf) - 1U)) {
                    pc_cmd_buf[pc_cmd_pos++] = c;
                } else {
                    pc_cmd_active = 0U;
                    pc_cmd_pos = 0U;
                    USART2_Print("ERR;COMMAND_TOO_LONG\n\r");
                }
            }
        }
    }
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


/* Strict machine-readable line for the PC Qt application.
   Human-readable UART log is left unchanged. */
static void Send_Data_To_PC(float lm75a_temp,
                            uint8_t lm75a_ok,
                            uint8_t os_active,
                            uint8_t alarm,
                            uint8_t devCount) {
    USART2_Print("DATA;LM75A=");

    if (lm75a_ok) {
        USART2_PrintFloat(lm75a_temp, "");
    } else {
        USART2_Print("ERR");
    }

    USART2_Print(";DS1=");

    if ((devCount > 0U) && (sensors[0].crc8_data_error == 0U)) {
        USART2_PrintFloat(sensors[0].temp, "");
    } else {
        USART2_Print("ERR");
    }

    USART2_Print(";DS2=");

    if ((devCount > 1U) && (sensors[1].crc8_data_error == 0U)) {
        USART2_PrintFloat(sensors[1].temp, "");
    } else {
        USART2_Print("ERR");
    }

    USART2_Print(";OS=");
    USART2_Print(os_active ? "LM75A" : "0");

    USART2_Print(";ALARM=");
    USART2_Print(alarm ? "1" : "0");

    USART2_Print("\n\r");
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
            if (res > 12U) {
                res = 12U;
            }
            ds18b20_resolution_bits[i] = res;
            ds18b20_tl_c[i] = 0;
            ds18b20_th_c[i] = 40;
            if (DS18B20_ApplyConfigToSlot(i)) {
                USART2_Print("DS18B20 startup config applied to slot ");
                USART2_PrintUInt((uint32_t)(i + 1U));
                USART2_Print("\n\r");
            } else {
                USART2_Print("DS18B20 startup config apply error on slot ");
                USART2_PrintUInt((uint32_t)(i + 1U));
                USART2_Print("\n\r");
            }
        }
    }

    USART2_Print("Course project: DS18B20 + LM75A I2C + OS + UART\n\r");
    USART2_Print("UART: USART2 PA2/PA3, I2C1: PB6/PB7, LM75A OS: PB10/D6\n\r");
    USART2_Print("Press m for LM75A config menu, d for DS18B20 config menu\n\r");
    if (!lm75b_os_config_ok) {
        USART2_Print("LM75A OS: threshold config warning; default Tos/Thyst may be used\n\r");
    }

    while (1) {
        float lm75b_temp = 0.0f;
        uint8_t lm75b_read_ok = 0U;
        uint8_t lm75b_os_active = LM75B_OS_IsActive();
        uint8_t any_valid_temp = 0U;
        float max_temp = -1000.0f;

        ProcessUartCommands(devCount);

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
                    sensors[i].resolution = DS18B20_ConfigByteToResolution(sensors[i].scratchpad_data[4]);
                    sensors[i].raw_temp = (int16_t)(((uint16_t)sensors[i].scratchpad_data[1] << 8U) |
                                                     sensors[i].scratchpad_data[0]);
                    sensors[i].raw_temp = DS18B20_RawApplyResolution(sensors[i].raw_temp, sensors[i].resolution);
                    sensors[i].temp = (float)sensors[i].raw_temp * 0.0625f;

                    ds18b20_tl_c[i] = (int8_t)sensors[i].scratchpad_data[3];
                    ds18b20_th_c[i] = (int8_t)sensors[i].scratchpad_data[2];

                    USART2_PrintFloat(sensors[i].temp, " C, resolution ");
                    USART2_PrintUInt(sensors[i].resolution);
                    USART2_Print(" bit, TL=");
                    USART2_PrintInt(ds18b20_tl_c[i]);
                    USART2_Print(" C, TH=");
                    USART2_PrintInt(ds18b20_th_c[i]);
                    USART2_Print(" C\n\r");

                    any_valid_temp = 1U;
                    if (sensors[i].temp > max_temp) {
                        max_temp = sensors[i].temp;
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
                USART2_Print("LM75A I2C: temp=");
                USART2_PrintFloat(lm75b_temp, " C\n\r");

                any_valid_temp = 1U;
                if (lm75b_temp > max_temp) {
                    max_temp = lm75b_temp;
                }
            } else {
                USART2_Print("LM75A I2C: read error\n\r");
            }

            lm75b_os_active = LM75B_OS_IsActive();
        } else {
            USART2_Print("LM75A I2C: init error\n\r");
        }

        if (any_valid_temp) {
            if (max_temp >= alarm_on_c) {
                alarm_latched = 1U;
            } else if (max_temp <= alarm_off_c) {
                alarm_latched = 0U;
            }
        } else {
            alarm_latched = 0U;
        }
        alarm = alarm_latched;

        USART2_Print("Alarm thresholds: ON=");
        USART2_PrintFloat(alarm_on_c, " C, OFF=");
        USART2_PrintFloat(alarm_off_c, " C\n\r");
        Send_Data_To_PC(lm75b_temp, lm75b_read_ok, lm75b_os_active, alarm, devCount);

        USART2_Print("\n\r");

        if (alarm) {
            LED_Blink(200000U);
        } else {
            LED_Off();
        }   
        ProcessUartCommands(devCount); 
        DelayMicro(1000000U);
    }
}
