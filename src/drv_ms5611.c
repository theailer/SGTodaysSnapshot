#include "board.h"
#include "mw.h"

#define BMP085_OFF     digitalLo(BARO_GPIO, BARO_PIN);
#define MS5611_ADDR    0x77                                      // MS5611, Standard address 0x77
#define CMD_RESET      0x1E                                      // ADC reset command
#define CMD_ADC_READ   0x00                                      // ADC read command
#define CMD_ADC_CONV   0x40                                      // ADC conversion command
#define CMD_ADC_D1     0x00                                      // ADC D1 conversion
#define CMD_ADC_D2     0x10                                      // ADC D2 conversion
#define CMD_ADC_256    0x00                                      // ADC OSR=256
#define CMD_ADC_512    0x02                                      // ADC OSR=512
#define CMD_ADC_1024   0x04                                      // ADC OSR=1024
#define CMD_ADC_2048   0x06                                      // ADC OSR=2048
#define CMD_ADC_4096   0x08                                      // ADC OSR=4096
#define CMD_PROM_RD    0xA0                                      // Prom read command
#define PROM_NB           8                                      // 8*16Bit = 128 Bit

static void     ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
static uint32_t ms5611_read_adc(void);
static void     ms5611_start_ut(void);
static void     ms5611_get_ut(void);
static void     ms5611_start_up(void);
static void     ms5611_get_up(void);
static bool     ms5611_crc_ok(void);
static float    ms5611_calculate(void);
static uint32_t ms5611_ut;                                       // static result of temperature measurement
static uint32_t ms5611_up;                                       // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];                               // on-chip ROM
static uint8_t  ms5611_osr = CMD_ADC_4096;

bool ms5611Detect(baro_t *baro)
{
    uint8_t i, k = 0;
    
    if (hse_value != 12000000) {
        // PC13 (BMP085's XCLR reset input, which we use to disable it). Only needed when running at 8MHz
        gpio_config_t gpio;
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_Out_PP;
        gpioInit(GPIOC, &gpio);
        BMP085_OFF;
    }

    if(!i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &i)) return false;  // If we have a MS5611, it will reply.
    do
    {
        delay(100);
        ms5611_reset();                                          // Reset it
        delay(100);
        for (i = 0; i < PROM_NB; i++) ms5611_c[i] = ms5611_prom(i);// read all coefficients // read 8 words word:0 = ID; word:1-6 = C1-C6; word:7 = ChkSum
        delay(50);
        k++;
        if (k == 10) break;                                      // Try 10 times to get correct Calibrationdata
    }
    while (!ms5611_crc_ok());
    baro->ut_delay     = 9040;                                   // baro->ut_delay = 10000;
    baro->up_delay     = 9040;                                   // baro->up_delay = 10000; // Not used baro->repeat_delay = 1; baro->repeat_delay = 4000;
    baro->baro_type    = 2;                                      // 1 = BMP 2 = MS
    baro->start_ut     = ms5611_start_ut;
    baro->get_ut       = ms5611_get_ut;
    baro->start_up     = ms5611_start_up;
    baro->get_up       = ms5611_get_up;
    baro->calculate    = ms5611_calculate;
    if (k < 10) return true;                                     // Readout of calibration data was not successful
    else return false;
}

static void ms5611_reset(void)
{
    i2cWrite(MS5611_ADDR, CMD_RESET, 1);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t  rxbuf[2] = { 0, 0 };
    delay(10);
    i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf);  // send PROM READ command
    return ((uint16_t)rxbuf[0] << 8) | (uint16_t)rxbuf[1];
}

static uint32_t ms5611_read_adc(void)
{
    uint8_t rxbuf[3];
    i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf);                // read ADC
    return ((uint32_t)rxbuf[0] << 16) | ((uint32_t)rxbuf[1] << 8) | (uint32_t)rxbuf[2];
}

static void ms5611_start_ut(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void)
{
    uint32_t tmp = ms5611_read_adc();
    if (tmp != 0) ms5611_ut = tmp;                               // Keep old on error
}

static void ms5611_start_up(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void)
{
    uint32_t tmp = ms5611_read_adc();
    if (tmp != 0) ms5611_up = tmp;                               // Keep old on error
}

static bool ms5611_crc_ok(void)
{
    uint16_t crc = 0, zerocheck = 0;
    uint8_t  i, k, crc_save;

    crc_save = (uint8_t)(ms5611_c[7] & 0xF);                     // Save last 4 bit
    ms5611_c[7] = 0xFF00 & ms5611_c[7];                          // Last byte must be cleared for crc calculation
    for (i = 0; i < 16; i++)
    {
        zerocheck = zerocheck | ms5611_c[i>>1];                  // To check for empty buffer
        if (i%2 == 1) crc ^= (ms5611_c[i>>1]) & 0x00FF;
        else crc ^= ms5611_c[i>>1] >> 8;
        for (k = 8; k > 0; k--)
        {
			      if (crc & (0x8000)) crc = (crc << 1) ^ 0x3000;
            else crc = crc << 1;
        }
    }
    crc = (crc >> 12) & 0xF;                                     // crc = CRC code
    if (crc_save == (uint8_t)crc && zerocheck != 0) return true;
    else return false;
}

#ifdef AlternativeMS5611Calc
static float ms5611_calculate(void)
{
    float dT, TEMP, OFF, OFF2, SENS, SENS2, Aux;
#ifdef barotempdebug
    float T2;
#endif
    dT   = (float)ms5611_ut - ((float)ms5611_c[5] * 256.0f);
    TEMP = (dT * (float)ms5611_c[6]) / 8388608;
    OFF  = (float)ms5611_c[2] * 65536.0f + (((float)ms5611_c[4] / 128) * dT);
    SENS = (float)ms5611_c[1] * 32768.0f + (((float)ms5611_c[3] / 256) * dT);

    if (TEMP < 0)
    {
#ifdef barotempdebug
        T2    = (dT   * dT) / 0x80000000;
        TEMP  = TEMP  - T2;
#endif
        Aux   = TEMP  * TEMP;
        OFF2  = 2.50f * Aux;
        SENS2 = 1.25f * Aux;

        OFF   = OFF   - OFF2;
        SENS  = SENS  - SENS2;
    }
#ifdef barotempdebug
    BaroActualTemp = (TEMP + 2000) * 0.01f;
#endif
    return (float)((((float)ms5611_up * (SENS / 2097152)) - OFF) / 32768);
}

#else

static float ms5611_calculate(void)
{
    int32_t  temp;
#ifdef barotempdebug
    float    T2 = 0;
#endif
    int64_t  off, sens;
    float    dTf, delt, off2 = 0, sens2 = 0;

    dTf  = (float)ms5611_ut - ((float)ms5611_c[5] * 256.0f);     // Summergames 2.2
    temp = 2000 + (((int64_t)dTf * ms5611_c[6]) >> 23);          // temperature
    off  = ((int64_t)ms5611_c[2] << 16) + (int64_t)((dTf / 128.0f) * (float)ms5611_c[4]);
    sens = ((int64_t)ms5611_c[1] << 15) + (int64_t)((dTf / 256.0f) * (float)ms5611_c[3]);

    if (temp < 2000)                                             // temperature lower than 20degC
    {
#ifdef barotempdebug
        T2    = (dTf * dTf) / 0x80000000;
#endif
        delt  = (float)(temp - 2000);
        delt  = delt * delt;
        off2  = delt * 2.50f;
        sens2 = delt * 1.25f;
        if (temp < -1500)                                        // temperature lower than -15degC
        {
            delt  = (float)(temp + 1500);
            delt  = delt * delt;
            off2  = off2  + delt * 7.0f;
            sens2 = sens2 + delt * 5.5f;
        }
    }
#ifdef barotempdebug
    temp = temp - T2;
    BaroActualTemp = (float)temp * 0.01f;// Put out Temp (in C)
#endif
    off     = off  - (int64_t)off2;
    sens    = sens - (int64_t)sens2;
    return (float)(((((int64_t)ms5611_up * sens ) >> 21) - off) >> 15);
}
#endif