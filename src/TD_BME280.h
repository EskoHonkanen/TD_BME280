
/*!
 * ----------------------------------------------------------------------------
 * @file TD_BME280.h
 * I2C driver for Bosch BME20 sensor (humidy, temperature and pressure).
 * Written by Esko Honkanen for Technode Design (info@technode.fi).
 * You may use this library as it is or change it without limitations. 
 * Beerware license.
 * @brief 'Simple is beatiful'
 * ----------------------------------------------------------------------------
 */

#ifndef TD_BME280_H
#define TD_BME280_H

//#define TD_BME280_DEBUG

#if defined(ARDUINO) && ARDUINO >= 100
#include "Wire.h"
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define U_LE 1
#define S_LE 2
#define U_BE 3
#define S_BE 4

/*!
 * ----------------------------------------------------------------------------
 * Defines
 * See BOSCH BST-BME280-DS001-23
 * ----------------------------------------------------------------------------
*/
#define SOFT_RESET    0xB6
#define IM_UPDATE     0x01
#define MEASURING     0x08

/* POWER MODE */
#define MODE_SLEEP    0x00
#define MODE_FORCED   0x01
#define MODE_NORMAL   0x03

/* OVERSAMPLING RATE */
#define OSR_NONE      0x00
#define OSR_X1        0x01
#define OSR_X2        0x02
#define OSR_X4        0x03
#define OSR_X8        0x04
#define OSR_X16       0x05

/* IIR FILTER */
#define FILTER_OFF    0x00
#define FILTER_2      0x01
#define FILTER_4      0x02
#define FILTER_8      0x03
#define FILTER_16     0x04

/* STANDBY DURATION */
#define TSB_0_5       0x00
#define TSB_62_5      0x01
#define TSB_125       0x02
#define TSB_250       0x03
#define TSB_500       0x04
#define TSB_1000      0x05
#define TSB_10        0x06
#define TSB_20        0x07

/*!
 * ----------------------------------------------------------------------------
 * REGISTERS (See BOSCH BST-BME280-DS001-23):
 * hum_lsb            0xFE
 * hum_msb            0xFD          BME280_REG_HUMIDY       (2ul)
 * temp_xlsb          0xFC
 * temp_lsb           0xFB
 * temp_msb           0xFA          BME280_REG_TEMPERATURE  (2ul)
 * pres_xlsb          0XF9
 * pres_lsb           0xF8
 * pres_msb           0xF7          BME280_REG_PRESSURE     (3ul)
 * config             0xF5          BME280_REG_CONFIG
 * ctrl_meas          0xF4          BME280_REG_CTRL_MEAS
 * status             0xF3          BME280_REG_STATUS
 * humidity           0xF2          BME280_REG_HUM
 * calib26..callib41  0xE1..0xF0
 * reset              0xE0          BME280_REG_RESET
 * id                 0xD0          BME280_REG_ID
 * calib00..callib25  0x88..0xA1 
 * ----------------------------------------------------------------------------
*/

/*!
 * ----------------------------------------------------------------------------
 * Register defines
 * See BOSCH BST-BME280-DS001-23
 * ----------------------------------------------------------------------------
*/
#define BME280_ADDRESS          0x77

#define BME280_REG_HUMIDY       0xFD
#define BME280_REG_TEMPERATURE  0xFA
#define BME280_REG_PRESSURE     0xF7
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_RESET        0xE0
#define BME280_REG_ID           0xD0

#define BME280_REG_DIG_T1       0x88
#define BME280_REG_DIG_T2       0x8A
#define BME280_REG_DIG_T3       0x8C

#define BME280_REG_DIG_P1       0x8E
#define BME280_REG_DIG_P2       0x90
#define BME280_REG_DIG_P3       0x92
#define BME280_REG_DIG_P4       0x94
#define BME280_REG_DIG_P5       0x96
#define BME280_REG_DIG_P6       0x98
#define BME280_REG_DIG_P7       0x9A
#define BME280_REG_DIG_P8       0x9C
#define BME280_REG_DIG_P9       0x9E

#define BME280_REG_DIG_H1       0xA1
#define BME280_REG_DIG_H2_L     0xE1
#define BME280_REG_DIG_H2_H     0xE2
#define BME280_REG_DIG_H3       0xE3
#define BME280_REG_DIG_H4_H     0xE4
#define BME280_REG_DIG_H4_L     0xE5
#define BME280_REG_DIG_H5_L     0xE5
#define BME280_REG_DIG_H5_H     0xE6
#define BME280_REG_DIG_H6       0xE7

/*!
 * ----------------------------------------------------------------------------
 * Error codes
 * ----------------------------------------------------------------------------
*/
#define NO_ERROR                 0b00000000
#define ERROR_TRANSMISSION_LEN   0b00000001
#define ERROR_END_TRANSMISSION   0b00000010
#define ERROR_REQUEST_LEN        0b00000100
#define ERROR_WRITE_LEN          0b00001000
#define ERROR_WRONG_SENSOR_ID    0b00010000
#define ERROR_FM_TIMEOUT         0b00100000

/*!
 * ----------------------------------------------------------------------------
 * class TD_BME280
 * ----------------------------------------------------------------------------
*/
class TD_BME280
{
  public:
    /*!
     * \brief Initialize TD_BME280 instance
     * \param I2C address of the BME280 device
     */
    TD_BME280(uint8_t i2c_device_address);
    ~TD_BME280();

    /*!
     * ----------------------------------------------------
     * \brief Begin
     * ----------------------------------------------------
    */
    void begin(void);

    /*!
     * ----------------------------------------------------
     * \brief Initialize BME280
     * \return function result (uint8_t)
     * ----------------------------------------------------
     */
    uint8_t init(void); 

    /*!
     * ----------------------------------------------------
     * \brief Read tempeature from BME280
     * \return data out (float *fT)
     * \return function result (uint8_t)
     * ----------------------------------------------------
     */
    uint8_t readTemperature(float *fT);

    /*!
     * ----------------------------------------------------
     * \brief Read pressure from BME280
     * \return data out (float *fP)
     * \return function result (uint8_t)
     * ----------------------------------------------------
     */
    uint8_t readPressure(float *fP);

    /*!
     * ----------------------------------------------------
     * \brief Read humidity from BME280
     * \return data out (float *fH)
     * \return function result (uint8_t)
     * ----------------------------------------------------
     */
    uint8_t readHumidity(float *fH); 

    /*!
     * ----------------------------------------------------
     * \brief Set sampling paramaters. Default values are
     * shown below.
     * ----------------------------------------------------
     */
    void setSampling(
        uint8_t mode            = MODE_NORMAL,
        uint8_t tempSampling    = OSR_X16,
        uint8_t pressSampling   = OSR_X16,
        uint8_t humSampling     = OSR_X16,
        uint8_t filter          = FILTER_OFF,
        uint8_t duration        = TSB_0_5
    );

    /*!
     * ----------------------------------------------------
     * \brief Start forced mode measurement
     * \return Function result (uint8_t)
     * ----------------------------------------------------
     */
    uint8_t startForced(void);  

    /*!
     * ----------------------------------------------------
     * \brief Get temperature compensation value.
     * \return Current compensation value in C°.
     * ----------------------------------------------------
     */
    float getTCompensation(void);

    /*!
     * ----------------------------------------------------
     * \brief Set temperature compensation value.
     * \param fine_tune compensation value in C°.
     * ----------------------------------------------------
     */
    void setTCompensation(float t_fine_tune);

    #if defined TD_BME280_DEBUG
    /* Debug variables */
    uint16_t pub_cal_T1;
    uint16_t pub_cal_T2;
    uint16_t pub_cal_T3;

    uint16_t pub_cal_P1;
    uint16_t pub_cal_P2;
    uint16_t pub_cal_P3;
    uint16_t pub_cal_P4;
    uint16_t pub_cal_P5;
    uint16_t pub_cal_P6;
    uint16_t pub_cal_P7;
    uint16_t pub_cal_P8;
    uint16_t pub_cal_P9;

    uint16_t pub_cal_H1;
    uint16_t pub_cal_H2;
    uint16_t pub_cal_H3;
    uint16_t pub_cal_H4;
    uint16_t pub_cal_H5;
    uint16_t pub_cal_H6;

    int32_t pub_adc_T;  
    int32_t pub_adc_H;
    int32_t pub_adc_P; 
    #endif

  private:
    /*!
     * ----------------------------------------------------
     * \brief Read one byte from BME280 register
     * \param register_address
     * \return data (uint8_t)
     * ----------------------------------------------------
    */
    uint8_t readByte(uint8_t register_address);

    /*!
     * ----------------------------------------------------
     * \brief Read two bytes (word) from BME280 registers
     * \param register_address, u8Sign
     * \return data (uint16_t)
     * ----------------------------------------------------
    */
    uint16_t readWord(uint8_t register_address, uint8_t u8Sign);

    /*!
     * ----------------------------------------------------
     * \brief Write one byte to BME280 register
     * \param register_address 
     * \param data (byte)
     * ----------------------------------------------------
     */
    void writeByte(uint8_t register_address, uint8_t data);

    uint8_t _i2c_device_address;
    uint16_t _cal_T1;
    uint16_t _cal_T2;
    uint16_t _cal_T3;
    //
    uint16_t _cal_P1;
    int16_t  _cal_P2;
    int16_t  _cal_P3;
    int16_t  _cal_P4;
    int16_t  _cal_P5;
    int16_t  _cal_P6;
    int16_t  _cal_P7;
    int16_t  _cal_P8;
    int16_t  _cal_P9;
    //
    uint8_t  _cal_H1;
    int16_t  _cal_H2;
    uint8_t  _cal_H3;
    int16_t  _cal_H4;
    int16_t  _cal_H5;
    int8_t   _cal_H6;
    //
    uint8_t _sensorID;
    uint8_t _error_code;
    int32_t t_fine;
    int32_t t_adjust = 0;
};

#endif //TD_BME280_H