
/*!
 * ----------------------------------------------------------------------------
 * @file TD_BME280.cpp
 * @section intro_sec Introduction
 * I2C driver for Bosch BME20 sensor (humidy, temperature and pressure)
 * @section author Author
 * Written by Esko Honkanen for Technode Design (info@technode.fi) 
 * @section license License
 * Beerware license.
 * @brief 'Simple is beatiful'
 * ----------------------------------------------------------------------------
 */

#include "Wire.h"
#include "TD_BME280.h"

/*!
 * ----------------------------------------------------------------------------
 * Defines
 * ----------------------------------------------------------------------------
*/
#define I2C Wire

TD_BME280::TD_BME280(uint8_t i2c_device_address)
{
    _i2c_device_address = i2c_device_address;
}

TD_BME280::~TD_BME280() {}

/*!
 * ----------------------------------------------------------------------------
 * Function begin()
 * ----------------------------------------------------------------------------
*/
void TD_BME280::begin(void)
{
     I2C.begin();
}

/*!
 * ----------------------------------------------------------------------------
 * Initilaze.
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::init(void)
{
    u_int8_t status;
    _error_code = 0;

    /*!
     * ----------------------------------------------------
     * Check if chip ID is correct.
     * ----------------------------------------------------
    */
    _sensorID = readByte(BME280_REG_ID);
    if ((_sensorID != 0x60) || (_error_code != NO_ERROR))
    {
        return _error_code | ERROR_WRONG_SENSOR_ID;
    }

    /*!
     * ----------------------------------------------------
     * Execute device soft-reset.
     * ----------------------------------------------------
    */
    writeByte(BME280_REG_RESET, SOFT_RESET); 
    delay(20);

    /*!
     * ----------------------------------------------------
     * Check if calibration is still active.
     * ----------------------------------------------------
    */
    status = 1;
    while (status)
    {
        status = readByte(BME280_REG_STATUS) & IM_UPDATE;
        if (_error_code != NO_ERROR) {
            return _error_code;
        }
        delay(10);
    }

    /*!
     * ----------------------------------------------------
     * Read calibration data.
     * See BOSCH BST-BME280-DS001-23
     *  ---------------------------------------------------
     * Big-Endian (BE) / Little-Endian (LE) of 0x1234:
     * Byte Index:      0  1
     * Big-Endian:     12 34
     * Little-Endian:  34 12
     * ----------------------------------------------------
    */
    /* Temperature */
    _cal_T1 = readWord(BME280_REG_DIG_T1, U_LE);
    _cal_T2 = readWord(BME280_REG_DIG_T2, S_LE);
    _cal_T3 = readWord(BME280_REG_DIG_T3, S_LE);

    #if defined TD_BME280_DEBUG
    pub_cal_T1 = _cal_T1;
    pub_cal_T2 = _cal_T2;
    pub_cal_T3 = _cal_T3;    
    #endif

    /* Pressure */
    _cal_P1 = readWord(BME280_REG_DIG_P1, U_LE);
    _cal_P2 = readWord(BME280_REG_DIG_P2, S_LE);
    _cal_P3 = readWord(BME280_REG_DIG_P3, S_LE);
    _cal_P4 = readWord(BME280_REG_DIG_P4, S_LE);
    _cal_P5 = readWord(BME280_REG_DIG_P5, S_LE);
    _cal_P6 = readWord(BME280_REG_DIG_P6, S_LE);
    _cal_P7 = readWord(BME280_REG_DIG_P7, S_LE);
    _cal_P8 = readWord(BME280_REG_DIG_P8, S_LE);
    _cal_P9 = readWord(BME280_REG_DIG_P9, S_LE);

    #if defined TD_BME280_DEBUG
    pub_cal_P1 = _cal_P1;
    pub_cal_P2 = _cal_P2;
    pub_cal_P3 = _cal_P3;
    pub_cal_P4 = _cal_P4;
    pub_cal_P5 = _cal_P5;
    pub_cal_P6 = _cal_P6;
    pub_cal_P7 = _cal_P7;
    pub_cal_P8 = _cal_P8;
    pub_cal_P9 = _cal_P9;
    #endif

    /* Humidity */
    _cal_H1 = readByte(BME280_REG_DIG_H1);
    _cal_H2 = readByte(BME280_REG_DIG_H2_L) | (readByte(BME280_REG_DIG_H2_H) << 8);
    _cal_H3 = readByte(BME280_REG_DIG_H3);
    _cal_H4 = (readByte(BME280_REG_DIG_H4_L) & 0x0F) | (readByte(BME280_REG_DIG_H4_H) << 4);
    _cal_H5 = ((readByte(BME280_REG_DIG_H5_L) & 0xF0) >> 4) | (readByte(BME280_REG_DIG_H5_H) << 4); 
    _cal_H6 = readByte(BME280_REG_DIG_H6);

    #if defined TD_BME280_DEBUG
    pub_cal_H1 = _cal_H1;
    pub_cal_H2 = _cal_H2;
    pub_cal_H3 = _cal_H3;
    pub_cal_H4 = _cal_H4;
    pub_cal_H5 = _cal_H5;
    pub_cal_H6 = _cal_H6;
    #endif


    /*!
     * ----------------------------------------------------
     * Set sampling.
     * ----------------------------------------------------
    */ 
    void setSampling();      
 
    return _error_code;
}

/*!
 * ----------------------------------------------------------------------------
 * Function readByte
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::readByte(uint8_t register_address)
{
    byte buffer[2];

    buffer[0] = register_address;
    I2C.beginTransmission(_i2c_device_address);
    if (I2C.write(buffer, 1) != 1)
    {
        _error_code |= ERROR_TRANSMISSION_LEN;
    }
    if (I2C.endTransmission())
    {
        _error_code |= ERROR_END_TRANSMISSION;
    }
    if (I2C.requestFrom(_i2c_device_address, 1ul) != 1)
    {
        _error_code |= ERROR_REQUEST_LEN;
    }
    return I2C.read();
}

/*!
 * ----------------------------------------------------------------------------
 * Function readWord
 * ----------------------------------------------------------------------------
*/
uint16_t TD_BME280::readWord(uint8_t register_address, uint8_t u8Sign)
{
    byte buffer[2];

    buffer[0] = readByte(register_address);
    buffer[1] = readByte(register_address + 1);
    switch (u8Sign)
    {
        case U_LE: 
            return ((buffer[0]) | (buffer[1]<<8));
            break;
        case S_LE: 
            return (int16_t)((buffer[0]) | (buffer[1]<<8));
            break;
        case U_BE: 
            return ((buffer[0]<<8) | (buffer[1]));
            break;
        case S_BE: 
            return (int16_t)((buffer[0]<<8) | (buffer[1]));
        default:
            return ((buffer[0]<<8) | (buffer[1]));
    }   
}

/*!
 * ----------------------------------------------------------------------------
 * Function writeByte
 * ----------------------------------------------------------------------------
*/
void TD_BME280::writeByte(uint8_t register_address, uint8_t data)
{   
    byte buffer[2];
    buffer[0] = register_address;
    buffer[1] = data;
    I2C.beginTransmission(_i2c_device_address);
    if (I2C.write(buffer, 2) != 0x02)
    {
        _error_code |= ERROR_WRITE_LEN;
    }
    if (I2C.endTransmission())
    {
        _error_code |= ERROR_END_TRANSMISSION;
    }    
}

/*!
 * ----------------------------------------------------------------------------
 * Function readTemperature
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::readTemperature(float *fT)
{
    byte buffer[3];
    int32_t adc_T, var1, var2, T;

    _error_code = NO_ERROR;
    buffer[0] = readByte(BME280_REG_TEMPERATURE + 0);
    buffer[1] = readByte(BME280_REG_TEMPERATURE + 1);
    buffer[2] = readByte(BME280_REG_TEMPERATURE + 2);
    adc_T = ((int32_t)(buffer[0]) << 16) | ((int32_t)(buffer[1]) << 8) | (int32_t)buffer[2];
    adc_T = (adc_T >> 4); 
    #if defined TD_BME280_DEBUG
    pub_adc_T = adc_T;
    #endif
    
    /* Calibration, see BOSCH BST-BME280-DS001-23 */
    var1  = ((((adc_T >> 3) - ((int32_t)_cal_T1<<1))) * ((int32_t)_cal_T2)) >> 11; 
    var2  = (((((adc_T >>4 ) - ((int32_t)_cal_T1)) * ((adc_T >> 4) - ((int32_t)_cal_T1))) >> 12) * ((int32_t)_cal_T3)) >> 14; 
    t_fine = var1 + var2 + t_adjust; 
    T  = (t_fine * 5 + 128) >> 8;
    *fT = ((float)T / 100);
    return _error_code;
}

/*!
 * ----------------------------------------------------------------------------
 * Function readPressure
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::readPressure(float *fP)
{    
    byte buffer[3];
    int32_t adc_P;
    int64_t var1, var2, p;

    _error_code = NO_ERROR;
    buffer[0] = readByte(BME280_REG_PRESSURE + 0);
    buffer[1] = readByte(BME280_REG_PRESSURE + 1);
    buffer[2] = readByte(BME280_REG_PRESSURE + 2);    
    adc_P = ((int32_t)(buffer[0]) << 16) | ((int32_t)(buffer[1]) << 8) | (int32_t)buffer[2];
    adc_P = adc_P >> 4;
    #if defined TD_BME280_DEBUG
    pub_adc_P = adc_P;
    #endif
 
    /* Calibration, see BOSCH BST-BME280-DS001-23 */
    var1 = ((int64_t)t_fine) - 128000; 
    var2 = var1 * var1 * (int64_t)_cal_P6; 
    var2 = var2 + ((var1*(int64_t)_cal_P5) << 17); 
    var2 = var2 + (((int64_t)_cal_P4) << 35); 
    var1 = ((var1 * var1 * (int64_t)_cal_P3)>>8) + ((var1 * (int64_t)_cal_P2) << 12); 
    var1 = (((((int64_t)1) << 47) + var1))*((int64_t)_cal_P1) >> 33; 
    if (var1 == 0) 
    { 
        return 0; // avoid exception caused by division by zero 
    } 
    p = 1048576 - adc_P; 
    p = (((p << 31)-var2)*3125)/var1; 
    var1 = (((int64_t)_cal_P9) * (p >> 13) * (p >> 13)) >> 25; 
    var2 = (((int64_t)_cal_P8) * p) >> 19; 
    p = ((p + var1 + var2) >> 8) + (((int64_t)_cal_P7) << 4);

    *fP = ((float)p / 256.0);  
    return _error_code;
}

/*!
 * ----------------------------------------------------------------------------
 * Function readHumidity
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::readHumidity(float *fH)
{
    byte buffer[2];
    float read_data = 0;
    uint32_t H;
    int32_t adc_H, v_x1_u32r;

    _error_code = NO_ERROR;
    buffer[0] = readByte(BME280_REG_HUMIDY + 0);
    buffer[1] = readByte(BME280_REG_HUMIDY + 1);    
    adc_H = (uint32_t)((buffer[0] << 8) | buffer[1]);
    #if defined TD_BME280_DEBUG 
    pub_adc_H = adc_H;
    #endif   
    
    /* Calibration, see BOSCH BST-BME280-DS001-23 */
    v_x1_u32r = (t_fine - ((int32_t)76800)); 
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)_cal_H4) << 20) - (((int32_t)_cal_H5) * \
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * \
    ((int32_t)_cal_H6)) >> 10) * (((v_x1_u32r * ((int32_t)_cal_H3)) >> 11) + \
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)_cal_H2) + \
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * \
    ((int32_t)_cal_H1)) >> 4)); 
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);  
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);     

    H = (uint32_t)(v_x1_u32r>>12);
    *fH = ((float)H / 1024.0);
    return _error_code;
}

/*!
 * ----------------------------------------------------------------------------
 * Function setSampling
 * ----------------------------------------------------------------------------
*/
void TD_BME280::setSampling(
    uint8_t mode,
    uint8_t tempSampling,
    uint8_t pressSampling,
    uint8_t humSampling,
    uint8_t filter,
    uint8_t duration)
{
    uint8_t config, ctrl_meas, ctrl_hum;
    config = (filter << 2) | (duration << 5);
    ctrl_meas = mode | (pressSampling << 2) | (tempSampling << 5);
    ctrl_hum = humSampling;
    //
    _error_code = NO_ERROR;
    writeByte(BME280_REG_CTRL_MEAS, MODE_SLEEP);
    writeByte(BME280_REG_CTRL_HUM, ctrl_hum);
    writeByte(BME280_REG_CONFIG, config);
    writeByte(BME280_REG_CTRL_MEAS, ctrl_meas);
}

/*!
 * ----------------------------------------------------------------------------
 * Start forced measuremet.
 * ----------------------------------------------------------------------------
*/
uint8_t TD_BME280::startForced(void)
 {
    uint8_t ctrl_meas;
    uint8_t ret_val = NO_ERROR;

    ctrl_meas = readByte(BME280_REG_CTRL_MEAS);
    ctrl_meas = ctrl_meas | MODE_FORCED;
    writeByte(BME280_REG_CTRL_MEAS, ctrl_meas);
    uint32_t start_time = millis();
    while (readByte(BME280_REG_STATUS) & MEASURING)
    {
        if ((millis() - start_time) > 1000) {
            ret_val = ERROR_FM_TIMEOUT;
            break; 
        }       
    }
    delay(1);
    return _error_code | ret_val;
}

/*!
 * ----------------------------------------------------------------------------
 *  Get temperature compensation value in C°.
 * ----------------------------------------------------------------------------
 */
float TD_BME280::getTCompensation(void) 
{
    return float((t_adjust * 5) >> 8) / 100.0;
}

/*!
 * ----------------------------------------------------------------------------
 * Set temperature compensation value in C°.
 * ----------------------------------------------------------------------------
 */
void TD_BME280::setTCompensation(float t_fine_tune) 
{
  t_adjust = ((int32_t(t_fine_tune * 100) << 8)) / 5;
}