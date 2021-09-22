#include "LSM6DSL.h"

//-------------------------------------------------------------------
// i2c_write - упрощенная функция чтения по i2c
//-------------------------------------------------------------------
uint8_t i2c_read(LSM6DSL_CONFIG *config, uint8_t Register)
{
    uint8_t value;
    HAL_I2C_Mem_Read(config->i2c, DEVICE_ADDRESS, Register, I2C_MEMADD_SIZE_8BIT, &value, 1, 0xFF);
    return value;
}

//-------------------------------------------------------------------
// i2c_write - упрощенная процедура записи по i2c
//-------------------------------------------------------------------
void i2c_write(LSM6DSL_CONFIG *config, uint8_t Register, uint8_t value)
{
    HAL_I2C_Mem_Write(config->i2c, DEVICE_ADDRESS, Register, I2C_MEMADD_SIZE_8BIT, &value, 1, 0xFF);
}

//-------------------------------------------------------------------
// LSM6DSL_ReadID - функция чтения из регистра WHO_I_AM
//-------------------------------------------------------------------
int8_t LSM6DSL_ReadID(LSM6DSL_CONFIG *config)
{
    int8_t value = 0x00;
    value = i2c_read(config, WHO_I_AM);
    return value;
}

//-------------------------------------------------------------------
// LSM6DSL_Init - процедура инициализация датчки с выбором фильтра
//-------------------------------------------------------------------
LSM6DSL_RESULT LSM6DSL_Init(LSM6DSL_CONFIG *config)
{
    //-------------------------------------------------------------------
    // Проверяем who_i_am микросхемы
    //-------------------------------------------------------------------
    if (LSM6DSL_ReadID(config) != WHO_I_AM_reg) //проверям ид датчика
        return RESULT_WHO_I_AM_ERROR;
    //-------------------------------------------------------------------
    // выключаем гироскоп и фильтры
    //-------------------------------------------------------------------
    Gyro_Disable(config);
    Gyro_Disable_Filters(config);
    //-------------------------------------------------------------------
    // задаем диапазон чувствительности
    //-------------------------------------------------------------------
    Gyro_Full_Scale_Select(config);
    //-------------------------------------------------------------------
    // выбираем какой фильтр включить
    //-------------------------------------------------------------------
    switch (config->filter_select)
    {
    case GYRO_FILTER_SELECT_NONE_FILTER:
        break;

    case GYRO_FILTER_SELECT_HP_FILTER:
        Gyro_Enable_HP_Filter(config);
        break;

    case GYRO_FILTER_SELECT_LP_FILTER:
        Gyro_Enable_LP_Filter(config);
        break;

    case GYRO_FILTER_SELECT_HP_LP_FILTER:
        Gyro_Enable_HP_Filter(config);
        Gyro_Enable_LP_Filter(config);
        break;
    }
    //-------------------------------------------------------------------
    // включаем гироскоп
    //-------------------------------------------------------------------
    Gyro_BDU_Enable(config);
    Gyro_Full_Scale_Select(config);
    Gyro_High_Perfomance_Mode_Select(config);
    Gyro_Enable(config);
    return RESULT_OK;
}

//-------------------------------------------------------------------
// Gyro_getX - получение значений по оси x
//-------------------------------------------------------------------
int16_t Gyro_getX(LSM6DSL_CONFIG *config)
{
    int16_t x = 0;
    uint8_t buff[2];
    HAL_I2C_Mem_Read(config->i2c, DEVICE_ADDRESS, OUTX_L_G, I2C_MEMADD_SIZE_8BIT, buff, 2, 1);
    x = (buff[1] << 8) + buff[0];
    return x;
}

//-------------------------------------------------------------------
// Gyro_getY - получение значений по оси y
//-------------------------------------------------------------------
int16_t Gyro_getY(LSM6DSL_CONFIG *config)
{
    int16_t y = 0;
    uint8_t buff[2];
    HAL_I2C_Mem_Read(config->i2c, DEVICE_ADDRESS, OUTY_L_G, I2C_MEMADD_SIZE_8BIT, buff, 2, 1);
    y = (buff[1] << 8) + buff[0];
    return y;
}

//-------------------------------------------------------------------
// Gyro_getZ - получение значений по оси z
//-------------------------------------------------------------------
int16_t Gyro_getZ(LSM6DSL_CONFIG *config)
{
    int16_t z = 0;
    uint8_t buff[2];
    HAL_I2C_Mem_Read(config->i2c, DEVICE_ADDRESS, OUTZ_L_G, I2C_MEMADD_SIZE_8BIT, buff, 2, 1);
    z = (buff[1] << 8) + buff[0];
    return z; //возвращаем z
}

//-------------------------------------------------------------------
// Gyro_Convert - перевод необработанных значений value в угловую скорость
//-------------------------------------------------------------------
float Gyro_Convert(LSM6DSL_CONFIG *config, int16_t value)
{
    float a = 0;
    switch (config->full_scale_select)
    {
    case GYRO_FULL_SCALE_SELECT_125:
        return a = (float)value * 0.004375;
    case GYRO_FULL_SCALE_SELECT_250:
        return a = (float)value * 0.00875;
    case GYRO_FULL_SCALE_SELECT_500:
        return a = (float)value * 0.0175;
    case GYRO_FULL_SCALE_SELECT_1000:
        return a = (float)value * 0.035;
    case GYRO_FULL_SCALE_SELECT_2000:
        return a = (float)value * 0.07;
    }
}

//-------------------------------------------------------------------
// Gyro_Calibrate - функция высчитывает калибровочные значения в буффер
//-------------------------------------------------------------------
void Gyro_Calibrate(LSM6DSL_CONFIG *config, float *buffer)
{
    HAL_Delay(1000);
    double buff[3] = {0};         //создаем массив из 3 целых чисел
    for (int i = 0; i < 100; i++) //цикл, в котором каждый эллемент массива будет суммироваться 100 раз
    {
        buff[0] += Gyro_getX(config); //в 0 элемент массива прибавляем значение гироскопа по оси х
        buff[1] += Gyro_getY(config); //в 0 элемент массива прибавляем значение гироскопа по оси y
        buff[2] += Gyro_getZ(config); //в 0 элемент массива прибавляем значение гироскопа по оси z
    }
    buffer[0] = buff[0] / 100; //вычисляем среднее по оси x
    buffer[1] = buff[1] / 100; //вычисляем среднее по оси y
    buffer[2] = buff[2] / 100; //вычисляем среднее по оси z
}

//-------------------------------------------------------------------
// Read_temp - функция чтения температуры и перевод в градусы
//-------------------------------------------------------------------
int16_t Read_temp(LSM6DSL_CONFIG *config)
{
    int16_t temp = 0;
    int16_t x = 0;
    x = ((i2c_read(config, OUT_TEMP_H) << 8) + i2c_read(config, OUT_TEMP_L));
    temp = (x / 256) + 25;
    return temp;
}
//-------------------------------------------------------------------
// процедура отключения гироскопа
//-------------------------------------------------------------------
void Gyro_Disable(LSM6DSL_CONFIG *config)
{
    i2c_write(config, CTRL2_G, 0x00);
}
//-------------------------------------------------------------------
// процедура включения гироскопа
//-------------------------------------------------------------------
void Gyro_Enable(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL2_G);
    value &= ~GYRO_ODR_MASK;
    value |= (config->odr << 4);
    i2c_write(config, CTRL2_G, value);
}
//-------------------------------------------------------------------
// процедура отключения фильтров гироскопа
//-------------------------------------------------------------------
void Gyro_Disable_Filters(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL7_G);
    value &= ~HP_FILTER_MASK;
    value |= HP_FILTER_DISABLED;
    i2c_write(config, CTRL7_G, value);

    value = i2c_read(config, CTRL4_C);
    value &= ~LP_FILTER_MASK;
    value |= LP_FILTER_DISABLED;
    i2c_write(config, CTRL4_C, value);
}
//-------------------------------------------------------------------
// процедура включения HP фильтра гироскопа
//-------------------------------------------------------------------
void Gyro_Enable_HP_Filter(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL7_G);
    value &= ~HP_FILTER_MASK;
    value |= HP_FILTER_ENABLED;

    value &= ~HP_FILTER_BANDWIDTH_MASK;
    value |= config->hpf_bandwidth_select;
    i2c_write(config, CTRL7_G, value);
}
//-------------------------------------------------------------------
// процедура включения LP фильтра гироскопа
//-------------------------------------------------------------------
void Gyro_Enable_LP_Filter(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL4_C);
    value &= ~LP_FILTER_MASK;
    value |= LP_FILTER_ENABLED;
    i2c_write(config, CTRL4_C, value);

    value = i2c_read(config, CTRL6_C);
    value &= ~LP_FILTER_BANDWIDTH_MASK;
    value |= config->lpf1_bandwidth_select;
    i2c_write(config, CTRL6_C, value);
}
//-------------------------------------------------------------------
// процедура задания диапазона чувствительности гироскопа
//-------------------------------------------------------------------
void Gyro_Full_Scale_Select(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL2_G);
    value &= ~GYRO_FULL_SCALE_SELECT_MASK;
    value |= (config->full_scale_select);
    i2c_write(config, CTRL2_G, value);
}
//-------------------------------------------------------------------
// процедура задания режима производительности
//-------------------------------------------------------------------
void Gyro_High_Perfomance_Mode_Select(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL7_G);
    value &= ~GYRO_HIGH_PERFOMANCE_MODE_SELECT;
    value |= (config->high_perfomance_mode_select << 7);
    i2c_write(config, CTRL7_G, value);
}
//-------------------------------------------------------------------
// Включение BDU
//-------------------------------------------------------------------
void Gyro_BDU_Enable(LSM6DSL_CONFIG *config)
{
    uint8_t value;
    value = i2c_read(config, CTRL3_C);
    value &= ~0x64;
    value |= (0x64);
    i2c_write(config, CTRL3_C, value);
}