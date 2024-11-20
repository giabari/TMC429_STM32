// ----------------------------------------------------------------------------
// TMC429_HALSTM32.c
//
// Porting to STM32 (HAL) lib from Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC429_HALSTM32.h"
#include <Math.h>

TMC429_Status TMC429_status_;
uint8_t TMC429_clock_frequency_;
uint8_t TMC429_pulse_div_[TMC429_MOTOR_COUNT];
uint8_t TMC429_ramp_div_[TMC429_MOTOR_COUNT];
uint16_t TMC429_chip_select_pin_;
GPIO_TypeDef* TMC429_chip_select_port_;
SPI_HandleTypeDef* TMC429_SPI_;


// Hardware communication functions

uint32_t TMC429_readRegister(uint8_t smda, uint8_t address)
{
  TMC429_MosiDatagram mosi_datagram;
  mosi_datagram.rrs = TMC429_RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = TMC429_RW_READ;
  mosi_datagram.data = 0;
  TMC429_MisoDatagram miso_datagram = TMC429_writeRead(mosi_datagram);
  return miso_datagram.data;
}

void TMC429_writeRegister(uint8_t smda, uint8_t address, uint32_t data)
{
  TMC429_MosiDatagram mosi_datagram;
  mosi_datagram.rrs = TMC429_RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = TMC429_RW_WRITE;
  mosi_datagram.data = data;
  TMC429_writeRead(mosi_datagram);
}

TMC429_MisoDatagram TMC429_writeRead(TMC429_MosiDatagram mosi_datagram)
{
  TMC429_MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;

  HAL_GPIO_WritePin(TMC429_chip_select_port_, TMC429_chip_select_pin_, GPIO_PIN_RESET);
  HAL_Delay(1);

  for (int i=(TMC429_DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    uint8_t byte_read;

    HAL_SPI_TransmitReceive(TMC429_SPI_, &byte_write, &byte_read, 1, HAL_MAX_DELAY);

    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }

  HAL_GPIO_WritePin(TMC429_chip_select_port_, TMC429_chip_select_pin_, GPIO_PIN_SET);
  HAL_Delay(1);

  __disable_irq();
  TMC429_status_ = miso_datagram.status;
  __enable_irq();

  return miso_datagram;
}





void TMC429_setup(SPI_HandleTypeDef* SPI_Bus, GPIO_TypeDef *CS_GPIOx, uint16_t CS_GPIO_Pin, uint8_t clock_frequency_mhz)
{
    TMC429_chip_select_pin_ = CS_GPIO_Pin;
    TMC429_chip_select_port_ = CS_GPIOx;
    TMC429_SPI_ = SPI_Bus;

    HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);

    TMC429_specifyClockFrequencyInMHz(clock_frequency_mhz);

    for (uint8_t motor = 0; motor < TMC429_MOTOR_COUNT; ++motor)
    {
        TMC429_pulse_div_[motor] = 0;
        TMC429_ramp_div_[motor] = 0;
    }

    TMC429_setStepDiv(TMC429_STEP_DIV_MAX);
    TMC429_stopAll();
    TMC429_initialize();
}

void TMC429_specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
    if (clock_frequency <= TMC429_CLOCK_FREQUENCY_MAX)
    {
        TMC429_clock_frequency_ = clock_frequency;
    }
    else
    {
        TMC429_clock_frequency_ = TMC429_CLOCK_FREQUENCY_MAX;
    }
}

void TMC429_setStepDiv(uint8_t step_div)
{
  TMC429_GlobalParameters global_parameters;
  global_parameters.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_GLOBAL_PARAMETERS);
  global_parameters.clk2_div = step_div & TMC429_STEP_DIV_MASK;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_GLOBAL_PARAMETERS, global_parameters.bytes);
}

uint8_t TMC429_getStepDiv()
{
  TMC429_GlobalParameters global_parameters;
  global_parameters.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_GLOBAL_PARAMETERS);
  return global_parameters.clk2_div & TMC429_STEP_DIV_MASK;
}

double TMC429_stepDivToStepTime(uint8_t step_div)
{
  double step_time = (double)(16*(1 + step_div))/(double)TMC429_clock_frequency_;
  return step_time;
}


void TMC429_stopAll()
{
  for (uint8_t motor=0; motor<TMC429_MOTOR_COUNT; ++motor)
  {
    TMC429_stop(motor);
  }
}

void TMC429_stop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_setMode(motor, VELOCITY_MODE);
  TMC429_setTargetVelocity(motor, 0);
}

void TMC429_setVelocityMode(size_t motor)
{
	TMC429_setMode(motor, VELOCITY_MODE);
}

void TMC429_setMode(size_t motor, TMC429_Mode mode)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }

  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.mode = (uint8_t)mode;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_setTargetVelocity(size_t motor, int16_t velocity)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_V_TARGET, velocity);
}

void TMC429_initialize()
{
  TMC429_setStepDirOutput();
}

void TMC429_setStepDirOutput()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_sd = 1;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_disableLeftSwitchStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 1;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_disableRightSwitches()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 0;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_setLimitsInHz(size_t motor, uint32_t velocity_min_hz, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }

  TMC429_setOptimalStepDivHz(velocity_max_hz);

  TMC429_setOptimalPulseDivHz(motor, velocity_max_hz);

  TMC429_setVelocityMinInHz(motor, velocity_min_hz);

  TMC429_setVelocityMaxInHz(motor, velocity_max_hz);

  TMC429_setOptimalRampDivHz(motor, velocity_max_hz, acceleration_max_hz_per_s);

  uint32_t a_max = TMC429_setAccelerationMaxInHzPerS(motor, velocity_max_hz, acceleration_max_hz_per_s);

  TMC429_setOptimalPropFactor(motor, a_max);
}

void TMC429_setOptimalStepDivHz(uint32_t velocity_max_hz)
{
  int step_div = TMC429_getStepDiv();

  double step_time = TMC429_stepDivToStepTime(step_div);

  uint32_t velocity_max_upper_limit = (double)TMC429_MHZ_PER_HZ/(step_time*2);

  while ((velocity_max_upper_limit < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = TMC429_stepDivToStepTime(step_div);
    velocity_max_upper_limit = (double)TMC429_MHZ_PER_HZ/(step_time*2);
  }

  TMC429_setStepDiv(step_div);
}

void TMC429_setOptimalPulseDivHz(size_t motor, uint32_t velocity_max_hz)
{
  uint8_t pulse_div = TMC429_findOptimalPulseDivHz(velocity_max_hz);
  TMC429_ClkConfig clk_config;
  clk_config.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.pulse_div = pulse_div;
  TMC429_writeRegister(motor, TMC429_ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  TMC429_pulse_div_[motor] = pulse_div;
}

uint8_t TMC429_findOptimalPulseDivHz(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = TMC429_PULSE_DIV_MAX + 1;
  uint32_t velocity_max_upper_limit = 0;
  while ((velocity_max_upper_limit < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    velocity_max_upper_limit = TMC429_getVelocityMaxUpperLimitInHz(pulse_div);
  }
  return pulse_div;
}

uint32_t TMC429_getVelocityMaxUpperLimitInHz(uint8_t pulse_div)
{
  return TMC429_convertVelocityToHz(pulse_div, TMC429_VELOCITY_REGISTER_MAX);
}

uint32_t TMC429_getVelocityMaxUpperLimitInHz_Zero()
{
  return TMC429_convertVelocityToHz(0, TMC429_VELOCITY_REGISTER_MAX);
}

int32_t TMC429_convertVelocityToHz(uint8_t pulse_div, int16_t velocity)
{
  double x = ((double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ)/(double)TMC429_VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div));
  return y;
}

void TMC429_setVelocityMinInHz(size_t motor, uint32_t velocity_min_hz)
{
  uint32_t velocity_min = TMC429_convertVelocityFromHz(TMC429_pulse_div_[motor], velocity_min_hz);
  if (velocity_min < TMC429_VELOCITY_MIN_MIN)
  {
    velocity_min = TMC429_VELOCITY_MIN_MIN;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_V_MIN, velocity_min);
}

void TMC429_setVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz)
{
  uint32_t velocity_max = TMC429_convertVelocityFromHz(TMC429_pulse_div_[motor], velocity_max_hz);
  uint32_t velocity_max_upper_limit = TMC429_getVelocityMaxUpperLimitInHz_Zero();
  if (velocity_max > velocity_max_upper_limit)
  {
    velocity_max = velocity_max_upper_limit;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_V_MAX, velocity_max);
}

int16_t TMC429_convertVelocityFromHz(uint8_t pulse_div, int32_t velocity)
{
  double x = ((double)velocity*(double)(1 << pulse_div))/((double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ);
  double y = x*(double)TMC429_VELOCITY_CONSTANT;
  return y;
}

void TMC429_setOptimalRampDivHz(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint8_t ramp_div = TMC429_findOptimalRampDivHz(velocity_max_hz, acceleration_max_hz_per_s);
  TMC429_ClkConfig clk_config;
  clk_config.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.ramp_div = ramp_div;
  TMC429_writeRegister(motor, TMC429_ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  TMC429_ramp_div_[motor] = ramp_div;
}

uint8_t TMC429_findOptimalRampDivHz(uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint8_t pulse_div = TMC429_findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = TMC429_RAMP_DIV_MAX;
  uint32_t acceleration_max_upper_limit = TMC429_getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);;
  uint32_t acceleration_max_lower_limit = TMC429_getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);

  while ((acceleration_max_upper_limit < acceleration_max_hz_per_s) &&
    (acceleration_max_lower_limit < acceleration_max_hz_per_s) &&
    (ramp_div >= 1) &&
    (ramp_div >= pulse_div))
  {
    --ramp_div;
    acceleration_max_upper_limit = TMC429_getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
    acceleration_max_lower_limit = TMC429_getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
  }
  return ramp_div;
}

uint32_t TMC429_getAccelerationMaxUpperLimitInHzPerS_Single(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = TMC429_findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div;
  if (pulse_div > 0)
  {
    ramp_div = pulse_div - 1;
  }
  else
  {
    ramp_div = TMC429_RAMP_DIV_MIN;
  }
  return TMC429_getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
}

uint32_t TMC429_getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div)
{
  uint32_t a_max_upper_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div + 1) >= 0)
  {
    a_max_upper_limit = TMC429_ACCELERATION_REGISTER_MAX;
  }
  else if (((int8_t)ramp_div - (int8_t)pulse_div + 12) < 1)
  {
    a_max_upper_limit = TMC429_ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_upper_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div + 12)) - 1;
  }
  if (a_max_upper_limit > TMC429_ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = TMC429_ACCELERATION_REGISTER_MAX;
  }
  if (a_max_upper_limit < TMC429_ACCELERATION_REGISTER_MIN)
  {
    a_max_upper_limit = TMC429_ACCELERATION_REGISTER_MIN;
  }
  return TMC429_convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_upper_limit);
}

uint32_t TMC429_getAccelerationMaxLowerLimitInHzPerS_Single(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = TMC429_findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = TMC429_RAMP_DIV_MAX;
  return TMC429_getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
}

uint32_t TMC429_getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t velocity_max)
{
  uint32_t a_max_lower_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div - 1) <= 0)
  {
    a_max_lower_limit = TMC429_ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_lower_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div - 1));
    if (TMC429_convertVelocityFromHz(pulse_div, velocity_max) <= (int16_t)TMC429_VELOCITY_REGISTER_THRESHOLD)
    {
      a_max_lower_limit /= 2;
    }
  }
  if (a_max_lower_limit > TMC429_ACCELERATION_REGISTER_MAX)
  {
    a_max_lower_limit = TMC429_ACCELERATION_REGISTER_MAX;
  }
  if (a_max_lower_limit < TMC429_ACCELERATION_REGISTER_MIN)
  {
    a_max_lower_limit = TMC429_ACCELERATION_REGISTER_MIN;
  }
  return TMC429_convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_lower_limit);
}

uint32_t TMC429_convertAccelerationToHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration)
{
  double a = ((double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ)/(double)TMC429_ACCELERATION_CONSTANT;
  double b = a*(double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div));
  double d = c/((double)(1 << ramp_div));
  uint32_t e = round(d*(double)acceleration);
  return e;
}

uint32_t TMC429_setAccelerationMaxInHzPerS(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint32_t acceleration_max_upper_limit = TMC429_getAccelerationMaxUpperLimitInHzPerS(TMC429_pulse_div_[motor], TMC429_ramp_div_[motor]);
  uint32_t acceleration_max_lower_limit = TMC429_getAccelerationMaxLowerLimitInHzPerS(TMC429_pulse_div_[motor], TMC429_ramp_div_[motor], velocity_max_hz);
  if (acceleration_max_hz_per_s > acceleration_max_upper_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_upper_limit;
  }
  if (acceleration_max_hz_per_s < acceleration_max_lower_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_lower_limit;
  }
  uint32_t acceleration_max = TMC429_convertAccelerationFromHzPerS(TMC429_pulse_div_[motor], TMC429_ramp_div_[motor], acceleration_max_hz_per_s);
  if (acceleration_max > TMC429_ACCELERATION_REGISTER_MAX)
  {
    acceleration_max = TMC429_ACCELERATION_REGISTER_MAX;
  }
  if (acceleration_max < TMC429_ACCELERATION_REGISTER_MIN)
  {
    acceleration_max = TMC429_ACCELERATION_REGISTER_MIN;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_A_MAX, acceleration_max);
  return acceleration_max;
}

uint32_t TMC429_convertAccelerationFromHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration)
{
  double a = ((double)acceleration*(double)(1 << pulse_div))/((double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ);
  double b = a*(double)TMC429_ACCELERATION_CONSTANT;
  double c = b/((double)TMC429_clock_frequency_*(double)TMC429_MHZ_PER_HZ);
  uint32_t d = round(c*(1 << ramp_div));
  return d;
}

void TMC429_setOptimalPropFactor(size_t motor, uint32_t acceleration_max)
{
  int pdiv, pmul, pm, pd ;
  double p_ideal, p_reduced;

  pm=-1; pd=-1; // -1 indicates : no valid pair found
  p_ideal = acceleration_max/(128.0*(1 << (TMC429_ramp_div_[motor] - TMC429_pulse_div_[motor])));
  p_reduced = p_ideal*0.99;
  for (pdiv=0; pdiv<=13; ++pdiv)
  {
    pmul = (int)(p_reduced*8.0*(1 << pdiv)) - 128;
    if ((0 <= pmul) && (pmul <= 127))
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }
  if ((pm == -1) || (pd == -1))
  {
    return;
  }
  TMC429_PropFactor prop_factor;
  prop_factor.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_PROP_FACTOR);
  prop_factor.pmul = pm;
  prop_factor.pdiv = pd;
  TMC429_writeRegister(motor, TMC429_ADDRESS_PROP_FACTOR, prop_factor.bytes);
}

void TMC429_setTargetVelocityInHz(size_t motor, int32_t velocity_hz)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_setTargetVelocity(motor, TMC429_convertVelocityFromHz(TMC429_pulse_div_[motor], velocity_hz));
}

int32_t TMC429_getActualVelocityInHz(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertVelocityToHz(TMC429_pulse_div_[motor], TMC429_getActualVelocity(motor));
}

int16_t TMC429_getActualVelocity(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_V_ACTUAL);
  return TMC429_unsignedToSigned(velocity_unsigned, TMC429_V_BIT_COUNT);
}

int32_t TMC429_unsignedToSigned(uint32_t input_value, uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

bool TMC429_atTargetVelocity(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return true;
  }
  int16_t actual_velocity = TMC429_getActualVelocity(motor);
  int16_t target_velocity = TMC429_getTargetVelocity(motor);
  return (actual_velocity == target_velocity);
}

int32_t TMC429_getTargetVelocityInHz(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertVelocityToHz(TMC429_pulse_div_[motor], TMC429_getTargetVelocity(motor));
}

int16_t TMC429_getTargetVelocity(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_V_TARGET);
  return TMC429_unsignedToSigned(velocity_unsigned, TMC429_V_BIT_COUNT);
}

bool TMC429_communicating()
{
  return (TMC429_getVersion() == TMC429_VERSION);
}

uint32_t TMC429_getVersion()
{
  return TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_TYPE_VERSION_429);
}

void TMC429_setRampMode(size_t motor)
{
	TMC429_setMode(motor, RAMP_MODE);
}

void TMC429_setSoftMode(size_t motor)
{
	TMC429_setMode(motor, SOFT_MODE);
}

void TMC429_setHoldMode(size_t motor)
{
	TMC429_setMode(motor, HOLD_MODE);
}

uint32_t TMC429_getVelocityMinInHz(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertVelocityToHz(TMC429_pulse_div_[motor], TMC429_getVelocityMin(motor));
}

uint32_t TMC429_getVelocityMaxInHz(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertVelocityToHz(TMC429_pulse_div_[motor], TMC429_getVelocityMax(motor));
}

uint16_t TMC429_getVelocityMin(size_t motor)
{
  return TMC429_readRegister(motor, TMC429_ADDRESS_V_MIN);
}

uint16_t TMC429_getVelocityMax(size_t motor)
{
  return TMC429_readRegister(motor, TMC429_ADDRESS_V_MAX);
}



void TMC429_setHoldVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }

  TMC429_setOptimalStepDivHz(velocity_max_hz);

  TMC429_setOptimalPulseDivHz(motor, velocity_max_hz);

  TMC429_setVelocityMaxInHz(motor, velocity_max_hz);
}

void TMC429_setHoldVelocityInHz(size_t motor, int32_t velocity_hz)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_setHoldVelocity(motor, TMC429_convertVelocityFromHz(TMC429_pulse_div_[motor], velocity_hz));
}

void TMC429_setHoldVelocity(size_t motor, int16_t velocity)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_V_ACTUAL, velocity);
}

uint32_t TMC429_getAccelerationMaxInHzPerS(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertAccelerationToHzPerS(TMC429_pulse_div_[motor], TMC429_ramp_div_[motor], TMC429_getAccelerationMax(motor));
}

uint32_t TMC429_getActualAccelerationInHzPerS(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  return TMC429_convertAccelerationToHzPerS(TMC429_pulse_div_[motor], TMC429_ramp_div_[motor], TMC429_getActualAcceleration(motor));
}

int32_t TMC429_getTargetPosition(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_X_TARGET);
  return TMC429_unsignedToSigned(position_unsigned, TMC429_X_BIT_COUNT);
}

void TMC429_setTargetPosition(size_t motor, int32_t position)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_X_TARGET, position);
}

bool TMC429_atTargetPosition(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return true;
  }
  int32_t actual_position = TMC429_getActualPosition(motor);
  int32_t target_position = TMC429_getTargetPosition(motor);
  return (actual_position == target_position);
}

int32_t TMC429_getActualPosition(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_X_ACTUAL);
  return TMC429_unsignedToSigned(position_unsigned, TMC429_X_BIT_COUNT);
}

void TMC429_setActualPosition(size_t motor, int32_t position)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_X_ACTUAL, position);
}

void TMC429_enableInverseStepPolarity()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 1;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_disableInverseStepPolarity()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 0;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_enableInverseDirPolarity()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 1;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_disableInverseDirPolarity()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 0;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_setSwitchesActiveLow()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 1;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_setSwitchesActiveHigh()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 0;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429_enableLeftSwitchStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 0;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429_leftSwitchStopEnabled(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return false;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_l;
}

bool TMC429_leftSwitchActive(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return false;
  }
  TMC429_SwitchState switch_state = TMC429_getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.l0;
      break;
    }
    case 1:
    {
      return switch_state.l1;
      break;
    }
    case 2:
    {
      return switch_state.l2;
      break;
    }
  }
  return false;
}

void TMC429_enableRightSwitches()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 1;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}



bool TMC429_rightSwitchesEnabled()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf.en_refr;
}

void TMC429_enableRightSwitchStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 0;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_disableRightSwitchStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 1;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429_rightSwitchStopEnabled(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return false;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_r;
}

bool TMC429_rightSwitchActive(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return false;
  }
  TMC429_SwitchState switch_state = TMC429_getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.r0;
      break;
    }
    case 1:
    {
      return switch_state.r1;
      break;
    }
    case 2:
    {
      return switch_state.r2;
      break;
    }
  }
  return false;
}

void TMC429_enableSwitchSoftStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 1;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_disableSwitchSoftStop(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 0;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429_switchSoftStopEnabled(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return false;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  return ref_conf_mode.ref_conf.soft_stop;
}

void TMC429_setReferenceSwitchToLeft(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 0;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_setReferenceSwitchToRight(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 1;
  TMC429_writeRegister(motor, TMC429_ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429_startLatchPositionWaiting(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_writeRegister(motor, TMC429_ADDRESS_X_LATCHED, 0);
}

bool TMC429_latchPositionWaiting(size_t motor)
{
  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.lp = false;
  if (motor < TMC429_MOTOR_COUNT)
  {
    ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.lp;
}

int32_t TMC429_getLatchPosition(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_X_LATCHED);
  return TMC429_unsignedToSigned(position_unsigned, TMC429_X_BIT_COUNT);
}

void TMC429_setPositionCompareMotor(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return;
  }
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.pos_comp_sel = motor;
  TMC429_writeRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

TMC429_Status TMC429_getStatus()
{
  TMC429_getVersion();
  return TMC429_status_;
}


TMC429_Mode TMC429_getMode(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return RAMP_MODE;
  }

  TMC429_RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  switch (ref_conf_mode.mode)
  {
    case RAMP_MODE:
      return RAMP_MODE;
      break;
    case SOFT_MODE:
      return SOFT_MODE;
      break;
    case VELOCITY_MODE:
      return VELOCITY_MODE;
      break;
    case HOLD_MODE:
      return HOLD_MODE;
      break;
  }
  return RAMP_MODE;
}



TMC429_ReferenceConfiguration TMC429_getReferenceConfiguration(size_t motor)
{
  TMC429_RefConfMode ref_conf_mode;
  if (motor < TMC429_MOTOR_COUNT)
  {
    ref_conf_mode.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.ref_conf;
}

TMC429_InterfaceConfiguration TMC429_getInterfaceConfiguration()
{
  TMC429_IfConf if_conf;
  if_conf.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf;
}

TMC429_SwitchState TMC429_getSwitchState()
{
  TMC429_SwState switch_state;
  switch_state.bytes = TMC429_readRegister(TMC429_SMDA_COMMON, TMC429_ADDRESS_SWITCHES);
  return switch_state.switch_state;
}

TMC429_ClockConfiguration TMC429_getClockConfiguration(size_t motor)
{
  TMC429_ClkConfig clk_config;
  if (motor < TMC429_MOTOR_COUNT)
  {
    clk_config.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.clk_config;
}

double TMC429_getProportionalityFactor(size_t motor)
{
  if (motor >= TMC429_MOTOR_COUNT)
  {
    return 0.0;
  }
  TMC429_PropFactor prop_factor;
  prop_factor.bytes = TMC429_readRegister(motor, TMC429_ADDRESS_PROP_FACTOR);
  int pm = prop_factor.pmul;
  int pd = prop_factor.pdiv;
  return ((double)(pm)) / ((double)(1 << (pd + 3)));
}

double TMC429_getStepTimeInMicroS()
{
  uint8_t step_div = TMC429_getStepDiv();
  return TMC429_stepDivToStepTime(step_div);
}


uint32_t TMC429_getAccelerationMax(size_t motor)
{
  return TMC429_readRegister(motor, TMC429_ADDRESS_A_MAX);
}


int16_t TMC429_getActualAcceleration(size_t motor)
{
  uint32_t acceleration_unsigned = TMC429_readRegister(motor, TMC429_ADDRESS_A_ACTUAL);
  return TMC429_unsignedToSigned(acceleration_unsigned, TMC429_A_BIT_COUNT);
}
