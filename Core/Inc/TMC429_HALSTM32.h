// ----------------------------------------------------------------------------
// TMC429_HALSTM32.h
//
// Porting to STM32 (HAL) lib from Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef TMC429_HAL_C_H
#define TMC429_HAL_C_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Constants (SPI/MOTORS)
#define TMC429_SPI_CLOCK 1125000;
//const static uint8_t TMC429_SPI_BIT_ORDER = MSBFIRST;
//const static uint8_t TMC429_SPI_MODE = SPI_MODE3;

#define TMC429_MOTOR_COUNT 3
#define TMC429_VERSION 0x429101
#define TMC429_CLOCK_FREQUENCY_MAX 32
#define TMC429_STEP_DIV_MAX 15
#define TMC429_MHZ_PER_HZ 1000000
#define TMC429_VELOCITY_CONSTANT 65536
#define TMC429_VELOCITY_REGISTER_MIN 0
#define TMC429_VELOCITY_REGISTER_MAX 2047
#define TMC429_VELOCITY_REGISTER_THRESHOLD 1448
#define TMC429_ACCELERATION_REGISTER_MIN 1
#define TMC429_ACCELERATION_REGISTER_MAX 2047
#define TMC429_ACCELERATION_CONSTANT 536870912 // (1 << 29)
#define TMC429_VELOCITY_MIN_MIN 1
#define TMC429_PULSE_DIV_MIN 0
#define TMC429_PULSE_DIV_MAX 13
#define TMC429_RAMP_DIV_MIN 0
#define TMC429_RAMP_DIV_MAX 13

// Datagrams
#define TMC429_DATAGRAM_SIZE 4

#define TMC429_RW_READ 1
#define TMC429_RW_WRITE 0

// IDX Addresses
#define TMC429_ADDRESS_X_TARGET 0b0000
#define TMC429_ADDRESS_X_ACTUAL 0b0001
#define TMC429_ADDRESS_V_MIN 0b0010
#define TMC429_ADDRESS_V_MAX 0b0011
#define TMC429_ADDRESS_V_TARGET 0b0100
#define TMC429_ADDRESS_V_ACTUAL 0b0101
#define TMC429_ADDRESS_A_MAX 0b0110
#define TMC429_ADDRESS_A_ACTUAL 0b0111
#define TMC429_ADDRESS_A_THRESHOLD 0b1000
#define TMC429_ADDRESS_PROP_FACTOR 0b1001
#define TMC429_ADDRESS_REF_CONF_MODE 0b1010
#define TMC429_ADDRESS_INTERRUPT 0b1011
#define TMC429_ADDRESS_CLOCK_CONFIGURATION 0b1100
#define TMC429_ADDRESS_DX_REF_TOLERANCE 0b1101
#define TMC429_ADDRESS_X_LATCHED 0b1110
#define TMC429_ADDRESS_USTEP_COUNT_429 0b1111

// JDX Addresses
#define TMC429_ADDRESS_DATAGRAM_LOW_WORD 0b0000
#define TMC429_ADDRESS_DATAGRAM_HIGH_WORD 0b0001
#define TMC429_ADDRESS_COVER_POS_LEN 0b0010
#define TMC429_ADDRESS_COVER_DATAGRAM 0b0011
#define TMC429_ADDRESS_IF_CONFIGURATION_429 0b0100
#define TMC429_ADDRESS_POS_COMP_429 0b0101
#define TMC429_ADDRESS_POS_COMP_INT 0b0110
#define TMC429_ADDRESS_POWER_DOWN 0b1000
#define TMC429_ADDRESS_TYPE_VERSION_429 0b1001
#define TMC429_ADDRESS_SWITCHES 0b1110
#define TMC429_ADDRESS_GLOBAL_PARAMETERS 0b1111

#define TMC429_SMDA_COMMON 0b11

#define TMC429_RRS_REGISTER 0
#define TMC429_RRS_RAM 1

// Masks
#define  TMC429_STEP_DIV_MASK 0xf

// Bit Count
#define  TMC429_X_BIT_COUNT 24
#define  TMC429_V_BIT_COUNT 12
#define  TMC429_A_BIT_COUNT 12



typedef struct
{
    uint8_t at_target_position_0 : 1;
    uint8_t switch_left_0 : 1;
    uint8_t at_target_position_1 : 1;
    uint8_t switch_left_1 : 1;
    uint8_t at_target_position_2 : 1;
    uint8_t switch_left_2 : 1;
    uint8_t cover_datagram_waiting : 1;
    uint8_t interrupt : 1;
} TMC429_Status;

typedef enum
{
    RAMP_MODE = 0b00,
    SOFT_MODE = 0b01,
    VELOCITY_MODE = 0b10,
    HOLD_MODE = 0b11,
}TMC429_Mode;

typedef struct
{
    uint8_t disable_stop_l : 1;
    uint8_t disable_stop_r : 1;
    uint8_t soft_stop : 1;
    uint8_t ref_rnl : 1;
    uint8_t space : 4;
}TMC429_ReferenceConfiguration;

typedef struct
{
    uint16_t inv_ref : 1;
    uint16_t sdo_int : 1;
    uint16_t step_half : 1;
    uint16_t inv_stp : 1;
    uint16_t inv_dir : 1;
    uint16_t en_sd : 1;
    uint16_t pos_comp_sel : 2;
    uint16_t en_refr : 1;
    uint16_t space : 7;
}TMC429_InterfaceConfiguration;

typedef struct
{
    uint8_t r0 : 1;
    uint8_t l0 : 1;
    uint8_t r1 : 1;
    uint8_t l1 : 1;
    uint8_t r2 : 1;
    uint8_t l2 : 1;
    uint8_t space : 2;
}TMC429_SwitchState;

typedef struct
{
    uint16_t usrs : 3;
    uint16_t space0 : 5;
    uint16_t ramp_div : 4;
    uint16_t pulse_div : 4;
}TMC429_ClockConfiguration;

// MOSI Datagrams
typedef union
{
    struct
    {
        uint32_t data : 24;
        uint32_t rw : 1;
        uint32_t address : 4;
        uint32_t smda : 2;
        uint32_t rrs : 1;
    };
    uint32_t bytes;
}TMC429_MosiDatagram;


// MISO Datagrams
typedef union
{
    struct
    {
        uint32_t data : 24;
        TMC429_Status status;
    };
    uint32_t bytes;
}TMC429_MisoDatagram;


// Union Structs
typedef union
{
    struct
    {
        uint32_t pdiv : 4;
        uint32_t space0 : 4;
        uint32_t pmul : 8;
        uint32_t space1 : 8;
        uint32_t space2 : 8;
    };
    uint32_t bytes;
}TMC429_PropFactor;

typedef union
{
    struct
    {
        uint32_t mode : 2;
        uint32_t space0 : 6;
        TMC429_ReferenceConfiguration ref_conf;
        uint32_t lp : 1;
        uint32_t space1 : 7;
        uint32_t space2 : 8;
    };
    uint32_t bytes;
}TMC429_RefConfMode;

typedef union
{
    struct
    {
        TMC429_InterfaceConfiguration if_conf;
        uint32_t space0 : 16;
    };
    uint32_t bytes;
}TMC429_IfConf;

typedef union
{
    struct
    {
        TMC429_SwitchState switch_state;
        uint32_t space0 : 16;
    };
    uint32_t bytes;
}TMC429_SwState;

typedef union
{
    struct
    {
        uint32_t lsmd : 2;
        uint32_t nscs_s : 1;
        uint32_t sck_s : 1;
        uint32_t ph_ab : 1;
        uint32_t fd_ab : 1;
        uint32_t dac_ab : 1;
        uint32_t cs_com_ind : 1;
        uint32_t clk2_div : 8;
        uint32_t cont_update : 1;
        uint32_t space0 : 3;
        uint32_t ref_mux : 1;
        uint32_t mot1r : 1;
        uint32_t space1 : 2;
    };
    uint32_t bytes;
}TMC429_GlobalParameters;

typedef union
{
    struct
    {
        TMC429_ClockConfiguration clk_config;
        uint32_t space0 : 16;
    };
    uint32_t bytes;
}TMC429_ClkConfig;



// Function declarations for TMC429 driver
void TMC429_setup(SPI_HandleTypeDef* SPI_Bus, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t clock_frequency_mhz);
void TMC429_specifyClockFrequencyInMHz(uint8_t clock_frequency);
void TMC429_setStepDiv(uint8_t step_div);
uint8_t TMC429_getStepDiv();
double TMC429_stepDivToStepTime(uint8_t step_div);
uint32_t TMC429_readRegister(uint8_t smda, uint8_t address);
void TMC429_writeRegister(uint8_t smda, uint8_t address, uint32_t data);
TMC429_MisoDatagram TMC429_writeRead(TMC429_MosiDatagram mosi_datagram);
void TMC429_stopAll();
void TMC429_stop(size_t motor);
void TMC429_setVelocityMode(size_t motor);
void TMC429_setMode(size_t motor, TMC429_Mode mode);
void TMC429_setTargetVelocity(size_t motor, int16_t velocity);
void TMC429_initialize();
void TMC429_setStepDirOutput();
void TMC429_disableLeftSwitchStop(size_t motor);
void TMC429_disableRightSwitches();
void TMC429_setLimitsInHz(size_t motor, uint32_t velocity_min_hz, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s);
void TMC429_setOptimalStepDivHz(uint32_t velocity_max_hz);
void TMC429_setOptimalPulseDivHz(size_t motor, uint32_t velocity_max_hz);
uint8_t TMC429_findOptimalPulseDivHz(uint32_t velocity_max_hz);
uint32_t TMC429_getVelocityMaxUpperLimitInHz(uint8_t pulse_div);
uint32_t TMC429_getVelocityMaxUpperLimitInHz_Zero();
int32_t TMC429_convertVelocityToHz(uint8_t pulse_div, int16_t velocity);
void TMC429_setVelocityMinInHz(size_t motor, uint32_t velocity_min_hz);
void TMC429_setVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz);
int16_t TMC429_convertVelocityFromHz(uint8_t pulse_div, int32_t velocity);
void TMC429_setOptimalRampDivHz(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s);
uint8_t TMC429_findOptimalRampDivHz(uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s);
uint32_t TMC429_getAccelerationMaxUpperLimitInHzPerS_Single(uint32_t velocity_max_hz);
uint32_t TMC429_getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div);
uint32_t TMC429_getAccelerationMaxLowerLimitInHzPerS_Single(uint32_t velocity_max_hz);
uint32_t TMC429_getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t velocity_max);
uint32_t TMC429_convertAccelerationToHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration);
uint32_t TMC429_setAccelerationMaxInHzPerS(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s);
uint32_t TMC429_convertAccelerationFromHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration);
void TMC429_setOptimalPropFactor(size_t motor, uint32_t acceleration_max);
void TMC429_setTargetVelocityInHz(size_t motor, int32_t velocity_hz);
int32_t TMC429_getActualVelocityInHz(size_t motor);
int16_t TMC429_getActualVelocity(size_t motor);
int32_t TMC429_unsignedToSigned(uint32_t input_value, uint8_t num_bits);
bool TMC429_atTargetVelocity(size_t motor);
int32_t TMC429_getTargetVelocityInHz(size_t motor);
int16_t TMC429_getTargetVelocity(size_t motor);

bool TMC429_communicating();
uint32_t TMC429_getVersion();
void TMC429_setRampMode(size_t motor);
void TMC429_setSoftMode(size_t motor);
void TMC429_setHoldMode(size_t motor);


uint32_t TMC429_getVelocityMinInHz(size_t motor);
uint32_t TMC429_getVelocityMaxInHz(size_t motor);
uint16_t TMC429_getVelocityMin(size_t motor);
uint16_t TMC429_getVelocityMax(size_t motor);



void TMC429_setHoldVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz);
void TMC429_setHoldVelocityInHz(size_t motor, int32_t velocity_hz);
void TMC429_setHoldVelocity(size_t motor, int16_t velocity);
uint32_t TMC429_getAccelerationMaxInHzPerS(size_t motor);
uint32_t TMC429_getActualAccelerationInHzPerS(size_t motor);
int32_t TMC429_getTargetPosition(size_t motor);
void TMC429_setTargetPosition(size_t motor, int32_t position);
bool TMC429_atTargetPosition(size_t motor);
int32_t TMC429_getActualPosition(size_t motor);
void TMC429_setActualPosition(size_t motor, int32_t position);
void TMC429_enableInverseStepPolarity();
void TMC429_disableInverseStepPolarity();
void TMC429_enableInverseDirPolarity();
void TMC429_disableInverseDirPolarity();
void TMC429_setSwitchesActiveLow();
void TMC429_setSwitchesActiveHigh();
void TMC429_enableLeftSwitchStop(size_t motor);
bool TMC429_leftSwitchStopEnabled(size_t motor);
bool TMC429_leftSwitchActive(size_t motor);
void TMC429_enableRightSwitches();
bool TMC429_rightSwitchesEnabled();
void TMC429_disableRightSwitchStop(size_t motor);
bool TMC429_rightSwitchStopEnabled(size_t motor);
bool TMC429_rightSwitchActive(size_t motor);
void TMC429_enableSwitchSoftStop(size_t motor);
void TMC429_disableSwitchSoftStop(size_t motor);
bool TMC429_switchSoftStopEnabled(size_t motor);
void TMC429_setReferenceSwitchToLeft(size_t motor);
void TMC429_setReferenceSwitchToRight(size_t motor);
void TMC429_startLatchPositionWaiting(size_t motor);
bool TMC429_latchPositionWaiting(size_t motor);
int32_t TMC429_getLatchPosition(size_t motor);
void TMC429_setPositionCompareMotor(size_t motor);
TMC429_Status TMC429_getStatus();
TMC429_Mode TMC429_getMode(size_t motor);
TMC429_ReferenceConfiguration TMC429_getReferenceConfiguration(size_t motor);
TMC429_InterfaceConfiguration TMC429_getInterfaceConfiguration();
TMC429_SwitchState TMC429_getSwitchState();
TMC429_ClockConfiguration TMC429_getClockConfiguration(size_t motor);
double TMC429_getProportionalityFactor(size_t motor);
double TMC429_getStepTimeInMicroS();
uint32_t TMC429_getAccelerationMax(size_t motor);
int16_t TMC429_getActualAcceleration(size_t motor);


#endif // TMC429_HALSTM32_H
