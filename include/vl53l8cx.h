#ifndef __VL53L8CX_H
#define __VL53L8CX_H


#include "libs/base/i2c.h"

#include "libs/base/gpio.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "platform.h"

#include "vl53l8cx_api.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_xtalk.h"

class VL53L8CX {
  public:
    VL53L8CX(coralmicro::I2c bus, int lpn_pin, int i2c_rst_pin = -1);
    virtual ~VL53L8CX(void);
    virtual int begin(void);
    virtual int end(void);
    virtual void on(void);
    virtual void off(void);
    virtual void i2c_reset(void);
    uint8_t is_alive(uint8_t *p_is_alive);
    uint8_t init(void);
    uint8_t set_i2c_address(uint16_t i2c_address);
    uint8_t get_power_mode(uint8_t *p_power_mode);
    uint8_t set_power_mode(uint8_t power_mode);
    uint8_t start_ranging(void);
    uint8_t stop_ranging(void);
    uint8_t check_data_ready(uint8_t *p_isReady);
    uint8_t get_ranging_data(VL53L8CX_ResultsData *p_results);
    uint8_t get_resolution(uint8_t *p_resolution);
    uint8_t set_resolution(uint8_t resolution);
    uint8_t get_ranging_frequency_hz(uint8_t *p_frequency_hz);
    uint8_t set_ranging_frequency_hz(uint8_t frequency_hz);
    uint8_t get_integration_time_ms(uint32_t *p_time_ms);
    uint8_t set_integration_time_ms(uint32_t integration_time_ms);
    uint8_t get_sharpener_percent(uint8_t *p_sharpener_percent);
    uint8_t set_sharpener_percent(uint8_t sharpener_percent);
    uint8_t get_target_order(uint8_t *p_target_order);
    uint8_t set_target_order(uint8_t target_order);
    uint8_t get_ranging_mode(uint8_t *p_ranging_mode);
    uint8_t set_ranging_mode(uint8_t ranging_mode);
    uint8_t get_external_sync_pin_enable(uint8_t *p_is_sync_pin_enabled);
    uint8_t set_external_sync_pin_enable(uint8_t enable_sync_pin);
    uint8_t get_VHV_repeat_count(uint32_t *p_repeat_count);
    uint8_t set_VHV_repeat_count(uint32_t repeat_count);
    uint8_t dci_read_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_replace_data(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);
    uint8_t get_detection_thresholds_enable(uint8_t *p_enabled);
    uint8_t set_detection_thresholds_enable(uint8_t enabled);
    uint8_t get_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds);
    uint8_t set_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds);
    uint8_t get_detection_thresholds_auto_stop(uint8_t *p_auto_stop);
    uint8_t set_detection_thresholds_auto_stop(uint8_t auto_stop);
    uint8_t motion_indicator_init(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t motion_indicator_set_distance_motion(VL53L8CX_Motion_Configuration  *p_motion_config, uint16_t distance_min_mm, uint16_t distance_max_mm);
    uint8_t motion_indicator_set_resolution(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t calibrate_xtalk(uint16_t reflectance_percent, uint8_t nb_samples, uint16_t distance_mm);
    uint8_t get_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t set_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t get_xtalk_margin(uint32_t *p_xtalk_margin);
    uint8_t set_xtalk_margin(uint32_t xtalk_margin);

    uint8_t IO_Write(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
    uint8_t IO_Read(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
    uint8_t IO_Wait(uint32_t ms);

    static uint8_t StaticIO_Write(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
    static uint8_t StaticIO_Read(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
    static uint8_t StaticIO_Wait(void *handle, uint32_t ms);


 private:
  coralmicro::I2cConfig i2c_config;
  int lpn_pin;
  int i2c_rst_pin;
  VL53L8CX_Configuration _dev;
  VL53L8CX_Configuration *p_dev;


};

#endif /* __VL53L8CX_H */
