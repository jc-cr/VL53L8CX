#include "vl53l8cx.h"

#define HIGH 1
#define LOW 0


VL53L8CX::VL53L8CX(coralmicro::I2c bus, int _lpn_pin, int _i2c_rst_pin)
    : lpn_pin(_lpn_pin), i2c_rst_pin(_i2c_rst_pin) {
  i2c_config = coralmicro::I2cGetDefaultConfig(bus);
  coralmicro::I2cInitController(i2c_config);

  memset((void *)&_dev, 0x0, sizeof(VL53L8CX_Configuration));
  _dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
  _dev.platform.Write = &VL53L8CX::StaticIO_Write;
  _dev.platform.Read = &VL53L8CX::StaticIO_Read;
  _dev.platform.Wait = &VL53L8CX::StaticIO_Wait;
  _dev.platform.handle = (void *)this;
  p_dev = &_dev;
}


VL53L8CX::~VL53L8CX() {}

int VL53L8CX::begin()
{
    if (lpn_pin >= 0) {
      coralmicro::GpioSetMode(static_cast<coralmicro::Gpio>(lpn_pin), coralmicro::GpioMode::kOutput);
      coralmicro::GpioSet(static_cast<coralmicro::Gpio>(lpn_pin), false);
      vTaskDelay(pdMS_TO_TICKS(10));
      coralmicro::GpioSet(static_cast<coralmicro::Gpio>(lpn_pin), true);
    }
    if (i2c_rst_pin >= 0) {
      coralmicro::GpioSetMode(static_cast<coralmicro::Gpio>(i2c_rst_pin), coralmicro::GpioMode::kOutput);
      coralmicro::GpioSet(static_cast<coralmicro::Gpio>(i2c_rst_pin), false);
    }
    return 0;
}

int VL53L8CX::end()
{
  if (lpn_pin >= 0) {
    coralmicro::GpioSetMode(static_cast<coralmicro::Gpio>(lpn_pin), coralmicro::GpioMode::kInput);
  }
  if (i2c_rst_pin >= 0) {
    coralmicro::GpioSetMode(static_cast<coralmicro::Gpio>(i2c_rst_pin), coralmicro::GpioMode::kInput);
  }
  return 0;
}

void VL53L8CX::on(void)
{
  if (lpn_pin >= 0) {
    coralmicro::GpioSet(static_cast<coralmicro::Gpio>(lpn_pin), true);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

void VL53L8CX::off(void)
{
  if (lpn_pin >= 0) {
    coralmicro::GpioSet(static_cast<coralmicro::Gpio>(lpn_pin), false);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

void VL53L8CX::i2c_reset(void)
{
  if (i2c_rst_pin >= 0) {
    coralmicro::GpioSet(static_cast<coralmicro::Gpio>(i2c_rst_pin), false);
    vTaskDelay(pdMS_TO_TICKS(10));
    coralmicro::GpioSet(static_cast<coralmicro::Gpio>(i2c_rst_pin), true);
    vTaskDelay(pdMS_TO_TICKS(10));
    coralmicro::GpioSet(static_cast<coralmicro::Gpio>(i2c_rst_pin), false);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


uint8_t VL53L8CX::is_alive(uint8_t  *p_is_alive)
{
  return vl53l8cx_is_alive(p_dev, p_is_alive);
}

/**
  * @brief Initialize the VL53L8CX sensor.
  * @return Status 0 if initialization is OK.
  */
uint8_t VL53L8CX::init(void)
{
  return vl53l8cx_init(p_dev);
}

/**
  * @brief Set the I2C address of the VL53L8CX sensor.
  * @param i2c_address The new I2C address to be set for the sensor.
  * @return Status 0 if the address is set correctly.
  */
uint8_t VL53L8CX::set_i2c_address(uint16_t i2c_address)
{
  return vl53l8cx_set_i2c_address(p_dev, i2c_address);
}

/**
  * @brief Get the current power mode of the VL53L8CX sensor.
  * @param p_power_mode Pointer to the variable that will be set to the current power mode.
  * @return Status 0 if the power mode is retrieved successfully.
  */
uint8_t VL53L8CX::get_power_mode(uint8_t *p_power_mode)
{
  return vl53l8cx_get_power_mode(p_dev, p_power_mode);
}

/**
  * @brief Set the power mode of the VL53L8CX sensor.
  * @param power_mode The power mode to be set(e.g., sleep or wakeup).
  * @return Status 0 if the power mode is set successfully, or 127 if the requested power mode is not valid.
  */
uint8_t VL53L8CX::set_power_mode(uint8_t power_mode)
{
  return vl53l8cx_set_power_mode(p_dev, power_mode);
}

/**
  * @brief Start a ranging session with the VL53L8CX sensor.
  * @return Status 0 if the ranging session starts successfully.
  */
uint8_t VL53L8CX::start_ranging(void)
{
  return vl53l8cx_start_ranging(p_dev);
}

/**
  * @brief Stop the current ranging session with the VL53L8CX sensor.
  * @return Status 0 if the ranging session stops successfully.
  */
uint8_t VL53L8CX::stop_ranging(void)
{
  return vl53l8cx_stop_ranging(p_dev);
}

/**
  * @brief Check if new ranging data is ready.
  * @param p_isReady Pointer to the variable that will be updated to indicate if new data is ready.
  * @return Status 0 if the data ready check is successful.
  */
uint8_t VL53L8CX::check_data_ready(uint8_t *p_isReady)
{
  return vl53l8cx_check_data_ready(p_dev, p_isReady);
}

/**
  * @brief Get the ranging data from the VL53L8CX sensor.
  * @param p_results Pointer to the results data structure where the ranging data will be stored.
  * @return Status 0 if the data is retrieved successfully.
  */
uint8_t VL53L8CX::get_ranging_data(VL53L8CX_ResultsData *p_results)
{
  return vl53l8cx_get_ranging_data(p_dev, p_results);
}

/**
  * @brief Get the current resolution of the VL53L8CX sensor.
  * @param p_resolution Pointer to the variable that will be set to the current resolution.
  * @return Status 0 if the resolution is retrieved successfully.
  */
uint8_t VL53L8CX::get_resolution(uint8_t *p_resolution)
{
  return vl53l8cx_get_resolution(p_dev, p_resolution);
}

/**
  * @brief Set a new resolution for the VL53L8CX sensor.
  * @param resolution The new resolution to be set.
  * @return Status 0 if the resolution is set successfully.
  */
uint8_t VL53L8CX::set_resolution(uint8_t resolution)
{
  return vl53l8cx_set_resolution(p_dev, resolution);
}

/**
  * @brief Get the current ranging frequency of the VL53L8CX sensor in Hz.
  * @param p_frequency_hz Pointer to the variable that will be set to the current ranging frequency.
  * @return Status 0 if the ranging frequency is retrieved successfully.
  */
uint8_t VL53L8CX::get_ranging_frequency_hz(uint8_t *p_frequency_hz)
{
  return vl53l8cx_get_ranging_frequency_hz(p_dev, p_frequency_hz);
}

/**
  * @brief Set a new ranging frequency for the VL53L8CX sensor in Hz.
  * @param frequency_hz The new ranging frequency to be set.
  * @return Status 0 if the ranging frequency is set successfully, or 127 if the value is not correct.
  */
uint8_t VL53L8CX::set_ranging_frequency_hz(uint8_t frequency_hz)
{
  return vl53l8cx_set_ranging_frequency_hz(p_dev, frequency_hz);
}

/**
  * @brief Get the current integration time of the VL53L8CX sensor in ms.
  * @param p_time_ms Pointer to the variable that will be set to the current integration time.
  * @return Status 0 if the integration time is retrieved successfully.
  */
uint8_t VL53L8CX::get_integration_time_ms(uint32_t *p_time_ms)
{
  return vl53l8cx_get_integration_time_ms(p_dev, p_time_ms);
}

/**
  * @brief Set a new integration time for the VL53L8CX sensor in ms.
  * @param integration_time_ms The new integration time to be set.
  * @return Status 0 if the integration time is set successfully.
  */
uint8_t VL53L8CX::set_integration_time_ms(uint32_t integration_time_ms)
{
  return vl53l8cx_set_integration_time_ms(p_dev, integration_time_ms);
}

/**
  * @brief Get the current sharpener percentage of the VL53L8CX sensor.
  * @param p_sharpener_percent Pointer to the variable that will be set to the current sharpener percentage.
  * @return Status 0 if the sharpener percentage is retrieved successfully.
  */
uint8_t VL53L8CX::get_sharpener_percent(uint8_t *p_sharpener_percent)
{
  return vl53l8cx_get_sharpener_percent(p_dev, p_sharpener_percent);
}

/**
  * @brief Set a new sharpener percentage for the VL53L8CX sensor.
  * @param sharpener_percent The new sharpener percentage to be set.
  * @return Status 0 if the sharpener percentage is set successfully.
  */
uint8_t VL53L8CX::set_sharpener_percent(uint8_t sharpener_percent)
{
  return vl53l8cx_set_sharpener_percent(p_dev, sharpener_percent);
}
/**
  * @brief Get the current target order of the VL53L8CX sensor(closest or strongest).
  * @param p_target_order Pointer to the variable that will be set to the current target order.
  * @return Status 0 if the target order is retrieved successfully.
  */
uint8_t VL53L8CX::get_target_order(uint8_t *p_target_order)
{
  return vl53l8cx_get_target_order(p_dev, p_target_order);
}

/**
  * @brief Set a new target order for the VL53L8CX sensor.
  * @param target_order The new target order to be set.
  * @return Status 0 if the target order is set successfully, or 127 if the target order is unknown.
  */
uint8_t VL53L8CX::set_target_order(uint8_t target_order)
{
  return vl53l8cx_set_target_order(p_dev, target_order);
}

/**
  * @brief Get the current ranging mode of the VL53L8CX sensor.
  * @param p_ranging_mode Pointer to the variable that will be set to the current ranging mode.
  * @return Status 0 if the ranging mode is retrieved successfully.
  */
uint8_t VL53L8CX::get_ranging_mode(uint8_t *p_ranging_mode)
{
  return vl53l8cx_get_ranging_mode(p_dev, p_ranging_mode);
}

/**
  * @brief Set the ranging mode of the VL53L8CX sensor.
  * @param ranging_mode The new ranging mode to be set.
  * @return Status 0 if the ranging mode is set successfully.
  */
uint8_t VL53L8CX::set_ranging_mode(uint8_t ranging_mode)
{
  return vl53l8cx_set_ranging_mode(p_dev, ranging_mode);
}

/**
  * @brief Check if the synchronization pin of the VL53L8CX sensor is enabled.
  * @param p_is_sync_pin_enabled Pointer to the variable that will be updated to indicate if the sync pin is enabled.
  * @return Status 0 if the sync pin status is retrieved successfully.
  */
uint8_t VL53L8CX::get_external_sync_pin_enable(uint8_t *p_is_sync_pin_enabled)
{
  return vl53l8cx_get_external_sync_pin_enable(p_dev, p_is_sync_pin_enabled);
}

/**
  * @brief Enable or disable the synchronization pin of the VL53L8CX sensor.
  * @param enable_sync_pin Set to 1 to enable the sync pin, or 0 to disable it.
  * @return Status 0 if the sync pin is set successfully.
  */
uint8_t VL53L8CX::set_external_sync_pin_enable(uint8_t enable_sync_pin)
{
  return vl53l8cx_set_external_sync_pin_enable(p_dev, enable_sync_pin);
}

/**
  * @brief Get the number of frames between two temperature compensations for the VL53L8CX sensor.
  * @param p_repeat_count Pointer to the variable that will be set to the number of frames before the next temperature compensation.
  * @return Status 0 if the repeat count is retrieved successfully.
  */
uint8_t VL53L8CX::get_VHV_repeat_count(uint32_t *p_repeat_count)
{
  return vl53l8cx_get_VHV_repeat_count(p_dev, p_repeat_count);
}


/**
  * @brief Set the number of frames between two temperature compensations for the VL53L8CX sensor.
  * @param repeat_count The number of frames between temperature compensations to be set.
  * @return Status 0 if the repeat count is set successfully.
  */
uint8_t VL53L8CX::set_VHV_repeat_count(uint32_t repeat_count)
{
  return vl53l8cx_set_VHV_repeat_count(p_dev, repeat_count);
}

/**
  * @brief Read 'extra data' from the VL53L8CX sensor using DCI(Device Configuration Interface).
  * @param data Pointer to the data array or casted structure where the read data will be stored.
  * @param index Index of the required value to be read.
  * @param data_size Size of the data array or casted structure(use sizeof() function).
  * @return Status 0 if the read operation is successful.
  */
uint8_t VL53L8CX::dci_read_data(uint8_t *data, uint32_t index, uint16_t data_size)
{
  return vl53l8cx_dci_read_data(p_dev, data, index, data_size);
}

/**
  * @brief Write 'extra data' to the VL53L8CX sensor using DCI(Device Configuration Interface).
  * @param data Pointer to the data array or casted structure containing the data to be written.
  * @param index Index of the required value to be written.
  * @param data_size Size of the data array or casted structure(use sizeof() function).
  * @return Status 0 if the write operation is successful.
  */
uint8_t VL53L8CX::dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size)
{
  return vl53l8cx_dci_write_data(p_dev, data, index, data_size);
}

/**
  * @brief Replace 'extra data' in the VL53L8CX sensor using DCI(Device Configuration Interface).
  * @param data Pointer to the data array or casted structure where the current data is stored.
  * @param index Index of the required value to be replaced.
  * @param data_size Size of the data array or casted structure(use sizeof() function).
  * @param new_data Pointer to the new data array containing the fields to be replaced.
  * @param new_data_size Size of the new data array.
  * @param new_data_pos Position of the new data in the buffer.
  * @return Status 0 if the replace operation is successful.
  */
uint8_t VL53L8CX::dci_replace_data(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos)
{
  return vl53l8cx_dci_replace_data(p_dev,
                                   data,
                                   index,
                                   data_size,
                                   new_data,
                                   new_data_size,
                                   new_data_pos);
}


/**
  * @brief Check if the detection thresholds are enabled on the VL53L8CX sensor.
  * @param p_enabled Pointer to the variable that will be set to 1 if thresholds are enabled, or 0 if disabled.
  * @return Status 0 if the check is successful.
  */
uint8_t VL53L8CX::get_detection_thresholds_enable(uint8_t *p_enabled)
{
  return vl53l8cx_get_detection_thresholds_enable(p_dev, p_enabled);
}

/**
  * @brief Enable or disable the detection thresholds on the VL53L8CX sensor.
  * @param enabled Set to 1 to enable thresholds, or 0 to disable them.
  * @return Status 0 if the thresholds are set successfully.
  */
uint8_t VL53L8CX::set_detection_thresholds_enable(uint8_t enabled)
{
  return vl53l8cx_set_detection_thresholds_enable(p_dev, enabled);
}

/**
  * @brief Get the detection thresholds from the VL53L8CX sensor.
  * @param p_thresholds Pointer to the array where the detection thresholds will be stored.
  * @return Status 0 if the thresholds are retrieved successfully.
  */
uint8_t VL53L8CX::get_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds)
{
  return vl53l8cx_get_detection_thresholds(p_dev, p_thresholds);
}

/**
  * @brief Set the detection thresholds on the VL53L8CX sensor.
  * @param p_thresholds Pointer to the array containing the new detection thresholds.
  * @return Status 0 if the thresholds are programmed successfully.
  */
uint8_t VL53L8CX::set_detection_thresholds(VL53L8CX_DetectionThresholds *p_thresholds)
{
  return vl53l8cx_set_detection_thresholds(p_dev, p_thresholds);
}

/**
  * @brief Get the status of the auto-stop feature on the VL53L8CX sensor when using detection thresholds.
  * @param p_auto_stop Pointer to the variable that will be set to 1 if auto-stop is enabled, or 0 if disabled.
  * @return Status 0 if the auto-stop status is retrieved successfully.
  */
uint8_t VL53L8CX::get_detection_thresholds_auto_stop(uint8_t *p_auto_stop)
{
  return vl53l8cx_get_detection_thresholds_auto_stop(p_dev, p_auto_stop);
}

/**
  * @brief Enable or disable the auto-stop feature on the VL53L8CX sensor when using detection thresholds.
  * @param auto_stop Set to 1 to enable auto-stop, or 0 to disable it.
  * @return Status 0 if the auto-stop feature is set successfully.
  */
uint8_t VL53L8CX::set_detection_thresholds_auto_stop(uint8_t auto_stop)
{
  return vl53l8cx_set_detection_thresholds_auto_stop(p_dev, auto_stop);
}


/**
  * @brief Initialize the motion indicator with the default monitoring range.
  * @param p_motion_config Pointer to the structure containing the initialized motion configuration.
  * @param resolution The desired resolution, defined by macros VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.
  * @return Status 0 if initialization is successful, or 127 if the resolution is unknown.
  */
uint8_t VL53L8CX::motion_indicator_init(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution)
{
  return vl53l8cx_motion_indicator_init(p_dev, p_motion_config, resolution);
}

/**
  * @brief Change the working distance range of the motion indicator.
  * @param p_motion_config Pointer to the structure containing the motion configuration.
  * @param distance_min_mm Minimum distance for the motion indicator(minimum value 400mm, maximum 4000mm).
  * @param distance_max_mm Maximum distance for the motion indicator(minimum value 400mm, maximum 4000mm).
  * @return Status 0 if the configuration is successful, or 127 if an argument is invalid.
  */
uint8_t VL53L8CX::motion_indicator_set_distance_motion(VL53L8CX_Motion_Configuration  *p_motion_config, uint16_t distance_min_mm, uint16_t distance_max_mm)
{
  return vl53l8cx_motion_indicator_set_distance_motion(p_dev, p_motion_config, distance_min_mm, distance_max_mm);
}

/**
  * @brief Update the internal motion indicator map to a new resolution.
  * @param p_motion_config Pointer to the structure containing the motion configuration.
  * @param resolution The desired SCI resolution, defined by macros VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.
  * @return Status 0 if the update is successful, or 127 if the resolution is unknown.
  */
uint8_t VL53L8CX::motion_indicator_set_resolution(VL53L8CX_Motion_Configuration *p_motion_config, uint8_t resolution)
{
  return vl53l8cx_motion_indicator_set_resolution(p_dev, p_motion_config, resolution);
}

/**
  * @brief Start the VL53L8CX sensor to calibrate Xtalk, recommended for use with a coverglass.
  * @param reflectance_percent Target reflectance in percent, between 1 and 99%. ST recommends a 3% target reflectance for better efficiency.
  * @param nb_samples Number of samples used for calibration. More samples increase accuracy but also calibration time. Minimum is 1, maximum is 16.
  * @param distance_mm Target distance in mm for calibration. Minimum allowed is 600mm, maximum is 3000mm. The target must stay in Full FOV.
  * @return Status 0 if calibration is successful, 127 if an argument has an incorrect value, or 255 if something failed.
  */
uint8_t VL53L8CX::calibrate_xtalk(uint16_t reflectance_percent, uint8_t nb_samples, uint16_t distance_mm)
{
  return vl53l8cx_calibrate_xtalk(p_dev, reflectance_percent, nb_samples, distance_mm);
}

/**
  * @brief Get the Xtalk calibration data buffer, available after using vl53l8cx_calibrate_xtalk().
  * @param p_xtalk_data Buffer to store Xtalk data, size defined by VL53L8CX_XTALK_SIZE macro.
  * @return Status 0 if buffer reading is successful.
  */
uint8_t VL53L8CX::get_caldata_xtalk(uint8_t *p_xtalk_data)
{
  return vl53l8cx_get_caldata_xtalk(p_dev, p_xtalk_data);
}

/**
  * @brief Set the Xtalk calibration data buffer, can be used to override the default Xtalk buffer.
  * @param p_xtalk_data Buffer containing Xtalk data, size defined by VL53L8CX_XTALK_SIZE macro.
  * @return Status 0 if buffer is set successfully.
  */
uint8_t VL53L8CX::set_caldata_xtalk(uint8_t *p_xtalk_data)
{
  return vl53l8cx_set_caldata_xtalk(p_dev, p_xtalk_data);
}

/**
  * @brief Get the Xtalk margin, used to increase the Xtalk threshold and avoid false positives after calibration.
  * @param p_xtalk_margin Pointer to store the current Xtalk margin in kcps/spads.
  * @return Status 0 if reading is successful.
  */
uint8_t VL53L8CX::get_xtalk_margin(uint32_t *p_xtalk_margin)
{
  return vl53l8cx_get_xtalk_margin(p_dev, p_xtalk_margin);
}

/**
  * @brief Set the Xtalk margin, used to increase the Xtalk threshold and avoid false positives after calibration.
  * @param xtalk_margin New Xtalk margin in kcps/spads. Minimum value is 0, maximum is 10,000 kcps/spads.
  * @return Status 0 if the new margin is set successfully, or 127 if the margin is invalid.
  */
uint8_t VL53L8CX::set_xtalk_margin(uint32_t xtalk_margin)
{
  return vl53l8cx_set_xtalk_margin(p_dev, xtalk_margin);
}

uint8_t VL53L8CX::IO_Read(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(RegisterAddress >> 8);
  buffer[1] = (uint8_t)(RegisterAddress & 0xFF);

  if (!coralmicro::I2cControllerWrite(i2c_config, _dev.platform.address, buffer, 2)) {
    return 1;
  }

  if (!coralmicro::I2cControllerRead(i2c_config, _dev.platform.address, p_values, size)) {
    return 1;
  }

  return 0;
}


uint8_t VL53L8CX::IO_Write(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
  std::vector<uint8_t> buffer(size + 2);
  buffer[0] = (uint8_t)(RegisterAddress >> 8);
  buffer[1] = (uint8_t)(RegisterAddress & 0xFF);
  std::copy(p_values, p_values + size, buffer.begin() + 2);

  if (!coralmicro::I2cControllerWrite(i2c_config, _dev.platform.address, buffer.data(), buffer.size())) {
    return 1;
  }

  return 0;
}


uint8_t VL53L8CX::IO_Wait(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
  return 0;
}

uint8_t VL53L8CX::StaticIO_Write(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
  return ((VL53L8CX *)handle)->IO_Write(RegisterAddress, p_values, size);
}

uint8_t VL53L8CX::StaticIO_Read(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
  return ((VL53L8CX *)handle)->IO_Read(RegisterAddress, p_values, size);
}

uint8_t VL53L8CX::StaticIO_Wait(void *handle, uint32_t ms) {
  return ((VL53L8CX *)handle)->IO_Wait(ms);
}