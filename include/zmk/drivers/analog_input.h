/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZEPHYR_INCLUDE_ANALOG_INPUT_H_
#define ZEPHYR_INCLUDE_ANALOG_INPUT_H_

/**
 * @file analog_input.h
 *
 * @brief Header file for the analog_input driver.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 校准相关常量定义
#define ANALOG_INPUT_AUTO_CALIBRATE_FLAG 32767  // 0x7FFF
#define ANALOG_INPUT_STABILIZE_DELAY_MS 50      // 校准前稳定时间(ms)，从100ms减少到50ms
#define ANALOG_INPUT_CALIBRATE_SAMPLES 5        // 每次校准采样次数，从10减少到5
#define ANALOG_INPUT_CALIBRATE_MAX_RETRIES 2    // 最大校准重试次数，从3减少到2
#define ANALOG_INPUT_CALIBRATE_VARIANCE_THRESHOLD 50  // 最大允许方差(mV)，从100增加到150

struct analog_input_data {
    const struct device *dev;
    struct adc_sequence as;
#if CONFIG_ADC_ASYNC
    struct k_poll_signal async_sig;
    struct k_poll_event async_evt;
#endif
    uint16_t *as_buff;
    int32_t *delta;
    int32_t *prev;
    struct k_work_delayable init_work;
    int async_init_step;
    bool ready;

    uint32_t sampling_hz;
    bool enabled;
    bool actived;

    struct k_work sampling_work;
    struct k_timer sampling_timer;
    int err;
    
    // 校准相关字段
    enum analog_input_calibration_state {
        CALIBRATION_NONE = 0,
        CALIBRATION_PENDING,
        CALIBRATION_IN_PROGRESS,
        CALIBRATION_COMPLETED,
        CALIBRATION_FAILED
    } calibration_state;
    uint8_t calibration_count;
    bool calibration_done;
    uint16_t *calibrated_mv_mid;
    
    // 校准统计
    struct {
        uint32_t total_calibrations;
        uint32_t successful_calibrations;
        uint32_t failed_calibrations;
        int64_t last_calibration_time;
        uint16_t calibration_variance[CONFIG_ANALOG_INPUT_MAX_CHANNELS];
    } calibration_stats;
    
    // 内存统计
    struct {
        size_t total_allocated;
        size_t peak_usage;
        uint32_t allocation_count;
        uint32_t deallocation_count;
    } mem_stats;
};

struct analog_input_io_channel { 
	struct adc_dt_spec adc_channel;
    uint16_t mv_mid;
    uint16_t mv_min_max;
    uint8_t mv_deadzone;
    bool invert;
    bool report_on_change_only;
    uint16_t scale_multiplier;
    uint16_t scale_divisor;
    uint8_t evt_type;
    uint8_t input_code;
};

struct analog_input_config {
    uint32_t sampling_hz;
    uint8_t io_channels_len;
	struct analog_input_io_channel io_channels[];
};

/* Helper macros used to convert sensor values. */
#define ANALOG_INPUT_SVALUE_TO_SAMPLING_HZ(svalue) ((uint32_t)(svalue).val1)
#define ANALOG_INPUT_SVALUE_TO_ENABLE(svalue) ((uint32_t)(svalue).val1)
#define ANALOG_INPUT_SVALUE_TO_ACTIVE(svalue) ((uint32_t)(svalue).val1)

/** @brief Sensor specific attributes of ANALOG_INPUT. */
enum analog_input_attribute {
    // 运行时校准控制
    ANALOG_INPUT_ATTR_CALIBRATE = 0x100,  // 触发重新校准
    ANALOG_INPUT_ATTR_CALIBRATION_STATS,  // 获取校准统计信息

    // setup polling timer
    ANALOG_INPUT_ATTR_SAMPLING_HZ,

    // ENABLE sampling timer
	ANALOG_INPUT_ATTR_ENABLE,

    // ACTIVE input reporting
    // or else, manually call sample_fetch & channel_get via sensor api.
	ANALOG_INPUT_ATTR_ACTIVE,

};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_ANALOG_INPUT_H_ */
