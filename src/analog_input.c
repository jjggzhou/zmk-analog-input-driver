/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP

#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ANALOG_INPUT, CONFIG_ANALOG_INPUT_LOG_LEVEL);

#include <zmk/drivers/analog_input.h>

// Forward declarations
static void* analog_input_safe_malloc(const struct device *dev, size_t size, const char* purpose);
static void analog_input_cleanup_resources(const struct device *dev);
static int analog_input_enhanced_calibrate(const struct device *dev, bool force_recalibrate);


static int analog_input_report_data(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

    struct adc_sequence* as = &data->as;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
        const struct device* adc = ch_cfg->adc_channel.dev;

        if (i == 0) {
#ifdef CONFIG_ADC_ASYNC
            int err = adc_read_async(adc, as, &data->async_sig);
            if (err < 0) {
                LOG_ERR("AIN%u read_async returned %d", i, err);
                return err;
            }
            err = k_poll(&data->async_evt, 1, K_FOREVER);
            if (err < 0) {
                LOG_ERR("AIN%u k_poll returned %d", i, err);
                return err;
            }
            if (!data->async_evt.signal->signaled) {
                return 0;
            }
            data->async_evt.signal->signaled = 0;
    	    data->async_evt.state = K_POLL_STATE_NOT_READY;
#else
            int err = adc_read(adc, as);
            if (err < 0) {
                LOG_ERR("AIN%u read returned %d", i, err);
                return err;
            }
#endif
        }

        int32_t raw = data->as_buff[i];
        int32_t mv = raw;
        adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, as->resolution, &mv);
#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)
        LOG_DBG("AIN%u raw: %d mv: %d", ch_cfg->adc_channel.channel_id, raw, mv);
#endif
        
        // 如果刚完成校准，持续输出校准信息
        if (data->calibration_done && data->calibration_count < 100) {
            LOG_INF("=== CALIBRATION INFO (count %d) ===", data->calibration_count);
            LOG_INF("Channel %d: raw=%d, mv=%d, config_mv_mid=%d, calibrated_mv_mid=%d", 
                    i, raw, mv, ch_cfg->mv_mid, data->calibrated_mv_mid[i]);
            if (i == config->io_channels_len - 1) {
                data->calibration_count++;
            }
        }
        
        // 使用校准后的mv_mid值（如果已校准），否则使用配置中的值
        uint16_t effective_mv_mid = (data->calibration_done) ? data->calibrated_mv_mid[i] : ch_cfg->mv_mid;
        int16_t v = mv - effective_mv_mid;
        int16_t dz = ch_cfg->mv_deadzone;
        if (dz) {
            if (v > 0) {
                if (v < dz) v = 0; else v -= dz;
            }
            if (v < 0) {
                if (v > -dz) v = 0; else v += dz;
            }
        }
        uint16_t mm = ch_cfg->mv_min_max;
        if (mm) {
            if (v > 0 && v > mm) v = mm;
            if (v < 0 && v < -mm) v = -mm;
        }

        if (ch_cfg->invert) v *= -1;
        v = (int16_t)((v * ch_cfg->scale_multiplier) / ch_cfg->scale_divisor);

#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)
        LOG_DBG("AIN%u processed: mv=%d, effective_mv_mid=%d, v_before_deadzone=%d, v_after_deadzone=%d, final_v=%d", 
                ch_cfg->adc_channel.channel_id, mv, effective_mv_mid, mv - effective_mv_mid, v, v);
#endif

        if (ch_cfg->report_on_change_only) {
            // track raw value to compare until next report interval
            data->delta[i] = v;
        }
        else {
            // accumulate delta until report in next iteration
            int32_t delta = data->delta[i];
            int32_t dv = delta + v;
            data->delta[i] = dv;
        }
    }

    // First read is setup as calibration
    as->calibrate = false;

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            data->delta[i] = 0;
            data->prev[i] = 0;
        }
    }
    last_smp_time = now;
#endif

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    if (!data->actived) {
        return 0;
    }

    int8_t idx_to_sync = -1;
    for (int8_t i = config->io_channels_len - 1; i >= 0; i--) {
        int32_t dv = data->delta[i];
        int32_t pv = data->prev[i];
        if (dv != pv) {
            idx_to_sync = i;
            break;
        }
    }

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
        // LOG_DBG("AIN%u get delta AGAIN", i);
        int32_t dv = data->delta[i];
        int32_t pv = data->prev[i];
        if (dv != pv) {
#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
            last_rpt_time = now;
#endif
            data->delta[i] = 0;
            if (ch_cfg->report_on_change_only) {
                data->prev[i] = dv;
            }

#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)
            LOG_DBG("input_report %u rv: %d  e:%d  c:%d", i, dv, ch_cfg->evt_type, ch_cfg->input_code);
#endif
            input_report(dev, ch_cfg->evt_type, ch_cfg->input_code, dv, i == idx_to_sync, K_NO_WAIT);
        } else {
#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)
            LOG_DBG("No report for channel %u: dv=%d, pv=%d", i, dv, pv);
#endif
        }
    }
    return 0;
}

K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);

static struct k_work_q analog_input_work_q;

static void sampling_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    // LOG_DBG("sampling work triggered");
    analog_input_report_data(data->dev);
}

static void sampling_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
    // LOG_DBG("sampling timer triggered");
    k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
}

static int active_set_value(const struct device *dev, bool active) {
    struct analog_input_data *data = dev->data;
    if (data->actived == active) return 0;
    LOG_DBG("%d", active ? 1 : 0);
    data->actived = active;
    return 0;
}

static int sample_hz_set_value(const struct device *dev, uint32_t hz) {
    struct analog_input_data *data = dev->data;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled) {
        LOG_DBG("Device is busy, would not update sampleing rate in enable state.");
        return -EBUSY;
    }

    LOG_DBG("%d", hz);
    data->sampling_hz = hz;
    return 0;
}

static int enable_set_value(const struct device *dev, bool enable) {
    struct analog_input_data *data = dev->data;
    // const struct tb6612fng_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled == enable) {
        return 0;
    }
    
    LOG_DBG("%d", enable ? 1 : 0);
    if (enable) {
        if (data->sampling_hz != 0) {
            uint32_t usec = 1000000UL / data->sampling_hz;
            k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
        } else {
            k_timer_start(&data->sampling_timer, K_NO_WAIT, K_NO_WAIT);
        }
        data->enabled = true;
    }
    else {
        k_timer_stop(&data->sampling_timer);
        data->enabled = false;
    }

    return 0;
}

static void analog_input_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct analog_input_data *data = CONTAINER_OF(work_delayable, 
                                                  struct analog_input_data, init_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    LOG_INF("=== ANALOG INPUT INITIALIZATION STARTED ===");
    // LOG_DBG("ANALOG_INPUT async init");
    uint32_t ch_mask = 0;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
        const struct device* adc = ch_cfg->adc_channel.dev;
        uint8_t channel_id = ch_cfg->adc_channel.channel_id;
        
        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = channel_id,
            #ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
                #ifdef CONFIG_ADC_NRFX_SAADC
                    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + channel_id,
                #else /* CONFIG_ADC_NRFX_SAADC */
                    .input_positive = channel_id,
                #endif /* CONFIG_ADC_NRFX_SAADC */
            #endif /* CONFIG_ADC_CONFIGURABLE_INPUTS */
        };

        ch_mask |= BIT(channel_id);

        if (!device_is_ready(adc)) {
            LOG_ERR("AIN%u device is not ready %s", i, adc->name);
            continue;
        }

        int err = adc_channel_setup(adc, &channel_cfg);
        if (err < 0) {
            LOG_ERR("AIN%u setup returned %d", i, err);
        }
    }

    uint16_t delta_size = config->io_channels_len * sizeof(int32_t);
    data->delta = malloc(delta_size);
    memset(data->delta, 0, delta_size);

    uint16_t prev_size = config->io_channels_len * sizeof(int32_t);
    data->prev = malloc(prev_size);
    memset(data->prev, 0, prev_size);

    uint16_t buff_size = config->io_channels_len * sizeof(uint16_t);
    data->as_buff = malloc(buff_size);
    memset(data->as_buff, 0, buff_size);

    // 分配校准后的mv_mid数组
    uint16_t mv_mid_size = config->io_channels_len * sizeof(uint16_t);
    data->calibrated_mv_mid = malloc(mv_mid_size);
    memset(data->calibrated_mv_mid, 0, mv_mid_size);

    data->as = (struct adc_sequence){
        .channels = ch_mask,
        .buffer = data->as_buff,
        .buffer_size = buff_size,
        .oversampling = 0,
        .resolution = 12,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_ASYNC
    k_poll_signal_init(&data->async_sig);
    struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                                             K_POLL_MODE_NOTIFY_ONLY,
                                                             &data->async_sig);
    data->async_evt = async_evt;
#endif

    data->ready = true;

    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&analog_input_work_q,
                        analog_input_q_stack, K_THREAD_STACK_SIZEOF(analog_input_q_stack),
                        CONFIG_ANALOG_INPUT_WORKQUEUE_PRIORITY, NULL);

    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);

    sample_hz_set_value(dev, config->sampling_hz);
    active_set_value(dev, true);
    if (data->sampling_hz) {
        enable_set_value(dev, true);
    }

    // 在初始化时检查配置中的mv_mid值，如果为0x7FFF则进行校准
    bool need_calibration = false;
    LOG_INF("=== CHECKING CALIBRATION NEED ===");
    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
        LOG_INF("Channel %d: checking mv_mid=%d (0x%X)", i, ch_cfg->mv_mid, ch_cfg->mv_mid);
        
        if (ch_cfg->mv_mid == 32767) {
            need_calibration = true;
            LOG_INF("=== CALIBRATION DETECTED ===");
            LOG_INF("Detected 32767 (0x7FFF) in config for channel %d, will perform calibration", i);
            break;
        }
    }
    
    if (need_calibration) {
        LOG_INF("=== WAITING 100ms FOR ADC STABILIZATION BEFORE CALIBRATION ===");
        k_msleep(100); // 等待100ms让硬件信号稳定
        LOG_INF("=== PERFORMING ADC READ FOR CALIBRATION ===");
        
        // 为每个通道单独读取ADC值
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
            const struct device* adc = ch_cfg->adc_channel.dev;
            
            // 为单个通道创建ADC序列
            struct adc_sequence single_as = {
                .channels = BIT(ch_cfg->adc_channel.channel_id),
                .buffer = &data->as_buff[i],
                .buffer_size = sizeof(uint16_t),
                .oversampling = 0,
                .resolution = 12,
                .calibrate = false,
            };
            
            LOG_INF("Reading ADC channel %d for calibration", ch_cfg->adc_channel.channel_id);
            int err = adc_read(adc, &single_as);
            if (err < 0) {
                LOG_ERR("ADC read failed for channel %d during calibration: %d", i, err);
                continue;
            }
            
            int32_t raw = data->as_buff[i];
            int32_t mv = raw;
            
            // 将原始值转换为毫伏
            adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, single_as.resolution, &mv);
            
            // 将当前读取到的值存储到data中的calibrated_mv_mid
            data->calibrated_mv_mid[i] = mv;
            LOG_INF("=== CALIBRATION RESULT ===");
            LOG_INF("Updated data: channel %d (ADC ch %d), new mv_mid=%d (raw=%d)", 
                    i, ch_cfg->adc_channel.channel_id, mv, raw);
        }
        LOG_INF("=== CALIBRATION COMPLETED ===");
        
        // 设置校准完成标志，在后续采样中持续输出校准信息
        data->calibration_done = true;
        data->calibration_count = 0;
    } else {
        LOG_INF("No calibration needed during initialization");
    }

    LOG_INF("=== ANALOG INPUT INITIALIZATION COMPLETED ===");
    
    // 延时5秒后输出校准信息，确保串口监视器能够捕获
    k_sleep(K_MSEC(5000));
    LOG_INF("=== 5 SECONDS DELAY COMPLETED ===");
    
    if (need_calibration) {
        LOG_INF("=== CALIBRATION SUMMARY ===");
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
            LOG_INF("Channel %d: config_mv_mid=%d, calibrated_mv_mid=%d", i, ch_cfg->mv_mid, data->calibrated_mv_mid[i]);
        }
    }
}

static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    int err = 0;

    // 初始化内存统计
    memset(&data->mem_stats, 0, sizeof(data->mem_stats));

    // 分配内存资源
    // 分配内存资源
    data->delta = analog_input_safe_malloc(dev, config->io_channels_len * sizeof(int32_t), "delta");
    data->prev = analog_input_safe_malloc(dev, config->io_channels_len * sizeof(int32_t), "prev");
    data->as_buff = analog_input_safe_malloc(dev, config->io_channels_len * sizeof(uint16_t), "ADC buffer");
    data->calibrated_mv_mid = analog_input_safe_malloc(dev, config->io_channels_len * sizeof(uint16_t), "calibration");
    if (!data->delta || !data->prev || !data->as_buff || !data->calibrated_mv_mid) {
        analog_input_cleanup_resources(dev);
        return -ENOMEM;
    }

    data->dev = dev;
    k_work_init_delayable(&data->init_work, analog_input_async_init);
    k_work_schedule(&data->init_work, K_MSEC(1));

    LOG_INF("Initialized with %d channels, allocated %zu bytes",
           config->io_channels_len, data->mem_stats.total_allocated);
    return err;
}

static int analog_input_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    int err;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case ANALOG_INPUT_ATTR_SAMPLING_HZ:
        err = sample_hz_set_value(dev, ANALOG_INPUT_SVALUE_TO_SAMPLING_HZ(*val));
        break;

    case ANALOG_INPUT_ATTR_ENABLE:
        err = enable_set_value(dev, ANALOG_INPUT_SVALUE_TO_ENABLE(*val));
        break;

    case ANALOG_INPUT_ATTR_ACTIVE:
        err = active_set_value(dev, ANALOG_INPUT_SVALUE_TO_ACTIVE(*val));
        break;

    case ANALOG_INPUT_ATTR_CALIBRATE:
        err = analog_input_enhanced_calibrate(dev, true);
        break;

    case ANALOG_INPUT_ATTR_CALIBRATION_STATS:
        LOG_INF("=== CALIBRATION STATISTICS ===");
        LOG_INF("Total calibrations: %u", data->calibration_stats.total_calibrations);
        LOG_INF("Successful calibrations: %u", data->calibration_stats.successful_calibrations);
        LOG_INF("Failed calibrations: %u", data->calibration_stats.failed_calibrations);
        LOG_INF("Last calibration time: %lld ms ago",
               k_uptime_get() - data->calibration_stats.last_calibration_time);
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            LOG_INF("Channel %d variance: %u mV", i,
                   data->calibration_stats.calibration_variance[i]);
        }
        err = 0;
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static int analog_input_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    int err = analog_input_report_data(data->dev);
    if (err < 0) {
        LOG_ERR("analog_input_report_data returned %d", err);
        return err;
    }

    return 0;
}

static int analog_input_channel_get(const struct device *dev, enum sensor_channel chan,
                                    struct sensor_value *val) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[i];
        if (!ch_cfg->report_on_change_only) {
            continue;
        }
        if (i == 0)      val->val1 = data->delta[i];
        else if (i == 1) val->val2 = data->delta[i];
    }

    return 0;
}

static const struct sensor_driver_api analog_input_driver_api = {
    .attr_set = analog_input_attr_set,
    .sample_fetch = analog_input_sample_fetch,
    .channel_get = analog_input_channel_get,
};

#define TRANSFORMED_IO_CHANNEL_ENTRY(node_id)                                                      \
    {                                                                                              \
        .adc_channel = ADC_DT_SPEC_GET_BY_IDX(node_id, 0),                                         \
        .mv_mid = DT_PROP(node_id, mv_mid),                                                        \
        .mv_min_max = DT_PROP(node_id, mv_min_max),                                                \
        .mv_deadzone = DT_PROP(node_id, mv_deadzone),                                              \
        .invert = DT_PROP(node_id, invert),                                                        \
        .report_on_change_only = DT_PROP(node_id, report_on_change_only),                          \
        .scale_multiplier = DT_PROP(node_id, scale_multiplier),                                    \
        .scale_divisor = DT_PROP(node_id, scale_divisor),                                          \
        .evt_type = DT_PROP(node_id, evt_type),                                                    \
        .input_code = DT_PROP(node_id, input_code),                                                \
    }

#define ANIN_IOC_CHILD_LEN_PLUS_ONE(node) 1 +

#define ANALOG_INPUT_DEFINE(n)                                                                     \
    static struct analog_input_data data##n = {                                                    \
    };                                                                                             \
    static const struct analog_input_config config##n = {                                          \
        .sampling_hz = DT_PROP(DT_DRV_INST(n), sampling_hz),                                       \
        .io_channels_len = (DT_FOREACH_CHILD(DT_DRV_INST(n), ANIN_IOC_CHILD_LEN_PLUS_ONE) 0),      \
        .io_channels = { DT_INST_FOREACH_CHILD_SEP(n, TRANSFORMED_IO_CHANNEL_ENTRY, (, )) },       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, analog_input_init, NULL, &data##n, &config##n, POST_KERNEL,           \
                          CONFIG_SENSOR_INIT_PRIORITY, &analog_input_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_INPUT_DEFINE)

// 安全的内存分配函数
static void* analog_input_safe_malloc(const struct device *dev, size_t size, const char* purpose) {
    if (!dev || !purpose || size == 0) {
        LOG_ERR("Invalid parameters for memory allocation");
        return NULL;
    }
    
    struct analog_input_data *data = dev->data;
    if (!data) {
        LOG_ERR("No device data available");
        return NULL;
    }
    void* ptr = malloc(size);
    if (!ptr) {
        LOG_ERR("Memory allocation failed for %s (size: %zu)", purpose, size);
        return NULL;
    }
    
    memset(ptr, 0, size);
    data->mem_stats.total_allocated += size;
    data->mem_stats.peak_usage = MAX(data->mem_stats.peak_usage, data->mem_stats.total_allocated);
    data->mem_stats.allocation_count++;
    LOG_DBG("Allocated %zu bytes for %s at %p", size, purpose, ptr);
    return ptr;
}

// 资源清理函数
static void analog_input_cleanup_resources(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    
    if (!data) return;
    
    LOG_DBG("Cleaning up resources for device %s", dev->name);
    
    // 停止所有硬件活动
    enable_set_value(dev, false);
    k_timer_stop(&data->sampling_timer);
    
    // 释放内存资源
    free(data->delta);
    free(data->prev);
    free(data->as_buff);
    free(data->calibrated_mv_mid);
    
    // 清理工作队列
    k_work_cancel(&data->sampling_work);
    k_work_queue_drain(&analog_input_work_q, false);
    
    LOG_INF("Device %s resources cleaned up", dev->name);
}

// 设备注销函数
static int analog_input_deinit(const struct device *dev) {
    analog_input_cleanup_resources(dev);
    return 0;
}

// 自动校准函数：将当前采样值设为mv_mid
static int analog_input_enhanced_calibrate(const struct device *dev, bool force_recalibrate) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    
    if (!force_recalibrate && data->calibration_state == CALIBRATION_COMPLETED) {
        return 0;
    }
    
    data->calibration_state = CALIBRATION_IN_PROGRESS;
    LOG_INF("=== ENHANCED CALIBRATION STARTED ===");
    
    k_msleep(ANALOG_INPUT_STABILIZE_DELAY_MS);
    
    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel *ch_cfg = &config->io_channels[i];
        
        if (ch_cfg->mv_mid != ANALOG_INPUT_AUTO_CALIBRATE_FLAG && !force_recalibrate) {
            continue;
        }
        
        uint32_t sum = 0;
        uint16_t valid_samples = 0;
        uint16_t min_val = UINT16_MAX;
        uint16_t max_val = 0;
        
        for (int retry = 0; retry < ANALOG_INPUT_CALIBRATE_MAX_RETRIES; retry++) {
            for (int sample = 0; sample < ANALOG_INPUT_CALIBRATE_SAMPLES; sample++) {
                int32_t mv;
                int err = analog_input_read_single_channel(dev, i, &mv);
                if (err == 0) {
                    sum += mv;
                    valid_samples++;
                    min_val = MIN(min_val, mv);
                    max_val = MAX(max_val, mv);
                    k_msleep(10);
                }
            }
            
            if (valid_samples >= ANALOG_INPUT_CALIBRATE_SAMPLES / 2) {
                break;
            }
            
            LOG_WRN("Calibration retry %d for channel %d", retry + 1, i);
            k_msleep(50);
        }
        
        if (valid_samples == 0) {
            LOG_ERR("Calibration failed for channel %d", i);
            data->calibration_state = CALIBRATION_FAILED;
            return -EIO;
        }
        
        uint16_t avg_mv = sum / valid_samples;
        uint16_t variance = max_val - min_val;
        
        if (variance > 100) {
            LOG_WRN("High variance (%d mV) in calibration", variance);
        }
        
        data->calibrated_mv_mid[i] = avg_mv;
        data->calibration_stats.calibration_variance[i] = variance;
        data->calibration_stats.total_calibrations++;
        
        LOG_INF("Channel %d calibrated: avg=%d mV, variance=%d", i, avg_mv, variance);
    }
    
    data->calibration_state = CALIBRATION_COMPLETED;
    data->calibration_stats.successful_calibrations++;
    data->calibration_stats.last_calibration_time = k_uptime_get();
    
    LOG_INF("=== ENHANCED CALIBRATION COMPLETED ===");
    return 0;
}

// 读取单个通道的函数
static int analog_input_read_single_channel(const struct device *dev, uint8_t channel_idx, int32_t *mv_out) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    
    if (channel_idx >= config->io_channels_len) {
        return -EINVAL;
    }
    
    struct analog_input_io_channel *ch_cfg = (struct analog_input_io_channel *)&config->io_channels[channel_idx];
    const struct device* adc = ch_cfg->adc_channel.dev;
    
    // 为单个通道创建ADC序列
    uint16_t buffer;
    struct adc_sequence single_as = {
        .channels = BIT(ch_cfg->adc_channel.channel_id),
        .buffer = &buffer,
        .buffer_size = sizeof(uint16_t),
        .oversampling = 0,
        .resolution = 12,
        .calibrate = false,
    };
    
    int err = adc_read(adc, &single_as);
    if (err < 0) {
        return err;
    }
    
    int32_t mv = buffer;
    adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, single_as.resolution, &mv);
    *mv_out = mv;
    
    return 0;
}
