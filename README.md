# ZMK Analog Input Driver

This driver groups ADC io channels into single input event for input subsystem. It provides config for thumbstick input with:

- Mid-point alignment with auto-calibration (set mv-mid=32767)
- Min-max limitation
- Adjustable deadzone
- Configurable sampling rate
- Reporting rate control
- Multiplier/divisor scaling
- Inversion option
- Runtime calibration control

> [!CAUTION]
> This poll mode driver has relativley high power consumption, its not recommended for wireless builds.

## Installation

Only GitHub actions builds are covered here. Local builds are different for each user, therefore it's not possible to cover all cases.

Include this project on your ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    ...
    # START #####
    - name: badjeff
      url-base: https://github.com/badjeff
    # END #######
  projects:
    ...
    # START #####
    - name: zmk-analog-input-driver
      remote: badjeff
      revision: main
    # END #######
    ...
```

Now, update your `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
#include <zephyr/dt-bindings/input/input-event-codes.h>
/* Reference: https://docs.zephyrproject.org/apidoc/latest/group__input__events.html */

&adc {
	status = "okay";
};

/ {
	anin0: analog_input_0 {
		compatible = "zmk,analog-input";
		sampling-hz = <100>;
		x-ch {
			io-channels = <&adc 2>; // <--- see ain-map.png for nRF52840
			// Set to 32767 (0x7FFF) to enable auto-calibration
			mv-mid = <1630>;
			mv-min-max = <1600>;
			mv-deadzone = <10>;
			scale-multiplier = <1>;
			scale-divisor = <70>;
			invert;
			evt-type = <INPUT_EV_REL>;
			input-code = <INPUT_REL_X>;
		};
		y-ch {
			io-channels = <&adc 3>; // <--- see ain-map.png for nRF52840
			mv-mid = <1630>;
			mv-min-max = <1600>;
			mv-deadzone = <10>;
			scale-multiplier = <3>;
			scale-divisor = <4>;
			invert;
			evt-type = <INPUT_EV_REL>;
			input-code = <INPUT_REL_Y>;

			/* enable report mdoe for gamepad axix or knob */
			/* to only call input_report on quantquantized value is updated */
			/* NOTE: mouse input does NOT need this */
			report-on-change-only;

		};
	};
};
```

Now enable the driver config in your `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
# Enable Analog Input
CONFIG_ADC=y
# Use async mode (Optional)
CONFIG_ADC_ASYNC=y

# Enable Analog Input Module
CONFIG_ANALOG_INPUT=y
# CONFIG_ANALOG_INPUT_LOG_LEVEL_DBG=y
# CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN=22

# Enable logging for pre/post processed value
CONFIG_ANALOG_INPUT_LOG_DBG_RAW=y
CONFIG_ANALOG_INPUT_LOG_DBG_REPORT=y

# Just in case, you don't RTFM
CONFIG_INPUT=y
```

## 高级功能

### 自动校准
设置 `mv-mid = <32767>` 以启用启动时自动校准。驱动将：
1. 采集多个样本确定中心位置
2. 将校准后的中点值存储在持久化存储中
3. 通过 API 提供校准统计信息

### 校准参数

```c
// 校准相关常量定义
#define ANALOG_INPUT_AUTO_CALIBRATE_FLAG 32767  // 0x7FFF
#define ANALOG_INPUT_STABILIZE_DELAY_MS 50      // 校准前稳定时间(ms)
#define ANALOG_INPUT_CALIBRATE_SAMPLES 5        // 每次校准采样次数
#define ANALOG_INPUT_CALIBRATE_MAX_RETRIES 2    // 最大校准重试次数
#define ANALOG_INPUT_CALIBRATE_VARIANCE_THRESHOLD 50  // 最大允许方差(mV)
```

### 通过按键绑定进行运行时重新校准

在 keymap 中添加校准行为：

```dts
/ {
    behaviors {
        cal_stick: calibrate_analog_stick {
            compatible = "zmk,behavior-analog-calibrate";
            #binding-cells = <0>;  
        };
    };

    keymap {
        compatible = "zmk,keymap";
        default_layer {
            bindings = <
                // 其他按键...
                &cal_stick
            >;
        };
    };
};
```

**当前实现说明**：
- 当前版本仅支持校准设备树中定义的 `anin0` 设备
- 参数值会被忽略，但为了兼容性仍需提供一个值（通常为0）
- 校准会强制重新计算中点值并更新校准参数

**使用示例**：
```dts
// 校准模拟输入设备
&cal_stick 0  // 参数值任意，当前实现中会被忽略
```

**日志输出**：
- 校准开始时会输出日志：`Calibration triggered for device: anin0`
- 校准成功：`Calibration completed successfully`
- 校准失败：`Calibration failed with error: <错误码>`

## 配置选项

### 设备树属性

```dts
anin0: analog_input_0 {
    compatible = "zmk,analog-input";
    sampling-hz = <100>;  // 采样率 (Hz)
    
    x-ch {
        io-channels = <&adc 7>;  // ADC 通道号
        // 设置为 32767 启用自动校准
        mv-mid = <32767>;
        mv-min-max = <800>;      // 最大偏移量 (mV)
        mv-deadzone = <50>;      // 死区大小 (mV)
        scale-multiplier = <1>;  // 缩放乘数
        scale-divisor = <6>;     // 缩放除数
        evt-type = <INPUT_EV_REL>;  // 事件类型
        input-code = <INPUT_REL_X>;  // 输入代码
        report-on-change-only;   // 仅在值变化时报告
    };
    
    y-ch {
        io-channels = <&adc 6>;  // ADC 通道号
        mv-mid = <32767>;        // 32767 启用自动校准
        mv-min-max = <800>;      // 最大偏移量 (mV)
        mv-deadzone = <50>;      // 死区大小 (mV)
        scale-multiplier = <1>;  // 缩放乘数
        scale-divisor = <6>;     // 缩放除数
        evt-type = <INPUT_EV_REL>;  // 事件类型
        input-code = <INPUT_REL_Y>;  // 输入代码
        report-on-change-only;   // 仅在值变化时报告
    };
};
```


## Troubleshooting

*What if it just work 1 minute?*

TL;DR: Set oversampling to zero at [here](https://github.com/zmkfirmware/zmk/blob/461f5c832fb8854d87dca54d113d306323697219/app/module/drivers/sensor/battery/battery_nrf_vddh.c#L90) in your zmk fork to use this module.

If you are running on nrf52840 board and analog reading get stuck after some moment, you need to ground all `uint8_t adc_sequence::oversampling` to zero in your ZMK branch in respect to `oversampling` setting is unsupported by given ADC hardware in a specific mode. [Reference](https://docs.zephyrproject.org/apidoc/latest/structadc__sequence.html#a233e8b20b57bb2fdbebf2c85f076c802).
