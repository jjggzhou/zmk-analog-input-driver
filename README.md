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

## Advanced Features

### Auto Calibration
Set `mv-mid = <32767>` to enable automatic calibration on startup. The driver will:
1. Take multiple samples to determine the center position
2. Store calibrated midpoint values in persistent storage
3. Provide calibration statistics via API

#### Runtime Recalibration via Key Binding

Add calibration behavior to your keymap:
```dts
/ {
    behaviors {
        cal_stick: calibrate_analog_stick {
            compatible = "zmk,behavior-analog-calibrate";
            label = "CALIBRATE_STICK";
            #binding-cells = <1>;
        };
    };

    keymap {
        compatible = "zmk,keymap";
        default_layer {
            bindings = <
                // Other keys...
                &cal_stick 0  // Parameter: device index (0=first analog input device)
            >;
        };
    };
};
```

**Parameter Usage:**
- `0` - Calibrate first analog input device (ANALOG_INPUT_0)
- `1` - Calibrate second analog input device (ANALOG_INPUT_1)
- `2` - Calibrate third analog input device (ANALOG_INPUT_2)
- `255` - Calibrate all analog input devices

**Usage Examples:**
```dts
// Single joystick keyboard
&cal_stick 0    // Calibrate the joystick

// Dual joystick keyboard
&cal_stick 0    // Calibrate left joystick
&cal_stick 1    // Calibrate right joystick

// Calibrate all devices at once
&cal_stick 255  // Calibrate all analog input devices
```
```


## Configuration Options

### Device Tree Properties
```dts
anin0: analog_input_0 {
    compatible = "zmk,analog-input";
    sampling-hz = <100>;
    
    // Calibration settings
    calibration {
        samples = <10>;           // Samples per calibration
        retries = <3>;           // Max retry attempts
        variance-threshold = <100>; // Max allowed variance (mV)
        auto-save = <1>;         // Auto-save to storage
    };
    
    x-ch {
        // Set to 32767 for auto-calibration
        mv-mid = <32767>;
        // Other properties...
    };
};
```


## Troubleshooting

*What if it just work 1 minute?*

TL;DR: Set oversampling to zero at [here](https://github.com/zmkfirmware/zmk/blob/461f5c832fb8854d87dca54d113d306323697219/app/module/drivers/sensor/battery/battery_nrf_vddh.c#L90) in your zmk fork to use this module.

If you are running on nrf52840 board and analog reading get stuck after some moment, you need to ground all `uint8_t adc_sequence::oversampling` to zero in your ZMK branch in respect to `oversampling` setting is unsupported by given ADC hardware in a specific mode. [Reference](https://docs.zephyrproject.org/apidoc/latest/structadc__sequence.html#a233e8b20b57bb2fdbebf2c85f076c802).
