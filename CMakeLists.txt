zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT src/analog_input.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT_CALIBRATION src/analog_input_calibration.c)
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT_CALIBRATION_PERSIST src/analog_calibration_storage.c)

zephyr_include_directories(include)
zephyr_include_directories(${ZEPHYR_BASE}/include)
zephyr_include_directories(${ZMK_DIR}/include)

if (CONFIG_ANALOG_INPUT)
    add_compile_definitions(
        ANALOG_INPUT_CALIBRATION_ENABLED=1
        ZMK_ANALOG_INPUT_CALIBRATION_LOG_LEVEL=${CONFIG_ANALOG_INPUT_CALIBRATION_LOG_LEVEL}
    )
endif()
