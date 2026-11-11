if(CONFIG_PVDD_SWITCH)
    set(srcs "pvdd_switch.c" "pvdd_switch_logic.c")
else()
    set(srcs "")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_adc
)
