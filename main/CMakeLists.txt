set(c5vrx_sources
    "main.c"
    "bs_relative_worker_probe.c"
    "bs_relative_middle_probe.c"
    "phy_phase_tap_probe.c"
    "bs_addctia_probe.c"
    "arc_phy.c"
    "arc_v3_controller.c"
    "arc_v5_autotune.c"
    "rx_auto_lab.c"
    "rf.c"
    "video.c"
    "direct_gain.c"
    "direct_gain_v2.c"
    "direct_gain_v3.c"
    "analog_video_detect.c"
    "menu_raster.c")

if(CONFIG_C5VRX_RSSI_METER)
    # Standalone meter image: proven RF state only, no video pipeline.
    set(c5vrx_sources
        "rssi_meter_main.c"
        "rf.c"
        "arc_phy.c"
        "rssi_pipeline.c"
        "rx5808_bus.c"
        "rssi_sdm.c"
        "rh_node.c")
endif()

idf_component_register(
    SRCS ${c5vrx_sources}
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_phy
        nvs_flash
        esp_event
        esp_netif
        esp_driver_parlio
        esp_driver_gpio
        esp_driver_bitscrambler
        esp_driver_usb_serial_jtag
        vfs
        esp_rom
        esp_mm
        esp_timer
        freertos
        heap
        hal
        esp_hw_support
        esp_hal_gpio
        esp_driver_gptimer
        esp_driver_sdm
)

# Proven default plus opt-in experimental demod/output paths. The meter
# image has no BitScrambler and skips every .bsasm program.
if(NOT CONFIG_C5VRX_RSSI_METER)
target_bitscrambler_add_src("fm.bsasm")
target_bitscrambler_add_src("fm_relative_golden.bsasm")
target_bitscrambler_add_src("fm_phase5_360.bsasm")
target_bitscrambler_add_src("bs_relative_worker_probe.bsasm")
target_bitscrambler_add_src("bs_relative_middle_probe.bsasm")
target_bitscrambler_add_src("fm_phase5_fsm_capture.bsasm")
target_bitscrambler_add_src("fm4.bsasm")
target_bitscrambler_add_src("bs_addctia_probe.bsasm")
target_bitscrambler_add_src("fm_phase8_hr_live.bsasm")
target_bitscrambler_add_src("fm_hc.bsasm")
if(C5VRX4_EXPERIMENT)
    include("${PROJECT_DIR}/component.cmake")
endif()
endif()
