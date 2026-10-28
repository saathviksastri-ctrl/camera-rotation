idf_component_register(
    SRCS "sd_card_main_derot.c" "derotate.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_image_effects esp_video esp_muxer fatfs driver esp_timer example_video_common
)
if(NOT CONFIG_SOC_SDMMC_HOST_SUPPORTED)
    fail_at_build_time(sdmmc ""
                             "SDMMC host is not supported."
                             "Please refer README.md for more details")
endif()