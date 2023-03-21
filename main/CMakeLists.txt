idf_component_register(SRCS "ctrl_subsystem.c" "itf_seven_seg.c" "itf_sd_card_writer.c" "itf_sd_card_setup.c" "itf_main.c" "itf_com_funcs.c" "main.c" 
                    INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
