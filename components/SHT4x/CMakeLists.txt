set(sht_srcs "src/sht4x.c")

idf_component_register(SRCS "${sht_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES "driver")