set(sgp_srcs "src/sgp4x.c")

idf_component_register(SRCS "${sgp_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES "driver")