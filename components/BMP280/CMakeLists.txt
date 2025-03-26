set(bmp_srcs "src/bmp280.c")

idf_component_register(SRCS "${bmp_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES "driver")