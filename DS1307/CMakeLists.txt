
add_library(DS1307_LIB STATIC
        DS1307.c
        I2C_Driver.c
        )
        
# pull in common dependencies
target_link_libraries(DS1307_LIB pico_stdlib hardware_i2c)

#include the 'include' directory with header files
target_include_directories(DS1307_LIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


