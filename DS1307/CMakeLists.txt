add_executable(DS1307
        DS1307.c
        I2C_Driver.c
        )

# pull in common dependencies
target_link_libraries(DS1307 pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(DS1307)


