idf_component_register(SRCS "main.c"
	"bus/bus_tree.c" "bus/uart.c" "bus/spi.c"
	"drivers/imu/icm20948.c"
	"sensors/imu.c"
	"flight/flight.c"
	REQUIRES "driver" "esp_wifi"
                    INCLUDE_DIRS ".")
