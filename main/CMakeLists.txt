idf_component_register(SRCS "main.c"
	"bus/bus.c" "bus/spi_dev.c" "bus/uart_dev.c"
	"drivers/bus/bus_tree.c" "drivers/bus/spi.c" "drivers/bus/uart.c"
	"drivers/imu/icm20948.c"
	"sensors/imu.c"
	"flight/flight.c"
	"misc/util.c"
	REQUIRES "driver" "esp_wifi"
                    INCLUDE_DIRS ".")
