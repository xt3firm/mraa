#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

/* mraa header */
#include "mraa/spi.h"

/* SPI declaration */
#define SPI_BUS 0
#define SPI_CS 0

/* SPI frequency in Hz */
#define SPI_FREQ 400000
int main(int argc, char** argv)
{
	mraa_result_t status = MRAA_SUCCESS;
	mraa_spi_context spi;
	int i, j;

	/* initialize mraa for the platform (not needed most of the times) */
	mraa_init();

	/* initialize SPI bus and SPI cs*/
	spi = mraa_spi_init_raw(SPI_BUS, SPI_CS);
	if (spi == NULL) {
		fprintf(stderr, "Failed to initialize SPI\n");
		mraa_deinit();
		return EXIT_FAILURE;
	}

	/* set SPI frequency */
	status = mraa_spi_frequency(spi, SPI_FREQ);
	if (status != MRAA_SUCCESS)
		goto err_exit;

	/* set big endian mode */
	status = mraa_spi_lsbmode(spi, 0);
	if (status != MRAA_SUCCESS) {
		goto err_exit;
	}
	while(1) {
		printf("0x%x\n",mraa_spi_write(spi, 0xaa));
	}
err_exit:
	mraa_result_print(status);

	/* stop spi */
	mraa_spi_stop(spi);

	/* deinitialize mraa for the platform (not needed most of the times) */
	mraa_deinit();

	return EXIT_FAILURE;
}
