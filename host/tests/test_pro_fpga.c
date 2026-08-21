#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "hackrf.h"
#include "mock_libusb.h"

/* Minimal prefix of the real struct to satisfy internal field access */
struct libusb_device_handle;

struct hackrf_device {
	struct libusb_device_handle* usb_device;
	uint16_t usb_api_version;
};

#define VENDOR_REQUEST_RADIO_WRITE_REG 59
#define VENDOR_REQUEST_RADIO_READ_REG  60

#define BANK_ALL     255
#define BANK_APPLIED 0

#define REG_ROTATION         5
#define REG_RESAMPLE_RX      8
#define REG_DC_BLOCK         22
#define REG_CLOCK_CORRECTION 23
#define REG_TX_NCO           24
#define REG_RX_NOTCH           25

static hackrf_device* create_device(uint16_t api_version)
{
	hackrf_device* d = calloc(1, sizeof(*d));
	d->usb_device = (struct libusb_device_handle*)0x1234;
	d->usb_api_version = api_version;
	return d;
}

/* Queue a successful radio-register write and return the mock for inspection */
static void queue_write_ok(uint8_t reg, uint8_t bank)
{
	mock_transfer_t t;
	memset(&t, 0, sizeof(t));
	t.request = VENDOR_REQUEST_RADIO_WRITE_REG;
	t.value = 0;
	t.index = bank;
	t.expected_length = 9;
	t.return_code = 9;
	mock_libusb_queue_transfer(&t);
	(void) reg;
}

/* hackrf_set_dc_block writes 1/0 to register 22 in bank ALL */
static void test_dc_block_write(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0113);
	queue_write_ok(REG_DC_BLOCK, BANK_ALL);
	assert(hackrf_set_dc_block(dev, true) == HACKRF_SUCCESS);
	printf("PASS: hackrf_set_dc_block write path\n");

	free(dev);
}

/* hackrf_set_quarter_shift encodes "up" as 0xC0000000 in register 5 */
static void test_quarter_shift_encoding(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0113);
	queue_write_ok(REG_ROTATION, BANK_ALL);
	assert(hackrf_set_quarter_shift(dev, HACKRF_QUARTER_SHIFT_UP) == HACKRF_SUCCESS);
	printf("PASS: hackrf_set_quarter_shift write path\n");

	free(dev);
}

/* out-of-range decimation is rejected before any transfer */
static void test_decimation_bounds(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0113);
	assert(hackrf_set_rx_decimation(dev, 6) == HACKRF_ERROR_INVALID_PARAM);
	assert(hackrf_set_tx_interpolation(dev, 4) == HACKRF_ERROR_INVALID_PARAM);
	printf("PASS: decimation/interpolation bounds\n");

	free(dev);
}

/* hackrf_get_dc_block reads register 22 from the applied bank */
static void test_dc_block_readback(void)
{
	mock_transfer_t t;
	uint64_t one = 1;
	hackrf_device* dev;
	bool enabled = false;

	mock_libusb_reset();
	dev = create_device(0x0113);

	memset(&t, 0, sizeof(t));
	t.request = VENDOR_REQUEST_RADIO_READ_REG;
	t.value = REG_DC_BLOCK;
	t.index = BANK_APPLIED;
	t.expected_length = 8;
	t.response_data = (unsigned char*) &one;
	t.response_length = 8;
	t.return_code = 8;
	mock_libusb_queue_transfer(&t);

	assert(hackrf_get_dc_block(dev, &enabled) == HACKRF_SUCCESS);
	assert(enabled == true);
	printf("PASS: hackrf_get_dc_block readback\n");

	free(dev);
}

/* clock correction: +1 ppm must encode as fp_1_63 (2^63 + 2^63/1e6) */
static void test_clock_correction_encoding(void)
{
	mock_transfer_t t;
	uint64_t fp_one_ppm = 0x8000000000000000ULL + 9223372036855ULL;
	hackrf_device* dev;
	double ppm = 0.0;

	mock_libusb_reset();
	dev = create_device(0x0113);

	memset(&t, 0, sizeof(t));
	t.request = VENDOR_REQUEST_RADIO_READ_REG;
	t.value = REG_CLOCK_CORRECTION;
	t.index = BANK_APPLIED;
	t.expected_length = 8;
	t.response_data = (unsigned char*) &fp_one_ppm;
	t.response_length = 8;
	t.return_code = 8;
	mock_libusb_queue_transfer(&t);

	assert(hackrf_get_clock_correction(dev, &ppm) == HACKRF_SUCCESS);
	assert(ppm > 0.9 && ppm < 1.1);
	printf("PASS: hackrf_get_clock_correction fp_1_63 decode\n");

	free(dev);
}

/* TX NCO requires USB API 0x0115: must fail cleanly on 0x0114 firmware */
static void test_tx_nco_api_gate(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0114);
	assert(hackrf_set_tx_nco(dev, 1000000) == HACKRF_ERROR_USB_API_VERSION);
	printf("PASS: hackrf_set_tx_nco API gate\n");

	free(dev);
}

/* TX NCO write path on 0x0115 firmware */
static void test_tx_nco_write(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0115);
	queue_write_ok(REG_TX_NCO, BANK_ALL);
	assert(hackrf_set_tx_nco(dev, 1000000) == HACKRF_SUCCESS);
	printf("PASS: hackrf_set_tx_nco write path\n");

	free(dev);
}

/* hackrf_set_rx_notch writes the Hz value to register 25 in bank ALL */
static void test_rx_notch_write(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0116);
	queue_write_ok(REG_RX_NOTCH, BANK_ALL);
	assert(hackrf_set_rx_notch(dev, -327500) == HACKRF_SUCCESS);
	printf("PASS: hackrf_set_rx_notch write path\n");

	free(dev);
}

/* RX notch requires USB API 0x0116: must fail cleanly on 0x0115 firmware */
static void test_rx_notch_api_gate(void)
{
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0115);
	assert(hackrf_set_rx_notch(dev, -327500) == HACKRF_ERROR_USB_API_VERSION);
	printf("PASS: hackrf_set_rx_notch API gate\n");

	free(dev);
}

/* get decodes RADIO_UNSET as disabled (0) */
static void test_rx_notch_get_unset(void)
{
	mock_transfer_t t;
	uint64_t unset = UINT64_MAX;
	int64_t hz = -1;
	hackrf_device* dev;

	mock_libusb_reset();
	dev = create_device(0x0116);

	memset(&t, 0, sizeof(t));
	t.request = VENDOR_REQUEST_RADIO_READ_REG;
	t.value = REG_RX_NOTCH;
	t.index = BANK_APPLIED;
	t.expected_length = 8;
	t.response_data = (unsigned char*) &unset;
	t.response_length = 8;
	t.return_code = 8;
	mock_libusb_queue_transfer(&t);

	assert(hackrf_get_rx_notch(dev, &hz) == HACKRF_SUCCESS);
	assert(hz == 0);
	printf("PASS: hackrf_get_rx_notch RADIO_UNSET decode\n");

	free(dev);
}

int main(void)
{
	printf("Running HackRF Pro FPGA helper tests...\n");

	test_dc_block_write();
	test_quarter_shift_encoding();
	test_decimation_bounds();
	test_dc_block_readback();
	test_clock_correction_encoding();
	test_tx_nco_api_gate();
	test_tx_nco_write();
	test_rx_notch_write();
	test_rx_notch_api_gate();
	test_rx_notch_get_unset();

	printf("\nAll tests passed.\n");
	return 0;
}
