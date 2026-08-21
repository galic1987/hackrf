/*
 * Copyright 2025-2026 Great Scott Gadgets <info@greatscottgadgets.com>
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <hackrf.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

static void usage()
{
	printf("hackrf_pro - High-level control of HackRF Pro FPGA features\n");
	printf("Usage:\n");
	printf("\t[-d serial_number] # Serial number of desired HackRF Pro.\n");
	printf("\t[--dc-block on|off] # Enable/disable FPGA DC block filter.\n");
	printf("\t[--decimation N] # RX decimation log2: 0=÷1, 1=÷2, 2=÷4, 3=÷8, 4=÷16, 5=÷32\n");
	printf("\t[--interpolation N] # TX interpolation log2: 0=÷1, 1=÷2, 2=÷4, 3=÷8\n");
	printf("\t[--quarter-shift up|down|none] # Digital spectrum shift by ±Fs/4.\n");
	printf("\t[--nco-freq HZ] # TX NCO offset in Hz (0 disables). Firmware computes\n");
	printf("\t                 # the phase step from the actual DAC clock.\n");
	printf("\t[--clock-corr PPM] # Reference clock correction in PPM (±10000).\n");
	printf("\t[--read-reg ADDR] # Read raw FPGA register (debug; bypasses radio management).\n");
	printf("\t[--write-reg ADDR VAL] # Write raw FPGA register (debug; bypasses radio management).\n");
	printf("\t[-h] # This help.\n");
	printf("\nSettings made via the managed options survive sample-rate and frequency\n");
	printf("changes. Raw register access is for debugging and may be overwritten by\n");
	printf("the firmware's radio configuration management.\n");
}

static int parse_on_off(const char* s, bool* out)
{
	if (strcmp(s, "on") == 0 || strcmp(s, "1") == 0) {
		*out = true;
		return HACKRF_SUCCESS;
	}
	if (strcmp(s, "off") == 0 || strcmp(s, "0") == 0) {
		*out = false;
		return HACKRF_SUCCESS;
	}
	return HACKRF_ERROR_INVALID_PARAM;
}

static int parse_shift(const char* s, uint8_t* out)
{
	if (strcmp(s, "none") == 0) {
		*out = HACKRF_QUARTER_SHIFT_NONE;
		return HACKRF_SUCCESS;
	}
	if (strcmp(s, "up") == 0) {
		*out = HACKRF_QUARTER_SHIFT_UP;
		return HACKRF_SUCCESS;
	}
	if (strcmp(s, "down") == 0) {
		*out = HACKRF_QUARTER_SHIFT_DOWN;
		return HACKRF_SUCCESS;
	}
	return HACKRF_ERROR_INVALID_PARAM;
}

static const char* shift_name(uint8_t mode)
{
	switch (mode) {
	case HACKRF_QUARTER_SHIFT_UP:
		return "up";
	case HACKRF_QUARTER_SHIFT_DOWN:
		return "down";
	default:
		return "none";
	}
}

int main(int argc, char** argv)
{
	int opt;
	int result = HACKRF_SUCCESS;
	const char* serial_number = NULL;
	hackrf_device* device = NULL;
	bool do_dc_block = false;
	bool dc_block = false;
	bool do_decimation = false;
	long decimation = 0;
	bool do_interpolation = false;
	long interpolation = 0;
	bool do_quarter_shift = false;
	uint8_t quarter_shift = 0;
	bool do_nco = false;
	int64_t nco_freq = 0;
	bool do_clock_corr = false;
	double clock_corr = 0.0;
	bool do_read_reg = false;
	uint8_t read_reg_addr = 0;
	bool do_write_reg = false;
	uint8_t write_reg_addr = 0;
	uint8_t write_reg_val = 0;

	static struct option long_options[] = {
		{"dc-block", required_argument, 0, 1},
		{"decimation", required_argument, 0, 2},
		{"interpolation", required_argument, 0, 3},
		{"quarter-shift", required_argument, 0, 4},
		{"nco-freq", required_argument, 0, 5},
		{"read-reg", required_argument, 0, 6},
		{"write-reg", required_argument, 0, 7},
		{"clock-corr", required_argument, 0, 8},
		{"device", required_argument, 0, 'd'},
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0},
	};

	while ((opt = getopt_long(argc, argv, "d:h", long_options, NULL)) != EOF) {
		result = HACKRF_SUCCESS;
		switch (opt) {
		case 1:
			result = parse_on_off(optarg, &dc_block);
			do_dc_block = true;
			break;
		case 2:
			decimation = strtol(optarg, NULL, 10);
			do_decimation = true;
			break;
		case 3:
			interpolation = strtol(optarg, NULL, 10);
			do_interpolation = true;
			break;
		case 4:
			result = parse_shift(optarg, &quarter_shift);
			do_quarter_shift = true;
			break;
		case 5:
			nco_freq = strtoll(optarg, NULL, 10);
			do_nco = true;
			break;
		case 6:
			read_reg_addr = (uint8_t) strtoul(optarg, NULL, 0);
			do_read_reg = true;
			break;
		case 7:
			write_reg_addr = (uint8_t) strtoul(optarg, NULL, 0);
			write_reg_val = (uint8_t) strtoul(argv[optind], NULL, 0);
			optind++;
			do_write_reg = true;
			break;
		case 8:
			clock_corr = strtod(optarg, NULL);
			do_clock_corr = true;
			break;
		case 'd':
			serial_number = optarg;
			break;
		case 'h':
		case '?':
			usage();
			return EXIT_SUCCESS;
		default:
			fprintf(stderr, "unknown argument\n");
			usage();
			return EXIT_FAILURE;
		}
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr, "argument error\n");
			usage();
			return EXIT_FAILURE;
		}
	}

	result = hackrf_init();
	if (result != HACKRF_SUCCESS) {
		fprintf(stderr,
			"hackrf_init() failed: %s (%d)\n",
			hackrf_error_name(result),
			result);
		return EXIT_FAILURE;
	}

	result = hackrf_open_by_serial(serial_number, &device);
	if (result != HACKRF_SUCCESS) {
		fprintf(stderr,
			"hackrf_open() failed: %s (%d)\n",
			hackrf_error_name(result),
			result);
		hackrf_exit();
		return EXIT_FAILURE;
	}

	{
		uint8_t board_id = BOARD_ID_UNDETECTED;
		result = hackrf_board_id_read(device, &board_id);
		if (result != HACKRF_SUCCESS || board_id != BOARD_ID_PRALINE) {
			fprintf(stderr,
				"hackrf_pro only supports HackRF Pro (praline) boards.\n");
			hackrf_close(device);
			hackrf_exit();
			return EXIT_FAILURE;
		}
	}

	if (do_dc_block) {
		result = hackrf_set_dc_block(device, dc_block);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"DC block failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			printf("DC block %s\n", dc_block ? "enabled" : "disabled");
		}
	}

	if (do_decimation) {
		result = hackrf_set_rx_decimation(device, (uint8_t) decimation);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Decimation failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			uint8_t applied = 0;
			hackrf_get_rx_decimation(device, &applied);
			printf("RX decimation set to ÷%u (applied log2=%u)\n",
			       1u << applied,
			       applied);
		}
	}

	if (do_interpolation) {
		result = hackrf_set_tx_interpolation(device, (uint8_t) interpolation);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Interpolation failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			uint8_t applied = 0;
			hackrf_get_tx_interpolation(device, &applied);
			printf("TX interpolation set to ÷%u (applied log2=%u)\n",
			       1u << applied,
			       applied);
		}
	}

	if (do_quarter_shift) {
		result = hackrf_set_quarter_shift(device, quarter_shift);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Quarter-shift failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			uint8_t applied = 0;
			hackrf_get_quarter_shift(device, &applied);
			printf("Quarter-shift set to %s (applied: %s)\n",
			       shift_name(quarter_shift),
			       shift_name(applied));
		}
	}

	if (do_nco) {
		result = hackrf_set_tx_nco(device, nco_freq);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"NCO setup failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			int64_t applied = 0;
			hackrf_get_tx_nco(device, &applied);
			printf("TX NCO offset set to %lld Hz (applied: %lld Hz)\n",
			       (long long) nco_freq,
			       (long long) applied);
		}
	}

	if (do_clock_corr) {
		result = hackrf_set_clock_correction(device, clock_corr);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Clock correction failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			double applied = 0.0;
			hackrf_get_clock_correction(device, &applied);
			printf("Clock correction set to %.2f ppm (applied: %.2f ppm)\n",
			       clock_corr,
			       applied);
		}
	}

	if (do_read_reg) {
		uint8_t value;
		result = hackrf_fpga_read_register(device, read_reg_addr, &value);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Register read failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			printf("FPGA reg[%u] = 0x%02x\n", read_reg_addr, value);
		}
	}

	if (do_write_reg) {
		fprintf(stderr,
			"warning: raw register writes bypass radio management and may be overwritten\n");
		result = hackrf_fpga_write_register(device, write_reg_addr, write_reg_val);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr,
				"Register write failed: %s (%d)\n",
				hackrf_error_name(result),
				result);
		} else {
			printf("FPGA reg[%u] = 0x%02x\n", write_reg_addr, write_reg_val);
		}
	}

	hackrf_close(device);
	hackrf_exit();
	return EXIT_SUCCESS;
}
