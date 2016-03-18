#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "limits.h"
#include "PCAP.h"


static uint32_t cap_t[8]; /* Raw capacitance values */
static uint8_t tx_data[8]; /*!< SPI TX buffer */
static uint8_t rx_data[8]; /*!< SPI RX buffer */
static uint8_t MSG_LEN;
static uint8_t prgdata[4096] = {0x0, 0x0, 0x0, 0x62, 0x63, 0x0, 0x65, 0xBE, 0x1, 0x20, 0x26, 0x42, 0x5C, 0x48, 0xA0, 0x3, 0x21, 0xE4, 0x20, 0x31, 0xA1, 0x3, 0x21, 0xE4, 0x20, 0x31, 0x84, 0x1, 0x23, 0x63, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0B, 0x43, 0x58, 0xC0, 0xFE, 0x43, 0xC0, 0x44, 0x7A, 0x7E, 0x20, 0x0B, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0xC0, 0xC0, 0xC0, 0xF6, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x41, 0x23, 0x94, 0xD0, 0x43, 0xEE, 0x44, 0xD2, 0x43, 0xEF, 0x44, 0x20, 0x5A, 0x70, 0x60, 0x71, 0x61, 0x78, 0x68, 0x2, 0x7A, 0xF3, 0x43, 0xC7, 0xFE, 0x41, 0xEB, 0x45, 0x5A, 0x21, 0xDF, 0x46, 0x46, 0x46, 0x46, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0x55, 0xED, 0x45, 0xEC, 0x51, 0xF4, 0x41, 0x23, 0x88, 0xEA, 0x45, 0xF5, 0x41, 0x23, 0x88 , 0xE9, 0x45, 0x1D, 0x41, 0x43, 0x58, 0xEA, 0x21, 0x99, 0xE9, 0x50, 0x46, 0xEB, 0x44, 0xA9, 0x2, 0xEB, 0x59, 0x43, 0xCA, 0xFE, 0x41, 0x5C, 0xA8, 0x3, 0xC0, 0x5A, 0xEB, 0x45, 0xEB, 0x41, 0xF2, 0x45, 0xF6, 0x41, 0x23, 0x88, 0xEA, 0x45, 0xF7, 0x41, 0x23, 0x88, 0xE9, 0x45, 0x1F, 0x41, 0x43, 0x58, 0xEA, 0x21, 0x99, 0xE9, 0x50, 0x46, 0xEB, 0x44, 0xA9, 0x2, 0xEB, 0x59, 0x43, 0xCA, 0xFE, 0x41, 0x5C, 0xA8, 0x3, 0xC0, 0x5A, 0xEB, 0x45, 0xEB, 0x41, 0xF3, 0x45, 0x2, 0xFF, 0xFF, 0xFF, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x7C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x6C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x6C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x2, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x2, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x2, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x2, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x2, 0x6A, 0xFD, 0x43, 0x40, 0x4F, 0x4F, 0x4F, 0xEB, 0x45, 0x7A, 0xF9, 0x41, 0x43, 0x58, 0xEB, 0x21, 0x99, 0xEA, 0x44, 0xC0, 0xC0, 0xC0, 0xF1, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x41, 0xED, 0x45, 0xC0, 0x41, 0xC0, 0xC0, 0xC0, 0xF8, 0xFF, 0x43, 0xE9, 0x44, 0x6A, 0x1D, 0x43, 0xAB, 0x1, 0xEA, 0x58, 0x8E, 0x3, 0xEC, 0x53, 0x1D, 0x50, 0x1F, 0x44, 0xEC, 0x53, 0xED, 0x53, 0xE9, 0x43, 0xEC, 0x58, 0xAC, 0xE6, 0x8E, 0x26, 0xC0, 0xC0, 0xC0, 0xF9, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0xC0, 0x41, 0xC0, 0xC0, 0xC0, 0xFC, 0xFF, 0x43, 0xE9, 0x44, 0x1D, 0x43, 0x1F, 0x59, 0xE9, 0x43, 0xED, 0x53, 0xEC, 0x53, 0x58, 0xAC, 0xF2, 0x7A, 0xC0, 0xC0, 0xC0, 0xC9, 0xFF, 0x43, 0xEC, 0x44, 0xE7, 0x44, 0xE8, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0x1F, 0x43, 0x4E, 0x4E, 0x4E, 0x44, 0xC0, 0xC0, 0xC0, 0xCF, 0xFF, 0x43, 0xE9, 0x44, 0x8E, 0x7, 0xC0, 0xC0, 0xC0, 0xCB, 0xFF, 0x43, 0xE9, 0x44, 0x40, 0x5D, 0x1D, 0x43, 0x1F, 0x21, 0xCA, 0xE8, 0x43, 0xEC, 0x44, 0x1D, 0x45, 0xF8, 0x43, 0xAB, 0x0C, 0xC0, 0x41, 0xED, 0x53, 0x53, 0x1F, 0x43, 0x4E, 0x4E, 0x4E, 0x44, 0xE7, 0x53, 0xC0, 0x41, 0xE8, 0x53, 0xE7, 0x53, 0x41, 0xEC, 0x45, 0xE9, 0x43, 0x5C, 0xAC, 0xD3, 0xC0, 0xC0, 0xC0, 0xCF, 0xFF, 0x43, 0xE9, 0x44, 0xE8, 0x41, 0xE9, 0x43, 0x5C, 0xA8, 0x0C, 0xC0, 0x41, 0xE8, 0x43, 0x53, 0xEC, 0x44, 0x1D, 0x44, 0x59, 0x43, 0xAB, 0xEB, 0xC8, 0x43, 0x46, 0x46, 0x46, 0x44, 0x7A, 0x8A, 0x1B, 0xC0, 0x43, 0x40, 0x5D, 0x5D, 0x90, 0x15, 0xC8, 0x45, 0xC9, 0x45, 0xF8, 0x43, 0xAA, 0x0B, 0xCA, 0x45, 0xCB, 0x45, 0xCC, 0x45, 0xCD, 0x45, 0xCE, 0x45, 0xCF, 0x45, 0x0, 0x2, 0xC0, 0x43, 0x4E, 0x4E, 0xEA, 0x44, 0xE9, 0x44, 0x8E, 0x6, 0xC0, 0x43, 0x4E, 0xEA, 0x44, 0xE9, 0x44, 0xF8, 0x43, 0xAB, 0x13, 0xC0, 0x43, 0x4E, 0xEA, 0x44, 0x4E, 0x50, 0xE9, 0x44, 0x8E, 0x8, 0xC0, 0x43, 0xEA, 0x44, 0x4E, 0x4E, 0x50, 0xE9, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0x4E, 0x4E, 0xE9, 0x51, 0x91, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x92, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x93, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x94, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x95, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x96, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x97, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x2, 0xE9, 0x43, 0x46, 0x46, 0xEC, 0x44, 0x1D, 0x45, 0x2, 0x7A, 0xFA, 0x41, 0x4F, 0x4F, 0x4F, 0xE7, 0x45, 0x5A, 0xFB, 0x43, 0xE7, 0x75, 0x21, 0xCA, 0x65, 0xD0, 0x45, 0x5A, 0xFC, 0x43, 0xE7, 0x21, 0xCA, 0xD1, 0x45, 0x5A, 0xFD, 0x43, 0xE7, 0x21, 0xCA, 0xD2, 0x45, 0x79, 0x69, 0x2, 0xD7, 0xFE, 0x43, 0xE7, 0x45, 0x5D, 0xAD, 0x1, 0x5D, 0x45, 0x41, 0x2, 0x1F, 0x43, 0x1D, 0x44, 0xC0, 0x43, 0xEC, 0x51, 0xED, 0x51, 0x5D, 0xAA, 0xF2, 0x2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2, 0x1, 0x3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
int rtc_flag = 0;
	/***** Bit Packing function to generate 32 bit function ****
 * @param a, b, c, d, e : Variables containing the bits to be combined
 * @param nb, nc, nd, ne : Position of the bits to be combined
 * @retval p 32bit integer to be returned
 */

uint32_t pack(uint32_t a, uint32_t b, uint8_t nb, uint32_t c, uint8_t nc, uint32_t d, uint8_t nd, uint32_t e, uint8_t ne)
    {
        uint32_t p = a;
        p = (p << nb)| b ;
        p = (p << nc)| c ;
        p = (p << nd)| d ;
        p = (p << ne)| e ;
        return p;
    }


/***** Transmits/Recieves data using SPI. ****
 * @param PCAP_spi_address Pointer containing the set spi address 
 * @param MSG_LEN length of the message to be transmitted
 * @param tx_data pointer containing the transmitted data
 * @param rx_data pointer containing the recieved data
 * @retval True 
 * @retval false Error occurred
 */
bool pcap_spi_tx_rx(uint32_t *PCAP_spi_address, uint8_t MSG_LEN, uint8_t *tx_data)
{
    bool PCAP_spi_chk = spi_master_tx_rx(PCAP_spi_address, MSG_LEN, (const uint8_t *)tx_data, (uint8_t *)rx_data);
  
  if (PCAP_spi_chk)
    return true;
    else 
        return false;
}

/**** PCAP DSP SPI write function ****
*
*
*/
bool pcap_dsp_write(uint32_t *PCAP_spi_address) 
{
    //uint8_t prgdata[4096];
    uint16_t regadd = 0;
    uint16_t x;
    uint8_t y;
    bool b;
    uint32_t p;
    MSG_LEN = 4;

    //standard measurement DSP file 
    //ANT chip crashes need better implementation.
        for (x = 0; x < 4096; x++)
            {
                regadd = x;                                 
                p = 0x09;      // Add SRAM write code
                p = (p << 12)|regadd;   // add SRAM Address 
                p = (p << 8)|prgdata[x];  // add program data
                
                // segmentation of into 3 segments of 8 bit variables and add to data packet
                for (y = 0; y < (3); y++)
                    {
                        tx_data[y] = (p >> ((2)-y)*CHAR_BIT)&(0xFF);
                    }
                b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); // test submission
                memset(tx_data, 0, 8);
                memset(rx_data, 0, 8);
                
            }
            return b;
}
    
/**** PCAP config register SPI write function ***
* @param PCAP_spi_address:  SPI address set by the SPI Set function 
* @param regdata: Array of 24 bit data containing the register values in sequential order from (0-9)
* @retval false for unsucessful set
*/
    
bool pcap_config_write(uint32_t *PCAP_spi_address, uint32_t *regdata) // why does the pointer work here
    {
        //Variable declaration & initialisation 
        // uint8_t n = 0;
        uint8_t regadd = 0;
				uint8_t x, y;
        uint32_t p;
        bool b; 
        
        MSG_LEN = 4;

        /*run bit configuration (register 20) */ 
        /* set run bit   = 0 for configuration set*/
        memset(tx_data, 0, 8);
				memset(rx_data, 0, 8);
        p = 0x03; // Add write code
        p = (p << 6)|20; // add Address 
        p = (p << 24)|(0); // add data
        for (y = 0; y < (4); y++)
            {
                tx_data[y] = (p >> ((3)-y)*CHAR_BIT) & (0xFF) ;
            }
        b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);

        /* Configuration registers (Registers 0-19) write loop */
        for (x = 0; x < 20; x++)
            {
                regadd = x;                                 
                p = 0x03;      // Add write code
                p = (p << 6)|regadd;   // add Address 
                p = (p << 24)|regdata[x];  // add data
                
                // segmentation of into 8 bit variables and add to data packet
                for (y = 0; y < (4); y++)
                    {
                        tx_data[y] = (p >> ((3)-y)*CHAR_BIT)&(0xFF) ;
                    }
                b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); // test submission
                // need to implement check function 
                //b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
                //nrf_delay_ms(DELAY_MS);   
                memset(tx_data, 0, 8);
                memset(rx_data, 0, 8);
                //n = n + 4; // next segmentation incremetn
            }
            //nrf_delay_ms(100); 
						
			NRF_RTC1->CC[0] = 3276; 
			rtc_flag = 1;
			NRF_RTC1->TASKS_START = 1;
			do 
			{ 
				// Enter System ON sleep mode 
				__WFE();   
				// Make sure any pending events are cleared 
				__SEV(); 
				__WFE();                 
			}while(rtc_flag); 
			
			NRF_RTC1->TASKS_STOP = 1; 
			NRF_RTC1->TASKS_CLEAR = 1; 

            //run bit confiuration i.e register 20
            p = 0x03; // Add write code
            p = (p << 6)|20; // add Address 
            p = (p << 24)|(1); // add data
            for (y = 0; y < (4); y++)
                {
                    tx_data[y] = (p >> ((3)-y)*CHAR_BIT) & (0xFF) ;
                    b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
                }
        b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
        //nrf_delay_ms(DELAY_MS);   
        // need to implement check function 
        return b;
    }
    
/**** PCAP config register set function ***
    * @PCAP_spi_address:  SPI address set by the SPI Set function 
  * Sets the indivual configuration registers and send it to the PCAP_config_write function for SPI write.
    * Return false for unsucessful set
*/
bool config_reg_set(uint32_t *PCAP_spi_address,int c_avg,int onoff, int cy_time, int rdc_sel) 
    { 
        uint32_t config_reg_d[20];
        uint8_t DSP_PRESET, PG_PRESET;
        bool w; 
        
        /* register 0 */
        config_reg_d[0] = pack((0x04), (MEMCOMP << 2)|2, 4, ECC_MODE, 8, AUTOBOOT_DIS, 4, MEM_LOCK_DIS,4);
                
        /* register 1 */
        config_reg_d[1] = 0x201022;
        
        /* register 2 */
        config_reg_d[2] = pack(CMEAS_PORT_EN, CMEAS_BITS, 4, rdc_sel, 4, 0x0B, 8, 0, 0);
        
        /* register 3 */
        config_reg_d[3] = pack(CY_CLK_SEL, SEQ_TIME, 6, CMEAS_FAKE, 3, c_avg, 13 , 0, 0);
        
        /* register 4 */
        config_reg_d[4] = pack(CMEAS_STARTPIN, CMEAS_TRIG_SEL, 2, cy_time, 10, TMEAS_CYTIME, 4, ((TMEAS_STARTPIN << 2)|onoff), 4);
        
        /* register 5 */
        config_reg_d[5] = pack(T_AVRG, TMEAS_TRIG_PREDIV, 22, 0, 0, 0, 0, 0 ,0);
        //config_reg_d[5] = 0x000000
            
        /* register 6 */
        config_reg_d[6] = pack(0, TMEAS_FAKE, 1, TMEAS_7BITS, 7, 0x40, 8, 0, 0);
        //config_reg_d[6] = 0x000040 ;
        
        /* register 7 */
        config_reg_d[7] = 0x1F0000 ;
        
        /* register 8 */
        DSP_PRESET = pack(DSP_SRAM_SEL, DSP_START, 1, DSP_STARTONOVL, 1, DSP_STARTONTEMP, 1, DSP_STARTPIN, 4);
        PG_PRESET = pack(INT2PG2, PG1_X_G3, 1, PG0_X_G2, 1, 0, 0, 0, 0);
        
        config_reg_d[8] = pack(DSP_PRESET, DSP_FF_IN, 4, (DSP_WATCHDOG_LENGTH << 2)|DSP_MOFLO_EN, 4, DSP_SPEED, 4, PG_PRESET, 4);
        //config_reg_d[8] = 0x800030 ;
        
        /* register 9 */
        config_reg_d[9] = pack(PG_DIR_IN, PG_PULL_UP, 4, PI_EN, 4, (PI1_CLK_SEL << 4)| PI0_CLK_SEL, 8, (PI1_RES << 2) | PI0_RES, 4); 
        //config_reg_d[9] = 0xFF000F ; 
        
        /* register 10 */
        config_reg_d[10] = pack(0x18, 00, 8, V_CORE_CTL, 8,0,0,0,0);
        //config_reg_d[10] = 0x180087;
        
        /* param register 11 */
        config_reg_d[11] = 0x000000 ;
        
        /* param register 12 */
        config_reg_d[12] = 0x000000 ;
        
        /* param register 13*/
        config_reg_d[13] = 0x000000 ;
        
        /* param register 14*/
        config_reg_d[14] = 0x000000 ;
        
        /* param register 15*/
        config_reg_d[15] = 0x000000 ;

        /* param register 16*/
        config_reg_d[16] = 0x000000 ;
        
        /* param register 17*/
        config_reg_d[17] = 0x000000 ;
        
        /* param register 18*/
        config_reg_d[18] = 0x000000 ;
        
        /* param register 19*/
        config_reg_d[19] = 0x200000 ; //gain correction register 
                
        w = pcap_config_write(PCAP_spi_address, config_reg_d);
        return w;
    }

    
/**** PCAP read register set function ***
    * Speifies the register to be read and send the command to the PCAP sensor
    * @param PCAP_spi_address:  SPI address set by the SPI Set function 
  * @param read register address see definition list fore register address
    * @retval Return false for unsucessful set or read
    * @retval Return register data for succesful set
*/

uint32_t read_reg(uint32_t *PCAP_spi_address, uint8_t wr_reg_add) 
    { 
        bool meas;
        uint32_t rx_reg_data;
        uint8_t p;
        
        MSG_LEN = 4; //Length of message to be transmitted
    
        // Read Register address 
        p = 0x01;      // Read code
        p = (p << 6)|wr_reg_add;   // Address 
        tx_data[0] = p; 
        
        //nrf_delay_ms(100);
        meas = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); //Intiate the read of capacitance data
        
        rx_reg_data = pack(rx_data[1], rx_data[2], 8, rx_data[3], 8, 0, 0, 0, 0 );
            
        //Return capacitance data
        if (meas == 0)
            return meas;
        else
            return rx_reg_data;
				memset(tx_data, 0, 8);
				memset(rx_data, 0, 8);
    }

/**** PCAP data extraction function ***
    * Extract the recieved read register data and converts it to decimal nature
    * @param data: data obtained from the read register 
    * @retval ext capacitance data for succesful set
*/ 
float data_extract(uint32_t data)
{
    //int p = (data >> 21) & 7;
    //uint32_t d = data & 0x1FFFFF;
    //float ext = (float) p + (float)(d/10000000);
    float p = data;
    float ext= p/(pow(2,21));
    return ext;
}

/**** PCAP communication check ***
    * @PCAP_spi_address:  SPI address set by the SPI Set function 
		* Checks the DSP of teh PCAP to ensure a sucessful SPI communication/DSP program in the PCAP.
    * Return false for unsucessful set
*/
bool pcap_commcheck(uint32_t *PCAP_spi_address)
{
		bool w; 
		/* SPI communication check */
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x10;	
		tx_data[1] = 0x08;				
	
		w = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
	
		if (rx_data[2] == 0x01)
		{
			return true;
		}
		else
		{
			return false;
		}
		
}

/**** PCAP config function ***
    * @PCAP_spi_address:  SPI address set by the SPI Set function 
		* Initiates configuration Register set and implements a partial reset.
    * Return false for unsucessful set
*/
bool 	pcap_config(uint32_t *PCAP_spi_address, int c_avg,int onoff, int cy_time, int rdc_sel)
{
		bool w,w1,w2; 
		/* Set configuration registers */
		w1 = config_reg_set(PCAP_spi_address, c_avg, onoff, cy_time, rdc_sel);
		
		/* Send a partial reset */
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x8A; // Partial Reset 
		
		//nrf_delay_ms(DELAY_MS);
		w2 = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
	
		if (w1 && w2 == 1)
		{
			w = true;
			return w ;
		}
		else
		{
			w = false;
			return w = false;
		}
}
	
/**** PCAP measure function ***
    * @PCAP_spi_address:  SPI address set by the SPI Set function 
		* Initiates measures and implements minimum delay before read.
    * Return false for unsucessful start
*/
bool 	pcap_measure(uint32_t *PCAP_spi_address,int c_avg, int onoff, int cy_time)
{
	bool w;
	uint8_t cap_n, n, pul_n;
	/* Start Capacitance Measurement */
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x8C; // Start Command 
		
		//nrf_delay_ms(DELAY_MS);
		w = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
		/* Measurement Delay */
		//nrf_delay_ms(1000);
		//rexamine this function Hope this works
		switch(CMEAS_BITS)
			{
				case 1: // grounded
					n = 1;
					cap_n = 1;
					do
					{
						uint16_t chk = (CMEAS_PORT_EN >> n) & 0x01;					
						if(chk == 1) 
						{
							cap_n++;
						}
							n++;
					}while(n < 8);
					pul_n = cap_n*2 + 1;
					break;
				case 4: // floating
					n = 1;
					cap_n = 1;
					do
					{
						uint16_t chk = (CMEAS_PORT_EN >> 2*n) & 0x03;

						/* Check for transmission and no of capacitors to be read*/
						if(chk == 3) 
						{
							cap_n++;
						}
						n++;
					} while(n < 4);
					pul_n = cap_n*3 + 1;
					/* Flag to ensure all capacitors values have been transmitted*/
					break;
			}
		switch(onoff)
		{
			case 0:
			//nrf_delay_ms((cap_n*C_AVRG*0.02)+200);
			//float time = (((float)cap_n*(float)((float)c_avg*CMEAS_CYTIME)*0.02)+2000)/1000;
			float time = ((float)pul_n*(float)((float)(cy_time+1)*0.02*(float)c_avg)+250)/1000; //used to be 5
			NRF_RTC1->CC[0] = time*32768; 
			rtc_flag = 1;
			NRF_RTC1->TASKS_START = 1;
			do 
			{ 
				// Enter System ON sleep mode 
				__WFE();   
				// Make sure any pending events are cleared 
				__SEV(); 
				__WFE();                 
			}while(rtc_flag); 
			
			NRF_RTC1->TASKS_STOP = 1; 
			NRF_RTC1->TASKS_CLEAR = 1; 
			break;
			
			case 1:
			//nrf_delay_ms((cap_n*c_avg*0.02) +(0.14*2*4) + 200); //4fold averaging harcoded //
			//need to edit the line below
			//time = ((float)pul_n*(float)(CMEAS_CYTIME*0.02*c_avg)+ 10 + (0.14*2*4))/1000;
			time = ((float)pul_n*(float)((float)(cy_time+1)*0.02*(float)c_avg)+250)/1000; //used to be 5
			NRF_RTC1->CC[0] = time*32768; 
			rtc_flag = 1;
			NRF_RTC1->TASKS_START = 1;
			do 
			{ 
				// Enter System ON sleep mode 
				__WFE();   
				// Make sure any pending events are cleared 
				__SEV(); 
				__WFE();                 
			}while(rtc_flag); 
			
			NRF_RTC1->TASKS_STOP = 1; 
			NRF_RTC1->TASKS_CLEAR = 1; 
			break;			
		}
		return w;
}
