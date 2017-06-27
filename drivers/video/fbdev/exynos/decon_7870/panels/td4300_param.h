#ifndef __TD4300_PARAM_H__
#define __TD4300_PARAM_H__
#include <linux/types.h>
#include <linux/kernel.h>

#define EXTEND_BRIGHTNESS	306
#define UI_MAX_BRIGHTNESS	255
#define UI_MIN_BRIGHTNESS	4
#define UI_DEFAULT_BRIGHTNESS	110
#if defined(CONFIG_PANEL_BRIGHTNESS_MAX_480CD)
#define DDI_MIN_BRIGHTNESS 3
#define DDI_DEFAULT_BRIGHTNESS 66
#define DDI_MAX_BRIGHTNESS 180
#define DDI_OUTDOOR_BRIGHTNESS 215
#else
#define DDI_MIN_BRIGHTNESS 3
#define DDI_DEFAULT_BRIGHTNESS 83
#define DDI_MAX_BRIGHTNESS 194
#define DDI_OUTDOOR_BRIGHTNESS 238
#endif

#define LEVEL_IS_HBM(brightness)	(brightness == EXTEND_BRIGHTNESS)

struct lcd_seq_info {
	unsigned char	*cmd;
	unsigned int	len;
	unsigned int	sleep;
};

struct ISL98611_rom_data {
	u8 addr;
	u8 val;
};

#if defined(CONFIG_PANEL_BRIGHTNESS_MAX_480CD)
static const struct ISL98611_rom_data ISL98611_INIT[] = {
	{0x01, 0x00},
	{0x02, 0xBF},
	{0x03, 0x02},
	{0x04, 0x0A},
	{0x05, 0x0A},
	{0x06, 0xF4},
	{0x10, 0xFF},
	{0x11, 0x07},
	{0x12, 0xBF}, //25mA
	{0x13, 0x87},
	{0x14, 0xFD},
	{0x16, 0xF5},
	{0x17, 0x8D},
};
#else
static const struct ISL98611_rom_data ISL98611_INIT[] = {
	{0x01, 0x00},
	{0x02, 0xBF},
	{0x03, 0x02},
	{0x04, 0x0A},
	{0x05, 0x0A},
	{0x06, 0xF4},
	{0x10, 0xFF},
	{0x11, 0x07},
	{0x12, 0x3F}, //20mA
	{0x13, 0x87},
	{0x14, 0xFD},
	{0x16, 0xF5},
	{0x17, 0x8D},
};
#endif

static const unsigned char SEQ_TD4300_BLON[] = {
	0x53,
	0x0C,
};

static const unsigned char SEQ_TD4300_BL[] = {
	0x51,
	0xFF,
};

static const unsigned char SEQ_TD4300_ADDRESS[] = {
	0x36,
	0x40,
};


static const unsigned char SEQ_SLEEP_OUT[] = {
	0x11,
	0x00, 0x00
};

const unsigned char SEQ_SLEEP_IN[] = {
	0x10,
	0x00, 0x00
};

const unsigned char SEQ_DISPLAY_OFF[] = {
	0x28,
	0x00, 0x00
};

static const unsigned char SEQ_DISPLAY_ON[] = {
	0x29,
	0x00, 0x00
};

static const unsigned char SEQ_TD4300_CABC_OFF[] = {
	0x55,
	0x00,
};

static const unsigned char SEQ_TD4300_B0[] = {
	0xB0,
	0x04
};

static const unsigned char SEQ_TD4300_B3[] = {
	0xB3,
	0x31, 0x00, 0x06,
};

static const unsigned char SEQ_TD4300_B4[] = {
	0xB4,
	0x00
};

static const unsigned char SEQ_TD4300_B6[] = {
	0xB6,
	0x33, 0xD3, 0x80, 0xFF, 0xFF,
};

static const unsigned char SEQ_TD4300_BA[] = {
	0xBA,
	0x55, 0xFF, 0x50, 0x00, 0x28, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_BB[] = {
	0xBB,
	0x14, 0x14,
};

static const unsigned char SEQ_TD4300_BC[] = {
	0xBC,
	0x37, 0x32,
};

static const unsigned char SEQ_TD4300_BD[] = {
	0xBD,
	0x64, 0x32,
};

static const unsigned char SEQ_TD4300_BE[] = {
	0xBE,
	0x04
};

static const unsigned char SEQ_TD4300_C0[] = {
	0xC0,
	0x00
};

static const unsigned char SEQ_TD4300_C1[] = {
	0xC1,
	0x84, 0x08, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0x67, 0xCD, 0xB9, 0xC6, 0xFE, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x62, 0x03, 0x02, 0x03,
	0x82, 0x00, 0x01, 0x00, 0x01,
};

static const unsigned char SEQ_TD4300_C1_GOA[] = {
	0xC1,
	0x84, 0x08, 0x01, 0xFF, 0xFF, 0x2F, 0x7F, 0xFC, 0x37, 0x9B,
	0xF3, 0xFE, 0x67, 0xCD, 0xB9, 0xC6, 0xFE, 0x07, 0xC7, 0xE4,
	0xFB, 0xC5, 0x1F, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x22, 0x03, 0x02, 0x03,
	0x82, 0x00, 0x01, 0x00, 0x01,
};

static const unsigned char SEQ_TD4300_C2[] = {
	0xC2,
	0x01, 0xF7, 0x80, 0x0D, 0x6F, 0x08, 0x0C, 0x10, 0x00, 0x08,
	0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_C3[] = {
	0xC3,
	0x78, 0x77, 0x87, 0x78, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3B, 0x63, 0xB6, 0x3C, 0x30, 0x3F, 0xC3, 0xFC, 0x3C, 0x40,
	0x01, 0x01, 0x03, 0x28, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
	0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00,
	0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x2B, 0x00,
	0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x20,
};

static const unsigned char SEQ_TD4300_C4[] = {
	0xC4,
	0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x02, 0x31, 0x01, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_C5[] = {
	0xC5,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
};

static const unsigned char SEQ_TD4300_C6[] = {
	0xC6,
	0x62, 0x05, 0x55, 0x02, 0x55, 0x01, 0x0e, 0x01, 0x02, 0x01,
	0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x02, 0x05, 0x15, 0x07,
	0x7F, 0xFF,
};

static const unsigned char SEQ_TD4300_C7[] = {
	0xC7,
	0x00, 0x16, 0x1D, 0x25, 0x31, 0x3E, 0x48, 0x57, 0x3C, 0x41,
	0x4D, 0x5B, 0x64, 0x6D, 0x7F, 0x00, 0x16, 0x1D, 0x25, 0x31,
	0x3E, 0x48, 0x57, 0x3A, 0x43, 0x4F, 0x5B, 0x64, 0x6D, 0x7F,
};

static const unsigned char SEQ_TD4300_C8[] = {
	0xC8,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00,
};

static const unsigned char SEQ_TD4300_C9[] = {
	0xC9,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00,
};

static const unsigned char SEQ_TD4300_CA[] = {
	0xCA,
	0x1D, 0xFC, 0xFC, 0xFC, 0x00, 0xD7, 0xD7, 0xBC, 0x00, 0xD8,
	0xFB, 0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFA, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x01, 0xFF, 0x00,
	0xFF, 0x00, 0x00, 0xFF, 0x03, 0xFF, 0x00, 0x00, 0xFF, 0x03,
	0x02, 0xFF, 0xFF,
};

static const unsigned char SEQ_TD4300_CB[] = {
	0xCB,
	0x00, 0x80, 0x1F, 0x00, 0x00, 0x50, 0x10, 0x80, 0xA0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_CB_GOA[] = {
	0xCB,
	0x50, 0x9F, 0x9F, 0xAF, 0x00, 0x00, 0x02, 0x00, 0x04, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_CC[] = {
	0xCC,
	0x0B,
};

static const unsigned char SEQ_TD4300_CD[] = {
	0xCD,
	0x00, 0x00, 0x22, 0x00, 0x22, 0x00, 0x22, 0x00, 0xAC, 0xAC,
	0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x15, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_CE[] = {
	0xCE,
	0x5D, 0x40, 0x49, 0x53, 0x59, 0x5E, 0x63, 0x68, 0x6E, 0x74,
	0x7E, 0x8A, 0x98, 0xA8, 0xBB, 0xD0, 0xFF, 0x04, 0x00, 0x01,
	0x01, 0x61, 0x00, 0x82, 0x00,
};

static const unsigned char SEQ_TD4300_CF[] = {
	0xCF,
	0x48, 0x10
};

static const unsigned char SEQ_TD4300_CABC_ON[] = {
	0x55,
	0x83,
};

static const unsigned char SEQ_TD4300_D0[] = {
	0xD0,
	0x33, 0x54, 0xD4, 0x31, 0x01, 0x10, 0x10, 0x10, 0x19, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const unsigned char SEQ_TD4300_D1[] = {
	0xD1,
	0x00
};

static const unsigned char SEQ_TD4300_D3[] = {
	0xD3,
	0x1B, 0x3B, 0xBB, 0x77, 0x77, 0x77, 0xBB, 0xB3, 0x33, 0x00,
	0x00, 0x69, 0x6F, 0xA0, 0xA0, 0x33, 0xBB, 0xF2, 0xFD, 0xC6,
};

static const unsigned char SEQ_TD4300_D6[] = {
	0xD6,
	0x41
};

static const unsigned char SEQ_TD4300_D7[] = {
	0xD7,
	0xF6, 0xFF, 0x03, 0x05, 0x41, 0x24, 0x80, 0x1F, 0xC7, 0x1F,
	0x1B, 0x00, 0x0C, 0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0C, 0xF0, 0x1F, 0x00, 0x0C, 0x00, 0x00, 0xAA, 0x67, 0x7E,
	0x5D, 0x06, 0x00,
};

static const unsigned char SEQ_TD4300_D9[] = {
	0xD9,
	0x00, 0x00
};

static const unsigned char SEQ_TD4300_DE[] = {
	0xDE,
	0x00, 0x3F, 0xFF, 0x90,
};

static const unsigned char SEQ_TD4300_F1[] = {
	0xF1,
	0x00, 0x00
};

#endif /* __S6D7AA0_PARAM_H__ */
