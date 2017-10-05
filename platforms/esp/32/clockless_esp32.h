/*
 * Integration into FastLED ClocklessController 2017 Thomas Basler
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com *
 *
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

FASTLED_NAMESPACE_BEGIN

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ARDUINO)
	#include "esp32-hal.h"
	#include "esp_intr.h"
	#include "driver/gpio.h"
	#include "driver/rmt.h"
	#include "driver/periph_ctrl.h"
	#include "freertos/semphr.h"
	#include "soc/rmt_struct.h"
#elif defined(ESP_PLATFORM)
	#include <esp_intr.h>
	#include <driver/gpio.h>
	#include <driver/rmt.h>
	#include <freertos/FreeRTOS.h>
	#include <freertos/semphr.h>
	#include <soc/dport_reg.h>
	#include <soc/gpio_sig_map.h>
	#include <soc/rmt_struct.h>
	#include <stdio.h>
#endif
#include "esp_log.h"

#ifdef __cplusplus
}
#endif

#define DIVIDER             4 /* 8 still seems to work, but timings become marginal */
#define MAX_PULSES         32 /* A channel has a 64 "pulse" buffer - we use half per pass */
#define RMT_DURATION_NS  12.5 /* minimum time of a single RMT duration based on clock ns */

#define CLKS_TO_NS(_CLKS) ((((long)(_CLKS)) * 1000 - 999) / F_CPU_MHZ)

#define FASTLED_HAS_CLOCKLESS 1

static uint8_t rmt_channels_used = 0;

template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 5>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {

	intr_handle_t rmt_intr_handle = NULL;
	uint16_t T0H, T1H, T0L, T1L;

public:

	uint16_t ws2812_pos, ws2812_len, ws2812_half, ws2812_bufIsDirty;
	xSemaphoreHandle ws2812_sem = NULL;
	rmt_item32_t ws2812_bitval_to_rmt_map[2];
	uint16_t TRS;
	uint8_t rmt_channel;
	PixelController<RGB_ORDER> *local_pixels;

    virtual void init() {
		ESP_LOGI("fastled", "T1: %d T2: %d T3: %d", T1, T2, T3);
		ESP_LOGI("fastled", "T1: %ld T2: %ld T3: %ld", CLKS_TO_NS(T1), CLKS_TO_NS(T2), CLKS_TO_NS(T3));


		TRS = 50000;
		T0H = CLKS_TO_NS(T1); // 350;
		T1H = CLKS_TO_NS(T1 + T2); //700;

		T0L = CLKS_TO_NS(T2 + T3); //800;
		T1L = CLKS_TO_NS(T3); //600;

		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

		rmt_channel = rmt_channels_used++;
		if (rmt_channel > 7) {
			assert("Only 8 RMT Channels are allowed");
		}

		ESP_LOGI("fastled", "RMT Channel Init: %d", rmt_channel);

		rmt_set_pin(static_cast<rmt_channel_t>(rmt_channel),
			RMT_MODE_TX,
			static_cast<gpio_num_t>(DATA_PIN));

		initRMTChannel(rmt_channel);

		RMT.tx_lim_ch[rmt_channel].limit = MAX_PULSES;

		RMT.int_ena.val |= BIT(24 + rmt_channel); // set ch*_tx_thr_event
		RMT.int_ena.val |= BIT(rmt_channel * 3); // set ch*_tx_end

		// RMT config for WS2812 bit val 0
		ws2812_bitval_to_rmt_map[0].level0 = 1;
		ws2812_bitval_to_rmt_map[0].level1 = 0;
		ws2812_bitval_to_rmt_map[0].duration0 = T0H / (RMT_DURATION_NS * DIVIDER);
		ws2812_bitval_to_rmt_map[0].duration1 = T0L / (RMT_DURATION_NS * DIVIDER);

		// RMT config for WS2812 bit val 1
		ws2812_bitval_to_rmt_map[1].level0 = 1;
		ws2812_bitval_to_rmt_map[1].level1 = 0;
		ws2812_bitval_to_rmt_map[1].duration0 = T1H / (RMT_DURATION_NS * DIVIDER);
		ws2812_bitval_to_rmt_map[1].duration1 = T1L / (RMT_DURATION_NS * DIVIDER);
    }

    virtual uint16_t getMaxRefreshRate() const { return 400; }

protected:

    virtual void showPixels(PixelController<RGB_ORDER> & pixels) {

		esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, ws2812_handleInterrupt, this, &rmt_intr_handle);

		ws2812_len = (pixels.size() * 3) * sizeof(uint8_t);
		ws2812_pos = 0;
		ws2812_half = 0;

		local_pixels = &pixels;

		copyToRmtBlock_half(*this);

		if (ws2812_pos < ws2812_len) {
			// Fill the other half of the buffer block
			copyToRmtBlock_half(*this);
		}

		ws2812_sem = xSemaphoreCreateBinary();

		RMT.conf_ch[rmt_channel].conf1.mem_rd_rst = 1;
		RMT.conf_ch[rmt_channel].conf1.tx_start = 1;

		xSemaphoreTake(ws2812_sem, portMAX_DELAY);
		vSemaphoreDelete(ws2812_sem);
		ws2812_sem = NULL;

		esp_intr_free(rmt_intr_handle);
    }

	void initRMTChannel(int rmtChannel)
	{
		RMT.apb_conf.fifo_mask = 1;  //enable memory access, instead of FIFO mode.
		RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
		RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
		RMT.conf_ch[rmtChannel].conf0.mem_size = 1;
		RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
		RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
		RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

		RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
		RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
		RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0;    //loop back mode.
		RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1;    // use apb clock: 80M
		RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
		RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;
	}

	static void ws2812_handleInterrupt(void *arg)
	{
		ClocklessController* c = static_cast<ClocklessController*>(arg);

		portBASE_TYPE taskAwoken = 0;

		if (RMT.int_st.val & BIT(24 + c->rmt_channel)) { // check if ch*_tx_thr_event is set
			copyToRmtBlock_half(*c);
			RMT.int_clr.val |= BIT(24 + c->rmt_channel); // set ch*_tx_thr_event
		}
		else if ((RMT.int_st.val & BIT(c->rmt_channel * 3)) && c->ws2812_sem) { // check if ch*_tx_end is set
			xSemaphoreGiveFromISR(c->ws2812_sem, &taskAwoken);
			RMT.int_clr.val |= BIT(c->rmt_channel * 3); // set ch*_tx_end
		}
	}

	static void copyToRmtBlock_half(ClocklessController& c)
	{
		// This fills half an RMT block
		// When wraparound is happening, we want to keep the inactive half of the RMT block filled
		uint16_t i, j, offset, len, byteval;
		static uint8_t rgb_channel = 0;

		offset = c.ws2812_half * MAX_PULSES;
		c.ws2812_half = !c.ws2812_half;

		len = c.ws2812_len - c.ws2812_pos;
		if (len > (MAX_PULSES / 8))
			len = (MAX_PULSES / 8);

		if (!len) {
			if (!c.ws2812_bufIsDirty) {
				return;
			}
			// Clear the channel's data block and return
			for (i = 0; i < MAX_PULSES; i++) {
				RMTMEM.chan[c.rmt_channel].data32[i + offset].val = 0;
			}
			c.ws2812_bufIsDirty = 0;
			return;
		}
		c.ws2812_bufIsDirty = 1;

		for (i = 0; i < len; i++) {
			switch (rgb_channel) {
			case 1:
				byteval = c.local_pixels->loadAndScale1();
				rgb_channel = 2;
				break;
			case 2:
				byteval = c.local_pixels->loadAndScale2();
				c.local_pixels->advanceData();
				c.local_pixels->stepDithering();
				rgb_channel = 0;
				break;
			default:
				byteval = c.local_pixels->loadAndScale0();
				rgb_channel = 1;
				break;
			}

			// Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the rmt_item32_t value corresponding to the buffered bit value
			for (j = 0; j < 8; j++, byteval <<= 1) {
				int bitval = (byteval >> 7) & 0x01;
				int data32_idx = i * 8 + offset + j;
				RMTMEM.chan[c.rmt_channel].data32[data32_idx].val = c.ws2812_bitval_to_rmt_map[bitval].val;
			}

			// Handle the reset bit by stretching duration1 for the final bit in the stream
			if (i + c.ws2812_pos == c.ws2812_len - 1) {
				RMTMEM.chan[c.rmt_channel].data32[i * 8 + offset + 7].duration1 =
					c.TRS / (RMT_DURATION_NS * DIVIDER);
			}
		}

		// Clear the remainder of the channel's data not set above
		for (i *= 8; i < MAX_PULSES; i++) {
			RMTMEM.chan[c.rmt_channel].data32[i + offset].val = 0;
		}

		c.ws2812_pos += len;
	}
};

FASTLED_NAMESPACE_END