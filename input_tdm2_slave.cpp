/* Audio Library for Teensy 3.X
 * Copyright (c) 2017, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#if defined(__IMXRT1062__)
#include <Arduino.h>
#include "input_tdm2_slave.h"
#include "output_tdm2.h"
#include "utility/imxrt_hw.h"

DMAMEM __attribute__((aligned(32)))
static uint32_t tdm_rx_buffer[AUDIO_BLOCK_SAMPLES*AudioInputTDM2Slave::noChannels];
audio_block_t * AudioInputTDM2Slave::block_incoming[AudioInputTDM2Slave::noChannels] = {
	nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
};
bool AudioInputTDM2Slave::update_responsibility = false;
DMAChannel AudioInputTDM2Slave::dma(false);
void AudioInputTDM2Slave::config_tdm(void)
{
	CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);

	// if either transmitter or receiver is enabled, do nothing
	if (I2S2_TCSR & I2S_TCSR_TE) return;
	if (I2S2_RCSR & I2S_RCSR_RE) return;
	
	CORE_PIN4_CONFIG  = 2;  //2:TX_BCLK
	CORE_PIN5_CONFIG = 2;  //2:RX_DATA0
	CORE_PIN3_CONFIG  = 2;  //2:TX_SYNC
	IOMUXC_SAI2_RX_BCLK_SELECT_INPUT =0;	//page 422 bitclock IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06
	IOMUXC_SAI2_RX_SYNC_SELECT_INPUT =0;  //IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05
	

	// configure transmitter
	int rsync = 1;
	int tsync = 0;
	uint32_t noBits = 16;	//the Atari transmits 16bit samples
	uint32_t noBitsM1=noBits-1;

	// configure transmitter
	I2S2_TMR = 0;
	I2S2_TCR1 = I2S_TCR1_RFW(1);  // watermark at half fifo size
	I2S2_TCR2 = I2S_TCR2_SYNC(tsync)
		| I2S_TCR2_DIV(0)// Bit clock divide by (DIV+1)*2 ->bitclock = 256*fs Todo: adapt for 16bit
		| I2S_TCR2_BCP;	//Bit Clock Polarity
	
	I2S2_TCR3 = I2S_TCR3_TCE;//Transmit Channel Enable, 0x10000=0001 0000 0000 0000 0000 -> channel 1 enabled
	
	I2S2_TCR4 = I2S_TCR4_FRSZ(noChannels-1)
		| I2S_TCR4_SYWD(noBitsM1)
		| I2S_TCR4_MF	//most signifacant bit is sent first 
		| I2S_TCR4_FSE	//frame sync early
		| I2S_TCR4_FSP;	// Frame Sync Polarity
	I2S2_TCR5 = I2S_TCR5_WNW(noBitsM1)//word N width in bits
	| I2S_TCR5_W0W(noBitsM1)//word 0 width in bits
	| I2S_TCR5_FBT(15);		//First Bit Shifted

	// configure receiver
	I2S2_RMR = 0;		//Receive Word Mask -> all words enabled
	I2S2_RCR1 = I2S_RCR1_RFW(1);//Receive FIFO Watermark
	I2S2_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_TCR2_BCP;
	I2S2_RCR3 = I2S_RCR3_RCE;

	
	I2S2_RCR4 = I2S_RCR4_FRSZ(noChannels-1)
				| I2S_RCR4_SYWD(noBitsM1)
				| I2S_RCR4_MF	//most signifacant bit is recieved first 
				| I2S_RCR4_FSE	//frame sync early
				| I2S_RCR4_FSP;	// Frame Sync Polarity: 0b - Frame sync is active high. 1b - Frame sync is active low.
	I2S2_RCR5 = I2S_RCR5_WNW(noBitsM1)
				| I2S_RCR5_W0W(noBitsM1)
				| I2S_RCR5_FBT(15);	// First Bit Shifted

	// I2S_RCR5_FBT: First Bit Shifted
	// Configures the bit index for the first bit received for each word in the frame. If configured for MSB First,
	// the index of the next bit received is one less than the current bit received. If configured for LSB First, the
	// index of the next bit received is one more than the current bit received. The value written must be greater
	// than or equal to the word width when configured for MSB First. The value written must be less than or
	// equal to 31-word width when configured for LSB First.

	I2S2_TCR2 &= ~I2S_TCR2_MSEL(3)	//master clock select	//has no effect if bit clock and frame clock are provided from an extern source
	& ~I2S_TCR2_BCD;		//bit clock direction: 1: master mode, 0: slave mode
	I2S2_TCR4 &= ~I2S_TCR4_FSD;	//frame sync direction: 1->master mode, 0->slave mode
	I2S2_RCR2 &= ~I2S_RCR2_BCD;					//Bit clock direction 1->master mode, 0->slave mode
	I2S2_RCR4 &= ~I2S_RCR4_FSD;					//frame sync direction: 1->master mode, 0->slave mode
}

void AudioInputTDM2Slave::begin(void)
{
	

	dma.begin(true); // Allocate the DMA channel first

	// TODO: should we set & clear the I2S_RCSR_SR bit here?
	config_tdm();

	CORE_PIN5_CONFIG = 2;  //2:RX_DATA0
	IOMUXC_SAI2_RX_DATA0_SELECT_INPUT = 0;
	dma.TCD->SADDR = &I2S2_RDR0;
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = tdm_rx_buffer;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->DLASTSGA = -sizeof(tdm_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI2_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S2_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
	I2S2_TCSR |= I2S_TCSR_TE | I2S_TCSR_BCE;
	dma.attachInterrupt(isr);	

}

// TODO: needs optimization...
static void memcpy_tdm_rx(uint32_t *dest1, uint32_t *dest2, const uint32_t *src)
{
	uint32_t i, in1, in2;

	for (i=0; i < AUDIO_BLOCK_SAMPLES/2; i++) {
		in1 = *src;
		in2 = *(src+8);
		src += 16;
		*dest1++ = (in1 >> 16) | (in2 & 0xFFFF0000);
		*dest2++ = (in1 << 16) | (in2 & 0x0000FFFF);
	}
}

void AudioInputTDM2Slave::isr(void)
{
	uint32_t daddr;
	const uint32_t *src;
	unsigned int i;

	daddr = (uint32_t)(dma.TCD->DADDR);
	dma.clearInterrupt();

	if (daddr < (uint32_t)tdm_rx_buffer + sizeof(tdm_rx_buffer) / 2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = &tdm_rx_buffer[AUDIO_BLOCK_SAMPLES*8];
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = &tdm_rx_buffer[0];
	}
	if (block_incoming[0] != nullptr) {
		#if IMXRT_CACHE_ENABLED >=1
		arm_dcache_delete((void*)src, sizeof(tdm_rx_buffer) / 2);
		#endif
		for (i=0; i < 16; i += 2) {
			uint32_t *dest1 = (uint32_t *)(block_incoming[i]->data);
			uint32_t *dest2 = (uint32_t *)(block_incoming[i+1]->data);
			memcpy_tdm_rx(dest1, dest2, src);
			src++;
		}
	}
	if (update_responsibility) update_all();
}


void AudioInputTDM2Slave::update(void)
{
	unsigned int i, j;
	audio_block_t *new_block[16];
	audio_block_t *out_block[16];

	// allocate 16 new blocks.  If any fails, allocate none
	for (i=0; i < 16; i++) {
		new_block[i] = allocate();
		if (new_block[i] == nullptr) {
			for (j=0; j < i; j++) {
				release(new_block[j]);
			}
			memset(new_block, 0, sizeof(new_block));
			break;
		}
	}
	__disable_irq();
	memcpy(out_block, block_incoming, sizeof(out_block));
	memcpy(block_incoming, new_block, sizeof(block_incoming));
	__enable_irq();
	if (out_block[0] != nullptr) {		
		// if we got 1 block, all 16 are filled
		for (i=0; i < 16; i++) {
			transmit(out_block[i], i);
			release(out_block[i]);
		}
	}
}


#endif
