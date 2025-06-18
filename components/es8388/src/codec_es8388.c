// #include "freertos/FreeRTOS.h"
#include "esp_system.h"
// #include "esp_event.h"
// #include "esp_event_loop.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "codec_es8388.h"

// #include "driver/i2c.h"
#include "gpi2c.h"

#define TAG "CODEC_ES8388"

#define I2C_MASTER_NUM	0
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

#define ES8388_ADDR 0b0010000

esp_err_t i2c_master_init(uint8_t sda, uint8_t scl)
{
    gpi2c_init(sda, scl, 400000);
    return 0;
}

static esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    gpi2c_writeRegister(ES8388_ADDR, reg_add, &data, 1);
    return ESP_OK;
}

static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    gpi2c_readRegister(ES8388_ADDR, reg_add, p_data, 1);
    return ESP_OK;
}

static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}

esp_err_t es8388_reg_init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    /* mute DAC during setup, power up all systems, slave mode */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

    /* power up DAC and enable LOUT1+2 / ROUT1+2, ADC sample rate = DAC sample rate */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3e);
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

    /* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

    /* DAC to output route mixer configuration: ADC MIX TO OUTPUT */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x1B);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

    /* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

    /* DAC volume control: 0dB (maximum, unattenuated)  */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

    /* power down ADC while configuring; volume: +9dB for both channels */
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x88); // +24db

    /* select LINPUT2 / RINPUT2 as ADC input; stereo; 16 bit word length, format right-justified, MCLK / Fs = 256 */
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0xf0); // 50
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x80); // 00
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0e);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);

    /* set ADC volume */
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x20);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x20);

    /* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

    /* set LOUT2 / ROUT2 volume: 0dB (unattenuated) */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x1e);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x1e);

    /* power up and enable DAC; power up ADC (no MIC bias) */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);

    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
    // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    // res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all
    // res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);   //vroi=0
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL6, 0x38);   //Left and Right channel dac inversion
    // res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db
    // ESP_LOGW(TAG, "Setting DAC Output: %02x", output );
    // res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, output );
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain
    // ESP_LOGW(TAG, "Setting ADC Input: %02x", input );
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, input);
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    // //ALC for Microphone
    // res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    // res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

    return res;
}

// This function sets the I2S format which can be one of
//		I2S_NORMAL
//		I2S_LEFT		Left Justified
//		I2S_RIGHT,      Right Justified
//		I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

esp_err_t es8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGI(TAG, "Setting I2S ADC Format\n");
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGI(TAG, "Setting I2S DAC Format\n");
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGI(TAG, "Setting I2S ADC Bits: %d\n", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGW(TAG, "Setting I2S DAC Bits: %d\n", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}

esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	ESP_LOGI(TAG, "Resetting State Machine\n");

        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	ESP_LOGI(TAG, "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	ESP_LOGI(TAG, "Powering up DAC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
    }

    return res;
}

esp_err_t es8388_defaultConfig() {
    esp_err_t res = ESP_OK;
    /* mute DAC during setup, power up all systems, slave mode */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04); // mute output
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

	/* power up DAC and enable only LOUT1 / ROUT1, ADC sample rate = DAC sample rate */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

	/* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

	/* DAC to output route mixer configuration */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

	/* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

	/* DAC volume control: 0dB (maximum, unattenuated)  */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

		/* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

	/* power up and enable DAC; power up ADC (no MIC bias) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x33); //This one is critical
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
    return res;
}

void es8388_init(uint32_t samplerate, i2s_data_bit_width_t bits_per_sample, uint8_t i2s_channel_nums, uint8_t sda, uint8_t scl) {
    ESP_LOGI(TAG, "Init ES8388...");
    i2c_master_init(sda, scl);
    vTaskDelay(20);
    es8388_defaultConfig();
    i2smanager_init(samplerate, bits_per_sample, i2s_channel_nums);
}
void es8388_setVolume(es_vol_t dev, int volume) {
    const uint32_t max_vol = 100; // max input volume value

    const int32_t max_vol_val = dev == ES_VOL_MAIN ? 96 : 0x21; // max register value for ES8388 out volume

    uint8_t lreg = 0, rreg = 0;
    switch(dev) {
        case ES_VOL_MAIN:
            lreg = ES8388_DACCONTROL4;
            rreg = ES8388_DACCONTROL5;
            break;
        case ES_VOL_OUT1:
            lreg = ES8388_DACCONTROL24;
            rreg = ES8388_DACCONTROL25;
            break;
        case ES_VOL_OUT2:
            lreg = ES8388_DACCONTROL26;
            rreg = ES8388_DACCONTROL27;
            break;
    }
    uint8_t vol_val = volume > max_vol ? max_vol_val : (max_vol_val * volume) / max_vol;

    // main dac volume control is reverse scale (lowest value is loudest)
    // hence we reverse the calculated value
    if (dev == ES_VOL_MAIN)
    {
        vol_val = max_vol_val - vol_val;
    }
    ESP_LOGI(TAG, "Set Volume values to 0x%02X", vol_val);
    es_write_reg(ES8388_ADDR, lreg, vol_val);
    es_write_reg(ES8388_ADDR, rreg, vol_val);
}
void es8388_zero_dma_buffer() {
    // i2s_zero_dma_buffer(I2S_NUM);
    i2smanager_zero_dma_buffer();
}
void es8388_read(void* data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait) {
    i2smanager_read((void *)data, size, bytes_read, ticks_to_wait);
}
void es8388_write(void *data, size_t size, size_t *bytes_written, TickType_t ticks_to_wait) {
    if(i2smanager_write((uint8_t*)(data), size, bytes_written, ticks_to_wait) != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to i2s");
    }
}