/**
 * @file max30100.cpp
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2018 Angelo Elias Dalzotto & Gabriel Boni Vicari
 * 
 * @brief This library was created to interface the MAX30100 pulse and oxymeter
 * sensor with ESP32 using the IDF-SDK. It includes functions to initialize with
 * the programmer's desired parameters and to update the readings, detecting pulse
 * and having the pulse saturation O2. It is based on Strogonovs Arduino library
 * (https://morf.lv).
 */

#include "max30100.h"

const uint8_t Max30100::Device::DEV_ADDR = 0x57;
const uint8_t Max30100::Device::REV_ID = 0xFE;
const uint8_t Max30100::Device::PART_ID = 0xFF;
const uint8_t Max30100::Device::INT_STATUS = 0x00;
const uint8_t Max30100::Device::INT_ENABLE = 0x01;
const uint8_t Max30100::Device::FIFO_WRITE = 0x02;
const uint8_t Max30100::Device::FIFO_OVF_COUNTER = 0x03;
const uint8_t Max30100::Device::FIFO_READ = 0x04;
const uint8_t Max30100::Device::FIFO_DATA = 0x05;
const uint8_t Max30100::Device::MODE_CONF = 0x06;
const uint8_t Max30100::Device::SPO2_CONF = 0x07;
const uint8_t Max30100::Device::LED_CONF = 0x09;
const uint8_t Max30100::Device::TEMP_INT = 0x16;
const uint8_t Max30100::Device::TEMP_FRAC = 0x17;

const uint8_t Max30100::Device::MODE_SHDN = (1 << 7);
const uint8_t Max30100::Device::MODE_RST = (1 << 6);
const uint8_t Max30100::Device::MODE_TEMP_EN = (1 << 3);
const uint8_t Max30100::Device::SPO2_HI_RES_EN = (1 << 6);

const char *Max30100::Device::TAG = "Max30100";

Max30100::Device::Device(i2c_port_t i2c_num, Current ir_current, 
                            Current start_red_current,
                            uint32_t acceptable_intensity_diff,
                            uint16_t red_current_adj_ms,
                            uint8_t rst_spo2_pulse_n, double alpha,
                            uint8_t mean_filter_sz, uint16_t pulse_min_thresh,
                            uint16_t pulse_max_thresh, uint8_t bpm_sample_size,
                            bool debug) :
                            i2c(DEV_ADDR, i2c_num),
                            i2c_num(i2c_num),
                            acceptable_intensity_diff(acceptable_intensity_diff),
                            red_current(start_red_current),
                            red_current_adj_ms(red_current_adj_ms),
                            rst_spo2_pulse_n(rst_spo2_pulse_n),
                            dc_alpha(alpha),
                            pulse_min_threshold(pulse_min_thresh),
                            pulse_max_threshold(pulse_max_thresh),
                            mean_filter_size(mean_filter_sz),
                            pulse_bpm_sample_size(bpm_sample_size),
                            debug(debug),
                            current_pulse_state(PulseState::IDLE),
                            ir_current(ir_current),
                            last_red_current_check(0)
{
    dc_filter_ir.w = 0;
    dc_filter_ir.result = 0;

    dc_filter_red.w = 0;
    dc_filter_red.result = 0;

    lpb_filter_ir.v[0] = 0;
    lpb_filter_ir.v[1] = 0;
    lpb_filter_ir.result = 0;

    mean_diff_ir.i = 0;
    mean_diff_ir.sum = 0;
    mean_diff_ir.count = 0;

    mean_diff_ir.i = 0;
    mean_diff_ir.sum = 0;
    mean_diff_ir.count = 0;

    ir_ac_sq_sum = 0;
    red_ac_sq_sum = 0;
    samples_recorded = 0;
    pulses_detected = 0;
    current_spO2 = 0;
}

void Max30100::Device::init(Mode mode, SamplingRate sampling_rate, PulseWidth pulse_width, bool high_res_mode)
{
    try {
        mtx_result = xSemaphoreCreateMutex();
        mean_diff_ir.values.reset(new double[mean_filter_size]);
        values_bpm.reset(new double[mean_filter_size]);
        set_mode(mode);
        set_sampling_rate(sampling_rate);
        set_pulse_width(pulse_width);
        set_led_current(red_current, ir_current);
        set_high_res(high_res_mode);
    } catch(const std::bad_alloc& ex){
        throw ex;
    }
}

void Max30100::Device::set_mode(Mode mode)
{
    try {
        i2c.modify_register(MODE_CONF, 0xF8, (uint8_t)mode);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }
}

void Max30100::Device::set_sampling_rate(SamplingRate rate)
{
    try {
        i2c.modify_register(SPO2_CONF, 0xE3, (uint8_t)rate << 2);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }
}

void Max30100::Device::set_pulse_width(PulseWidth pw)
{
    try {
        i2c.modify_register(SPO2_CONF, 0xFC, (uint8_t)pw);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }
}

void Max30100::Device::set_led_current(Current red_current, Current ir_current)
{
    try {
        i2c.write_register(LED_CONF, ((uint8_t)red_current << 4) | (uint8_t)ir_current);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }
}

void Max30100::Device::set_high_res(bool high_res)
{
    try {
        if(high_res)
            i2c.modify_register(SPO2_CONF, 0xFF, SPO2_HI_RES_EN);
        else
            i2c.modify_register(SPO2_CONF, ~SPO2_HI_RES_EN, 0x00);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }
}

void Max30100::Device::update()
{
    Fifo raw_data;
    
    try {
        uint8_t buffer[4];
        i2c.read_buffer(FIFO_DATA, buffer, 4);
        raw_data.raw_ir = (uint16_t)(buffer[0] << 8) | buffer[1];
        raw_data.raw_red = (uint16_t)(buffer[2] << 8) | buffer[3];
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }

    dc_filter_ir = dc_removal((double)raw_data.raw_ir, dc_filter_ir.w, dc_alpha);
    dc_filter_red = dc_removal((double)raw_data.raw_red, dc_filter_red.w, dc_alpha);

    lpb_filter(mean_diff(dc_filter_ir.result)/*-dc_filter_ir.result*/);

    ir_ac_sq_sum += dc_filter_ir.result * dc_filter_ir.result;
    red_ac_sq_sum += dc_filter_red.result * dc_filter_red.result;
    samples_recorded++;

    if(detect_pulse(lpb_filter_ir.result) && samples_recorded){
        xSemaphoreTake(mtx_result, portMAX_DELAY);
        pulse_detected = true;
        xSemaphoreGive(mtx_result);
        pulses_detected++;

        double ratio_rms = log(sqrt(red_ac_sq_sum / (double)samples_recorded)) /
                            log(sqrt(ir_ac_sq_sum / (double)samples_recorded));

        if(debug)
            ESP_LOGD(TAG, "RMS Ratio: %lf\n", ratio_rms);

        //This is my adjusted standard model, so it shows 0.89 as 94% saturation.
        //It is probably far from correct, requires proper empirical calibration.
        xSemaphoreTake(mtx_result, portMAX_DELAY);
        current_spO2 = 110.0 - 18.0 * ratio_rms;
        xSemaphoreGive(mtx_result);

        if(!(pulses_detected % rst_spo2_pulse_n)){
            ir_ac_sq_sum = 0;
            red_ac_sq_sum = 0;
            samples_recorded = 0;
        }
    }

    balance_intensities(dc_filter_red.w, dc_filter_ir.w);
}

struct Max30100::Device::Filter Max30100::Device::dc_removal(double x, double prev_w, double alpha)
{
    struct Filter filtered;
    filtered.w = x + alpha * prev_w;
    filtered.result = filtered.w - prev_w;

    return filtered;
}

void Max30100::Device::lpb_filter(double x)
{
    lpb_filter_ir.v[0] = lpb_filter_ir.v[1];

    //Fs = 100Hz and Fc = 10Hz
    lpb_filter_ir.v[1] = (2.452372752527856026e-1 * x) +
                            (0.50952544949442879485 * lpb_filter_ir.v[0] );

    //Fs = 100Hz and Fc = 4Hz
    //lpb_filter_ir.v[1] = (1.367287359973195227e-1 * x)
    //                   + (0.72654252800536101020 * lpb_filter_ir.v[0]);
    //Very precise butterworth filter

    lpb_filter_ir.result = lpb_filter_ir.v[0] + lpb_filter_ir.v[1];
}

double Max30100::Device::mean_diff(double M)
{
    double avg = 0;

    mean_diff_ir.sum -= mean_diff_ir.values[mean_diff_ir.i];
    mean_diff_ir.values[mean_diff_ir.i] = M;
    mean_diff_ir.sum += mean_diff_ir.values[mean_diff_ir.i];

    mean_diff_ir.i++;
    mean_diff_ir.i = mean_diff_ir.i % mean_filter_size;

    if(mean_diff_ir.count < mean_filter_size)
        mean_diff_ir.count++;

    avg = mean_diff_ir.sum / mean_diff_ir.count;
    return avg - M;
}


bool Max30100::Device::detect_pulse(double sensor_value)
{
    static float prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t current_beat = 0;
    static uint32_t last_beat = 0;

    if(sensor_value > pulse_max_threshold) {
        current_pulse_state = PulseState::IDLE;
        prev_sensor_value = 0;
        last_beat = 0;
        current_beat = 0;
        values_went_down = 0;
        return false;
    }

    switch(current_pulse_state) {
    case PulseState::IDLE:
        if(sensor_value >= this->pulse_min_threshold) {
            current_pulse_state = PulseState::TRACE_UP;
            values_went_down = 0;
        }
        break;
    case PulseState::TRACE_UP:
        if(sensor_value > prev_sensor_value) {
            current_beat = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        } else {
            if(debug) {
                printf("Peak reached: %f %f\n",
                       sensor_value,
                       prev_sensor_value);
            }

            uint32_t beat_duration = current_beat - last_beat;
            last_beat = current_beat;

            float raw_bpm = 0;
            if(beat_duration > 0)
                raw_bpm = 60000.0 / (float)beat_duration;

            if(this->debug)
                printf("%f\n", raw_bpm);

            //This method sometimes glitches,
            //it's better to go through whole moving average everytime.
            //IT's a neat idea to optimize the amount of work for moving avg,
            //but while placing, removing finger it can screw up
            //this->values_bpmSum -= this->values_bpm[bpmIndex];
            //this->values_bpm[bpmIndex] = rawBPM;
            //this->values_bpmSum += this->values_bpm[bpmIndex];

            values_bpm[bpm_i] = raw_bpm;
            bpm_sum = 0;
            for(int i = 0; i < pulse_bpm_sample_size; i++)
                bpm_sum += values_bpm[i];

            if(debug) {
                printf("CurrentMoving Avg: ");

                for(int i = 0; i < pulse_bpm_sample_size; i++)
                    printf("%f ", values_bpm[i]);

                printf(" \n");
            }

            bpm_i++;
            bpm_i = bpm_i % pulse_bpm_sample_size;

            if(bpm_count < pulse_bpm_sample_size)
                bpm_count++;

            xSemaphoreTake(mtx_result, portMAX_DELAY);
            current_bpm = bpm_sum / bpm_count;

            if(debug)
                printf("AVg. BPM: %f\n", current_bpm);
            xSemaphoreGive(mtx_result);

            current_pulse_state = PulseState::TRAC_DOWN;

            return true;
        }
        break;
    case PulseState::TRAC_DOWN:
        if(sensor_value < prev_sensor_value)
            values_went_down++;

        if(sensor_value < pulse_min_threshold)
            current_pulse_state = PulseState::IDLE;

        break;
    }

    prev_sensor_value = sensor_value;
    return false;
}

void Max30100::Device::balance_intensities(double red_dc, double ir_dc)
{
    if(xTaskGetTickCount()*portTICK_PERIOD_MS-last_red_current_check >= red_current_adj_ms){
        if(ir_dc - red_dc > acceptable_intensity_diff && red_current != Current::CURRENT_50MA){
            red_current = static_cast<Max30100::Current>((uint8_t)red_current+1);
            set_led_current(red_current, ir_current);
            if(debug)
                ESP_LOGD(TAG, "Red LED Current+");
        } else if(red_dc - ir_dc > acceptable_intensity_diff && (uint8_t)red_current > 0){
            red_current = static_cast<Max30100::Current>((uint8_t)red_current-1);
            set_led_current(red_current, ir_current);
            if(debug)
                ESP_LOGD(TAG, "Red LED Current-");
        }
    }
    last_red_current_check = xTaskGetTickCount()*portTICK_PERIOD_MS;
}

double Max30100::Device::read_temperature()
{
    try {
        i2c.modify_register(MODE_CONF, 0xFF, MODE_TEMP_EN);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }

    //@TODO
    //This can be changed to a while loop checking the interrupt flag.
    //There is an interrupt flag for when temperature has been read.
    vTaskDelay(100/portTICK_PERIOD_MS);

    int8_t temp;
    double temp_frac;
    try {
        temp = i2c.read_register(TEMP_INT);
        temp_frac = i2c.read_register(TEMP_FRAC);
    } catch(const I2CExcept::CommandFailed& ex){
        throw ex;
    }

    temp_frac *= 0.0625;

    return (double)temp+temp_frac;
}

void Max30100::Device::print_registers()
{
    if(!debug)
        throw std::runtime_error("Not in debug mode");

    try {
        ESP_LOGD(TAG, "INT_STATUS: %x\n", i2c.read_register(INT_STATUS));
        ESP_LOGD(TAG, "INT_EN: %x\n", i2c.read_register(INT_ENABLE));
        ESP_LOGD(TAG, "FIFO_WRITE: %x\n", i2c.read_register(FIFO_WRITE));
        ESP_LOGD(TAG, "FIFO_OVF_CNT: %x\n", i2c.read_register(FIFO_OVF_COUNTER));
        ESP_LOGD(TAG, "FIFO_READ: %x\n", i2c.read_register(FIFO_READ));
        ESP_LOGD(TAG, "FIFO_DATA: %x\n", i2c.read_register(FIFO_DATA));
        ESP_LOGD(TAG, "MODE_CONF: %x\n", i2c.read_register(MODE_CONF));
        ESP_LOGD(TAG, "SPO2_CONF: %x\n", i2c.read_register(SPO2_CONF));
        ESP_LOGD(TAG, "LED_CONF: %x\n", i2c.read_register(LED_CONF));
        ESP_LOGD(TAG, "TEMP_INT: %x\n", i2c.read_register(TEMP_INT));
        ESP_LOGD(TAG, "TEMP_FRAC: %x\n", i2c.read_register(TEMP_FRAC));
        ESP_LOGD(TAG, "REV_ID: %x\n", i2c.read_register(REV_ID));
        ESP_LOGD(TAG, "PART_ID: %x\n", i2c.read_register(PART_ID));
    } catch(const I2CExcept::CommandFailed &ex){
        throw ex;
    }
}

double Max30100::Device::get_bpm()
{
    xSemaphoreTake(mtx_result, portMAX_DELAY);
    double bpm = current_bpm;
    xSemaphoreGive(mtx_result);
    return bpm;
}

double Max30100::Device::get_spo2()
{
    xSemaphoreTake(mtx_result, portMAX_DELAY);
    double spo2 = current_spO2;
    xSemaphoreGive(mtx_result);
    return spo2;
}

const char* Max30100::Device::get_tag()
{
    return TAG;
}

bool Max30100::Device::is_pulse_detected()
{
    xSemaphoreTake(mtx_result, portMAX_DELAY);
    bool detected = pulse_detected;
    pulse_detected = false;
    xSemaphoreGive(mtx_result);
    return detected;
}

/**
 * MIT License
 * 
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/