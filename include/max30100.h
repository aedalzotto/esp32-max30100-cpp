/**
 * @file max30100.h
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

#ifndef MAX30100_H
#define MAX30100_H

#include <cmath>
#include <memory>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "i2c_device.h"

namespace Max30100 {
    class Device;

    enum class Mode;
    enum class SamplingRate;
    enum class PulseWidth;
    enum class Current;
    enum class PulseState;
}

/**
 * Operation Mode Enum.
 * Heart rate only or Oxigen saturation + Heart rate.
 */
enum class Max30100::Mode {
    HR_ONLY = 0x02,
    SPO2_HR = 0x03
};

/**
 * Sampling rate enum.
 * Internal sampling rates from 50Hz up to 1KHz.
 */
enum class Max30100::SamplingRate {
    RATE_50HZ   = 0x00,
    RATE_100HZ  = 0x01,
    RATE_167HZ  = 0x02,
    RATE_200HZ  = 0x03,
    RATE_400HZ  = 0x04,
    RATE_600HZ  = 0x05,
    RATE_800HZ  = 0x06,
    RATE_1000HZ = 0x07
};

/**
 * Led pulse width enum.
 * Sets from 200us to 1600us.
 */
enum class Max30100::PulseWidth {
    WIDTH_200US_ADC_13  = 0x00,
    WIDTH_400US_ADC_14  = 0x01,
    WIDTH_800US_ADC_15  = 0x02,
    WIDTH_1600US_ADC_16 = 0x03
};

/**
 * Led current enum.
 * Sets the current for any of the leds. From 0mA to 50mA.
 */
enum class Max30100::Current {
    CURRENT_0MA    = 0x00,
    CURRENT_4_4MA  = 0x01,
    CURRENT_7_6MA  = 0x02,
    CURRENT_11MA   = 0x03,
    CURRENT_14_2MA = 0x04,
    CURRENT_17_4MA = 0x05,
    CURRENT_20_8MA = 0x06,
    CURRENT_24MA   = 0x07,
    CURRENT_27_1MA = 0x08,
    CURRENT_30_6MA = 0x09,
    CURRENT_33_8MA = 0x0A,
    CURRENT_37MA   = 0x0B,
    CURRENT_40_2MA = 0x0C,
    CURRENT_43_6MA = 0x0D,
    CURRENT_46_8MA = 0x0E,
    CURRENT_50MA   = 0x0F
};

/**
 * Pulse State Machine
 */
enum class Max30100::PulseState {
    IDLE,
    TRACE_UP,
    TRAC_DOWN
};

/**
 * Main device class
 */
class Max30100::Device {
public:
    /**
     * @brief Device constructor. Must have I2C already initialized.
     * 
     * @details All parameters are optional.
     * 
     * @param i2c_num is the I2C port of the ESP32 with the MAX30100 is attached to.
     * @param ir_current is the current to the IR Led.
     * @param start_red_current is the starting value to Red Led current.
     * @param acceptable_intensity_diff TODO
     * @param red_current_adj_ms Minimum time to update red led current (ms).
     * @param rst_spo2_pulse_n TODO
     * @param alpha Filter parameter.
     * @param mean_filter_sz is the sampling vector size to the filter.
     * @param pulse_min_thresh TODO
     * @param pulse_max_thresh TODO
     * @param bpm_sample_size is the heart rate sampling vector size.
     * @param debug is to set if registers output will be done to serial monitoring.
     */
    Device(i2c_port_t i2c_num = I2C_NUM_0,
            Current ir_current = Current::CURRENT_50MA,
            Current start_red_current = Current::CURRENT_27_1MA,
            uint32_t acceptable_intensity_diff = 65000,
            uint16_t red_current_adj_ms = 500, uint8_t rst_spo2_pulse_n = 4,
            double alpha = 0.95, uint8_t mean_filter_sz = 15,
            uint16_t pulse_min_thresh = 300, uint16_t pulse_max_thresh = 2000,
            uint8_t bpm_sample_size = 10,
            bool debug = false);

    /**
     * @brief Device initializer
     * 
     * @details Allocate resources that can throw exceptions
     * 
     * @param mode is the working mode of the sensor (HR only or HR+SPO2)
     * @param sampling_rate is the frequency which samples are taken internally.
     * @param pulse_width is the led pulse width. 
     * @param high_res_mode is to set if high resolution mode or not.
     * 
     * @throw std::bac_alloc Case can't allocate memory for sample buffers.
     * @throw I2CExcept::CommandFailed Case some i2c communication fail.
     */
    void init(Mode mode = Mode::SPO2_HR, 
            SamplingRate sampling_rate = SamplingRate::RATE_100HZ,
            PulseWidth pulse_width = PulseWidth::WIDTH_1600US_ADC_16,bool high_res_mode = true);

    /**
     * @brief Reads the MAX30100 registers and update the object.
     * 
     * @details This must be called at the same rate as sampling rate defined.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void update();

    /**
     * @brief Reads the internal temperature of the MAX30100.
     * 
     * @returns Temperature in celsius.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    double read_temperature();

    /**
     * @brief Prints the registers for debugging purposes.
     * 
     * @throw
     *     - std::runtime_error Case not in debug mode
     *     - I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void print_registers();

    /**
     * @brief Sets the operation mode. HR Only or HR+SPO2.
     * 
     * @param mode is the mode of operation desired.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void set_mode(Mode mode = Mode::SPO2_HR);

    /**
     * @brief Sets the resolution. High or standard.
     * 
     * @param high_res is if high resolution will be enabled.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void set_high_res(bool high_res = true);

    /**
     * @brief Sets the led currents.
     * 
     * @param red_current is the current value of the Red Led.
     * @param ir_current is the current value of the IR Led.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void set_led_current(Current red_current = Current::CURRENT_27_1MA,
                            Current ir_current = Current::CURRENT_50MA);

    /**
     * @brief Sets the pulse width.
     * 
     * @param pw is the pulse width desired.
     */
    void set_pulse_width(PulseWidth pw = PulseWidth::WIDTH_1600US_ADC_16);

    /**
     * @brief Set the internal sampling rate.
     * 
     * @param rate is the sampling rate desired.
     * 
     * @throw I2CExcept::CommandFailed Case some i2c comm fails.
     */
    void set_sampling_rate(SamplingRate rate = SamplingRate::RATE_100HZ);

    /**
     * @brief Get the last bpm value.
     * 
     * @return current_bpm
     */
    double get_bpm();

    /**
     * @brief Get the last spo2 value.
     * 
     * @return current_spo2
     */
    double get_spo2(); 

    /**
     * @brief Get the device tag for debug.
     * 
     * @return *TAG
     */
    const char* get_tag();

private:
    /**
     * FIFO reading structure.
     */
    struct Fifo {
        uint16_t raw_ir;
        uint16_t raw_red;
    };

    /**
     * DC filter results.
     */
    struct Filter {
        double w;
        double result;
    };

    /**
     * Butterworth filter results.
     */
    struct Butterworth {
        double v[2];
        double result;
    };

    /**
     * Mean diff filter results.
     */
    struct MeanDiff {
        std::unique_ptr<double[]> values;
        uint8_t i;
        double sum;
        uint8_t count;
    };

    /**
     * @brief Pulse detection algorithm.
     * 
     * @param sensor_value is the value read from the sensor after the filters
     * 
     * @returns true if detected.
     */
    bool detect_pulse(double sensor_value);

    /**
     * @brief Balance intensities filter.
     * 
     * @details DC filter for the raw values.
     * 
     * @param red_dc is the w in red led dc filter structure.
     * @param ir_dc is the w in ir led dc filter structure.
     */
    void balance_intensities(double red_dc, double ir_dc);

    /**
     * @brief Removes the DC offset of the sensor value.
     * 
     * @param x is the raw value read from the led.
     * @param prev_w is the previous filtered w from the dc filter structure.
     * @param alpha is the dc filter alpha parameter.
     * 
     * @returns a dc filter structure.
     */
    struct Filter dc_removal(double x, double prev_w, double alpha);

    /**
     * @brief Applies the mean diff filter.
     * 
     * @param M is the filtered DC result of the dc filter structure.
     * 
     * @returns a filtered value.
     */
    double mean_diff(double M);

    /**
     * @brief Applies a low-pass butterworth filter.
     * 
     * @param this is the address of the configuration structure.
     * @param x is the mean diff filtered value.
     */
    void lpb_filter(double x);

    I2CDevice i2c;

    i2c_port_t i2c_num;
    uint32_t acceptable_intensity_diff;
    Current red_current;
    uint32_t red_current_adj_ms;
    uint8_t rst_spo2_pulse_n;
    double dc_alpha;
    uint16_t pulse_min_threshold;
    uint16_t pulse_max_threshold;

    uint8_t mean_filter_size;
    uint8_t pulse_bpm_sample_size;

    bool debug;
    PulseState current_pulse_state;

    Current ir_current;
    double last_red_current_check;

    struct MeanDiff mean_diff_ir;
    std::unique_ptr<double[]> values_bpm;

    SemaphoreHandle_t mtx_result;

    double current_bpm;
    double bpm_sum;
    uint8_t bpm_count;
    uint8_t bpm_i;

    struct Fifo prev_fifo;
    struct Filter dc_filter_ir;
    struct Filter dc_filter_red;
    struct Butterworth lpb_filter_ir;
    
    double ir_ac_sq_sum;
    double red_ac_sq_sum;
    uint16_t samples_recorded;
    uint16_t pulses_detected;
    double current_spO2;

    bool pulse_detected;
    double heart_bpm;
    double ir_cardiogram;
    double ir_dc_val;
    double red_dc_val;
    double spo2;
    uint32_t last_beat_thresh;
    double dc_filtered_red;
    double dc_filtered_ir;

    static const uint8_t DEV_ADDR;
    static const uint8_t REV_ID;
    static const uint8_t PART_ID;
    static const uint8_t INT_STATUS;
    static const uint8_t INT_ENABLE;
    static const uint8_t FIFO_WRITE;
    static const uint8_t FIFO_OVF_COUNTER;
    static const uint8_t FIFO_READ;
    static const uint8_t FIFO_DATA;
    static const uint8_t MODE_CONF;
    static const uint8_t SPO2_CONF;
    static const uint8_t LED_CONF;
    static const uint8_t TEMP_INT;
    static const uint8_t TEMP_FRAC;

    static const uint8_t MODE_SHDN;
    static const uint8_t MODE_RST;
    static const uint8_t MODE_TEMP_EN;
    static const uint8_t SPO2_HI_RES_EN;

    static const char *TAG;
};

#endif

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