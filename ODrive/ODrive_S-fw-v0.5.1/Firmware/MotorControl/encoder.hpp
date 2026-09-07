#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        // Default INCREMENTAL, not MT6835: pwm_trig_adc_cb() starts an abs-SPI
        // transaction every cycle for ANY axis whose mode has MODE_FLAG_ABS, even
        // when that axis is idle. With the factory default of MODE_SPI_ABS_MT6835
        // an unconfigured axis1 hammers the SPI3 bus shared with axis0's encoder
        // (and, sharing abs_spi_cs_gpio_pin=6, toggles the same CS), which
        // corrupts axis0's reads (spi_error_rate -> 1). This came back after every
        // erase_configuration()/NVM-load-fail. Configure axis0.encoder.config.mode
        // explicitly, like stock ODrive.
        Mode mode = MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t cpr = (1 << 21);   // MT6835: 21-bit angle (2^21 counts per turn)
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 6;  // GPIO_6 (PB2)
        uint16_t sincos_gpio_pin_sin = 3;
        uint16_t sincos_gpio_pin_cos = 4;

        // custom setters
        Encoder* parent = nullptr;
        void set_use_index(bool value) { use_index = value; parent->set_idx_subscribe(); }
        void set_find_idx_on_lockin_only(bool value) { find_idx_on_lockin_only = value; parent->set_idx_subscribe(); }
        void set_abs_spi_cs_gpio_pin(uint16_t value) { abs_spi_cs_gpio_pin = value; parent->abs_spi_cs_pin_init(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
            Config_t& config, const Motor::Config_t& motor_config);
    
    void setup();
    void set_error(Error error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();

    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;        // [count]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    float pos_estimate_ = 0.0f; // [turn]
    float vel_estimate_ = 0.0f; // [turn/s]
    float pos_cpr_ = 0.0f;      // [turn]
    float pos_circular_ = 0.0f; // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_init();
    bool abs_spi_start_transaction();
    void abs_spi_cb();
    void abs_spi_cs_pin_init();
    /** Seed ABZ timer from last SPI absolute position; see MODE_SPI_THEN_ABZ_MT6835. */
    void spi_then_abz_handoff_from_abs_spi();
    /** SPI3 is shared with the DRV8301 gate driver. A DRV8301 transaction calls
     *  spi3_lock_for_drv() so the ADC-ISR-driven absolute-encoder transfer skips
     *  its turn instead of colliding / running with the wrong frame format, then
     *  spi3_unlock_for_drv() when done. See Motor::check_DRV_fault(). */
    static void spi3_lock_for_drv();
    static void spi3_unlock_for_drv();
    static bool spi3_locked_for_drv();
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    static constexpr size_t MT6835_SPI_XFER_BYTES = 6;
    uint8_t mt6835_dma_tx_[6] = {0};
    uint8_t mt6835_dma_rx_[6];
    uint8_t mt6835_accum_[4] = {0};
    bool abs_spi_pos_updated_ = false;
    /** After first good MT6835 SPI read: timer/PLL seeded, ABZ incremental path used (MODE_SPI_THEN_ABZ_MT6835 only). */
    bool spi_then_abz_handoff_done_ = false;
    Mode mode_ = MODE_INCREMENTAL;
    GPIO_TypeDef* abs_spi_cs_port_;
    uint16_t abs_spi_cs_pin_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }
};

#endif // __ENCODER_HPP
