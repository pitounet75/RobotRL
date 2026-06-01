
#include "odrive_main.h"

namespace {

enum class AbsSpiFmt : uint8_t {
    W16Std = 0,
    W16Aeat = 1,
    W8Mt6835 = 2,
    Unset = 0xff,
};

static SPI_HandleTypeDef* s_abs_spi = nullptr;
static AbsSpiFmt s_abs_fmt = AbsSpiFmt::Unset;

static void abs_spi_apply_hw(SPI_HandleTypeDef* spi, Encoder::Mode mode) {
    AbsSpiFmt want = AbsSpiFmt::Unset;
    if (mode == Encoder::MODE_SPI_ABS_MT6835 || mode == Encoder::MODE_SPI_THEN_ABZ_MT6835) {
        want = AbsSpiFmt::W8Mt6835;
    } else if (mode == Encoder::MODE_SPI_ABS_AEAT) {
        want = AbsSpiFmt::W16Aeat;
    } else if (mode & Encoder::MODE_FLAG_ABS) {
        want = AbsSpiFmt::W16Std;
    } else {
        return;
    }

    if (s_abs_spi == spi && s_abs_fmt == want) {
        return;
    }
    s_abs_spi = spi;
    s_abs_fmt = want;

    spi->Init.Mode = SPI_MODE_MASTER;
    spi->Init.Direction = SPI_DIRECTION_2LINES;
    spi->Init.NSS = SPI_NSS_SOFT;
    spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi->Init.TIMode = SPI_TIMODE_DISABLE;
    spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->Init.CRCPolynomial = 10;

    if (want == AbsSpiFmt::W8Mt6835) {
        /* MT6835 datasheet §7.6.2: SPI mode 3 (CPOL=1, CPHA=1). */
        spi->Init.DataSize = SPI_DATASIZE_8BIT;
        spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    } else {
        spi->Init.DataSize = SPI_DATASIZE_16BIT;
        spi->Init.CLKPolarity = (want == AbsSpiFmt::W16Aeat) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    }
    HAL_SPI_DeInit(spi);
    HAL_SPI_Init(spi);
}

/* MT6835 datasheet §7.6.9 burst read: C3~C0=1010 with address 0x003, CSN held low
 * while the IC outputs regs 0x003~0x006. One 8-bit SPI transaction of 6 bytes:
 * TX-first 3 bytes = command, RX[2..5] = data from 0x003..0x006; send 0xFF on MOSI
 * for the last 3 bytes to clock out the burst. */
static void mt6835_fill_burst_tx(uint8_t* tx6) {
    const uint16_t cmd = (uint16_t)(0xA000u | (0x003u & 0x0FFFu));
    tx6[0] = (uint8_t)(cmd >> 8);
    tx6[1] = (uint8_t)(cmd & 0xFFu);
    tx6[2] = 0xFFu;
    tx6[3] = 0xFFu;
    tx6[4] = 0xFFu;
    tx6[5] = 0xFFu;
}

/* CRC-8 (polynomial x^8+x^2+x+1), same table as MagnTek reference sketches */
static uint8_t mt6835_crc8(const uint8_t* data, size_t len) {
    static const uint8_t kCrcTable[256] = {
        0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d,
        0x70,0x77,0x7e,0x79,0x6c,0x6b,0x62,0x65,0x48,0x4f,0x46,0x41,0x54,0x53,0x5a,0x5d,
        0xe0,0xe7,0xee,0xe9,0xfc,0xfb,0xf2,0xf5,0xd8,0xdf,0xd6,0xd1,0xc4,0xc3,0xca,0xcd,
        0x90,0x97,0x9e,0x99,0x8c,0x8b,0x82,0x85,0xa8,0xaf,0xa6,0xa1,0xb4,0xb3,0xba,0xbd,
        0xc7,0xc0,0xc9,0xce,0xdb,0xdc,0xd5,0xd2,0xff,0xf8,0xf1,0xf6,0xe3,0xe4,0xed,0xea,
        0xb7,0xb0,0xb9,0xbe,0xab,0xac,0xa5,0xa2,0x8f,0x88,0x81,0x86,0x93,0x94,0x9d,0x9a,
        0x27,0x20,0x29,0x2e,0x3b,0x3c,0x35,0x32,0x1f,0x18,0x11,0x16,0x03,0x04,0x0d,0x0a,
        0x57,0x50,0x59,0x5e,0x4b,0x4c,0x45,0x42,0x6f,0x68,0x61,0x66,0x73,0x74,0x7d,0x7a,
        0x89,0x8e,0x87,0x80,0x95,0x92,0x9b,0x9c,0xb1,0xb6,0xbf,0xb8,0xad,0xaa,0xa3,0xa4,
        0xf9,0xfe,0xf7,0xf0,0xe5,0xe2,0xeb,0xec,0xc1,0xc6,0xcf,0xc8,0xdd,0xda,0xd3,0xd4,
        0x69,0x6e,0x67,0x60,0x75,0x72,0x7b,0x7c,0x51,0x56,0x5f,0x58,0x4d,0x4a,0x43,0x44,
        0x19,0x1e,0x17,0x10,0x05,0x02,0x0b,0x0c,0x21,0x26,0x2f,0x28,0x3d,0x3a,0x33,0x34,
        0x4e,0x49,0x40,0x47,0x52,0x55,0x5c,0x5b,0x76,0x71,0x78,0x7f,0x6a,0x6d,0x64,0x63,
        0x3e,0x39,0x30,0x37,0x22,0x25,0x2c,0x2b,0x06,0x01,0x08,0x0f,0x1a,0x1d,0x14,0x13,
        0xae,0xa9,0xa0,0xa7,0xb2,0xb5,0xbc,0xbb,0x96,0x91,0x98,0x9f,0x8a,0x8d,0x84,0x83,
        0xde,0xd9,0xd0,0xd7,0xc2,0xc5,0xcc,0xcb,0xe6,0xe1,0xe8,0xef,0xfa,0xfd,0xf4,0xf3,
    };
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        crc = kCrcTable[crc];
    }
    return crc;
}

} // namespace


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                Config_t& config, const Motor::Config_t& motor_config) :
        hw_config_(hw_config),
        config_(config)
{
    update_pll_gains();

    if (config.pre_calibrated) {
        if (config.mode == Encoder::MODE_HALL || config.mode == Encoder::MODE_SINCOS)
            is_ready_ = true;
        if (motor_config.motor_type == Motor::MOTOR_TYPE_ACIM)
            is_ready_ = true;
    }
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    set_idx_subscribe();

    mode_ = config_.mode;
    spi_then_abz_handoff_done_ = false;
    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();
        abs_spi_init();
        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}

void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    if (config_.use_index) {
        set_circular_count(0, false);
        if (config_.zero_count_on_find_idx)
            set_linear_count(0); // Avoid position control transient after search
        if (config_.pre_calibrated) {
            is_ready_ = true;
            if(axis_->controller_.config_.anticogging.pre_calibrated){
                axis_->controller_.anticogging_valid_ = true;
            }
        } else {
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }

    // Disable interrupt
    GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
}

void Encoder::set_idx_subscribe(bool override_enable) {
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_PULLDOWN,
                enc_index_cb_wrapper, this);
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
    }
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (!is_ready_ && axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM)
        config_.pre_calibrated = false;
    if (mode_ == MODE_INCREMENTAL && !index_found_)
        config_.pre_calibrated = false;
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;
    tim_cnt_sample_ = count;

    //Write hardware last
    hw_config_.timer->Instance->CNT = count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.offset += count - count_in_cpr_;
        config_.offset = mod(config_.offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    if (!config_.idx_search_unidirectional && axis_->motor_.config_.direction == 0) {
        axis_->motor_.config_.direction = 1;
    }
    set_idx_subscribe();

    bool status = axis_->run_lockin_spin(axis_->config_.calibration_lockin);
    return status;
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;
    axis_->motor_.config_.direction = 1; // Must test spin forwards for direction detect logic

    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    bool status = axis_->run_lockin_spin(lockin_config);

    if (status) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            axis_->motor_.config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            axis_->motor_.config_.direction = -1;
        } else {
            axis_->motor_.config_.direction = 0;
        }
    }

    return status;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;
    const int num_steps = (int)(config_.calib_scan_distance / config_.calib_scan_omega * (float)current_meas_hz);

    // Require index found if enabled
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        if (!axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(TIMING_LOG_ENC_CALIB);
        return ++i < start_lock_duration * current_meas_hz;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    int32_t init_enc_val = shadow_count_;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis_->run_control_loop([&]() {
        float phase = wrap_pm_pi(config_.calib_scan_distance * (float)i / (float)num_steps - config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        return false;
    }

    //TODO avoid recomputing elec_rad_per_enc every time
    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&]() {
        float phase = wrap_pm_pi(-config_.calib_scan_distance * (float)i / (float)num_steps + config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    config_.offset = encvaluesum / (num_steps * 2);
    int32_t residual = encvaluesum - ((int64_t)config_.offset * (int64_t)(num_steps * 2));
    config_.offset_float = (float)residual / (float)(num_steps * 2) + 0.5f;  // add 0.5 to center-align state to phase

    /* Switch to ABZ only after offset cal: CPR check uses shadow_count_ in MT6835 count
     * space; early handoff made shadow track 16-bit quadrature and broke the ratio test. */
    spi_then_abz_handoff_from_abs_spi();

    is_ready_ = true;
    return true;
}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}

void Encoder::sample_now() {
    switch (mode_) {
        case MODE_INCREMENTAL: {
            tim_cnt_sample_ = (int16_t)hw_config_.timer->Instance->CNT;
        } break;

        case MODE_HALL: {
            // do nothing: samples already captured in general GPIO capture
        } break;

        case MODE_SINCOS: {
            sincos_sample_s_ = (get_adc_voltage(get_gpio_port_by_pin(config_.sincos_gpio_pin_sin), get_gpio_pin_by_pin(config_.sincos_gpio_pin_sin)) / 3.3f) - 0.5f;
            sincos_sample_c_ = (get_adc_voltage(get_gpio_port_by_pin(config_.sincos_gpio_pin_cos), get_gpio_pin_by_pin(config_.sincos_gpio_pin_cos)) / 3.3f) - 0.5f;
        } break;

        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI:
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_MT6835:
        {
            axis_->motor_.log_timing(TIMING_LOG_SAMPLE_NOW);
            // Do nothing
        } break;

        case MODE_SPI_THEN_ABZ_MT6835: {
            if (spi_then_abz_handoff_done_) {
                tim_cnt_sample_ = (int16_t)hw_config_.timer->Instance->CNT;
            } else {
                axis_->motor_.log_timing(TIMING_LOG_SAMPLE_NOW);
            }
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
}

bool Encoder::abs_spi_init(){
    if ((mode_ & MODE_FLAG_ABS) == 0x0)
        return false;

    abs_spi_apply_hw(hw_config_.spi, mode_);
    return true;
}

bool Encoder::abs_spi_start_transaction(){
    if (mode_ & MODE_FLAG_ABS){
        if (mode_ == MODE_SPI_THEN_ABZ_MT6835 && spi_then_abz_handoff_done_) {
            return true;
        }
        abs_spi_apply_hw(hw_config_.spi, mode_);
        axis_->motor_.log_timing(TIMING_LOG_SPI_START);
        if(hw_config_.spi->State != HAL_SPI_STATE_READY){
            set_error(ERROR_ABS_SPI_NOT_READY);
            return false;
        }
        HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_RESET);
        if (mode_ == MODE_SPI_ABS_MT6835 || (mode_ == MODE_SPI_THEN_ABZ_MT6835 && !spi_then_abz_handoff_done_)) {
            mt6835_fill_burst_tx(mt6835_dma_tx_);
            HAL_SPI_TransmitReceive_DMA(hw_config_.spi, mt6835_dma_tx_, mt6835_dma_rx_,
                                        MT6835_SPI_XFER_BYTES);
        } else {
            HAL_SPI_TransmitReceive_DMA(hw_config_.spi, (uint8_t*)abs_spi_dma_tx_, (uint8_t*)abs_spi_dma_rx_, 1);
        }
    }
    return true;
}

uint8_t ams_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

uint8_t cui_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    return ~v & 3;
}

void Encoder::abs_spi_cb(){
    axis_->motor_.log_timing(TIMING_LOG_SPI_END);

    const bool mt6835_xfer = (mode_ == MODE_SPI_ABS_MT6835)
        || (mode_ == MODE_SPI_THEN_ABZ_MT6835 && !spi_then_abz_handoff_done_);

    if (mt6835_xfer) {
        HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_SET);

        mt6835_accum_[0] = mt6835_dma_rx_[2];
        mt6835_accum_[1] = mt6835_dma_rx_[3];
        mt6835_accum_[2] = mt6835_dma_rx_[4];
        mt6835_accum_[3] = mt6835_dma_rx_[5];

        if (mt6835_crc8(mt6835_accum_, 3) != mt6835_accum_[3]) {
            return;
        }

        uint32_t raw21 = ((uint32_t)mt6835_accum_[0] << 13)
                       | ((uint32_t)mt6835_accum_[1] << 5)
                       | ((uint32_t)mt6835_accum_[2] >> 3);
        raw21 &= 0x1FFFFFu;
        pos_abs_ = (int32_t)((raw21 * (uint64_t)(uint32_t)config_.cpr) >> 21);

        abs_spi_pos_updated_ = true;
        /* Defer SPI→ABZ handoff until offset cal ends so CPR check sees MT6835-scaled
         * counts. If pre_calibrated skips offset cal, hand off on first good read. */
        if (mode_ == MODE_SPI_THEN_ABZ_MT6835 && !spi_then_abz_handoff_done_ && config_.pre_calibrated) {
            spi_then_abz_handoff_from_abs_spi();
        }
        if (config_.pre_calibrated) {
            is_ready_ = true;
        }
        return;
    }

    HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_SET);

    uint16_t pos;

    switch (mode_) {
        case MODE_SPI_ABS_AMS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct (even) and error flag clear
            if (ams_parity(rawVal) || ((rawVal >> 14) & 1)) {
                return;
            }
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_CUI: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct
            if (cui_parity(rawVal)) {
                return;
            }
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_RLS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            pos = (rawVal >> 2) & 0x3fff;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return;
        } break;
    }

    pos_abs_ = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated) {
        is_ready_ = true;
    }
}

void Encoder::spi_then_abz_handoff_from_abs_spi() {
    if (mode_ != MODE_SPI_THEN_ABZ_MT6835 || spi_then_abz_handoff_done_) {
        return;
    }
    int32_t p = mod(pos_abs_, config_.cpr);
    set_circular_count(p, false);
    set_linear_count(p);
    spi_then_abz_handoff_done_ = true;
    spi_error_rate_ = 0.0f;
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode cs pin
    abs_spi_cs_port_ = get_gpio_port_by_pin(config_.abs_spi_cs_gpio_pin);
    abs_spi_cs_pin_ = get_gpio_pin_by_pin(config_.abs_spi_cs_gpio_pin);

    // Init cs pin
    HAL_GPIO_DeInit(abs_spi_cs_port_, abs_spi_cs_pin_);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = abs_spi_cs_pin_;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(abs_spi_cs_port_, &GPIO_InitStruct);

    // Write pin high
    HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_SET);
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH

    switch (mode_) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            int32_t hall_cnt;
            if (decode_hall(hall_state_, &hall_cnt)) {
                delta_enc = hall_cnt - count_in_cpr_;
                delta_enc = mod(delta_enc, 6);
                if (delta_enc > 3)
                    delta_enc -= 6;
            } else {
                if (!config_.ignore_illegal_hall_state) {
                    set_error(ERROR_ILLEGAL_HALL_STATE);
                    return false;
                }
            }
        } break;

        case MODE_SINCOS: {
            float phase = fast_atan2(sincos_sample_s_, sincos_sample_c_);
            int fake_count = (int)(1000.0f * phase);
            //CPR = 6283 = 2pi * 1k

            delta_enc = fake_count - count_in_cpr_;
            delta_enc = mod(delta_enc, 6283);
            if (delta_enc > 6283/2)
                delta_enc -= 6283;
        } break;
        
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI: 
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_MT6835: {
            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.005f)
                    set_error(ERROR_ABS_SPI_COM_FAIL);
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2) {
                delta_enc -= config_.cpr;
            }

        }break;

        case MODE_SPI_THEN_ABZ_MT6835: {
            if (spi_then_abz_handoff_done_) {
                int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
                delta_enc = (int32_t)delta_enc_16;
            } else {
                if (abs_spi_pos_updated_ == false) {
                    spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                    if (spi_error_rate_ > 0.005f)
                        set_error(ERROR_ABS_SPI_COM_FAIL);
                } else {
                    spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
                }
                abs_spi_pos_updated_ = false;
                delta_enc = pos_abs_latched - count_in_cpr_;
                delta_enc = mod(delta_enc, config_.cpr);
                if (delta_enc > config_.cpr/2) {
                    delta_enc -= config_.cpr;
                }
            }
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if ((mode_ & MODE_FLAG_ABS) && !(mode_ == MODE_SPI_THEN_ABZ_MT6835 && spi_then_abz_handoff_done_)) {
        count_in_cpr_ = pos_abs_latched;
    }

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - (int32_t)std::floor(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)std::floor(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    float pos_cpr_last = pos_cpr_;
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;
    pos_cpr_= pos_cpr_counts_ / (float)config_.cpr;
    float delta_pos_cpr = wrap_pm(pos_cpr_ - pos_cpr_last, 0.5f);
    pos_circular_ += delta_pos_cpr;
    pos_circular_ = fmodf_pos(pos_circular_, axis_->controller_.config_.circular_setpoint_range);

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

    vel_estimate_valid_ = true;
    pos_estimate_valid_ = true;
    return true;
}
