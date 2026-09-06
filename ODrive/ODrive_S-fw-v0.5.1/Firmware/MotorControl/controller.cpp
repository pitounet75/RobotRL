
#include "odrive_main.h"
#include <algorithm>

Controller::Controller(Config_t& config) :
    config_(config)
{
    update_filter_gains();
}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
}

void Controller::set_error(Error error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------


bool Controller::select_encoder(size_t encoder_num) {
    if (encoder_num < AXIS_COUNT) {
        Axis* ax = axes[encoder_num];
        pos_estimate_circular_src_ = &ax->encoder_.pos_circular_;
        pos_wrap_src_ = &config_.circular_setpoint_range;
        pos_estimate_linear_src_ = &ax->encoder_.pos_estimate_;
        pos_estimate_valid_src_ = &ax->encoder_.pos_estimate_valid_;
        vel_estimate_src_ = &ax->encoder_.vel_estimate_;
        vel_estimate_valid_src_ = &ax->encoder_.vel_estimate_valid_;
        return true;
    } else {
        return set_error(Controller::ERROR_INVALID_LOAD_ENCODER), false;
    }
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.index = 0;
        anticogging_settle_streak_ = 0;
        anticogging_calib_phase_ = 0;
        anticogging_finalize_idx_ = 0;
        anticogging_finalize_sum_ = 0.0f;
        for (float& v : anticogging_rev_buffer_) {
            v = 0.0f;
        }
        config_.anticogging.calib_anticogging = true;
    }
}

float Controller::get_anticogging_value(uint32_t index) {
    const uint32_t i = std::min<uint32_t>(index, 3599u);
    return config_.anticogging.cogging_map[i];
}

// Host-side restore of a previously dumped cogging map (no calibration sweep).
// Out-of-range indices are ignored so a partial/garbled stream cannot corrupt
// neighbouring config. The caller is expected to disable anticogging while
// streaming, then set anticogging.pre_calibrated and call save_configuration().
void Controller::set_anticogging_value(uint32_t index, float value) {
    if (index < 3600u) {
        config_.anticogging.cogging_map[index] = value;
    }
}

/*
 * Anti-cogging: forward sweep (bins 0..3599), then reverse sweep (bins 3599..0), same hold
 * per bin. Reverse samples are stored per bin index. Combined map = (forward + reverse) / 2,
 * then subtract global mean (zero-mean / balanced map).
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    // ~15 ms at 8 kHz: require stable settle to avoid advancing on limit-cycle zero-crossings.
    static constexpr uint16_t kSettleCycles = 120;
    static constexpr uint32_t kBins = 3600;
    const float cpr = (float)axis_->encoder_.config_.cpr;
    const float ratio = axis_->encoder_.getCoggingRatio();
    // Match position-loop geometry: circular mode uses wrapped error vs pos_circular_,
    // not linear pos_estimate, or settling never happens after the first bin.
    float pos_err;
    if (config_.circular_setpoints) {
        const float range = config_.circular_setpoint_range;
        const float cmd = fmodf_pos(input_pos_, range);
        pos_err = cmd - axis_->encoder_.pos_circular_;
        pos_err = wrap_pm(pos_err, 0.5f * range);
    } else {
        pos_err = input_pos_ - pos_estimate;
    }
    const bool settled =
        (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / cpr &&
         std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / cpr);
    if (settled) {
        if (anticogging_settle_streak_ < UINT16_MAX) {
            anticogging_settle_streak_++;
        }
    } else {
        anticogging_settle_streak_ = 0;
    }
    const bool settled_stable = settled && (anticogging_settle_streak_ >= kSettleCycles);

    if (settled_stable) {
        anticogging_settle_streak_ = 0;
        const uint32_t i = std::min<uint32_t>(config_.anticogging.index, 3599u);
        if (anticogging_calib_phase_ == 0) {
            // Forward pass: sample bin i, then advance toward 3600
            config_.anticogging.cogging_map[i] = vel_integrator_torque_;
            config_.anticogging.index++;
            if (config_.anticogging.index >= kBins) {
                anticogging_calib_phase_ = 1;
                config_.anticogging.index = 3599;
            }
        } else if (anticogging_calib_phase_ == 1) {
            // Reverse pass: visit bins 3599..0; store at bin i (aligned with forward indexing)
            anticogging_rev_buffer_[i] = vel_integrator_torque_;
            if (i == 0) {
                anticogging_calib_phase_ = 2;
            } else {
                config_.anticogging.index--;
            }
        }
    }

    // Finalize (average fwd+rev, then subtract the global mean) is chunked across control cycles:
    // doing the full 2x3600 float pass inline in one 8 kHz iteration overran the control deadline.
    static constexpr uint32_t kFinalizeChunk = 128;

    if (anticogging_calib_phase_ == 2 || anticogging_calib_phase_ == 3) {
        const uint32_t end = std::min<uint32_t>(anticogging_finalize_idx_ + kFinalizeChunk, kBins);
        if (anticogging_calib_phase_ == 2) {
            // Pass A: cogging_map[j] = 0.5*(forward[j] + reverse[j]); accumulate sum for the mean.
            for (uint32_t j = anticogging_finalize_idx_; j < end; j++) {
                const float v = 0.5f * (config_.anticogging.cogging_map[j] + anticogging_rev_buffer_[j]);
                config_.anticogging.cogging_map[j] = v;
                anticogging_finalize_sum_ += v;
            }
        } else {
            // Pass B: subtract the global mean -> zero-mean map.
            for (uint32_t j = anticogging_finalize_idx_; j < end; j++) {
                config_.anticogging.cogging_map[j] -= anticogging_finalize_mean_;
            }
        }
        anticogging_finalize_idx_ = end;

        // Keep holding position (motor is parked at bin 0 from the end of the reverse sweep).
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();

        if (anticogging_finalize_idx_ >= kBins) {
            anticogging_finalize_idx_ = 0;
            if (anticogging_calib_phase_ == 2) {
                anticogging_finalize_mean_ = anticogging_finalize_sum_ / (float)kBins;
                anticogging_calib_phase_ = 3;
            } else {
                config_.anticogging.index = 0;
                input_pos_ = 0.0f;
                input_vel_ = 0.0f;
                input_torque_ = 0.0f;
                input_pos_updated();
                anticogging_valid_ = true;
                config_.anticogging.calib_anticogging = false;
                anticogging_calib_phase_ = 0;
                anticogging_finalize_sum_ = 0.0f;
                return true;
            }
        }
        return false;
    }

    if (anticogging_calib_phase_ == 0 && config_.anticogging.index < kBins) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * ratio;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    }
    if (anticogging_calib_phase_ == 1) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * ratio;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    }

    return false;
}

void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update(float* torque_setpoint_output) {
    float* pos_estimate_linear = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_linear_src_ : nullptr;
    float* pos_estimate_circular = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_circular_src_ : nullptr;
    float* vel_estimate_src = (vel_estimate_valid_src_ && *vel_estimate_valid_src_)
            ? vel_estimate_src_ : nullptr;

    // Calib_anticogging is only true when calibration is occurring, so we can't block anticogging_pos
    float anticogging_pos = axis_->encoder_.pos_estimate_ / axis_->encoder_.getCoggingRatio();
    if (config_.anticogging.calib_anticogging) {
        if (!axis_->encoder_.pos_estimate_valid_ || !axis_->encoder_.vel_estimate_valid_) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(axis_->encoder_.pos_estimate_, axis_->encoder_.vel_estimate_);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints && !config_.anticogging.calib_anticogging) {
        // Keep pos setpoint from drifting
        input_pos_ = fmodf_pos(input_pos_, config_.circular_setpoint_range);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                pos_setpoint_ = axes[config_.axis_to_mirror]->encoder_.pos_estimate_ * config_.mirror_ratio;
                vel_setpoint_ = axes[config_.axis_to_mirror]->encoder_.vel_estimate_ * config_.mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = input_pos_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if(!pos_estimate_circular) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap_src_);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, 0.5f * *pos_wrap_src_);
        } else {
            if(!pos_estimate_linear) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate_src) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->motor_.current_control_.acim_rotor_flux;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (fabsf(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        // Interpolate linearly between adjacent bins so the feed-forward torque is continuous.
        // The 3600-entry map is circular over one turn; anticogging_pos is in bin units
        // (pos * 3600), or the trajectory setpoint in TRAP_TRAJ mode (see above).
        const float turns = anticogging_pos * (1.0f / 3600.0f);
        const float binf  = (turns - floorf(turns)) * 3600.0f;   // [0, 3600)
        const int   i0    = std::clamp((int)binf, 0, 3599);
        const int   i1    = (i0 + 1) % 3600;                      // circular wrap
        const float frac  = binf - (float)i0;                     // [0, 1)
        const float* m    = config_.anticogging.cogging_map;
        torque += m[i0] + frac * (m[i1] - m[i0]);
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate_src;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_current_mode_vel_limit) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate_src, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    float Tlim = axis_->motor_.max_available_torque();
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
    }

    if (torque_setpoint_output) *torque_setpoint_output = torque;
    return true;
}
