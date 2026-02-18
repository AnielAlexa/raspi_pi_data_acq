#include "camera_display_node/ae_controller.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>

#include "auto_exposure_control/auto_exposure_percentile.h"
#include "auto_exposure_control/auto_exposure_intensity.h"
#include "photometric_camera/photometric_model.h"

namespace camera_display_node {

static std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

bool loadTuningFile(const std::string& path, TuningParams* params) {
    if (!params) return false;
    std::ifstream in(path);
    if (!in.is_open()) return false;

    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no++;
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if (key.empty() || value.empty()) continue;

        try {
            if (key == "max_exposure_ms") params->max_exposure_ms = std::stod(value);
            else if (key == "min_gain_x") params->min_gain_x = std::stof(value);
            else if (key == "max_gain_x") params->max_gain_x = std::stof(value);
            else if (key == "gain_change_step_x") params->gain_change_step_x = std::stof(value);
            else if (key == "mean_method_max_step_ratio") params->mean_method_max_step_ratio = std::stod(value);
            else if (key == "percentile_damping") params->percentile_damping = std::stod(value);
            else if (key == "percentile_max_step_ratio") params->percentile_max_step_ratio = std::stod(value);
            else if (key == "percentile_deadband_us") params->percentile_deadband_us = std::stoi(value);
            else if (key == "gain_cooldown_frames") params->gain_cooldown_frames = std::stoi(value);
        } catch (const std::exception&) {
            std::cerr << "AE tuning: invalid value for '" << key << "': " << value << std::endl;
        }
    }

    if (params->min_gain_x > params->max_gain_x)
        std::swap(params->min_gain_x, params->max_gain_x);
    params->max_exposure_ms = std::max(0.2, params->max_exposure_ms);
    params->gain_change_step_x = std::max(0.01f, params->gain_change_step_x);
    params->mean_method_max_step_ratio = std::max(0.01, params->mean_method_max_step_ratio);
    params->percentile_damping = std::max(0.01, std::min(1.0, params->percentile_damping));
    params->percentile_max_step_ratio = std::max(0.01, params->percentile_max_step_ratio);
    params->percentile_deadband_us = std::max(0, params->percentile_deadband_us);
    params->gain_cooldown_frames = std::max(0, params->gain_cooldown_frames);
    return true;
}

AEController::~AEController() = default;

bool AEController::init(const std::string& method,
                         const std::string& calib_dir,
                         const std::string& ga_profile,
                         const TuningParams& tuning) {
    method_ = method;
    tuning_ = tuning;
    gain_cooldown_ = 0;

    // Always create the intensity controller (used as fallback and for mean method)
    intensity_ctrl_ = std::make_unique<auto_exposure::AutoExposureIntensity>();

    if (method_ == "percentile") {
        // Load photometric model
        photo_model_ = photometric_camera::PhotometricModel::loadModel(calib_dir, -1.0);
        if (!photo_model_) {
            std::cerr << "AE: Failed to load photometric model from " << calib_dir << std::endl;
            return false;
        }

        // Configure percentile options
        auto_exposure::AutoExposurePercentileOptions options;
        options.use_intensity_high_bound = 170;
        options.rate_over_comp_thresh = 0.15;
        options.ga_rate_profile = ga_profile;

        if (ga_profile.empty()) {
            std::cerr << "AE: ga_profile path is empty, disabling auto_ga_rate" << std::endl;
            options.auto_ga_rate = false;
        }

        auto_exposure_utils::OverUnderExposureCompOptions comp_options;
        percentile_ctrl_ = std::make_unique<auto_exposure::AutoExposurePercentile>(
            options, comp_options, photo_model_);
    }

    return true;
}

int AEController::applyExposureStepLimit(int proposed_us, int last_us,
                                          double max_step_ratio) {
    max_step_ratio = std::max(0.0, max_step_ratio);
    if (last_us <= 0 || max_step_ratio <= 0.0) return proposed_us;

    const int max_delta = static_cast<int>(std::round(last_us * max_step_ratio));
    const int delta = proposed_us - last_us;
    if (std::abs(delta) <= max_delta) return proposed_us;
    return last_us + (delta > 0 ? max_delta : -max_delta);
}

AEResult AEController::compute(const cv::Mat& small_gray,
                                int current_exp_us,
                                float current_gain_x) {
    AEResult result;
    result.desired_exp_us = current_exp_us;
    result.desired_gain_x = current_gain_x;
    result.changed = false;

    const int last_exp_us = current_exp_us;
    const int max_exp_us = static_cast<int>(std::round(tuning_.max_exposure_ms * 1000.0));
    const int min_exp_us = 20;

    // --- Step 1: compute desired exposure (mirrors uzh-rpg calculateNewSettings) ---
    int desired_exp_us = last_exp_us;
    if (method_ == "mean") {
        desired_exp_us = intensity_ctrl_->computeDesiredExposureTimeIntensity(
            small_gray, last_exp_us);
    } else {
        // percentile: the controller internally switches to intensity-based
        // control when mean > use_intensity_high_bound (170), so no extra
        // blending or damping is needed here — trust the controller output.
        desired_exp_us = percentile_ctrl_->computeDesiredExposureWeightedGradient(
            small_gray, last_exp_us, current_gain_x);
    }

    // --- Step 2: clamp (mirrors uzh-rpg calculateNewSettings clamp block) ---
    desired_exp_us = std::max(min_exp_us, std::min(desired_exp_us, max_exp_us));

    // --- Step 3: adjust gain (mirrors uzh-rpg adjustGain()) ---
    const int dec_gain_exp_us = static_cast<int>(0.2 * max_exp_us);
    const int inc_gain_exp_us = static_cast<int>(0.9 * max_exp_us);

    float desired_gain_x = current_gain_x;
    bool allow_gain_change = (gain_cooldown_ <= 0);

    if (allow_gain_change && desired_exp_us < dec_gain_exp_us) {
        desired_gain_x = std::max(tuning_.min_gain_x,
                                   current_gain_x - tuning_.gain_change_step_x);
        if (photo_model_) {
            desired_exp_us = photo_model_->getExpForNewGain(
                current_gain_x, desired_gain_x, desired_exp_us);
        }
    } else if (allow_gain_change && desired_exp_us > inc_gain_exp_us) {
        desired_gain_x = std::min(tuning_.max_gain_x,
                                   current_gain_x + tuning_.gain_change_step_x);
        if (photo_model_) {
            desired_exp_us = photo_model_->getExpForNewGain(
                current_gain_x, desired_gain_x, desired_exp_us);
        }
    }

    // Final clamp after gain compensation
    desired_exp_us = std::max(min_exp_us, std::min(desired_exp_us, max_exp_us));

    // Update gain cooldown
    if (std::abs(desired_gain_x - current_gain_x) > 0.01f) {
        gain_cooldown_ = tuning_.gain_cooldown_frames;
    } else if (gain_cooldown_ > 0) {
        gain_cooldown_--;
        desired_gain_x = current_gain_x;  // no change during cooldown
    }

    result.desired_exp_us = desired_exp_us;
    result.desired_gain_x = desired_gain_x;
    result.changed = (std::abs(desired_exp_us - last_exp_us) > 1) ||
                     (std::abs(desired_gain_x - current_gain_x) > 0.01f);

    return result;
}

}  // namespace camera_display_node
