#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "auto_exposure_control/auto_exposure_percentile.h"
#include "auto_exposure_control/auto_exposure_intensity.h"
#include "photometric_camera/photometric_model.h"

namespace camera_display_node {

struct TuningParams {
    double max_exposure_ms = 6.8;
    float min_gain_x = 1.0f;
    float max_gain_x = 4.0f;
    float gain_change_step_x = 0.5f;
    double mean_method_max_step_ratio = 0.12;
    double percentile_damping = 0.29;
    double percentile_max_step_ratio = 0.07;
    int percentile_deadband_us = 75;
    int gain_cooldown_frames = 7;
};

struct AEResult {
    int desired_exp_us;
    float desired_gain_x;
    bool changed;
};

bool loadTuningFile(const std::string& path, TuningParams* params);

class AEController {
public:
    AEController() = default;
    ~AEController();

    // Non-copyable
    AEController(const AEController&) = delete;
    AEController& operator=(const AEController&) = delete;

    // Initialize with calibration data and method selection.
    // method: "mean" or "percentile"
    // calib_dir: path to photometric calibration (needed for percentile)
    // ga_profile: path to ga_profile.txt (needed for percentile)
    bool init(const std::string& method,
              const std::string& calib_dir,
              const std::string& ga_profile,
              const TuningParams& tuning);

    // Compute desired exposure/gain given a downscaled grayscale frame.
    // current_exp_us and current_gain_x are the current V4L2 settings.
    AEResult compute(const cv::Mat& small_gray,
                     int current_exp_us,
                     float current_gain_x);

    const TuningParams& tuning() const { return tuning_; }

private:
    static int applyExposureStepLimit(int proposed_us, int last_us,
                                       double max_step_ratio);

    std::string method_;
    TuningParams tuning_;
    int gain_cooldown_ = 0;

    std::unique_ptr<auto_exposure::AutoExposurePercentile> percentile_ctrl_;
    std::unique_ptr<auto_exposure::AutoExposureIntensity> intensity_ctrl_;
    std::shared_ptr<photometric_camera::PhotometricModel> photo_model_;
};

}  // namespace camera_display_node
