#include "cn105.h"
#include "Globals.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
#include <utility>

using namespace esphome;

void CN105Climate::checkPendingWantedSettings() {
    long now = CUSTOM_MILLIS;
    if (!(this->wantedSettings.hasChanged) || (now - this->wantedSettings.lastChange < this->debounce_delay_)) {
        return;
    }

    ESP_LOGI(LOG_ACTION_EVT_TAG, "checkPendingWantedSettings - wanted settings have changed, sending them to the heatpump...");
    this->sendWantedSettings();
}

void CN105Climate::checkPendingWantedRunStates() {
    long now = CUSTOM_MILLIS;
    if (!(this->wantedRunStates.hasChanged) || (now - this->wantedRunStates.lastChange < this->debounce_delay_)) {
        return;
    }
    ESP_LOGI(LOG_ACTION_EVT_TAG, "checkPendingWantedRunStates - wanted run states have changed, sending them to the heatpump...");
    this->sendWantedRunStates();
}

void logCheckWantedSettingsMutex(wantedHeatpumpSettings& settings) {

    if (settings.hasBeenSent) {
        ESP_LOGE("control", "Mutex lock faillure: wantedSettings should be locked while sending.");
        ESP_LOGD("control", "-- This is an assertion test on wantedSettings.hasBeenSent");
        ESP_LOGD("control", "-- wantedSettings.hasBeenSent = true is unexpected");
        ESP_LOGD("control", "-- should be false because mutex should prevent running this while sending");
        ESP_LOGD("control", "-- and mutex should be released only when hasBeenSent is false");
    }

}

void CN105Climate::controlDelegate(const esphome::climate::ClimateCall& call) {
    ESP_LOGD("control", "espHome control() interface method called...");
    bool updated = false;

    logCheckWantedSettingsMutex(this->wantedSettings);

    updated = this->processModeChange(call) || updated;
    updated = this->processTemperatureChange(call) || updated;
    updated = this->processFanChange(call) || updated;
    updated = this->processSwingChange(call) || updated;

    this->finalizeControlIfUpdated(updated);
}

bool CN105Climate::processModeChange(const esphome::climate::ClimateCall& call) {
    if (!call.get_mode().has_value()) {
        return false;
    }

    ESP_LOGD("control", "Mode change asked");
    this->mode = *call.get_mode();
    this->controlMode();
    this->controlTemperature();
    return true;
}

void CN105Climate::handleDualSetpointBoth(float low, float high) {
    ESP_LOGD("control", "handleDualSetpointBoth - low: %.1f, high: %.1f", low, high);
    this->setTargetTemperatureLow(low);
    this->setTargetTemperatureHigh(high);
    this->last_dual_setpoint_side_ = 'N';
    this->last_dual_setpoint_change_ms_ = CUSTOM_MILLIS;
    this->currentSettings.dual_low_target = this->getTargetTemperatureLow();
    this->currentSettings.dual_high_target = this->getTargetTemperatureHigh();
}

void CN105Climate::handleDualSetpointLowOnly(float low) {
    ESP_LOGD("control", "handleDualSetpointLowOnly - LOW: %.1f", low);
    if (this->last_dual_setpoint_side_ == 'H' && (CUSTOM_MILLIS - this->last_dual_setpoint_change_ms_) < UI_SETPOINT_ANTIREBOUND_MS) {
        ESP_LOGD("control", "IGNORED low setpoint due to UI anti-rebound after high change");
        return;
    }
    if (!std::isnan(this->getTargetTemperatureLow()) && fabsf(low - this->getTargetTemperatureLow()) < 0.05f) {
        ESP_LOGD("control", "IGNORED low setpoint: no effective change vs current low target");
        return;
    }
    this->setTargetTemperatureLow(low);
    if (this->mode == climate::CLIMATE_MODE_AUTO) {
        const float amplitude = 4.0f;
        this->setTargetTemperatureHigh(this->getTargetTemperatureLow() + amplitude);
        ESP_LOGD("control", "mode auto: sliding high to preserve amplitude %.1f => [%.1f - %.1f]", amplitude, this->getTargetTemperatureLow(), this->getTargetTemperatureHigh());
    }
    this->last_dual_setpoint_side_ = 'L';
    this->last_dual_setpoint_change_ms_ = CUSTOM_MILLIS;
    this->currentSettings.dual_low_target = this->getTargetTemperatureLow();
    this->currentSettings.dual_high_target = this->getTargetTemperatureHigh();
}

void CN105Climate::handleDualSetpointHighOnly(float high) {
    ESP_LOGI("control", "HIGH: handleDualSetpointHighOnly - HIGH : %.1f", high);
    if (this->last_dual_setpoint_side_ == 'L' && (CUSTOM_MILLIS - this->last_dual_setpoint_change_ms_) < UI_SETPOINT_ANTIREBOUND_MS) {
        ESP_LOGD("control", "ignored high setpoint due to UI anti-rebound after low change");
        return;
    }
    if (!std::isnan(this->getTargetTemperatureHigh()) && fabsf(high - this->getTargetTemperatureHigh()) < 0.05f) {
        ESP_LOGD("control", "ignored high setpoint: no effective change vs current high target");
        return;
    }
    this->setTargetTemperatureHigh(high);
    if (this->mode == climate::CLIMATE_MODE_AUTO) {
        const float amplitude = 4.0f;
        this->setTargetTemperatureLow(this->getTargetTemperatureHigh() - amplitude);
        ESP_LOGD("control", "mode auto: sliding low to preserve amplitude %.1f => [%.1f - %.1f]", amplitude, this->getTargetTemperatureLow(), this->getTargetTemperatureHigh());
    }
    this->last_dual_setpoint_side_ = 'H';
    this->last_dual_setpoint_change_ms_ = CUSTOM_MILLIS;
    this->currentSettings.dual_low_target = this->getTargetTemperatureLow();
    this->currentSettings.dual_high_target = this->getTargetTemperatureHigh();
}

void CN105Climate::handleSingleTargetInAutoOrDry(float requested) {
    ESP_LOGD("control", "handleSingleTargetInAutoOrDry - SINGLE: %.1f", requested);
    if (this->mode == climate::CLIMATE_MODE_AUTO) {
        const float half_span = 2.0f;
        this->setTargetTemperatureLow(requested - half_span);
        this->setTargetTemperatureHigh(requested + half_span);
        this->last_dual_setpoint_side_ = 'N';
        this->last_dual_setpoint_change_ms_ = CUSTOM_MILLIS;
        this->currentSettings.dual_low_target = this->getTargetTemperatureLow();
        this->currentSettings.dual_high_target = this->getTargetTemperatureHigh();
        this->setTargetTemperature(requested);
        ESP_LOGD("control", "AUTO received single target: median=%.1f => [%.1f - %.1f]", requested, this->getTargetTemperatureLow(), this->getTargetTemperatureHigh());
    }
    if (this->mode == climate::CLIMATE_MODE_DRY) {
        this->setTargetTemperatureHigh(requested);
        if (std::isnan(this->getTargetTemperatureLow())) {
            this->setTargetTemperatureLow(requested);
        }
        this->last_dual_setpoint_side_ = 'H';
        this->last_dual_setpoint_change_ms_ = CUSTOM_MILLIS;
        this->currentSettings.dual_low_target = this->getTargetTemperatureLow();
        this->currentSettings.dual_high_target = this->getTargetTemperatureHigh();
        this->setTargetTemperature(requested);
        ESP_LOGD("control", "DRY received single target: high=%.1f (low now %.1f)", this->getTargetTemperatureHigh(), this->getTargetTemperatureLow());
    }
}

bool CN105Climate::processTemperatureChange(const esphome::climate::ClimateCall& call) {
    // Accept any of: low/high/single (AUTO/DRY can send single even when using dual internally)
    bool tempHasValue = (call.get_target_temperature_low().has_value() ||
        call.get_target_temperature_high().has_value() || call.get_target_temperature().has_value());

    if (!tempHasValue) {
        return false;
    } else {
        ESP_LOGD("control", "A temperature setpoint value has been provided...");
    }

    float temp_low = NAN;
    float temp_high = NAN;
    float temp_single = NAN;
    if (call.get_target_temperature_low().has_value()) {
        temp_low = this->fahrenheitSupport_.normalizeUiTemperatureToHeatpumpTemperature(*call.get_target_temperature_low());
    }
    if (call.get_target_temperature_high().has_value()) {
        temp_high = this->fahrenheitSupport_.normalizeUiTemperatureToHeatpumpTemperature(*call.get_target_temperature_high());
    }
    if (call.get_target_temperature().has_value()) {
        temp_single = this->fahrenheitSupport_.normalizeUiTemperatureToHeatpumpTemperature(*call.get_target_temperature());
    }

    // 🔧 FIX: has_feature_flags/CLIMATE_REQUIRES... is not available in your ESPHome build.
    // Use the traits API that exists: supports two-point target temperature.
    if (this->traits_.get_supports_two_point_target_temperature()) {
        ESP_LOGD("control", "Processing with dual setpoint support...");
        if (call.get_target_temperature_low().has_value() && call.get_target_temperature_high().has_value()) {
            this->handleDualSetpointBoth(temp_low, temp_high);
        } else if (call.get_target_temperature_low().has_value()) {
            this->handleDualSetpointLowOnly(temp_low);
        } else if (call.get_target_temperature_high().has_value()) {
            this->handleDualSetpointHighOnly(temp_high);
        } else if (call.get_target_temperature().has_value() &&
            (this->mode == climate::CLIMATE_MODE_AUTO || this->mode == climate::CLIMATE_MODE_DRY)) {
            this->handleSingleTargetInAutoOrDry(temp_single);
        }
    } else {
        ESP_LOGD("control", "Processing without dual setpoint support...");
        if (call.get_target_temperature().has_value()) {
            this->setTargetTemperature(temp_single);
            ESP_LOGI("control", "Setting heatpump setpoint : %.1f", this->getTargetTemperature());
        }
    }

    this->controlTemperature();
    ESP_LOGD("control", "controlled temperature to: %.1f", this->wantedSettings.temperature);
    return true;
}

bool CN105Climate::processFanChange(const esphome::climate::ClimateCall& call) {
    if (!call.get_fan_mode().has_value()) {
        return false;
    }
    ESP_LOGD("control", "Fan change asked");
    this->fan_mode = *call.get_fan_mode();
    this->controlFan();
    return true;
}

bool CN105Climate::processSwingChange(const esphome::climate::ClimateCall& call) {
    if (!call.get_swing_mode().has_value()) {
        return false;
    }
    ESP_LOGD("control", "Swing change asked");
    this->swing_mode = *call.get_swing_mode();
    this->controlSwing();
    return true;
}

void CN105Climate::finalizeControlIfUpdated(bool updated) {
    if (!updated) {
        return;
    }
    ESP_LOGD(LOG_ACTION_EVT_TAG, "clim.control() -> User changed something...");
    logCheckWantedSettingsMutex(this->wantedSettings);
    this->wantedSettings.hasChanged = true;
    this->wantedSettings.hasBeenSent = false;
    this->wantedSettings.lastChange = CUSTOM_MILLIS;
    this->debugSettings("control (wantedSettings)", this->wantedSettings);
    this->publish_state();
}

void CN105Climate::control(const esphome::climate::ClimateCall& call) {
#ifdef USE_ESP32
    std::lock_guard<std::mutex> guard(wantedSettingsMutex);
    this->controlDelegate(call);
#else
    this->emulateMutex("CONTROL_WANTED_SETTINGS", std::bind(&CN105Climate::controlDelegate, this, call));
#endif
}

void CN105Climate::controlSwing() {
    bool wideVaneSupported = this->traits_.supports_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
    bool vane_is_swing = (this->currentSettings.vane != nullptr) && (strcmp(this->currentSettings.vane, "SWING") == 0);
    bool wide_is_swing = (this->currentSettings.wideVane != nullptr) && (strcmp(this->currentSettings.wideVane, "SWING") == 0);

    switch (this->swing_mode) {
    case climate::CLIMATE_SWING_OFF:
        if (vane_is_swing) {
            this->setVaneSetting("AUTO");
        }
        if (wideVaneSupported && wide_is_swing) {
            this->setWideVaneSetting("|");
        }
        break;

    case climate::CLIMATE_SWING_VERTICAL:
        this->setVaneSetting("SWING");
        if (wideVaneSupported && wide_is_swing) {
            this->setWideVaneSetting("|");
        }
        break;

    case climate::CLIMATE_SWING_HORIZONTAL:
        if (vane_is_swing) {
            this->setVaneSetting("AUTO");
        }
        if (wideVaneSupported) {
            this->setWideVaneSetting("SWING");
        }
        break;

    case climate::CLIMATE_SWING_BOTH:
        this->setVaneSetting("SWING");
        if (wideVaneSupported) {
            this->setWideVaneSetting("SWING");
        }
        break;

    default:
        ESP_LOGW(TAG, "control - received unsupported swing mode request.");
        break;
    }
}

void CN105Climate::controlFan() {
    switch (this->fan_mode.value()) {
    case climate::CLIMATE_FAN_OFF:
        this->setPowerSetting("OFF");
        break;
    case climate::CLIMATE_FAN_QUIET:
        this->setFanSpeed("QUIET");
        break;
    case climate::CLIMATE_FAN_DIFFUSE:
        this->setFanSpeed("QUIET");
        break;
    case climate::CLIMATE_FAN_LOW:
        this->setFanSpeed("1");
        break;
    case climate::CLIMATE_FAN_MEDIUM:
        this->setFanSpeed("2");
        break;
    case climate::CLIMATE_FAN_MIDDLE:
        this->setFanSpeed("3");
        break;
    case climate::CLIMATE_FAN_HIGH:
        this->setFanSpeed("4");
        break;
    case climate::CLIMATE_FAN_ON:
    case climate::CLIMATE_FAN_AUTO:
    default:
        this->setFanSpeed("AUTO");
        break;
    }
}

void CN105Climate::controlTemperature() {
    float setting;

    // 🔧 FIX: use two-point target support instead of has_feature_flags
    if (this->traits_.get_supports_two_point_target_temperature()) {
        this->sanitizeDualSetpoints();

        switch (this->mode) {
        case climate::CLIMATE_MODE_AUTO:
            if (this->traits_.get_supports_two_point_target_temperature()) {
                if ((!std::isnan(currentSettings.temperature)) && (currentSettings.temperature > 0)) {
                    this->setTargetTemperatureLow(currentSettings.temperature - 2.0f);
                    this->setTargetTemperatureHigh(currentSettings.temperature + 2.0f);
                    ESP_LOGI("control", "Initializing AUTO mode temps from current PAC temp: %.1f -> [%.1f - %.1f]",
                        currentSettings.temperature, this->getTargetTemperatureLow(), this->getTargetTemperatureHigh());
                }
                setting = currentSettings.temperature;
                ESP_LOGD("control", "AUTO mode : getting median temperature from current PAC temp: %.1f", setting);
            } else {
                setting = this->getTargetTemperature();
            }
            break;

        case climate::CLIMATE_MODE_HEAT:
            setting = this->getTargetTemperatureLow();
            ESP_LOGD("control", "HEAT mode : getting temperature low:%1.f", this->getTargetTemperatureLow());
            break;

        case climate::CLIMATE_MODE_COOL:
            setting = this->getTargetTemperatureHigh();
            ESP_LOGD("control", "COOL mode : getting temperature high:%1.f", this->getTargetTemperatureHigh());
            break;

        case climate::CLIMATE_MODE_DRY:
            setting = this->getTargetTemperatureHigh();
            ESP_LOGD("control", "DRY mode : getting temperature high:%1.f", this->getTargetTemperatureHigh());
            break;

        default:
            // Other modes : use median temperature
            if (this->traits_.get_supports_two_point_target_temperature()) {
                setting = (this->getTargetTemperatureLow() + this->getTargetTemperatureHigh()) / 2.0f;
            } else {
                setting = this->getTargetTemperature();
            }
            ESP_LOGD("control", "DEFAULT mode : getting temperature median:%1.f", setting);
            break;
        }
    } else {
        setting = this->getTargetTemperature();
    }

    setting = this->calculateTemperatureSetting(setting);
    this->wantedSettings.temperature = setting;
    ESP_LOGI("control", "setting wanted temperature to %.1f", setting);
}

void CN105Climate::controlMode() {
    switch (this->mode) {
    case climate::CLIMATE_MODE_COOL:
        ESP_LOGI("control", "changing mode to COOL");
        this->setModeSetting("COOL");
        this->setPowerSetting("ON");
        break;
    case climate::CLIMATE_MODE_HEAT:
        ESP_LOGI("control", "changing mode to HEAT");
        this->setModeSetting("HEAT");
        this->setPowerSetting("ON");
        break;
    case climate::CLIMATE_MODE_DRY:
        ESP_LOGI("control", "changing mode to DRY");
        this->setModeSetting("DRY");
        this->setPowerSetting("ON");
        break;
    case climate::CLIMATE_MODE_AUTO:
        ESP_LOGI("control", "changing mode to AUTO");
        this->setModeSetting("AUTO");
        this->setPowerSetting("ON");
        break;
    case climate::CLIMATE_MODE_FAN_ONLY:
        ESP_LOGI("control", "changing mode to FAN_ONLY");
        this->setModeSetting("FAN");
        this->setPowerSetting("ON");
        break;
    case climate::CLIMATE_MODE_OFF:
        ESP_LOGI("control", "changing mode to OFF");
        this->setPowerSetting("OFF");
        break;
    default:
        ESP_LOGW("control", "unsupported mode");
    }
}

void CN105Climate::setActionIfOperatingTo(climate::ClimateAction action_if_operating) {
    bool stage_is_active = this->use_stage_for_operating_status_ &&
        this->currentSettings.stage != nullptr &&
        strcmp(this->currentSettings.stage, STAGE_MAP[0 /*IDLE*/]) != 0;

    ESP_LOGD(LOG_OPERATING_STATUS_TAG, "Setting action (operating: %s, stage_fallback_enabled: %s, stage: %s, stage_is_active: %s)",
        this->currentStatus.operating ? "true" : "false",
        this->use_stage_for_operating_status_ ? "yes" : "no",
        getIfNotNull(this->currentSettings.stage, "N/A"),
        stage_is_active ? "yes" : "no");

    if (this->currentStatus.operating) {
        this->action = action_if_operating;
        ESP_LOGD(LOG_OPERATING_STATUS_TAG, "Action set by operating status (compressor running)");
    } else if (stage_is_active) {
        this->action = action_if_operating;
        ESP_LOGD(LOG_OPERATING_STATUS_TAG, "Action set by stage fallback (stage: %s)", this->currentSettings.stage);
    } else {
        this->action = climate::CLIMATE_ACTION_IDLE;
        ESP_LOGD(LOG_OPERATING_STATUS_TAG, "Action set to IDLE (no activity detected)");
    }
}

void CN105Climate::setActionIfOperatingAndCompressorIsActiveTo(climate::ClimateAction action) {
    ESP_LOGW(TAG, "Warning: the use of compressor frequency as an active indicator is deprecated. Please use operating status instead.");

    if (currentStatus.compressorFrequency <= 0) {
        this->action = climate::CLIMATE_ACTION_IDLE;
    } else {
        this->setActionIfOperatingTo(action);
    }
}

void CN105Climate::updateAction() {
    ESP_LOGV(TAG, "updating action back to espHome...");

    // 🔧 FIX: use two-point target support instead of has_feature_flags
    if (this->traits().get_supports_two_point_target_temperature()) {
        this->sanitizeDualSetpoints();
    }

    switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT:
        this->setActionIfOperatingTo(climate::CLIMATE_ACTION_HEATING);
        break;
    case climate::CLIMATE_MODE_COOL:
        this->setActionIfOperatingTo(climate::CLIMATE_ACTION_COOLING);
        break;
    case climate::CLIMATE_MODE_AUTO:
        if (this->traits().supports_mode(climate::CLIMATE_MODE_HEAT) &&
            this->traits().supports_mode(climate::CLIMATE_MODE_COOL)) {
            if (this->getCurrentTemperature() >= this->getTargetTemperatureHigh()) {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_COOLING);
            } else if (this->getCurrentTemperature() <= this->getTargetTemperatureLow()) {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_HEATING);
            } else {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_IDLE);
            }
        } else if (this->traits().supports_mode(climate::CLIMATE_MODE_COOL)) {
            if (this->getCurrentTemperature() < this->getTargetTemperatureHigh()) {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_IDLE);
            } else {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_COOLING);
            }
        } else if (this->traits().supports_mode(climate::CLIMATE_MODE_HEAT)) {
            if (this->getCurrentTemperature() >= this->getTargetTemperatureLow()) {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_IDLE);
            } else {
                this->setActionIfOperatingTo(climate::CLIMATE_ACTION_HEATING);
            }
        } else {
            ESP_LOGE(TAG, "AUTO mode is not supported by this unit");
            this->setActionIfOperatingTo(climate::CLIMATE_ACTION_FAN);
        }
        break;

    case climate::CLIMATE_MODE_DRY:
        this->setActionIfOperatingTo(climate::CLIMATE_ACTION_DRYING);
        break;
    case climate::CLIMATE_MODE_FAN_ONLY:
        this->action = climate::CLIMATE_ACTION_FAN;
        break;
    default:
        this->action = climate::CLIMATE_ACTION_OFF;
    }

    ESP_LOGD(TAG, "Climate mode is: %i", this->mode);
    ESP_LOGD(TAG, "Climate action is: %i", this->action);
}

climate::ClimateTraits CN105Climate::traits() {
    return traits_;
}

climate::ClimateTraits& CN105Climate::config_traits() {
    return traits_;
}

void CN105Climate::setModeSetting(const char* setting) {
    int index = lookupByteMapIndex(MODE_MAP, 5, setting);
    if (index > -1) {
        wantedSettings.mode = MODE_MAP[index];
    } else {
        wantedSettings.mode = MODE_MAP[0];
    }
}

void CN105Climate::setPowerSetting(const char* setting) {
    int index = lookupByteMapIndex(POWER_MAP, 2, setting);
    if (index > -1) {
        wantedSettings.power = POWER_MAP[index];
    } else {
        wantedSettings.power = POWER_MAP[0];
    }
}

void CN105Climate::setFanSpeed(const char* setting) {
    int index = lookupByteMapIndex(FAN_MAP, 6, setting);
    if (index > -1) {
        wantedSettings.fan = FAN_MAP[index];
    } else {
        wantedSettings.fan = FAN_MAP[0];
    }
}

void CN105Climate::setVaneSetting(const char* setting) {
    int index = lookupByteMapIndex(VANE_MAP, 7, setting);
    if (index > -1) {
        wantedSettings.vane = VANE_MAP[index];
    } else {
        wantedSettings.vane = VANE_MAP[0];
    }
}

void CN105Climate::setWideVaneSetting(const char* setting) {
    int index = lookupByteMapIndex(WIDEVANE_MAP, 8, setting);
    if (index > -1) {
        wantedSettings.wideVane = WIDEVANE_MAP[index];
    } else {
        wantedSettings.wideVane = WIDEVANE_MAP[0];
    }
}

void CN105Climate::setAirflowControlSetting(const char* setting) {
    int index = lookupByteMapIndex(AIRFLOW_CONTROL_MAP, 3, setting);
    if (index > -1) {
        wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[index];
    } else {
        wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
    }
}

void CN105Climate::set_remote_temperature(float setting) {
    if (std::isnan(setting)) {
        ESP_LOGW(LOG_REMOTE_TEMP, "Remote temperature is NaN, ignoring.");
        return;
    }

    // Toujours renvoyer la température distante lorsqu’un nouvel échantillon arrive,
    // même si la valeur n’a pas changé, afin d’éviter que l’unité Mitsubishi
    // ne repasse sur la sonde interne faute de mise à jour régulière (#474).
    this->remoteTemperature_ = setting;
    this->shouldSendExternalTemperature_ = true;
    ESP_LOGD(LOG_REMOTE_TEMP, "setting remote temperature to %f", this->remoteTemperature_);
}
