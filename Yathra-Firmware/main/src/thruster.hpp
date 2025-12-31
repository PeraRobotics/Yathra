#pragma once

#include <vector>
#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

class ThrusterController {
public:
    ThrusterController();
    ~ThrusterController();

    /**
     * @brief Initialize the hardware for a list of pins.
     * Automatically handles resource allocation for 6+ motors.
     * * @param pins A vector of GPIO pin numbers (e.g., {18, 19, 21, ...})
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t init(const std::vector<int>& pins);

    /**
     * @brief Set speeds for all initialized thrusters.
     * * @param speeds Vector of percentages (-100.0 to 100.0). 
     * Size must match the number of pins initialized.
     */
    void setSpeeds(const std::vector<float>& speeds);

    /**
     * @brief Immediately stop all thrusters (send 1500us).
     */
    void stopAll();

private:
    // Constants for Blue Robotics ESCs
    static constexpr int ESC_MIN_PULSE = 1100;
    static constexpr int ESC_NEUTRAL_PULSE = 1500;
    static constexpr int ESC_MAX_PULSE = 1900;
    static constexpr int MCPWM_FREQ_HZ = 50;
    static constexpr int MCPWM_PERIOD_TICKS = 20000; // 20ms

    // Internal storage for handles
    struct ThrusterChannel {
        int gpio_num;
        mcpwm_cmpr_handle_t comparator;
    };

    std::vector<ThrusterChannel> channels;
    mcpwm_timer_handle_t timer_group0 = nullptr;
    mcpwm_timer_handle_t timer_group1 = nullptr;

    // Helper to calculate pulse width
    uint32_t calculatePulseWidth(float percent);
    
    // Internal resource allocation helper
    mcpwm_timer_handle_t getOrInitTimer(int group_id);
};