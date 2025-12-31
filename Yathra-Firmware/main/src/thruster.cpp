#include "thruster.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

static const char *TAG = "THRUSTER_CPP";

ThrusterController::ThrusterController() {}

ThrusterController::~ThrusterController() {
    // Basic cleanup could go here, though ESP-IDF drivers usually stay alive 
    // for the duration of the firmware.
}

mcpwm_timer_handle_t ThrusterController::getOrInitTimer(int group_id) {
    mcpwm_timer_handle_t* target_timer = (group_id == 0) ? &timer_group0 : &timer_group1;

    // If timer already exists, return it
    if (*target_timer != nullptr) {
        return *target_timer;
    }

    // Create new timer
    ESP_LOGI(TAG, "Initializing MCPWM Timer for Group %d", group_id);
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = group_id;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 1000000; // 1MHz, 1us per tick
    timer_config.period_ticks = MCPWM_PERIOD_TICKS;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, target_timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(*target_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*target_timer, MCPWM_TIMER_START_NO_STOP));

    return *target_timer;
}

esp_err_t ThrusterController::init(const std::vector<int>& pins) {
    channels.clear();
    channels.reserve(pins.size());

    ESP_LOGI(TAG, "Initializing %d Thrusters...", (int)pins.size());

    // Loop through requested pins
    for (size_t i = 0; i < pins.size(); i++) {
        // --- Resource Allocation Strategy ---
        // ESP32 has 2 Groups (0, 1). Each Group has 3 Operators. Each Operator handles 2 Generators.
        // Capacity per Group = 6 Motors.
        // i=0..5 -> Group 0. i=6..11 -> Group 1.
        
        int group_id = (i >= 6) ? 1 : 0;
        int operator_index_in_group = (i % 6) / 2; // 0, 1, or 2
        
        // 1. Get correct Timer
        mcpwm_timer_handle_t timer = getOrInitTimer(group_id);

        // 2. Create Operator 
        // Note: We need a new operator handle for every PAIR.
        // We use static handles here to remember the operator for the second motor of the pair.
        static mcpwm_oper_handle_t current_oper = nullptr;
        static int current_oper_group = -1;
        static int current_oper_index = -1;

        bool is_new_operator_needed = (i % 2 == 0) || (group_id != current_oper_group);

        if (is_new_operator_needed) {
            mcpwm_operator_config_t operator_config = {};
            operator_config.group_id = group_id;
            
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &current_oper));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(current_oper, timer));
            
            current_oper_group = group_id;
            current_oper_index = operator_index_in_group;
        }

        // 3. Create Comparator
        mcpwm_cmpr_handle_t comparator = nullptr;
        mcpwm_comparator_config_t comparator_config = {};
        comparator_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(current_oper, &comparator_config, &comparator));

        // 4. Create Generator
        mcpwm_gen_handle_t generator = nullptr;
        mcpwm_generator_config_t generator_config = {};
        generator_config.gen_gpio_num = pins[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(current_oper, &generator_config, &generator));

        // 5. Setup Generator Actions (High at 0, Low at Compare)
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

        // Store the handle
        channels.push_back({pins[i], comparator});
        
        // Init at neutral
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, ESC_NEUTRAL_PULSE));
    }

    // Arming delay needs to be handled by the user (usually in main)
    return ESP_OK;
}

void ThrusterController::setSpeeds(const std::vector<float>& speeds) {
    size_t count = std::min(speeds.size(), channels.size());

    for (size_t i = 0; i < count; i++) {
        uint32_t pulse = calculatePulseWidth(speeds[i]);
        mcpwm_comparator_set_compare_value(channels[i].comparator, pulse);
    }
}

void ThrusterController::stopAll() {
    for (auto& ch : channels) {
        mcpwm_comparator_set_compare_value(ch.comparator, ESC_NEUTRAL_PULSE);
    }
}

uint32_t ThrusterController::calculatePulseWidth(float percent) {
    float clamped = std::clamp(percent, -100.0f, 100.0f);
    
    if (clamped >= 0) {
        return ESC_NEUTRAL_PULSE + (uint32_t)(clamped * (ESC_MAX_PULSE - ESC_NEUTRAL_PULSE) / 100.0f);
    } else {
        return ESC_NEUTRAL_PULSE + (uint32_t)(clamped * (ESC_NEUTRAL_PULSE - ESC_MIN_PULSE) / 100.0f);
    }
}