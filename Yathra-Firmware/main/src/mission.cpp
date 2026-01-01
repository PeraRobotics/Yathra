#include "mission.hpp"
#include "esp_log.h"

static const char *M_TAG = "MISSION";

MissionManager::MissionManager() : current_step_idx(-1), step_timer(0.0f), is_active(false) {}

void MissionManager::loadMission(const std::vector<MissionStep>& new_plan) {
    plan = new_plan;
    current_step_idx = 0;
    step_timer = 0.0f;
    is_active = true;
    ESP_LOGI(M_TAG, "Mission Loaded: %d steps. Starting...", plan.size());
}

void MissionManager::stop() {
    is_active = false;
    current_step_idx = -1;
    ESP_LOGI(M_TAG, "Mission Stopped.");
}

bool MissionManager::isActive() const {
    return is_active;
}

bool MissionManager::update(float dt, robot_shared_state_t *state) {
    if (!is_active) return false;

    // Check if we ran out of steps
    if (current_step_idx >= plan.size()) {
        ESP_LOGI(M_TAG, "Mission Completed Successfully.");
        stop();
        return false;
    }

    // Get current step
    MissionStep &step = plan[current_step_idx];

    // --- APPLY TARGETS TO SHARED STATE ---
    state->target.h = step.target_depth;
    state->target.v = step.target_surge;
    state->target.w = step.target_heading; 
    
    // Critical: Tell control loop this W is an ANGLE (deg), not a RATE (deg/s)
    state->target.is_w_head = step.heading_is_absolute; 

    // --- TIMER LOGIC ---
    step_timer += dt;
    if (step_timer >= step.duration_sec) {
        ESP_LOGI(M_TAG, "Step [%d] '%s' Done (%.1fs).", current_step_idx, step.name.c_str(), step_timer);
        current_step_idx++;
        step_timer = 0.0f; // Reset for next step
    }

    return true;
}