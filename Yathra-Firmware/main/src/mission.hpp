#pragma once

#include <vector>
#include <string>
#include "telemetry.h" // Needed for robot_shared_state_t

// Types of mission steps
enum MissionType {
    MISSION_HOLD,   // Hold position/heading
    MISSION_MOVE    // Active movement (surge)
};

struct MissionStep {
    std::string name;       // Debug name
    MissionType type;
    float duration_sec;     // Duration to run this step
    
    // Setpoints
    float target_depth;     // Meters
    float target_heading;   // Degrees (0-360)
    float target_surge;     // -1.0 to 1.0
    
    // Control Flags
    bool heading_is_absolute; // true = go to specific angle; false = spin at rate
};

class MissionManager {
public:
    MissionManager();

    // Load a new mission plan and start immediately
    void loadMission(const std::vector<MissionStep>& new_plan);

    // Stop current mission
    void stop();

    // Update the robot state based on current step. Returns false if mission finished.
    bool update(float dt, robot_shared_state_t *state);

    // Check if mission is running
    bool isActive() const;

private:
    std::vector<MissionStep> plan;
    int current_step_idx;
    float step_timer;
    bool is_active;
};