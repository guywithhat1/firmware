#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/state.hpp"
#include "../controls/controller_manager.hpp"
#include "../controls/estimator_manager.hpp"
#include <map>
#include <string>
#define CONFIG_LAYER_DEBUG

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

/// @brief map section names to YAML section IDs
static const std::map<std::string, u_int8_t> yaml_section_id_mappings = {
    {"robot", 0},
    {"num_motors", 1},
    {"num_estimators", 2},
    {"num_gains", 3},
    {"num_controller_levels", 4},
    {"encoder_offsets", 5},
    {"yaw_axis_vector", 6},
    {"pitch_axis_vector", 7},
    {"defualt_gimbal_starting_angles", 8},
    {"defualt_chassis_starting_angles", 9},
    {"length_of_barrel_from_pitch_axis", 10},
    {"height_of_pitch_axis", 11},
    {"height_of_camera_above_barrel", 12},
    {"num_sensors", 13},
    {"estimators", 14},
    {"kinematics_p", 15},
    {"kinematics_v", 16},
    {"reference_limits", 17},
    {"controller_types", 18},
    {"gains", 19},
    {"num_states_per_estimator", 20},
    {"assigned_states", 21}
};

class ConfigLayer {
private:
    /// @brief array to save config packets
    CommsPacket config_packets[MAX_CONFIG_PACKETS];

    /// @brief flag indicating if all config packets have been received
    bool configured = false;

    /// @brief current YAML section we are seeking
    int seek_sec = -1;

    /// @brief current YAML subsection we are seeking
    int seek_subsec = 0;

    /// @brief number of YAML sections
    uint16_t num_sec;

    /// @brief array to store number of subsections per YAML section
    uint8_t subsec_sizes[MAX_CONFIG_PACKETS] = { 0 };

public:
    /// @brief default constructor
    ConfigLayer() {}

    /// @brief check incoming packet from the comms layer and update outgoing packet accordingly to request next config packet
    /// @param in incoming comms packet
    /// @param out outgoing comms packet to write config requests to
    void process(CommsPacket *in, CommsPacket *out);

    /// @brief return configured flag (check if all config packets have been received)
    /// @return the configured flag
    bool is_configured() { return configured; }

    void get_config_packets(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]){
        memcpy(packets, config_packets, sizeof(CommsPacket) * MAX_CONFIG_PACKETS);
        memcpy(sizes, subsec_sizes, sizeof(uint8_t) * MAX_CONFIG_PACKETS);
    }
};

struct Config {
    float num_motors;
    float num_gains;
    float num_controller_levels;
    float encoder_offsets[16];  
    float num_sensors[16];
    float kinematics_p[NUM_MOTORS][STATE_LEN];
    float kinematics_v[NUM_MOTORS][STATE_LEN];

    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS];
    float assigned_states[NUM_ESTIMATORS][STATE_LEN];
    float num_states_per_estimator[NUM_ESTIMATORS];
    float set_reference_limits[STATE_LEN][3][2];

    float yaw_axis_vector[2];
    float pitch_axis_vector[2];
    float defualt_gimbal_starting_angles[2];
    float defualt_chassis_starting_angles[2];
    float controller_types[NUM_MOTORS][NUM_CONTROLLER_LEVELS];

    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {

        for(int i = 0; i < MAX_CONFIG_PACKETS; i++){
            if(packets[i].get_id() == yaml_section_id_mappings.at("num_gains")){
                memcpy(&num_gains, packets[i].raw + 8, sizeof(float));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("num_controller_levels")){
                memcpy(&num_controller_levels, packets[i].raw + 8, sizeof(float));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("num_sensors")){
                memcpy(&num_sensors, packets[i].raw + 8, sizeof(float));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("kinematics_p")){
                memcpy(kinematics_p, packets[i].raw + 8, sizeof(kinematics_p));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("kinematics_v")){
                memcpy(kinematics_v, packets[i].raw + 8, sizeof(kinematics_v));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("gains")) {   
                memcpy(gains, packets[i].raw + 8, sizeof(gains));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("assigned_states")){
                memcpy(assigned_states, packets[i].raw + 8, sizeof(assigned_states));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("num_states_per_estimator")){
                memcpy(num_states_per_estimator, packets[i].raw + 8, sizeof(num_states_per_estimator));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("reference_limits")){
                memcpy(set_reference_limits, packets[i].raw + 8, sizeof(set_reference_limits));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("yaw_axis_vector")){
                memcpy(yaw_axis_vector, packets[i].raw + 8, sizeof(yaw_axis_vector));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("pitch_axis_vector")){
                memcpy(pitch_axis_vector, packets[i].raw + 8, sizeof(pitch_axis_vector));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("defualt_gimbal_starting_angles")){
                memcpy(defualt_gimbal_starting_angles, packets[i].raw + 8, sizeof(defualt_gimbal_starting_angles));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("defualt_chassis_starting_angles")){
                memcpy(defualt_chassis_starting_angles, packets[i].raw + 8, sizeof(defualt_chassis_starting_angles));
            }
            if(packets[i].get_id() == yaml_section_id_mappings.at("controller_types")){
                memcpy(controller_types, packets[i].raw + 8, sizeof(controller_types));
            }
        }
    }

    void print() {
        Serial.println("Config data:");
        // for(int i = 0; i < NUM_MOTORS; i++){
        //     for(int j = 0; j < STATE_LEN; j++){
        //         Serial.printf("kinematics_p[%d][%d]: %f\n", i, j, kinematics_p[i][j]);
        //     }
        // }

        for(int i = 0; i < NUM_MOTORS; i++){
            for(int j = 0; j < NUM_CONTROLLER_LEVELS; j++){
                for(int k = 0; k < NUM_GAINS; k++){
                    Serial.printf("gains[%d][%d][%d]: %f\n", i, j, k, gains[i][j][k]);
                }
            }
        }
    }

    private: 
        int gains_index = 0;
        int assigned_states_index = 0;
        int num_states_per_estimator_index = 0;
        int set_reference_limits_index = 0;
        int yaw_axis_vector_index = 0;
        int pitch_axis_vector_index = 0;
        int defualt_gimbal_starting_angles_index = 0;
        int defualt_chassis_starting_angles_index = 0;
        int controller_types_index = 0;
        int kinematics_p_index = 0;
        int kinematics_v_index = 0;
        int num_sensors_index = 0;
        int encoder_offsets_index = 0;
};

#endif