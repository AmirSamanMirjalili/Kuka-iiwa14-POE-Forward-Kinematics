#include "mujoco_utils.h"
#include <stdio.h>
#include "Operations.h"
#include <memory>

extern const double CTRL_UPDATE_FREQ;
extern bool debug_mode;
extern const char* FILENAME;
extern mjModel* m;
extern mjData* d;
extern int mocap_body_id;
extern int* actuator_ids;
extern bool isPushing;
extern int grabbedBodyId;
extern mjtNum pushForce[3];
bool kinematic_debug_mode = true;

void init_mujoco() {

    char error[1000] = "Could not load binary model";
    m = mj_loadXML(FILENAME, 0, error, 1000);
    if (!m) mju_error_s("Load model error: %s", error);
    d = mj_makeData(m);
    mocap_body_id = mj_name2id(m, mjOBJ_BODY, "mocap_sphere");
}


std::shared_ptr<mjtNum[]> calculate_joint_distances() {
    // Use shared_ptr for automatic memory management
    std::shared_ptr<mjtNum[]> link_length_array(new mjtNum[m->njnt], std::default_delete<mjtNum[]>());


    for (int i = 1; i < m->njnt; i++) {
        // Get joint names
        char joint1_name[100];
        char joint2_name[100];

        sprintf(joint1_name, "joint%d", i);
        sprintf(joint2_name, "joint%d", i + 1);

        // Get joint IDs
        int joint1_id = mj_name2id(m, mjOBJ_JOINT, joint1_name);
        int joint2_id = mj_name2id(m, mjOBJ_JOINT, joint2_name);

        // If either joint is not found, skip to the next link
        if (joint1_id == -1 || joint2_id == -1) {
            DEBUG_PRINT("Skipping link %d: One or both joints not found.\n", i);
            continue;
        }

        // Get the global positions of the joints
        mjtNum joint1_pos[3];
        mjtNum joint2_pos[3];
        mj_local2Global(d, joint1_pos, NULL, m->jnt_pos + 3*joint1_id, NULL, m->jnt_bodyid[joint1_id], 0);
        mj_local2Global(d, joint2_pos, NULL, m->jnt_pos + 3*joint2_id, NULL, m->jnt_bodyid[joint2_id], 0);

        // Calculate the link length
        mjtNum link_length = mju_dist3(joint1_pos, joint2_pos); 

        //store the link length
        link_length_array[i] = link_length;


        DEBUG_PRINT("Link %d length: %f\n", i, link_length);
    }

    return link_length_array;
}

void get_joint_information() {
    
    // mj_kinematics(m, d);   

         // Run forward kinematics
    for (int i = 0; i < m->njnt; i++) {
        mjtNum global_joint_pos[3];
        mj_local2Global(d, global_joint_pos, NULL, m->jnt_pos + 3*i, NULL, m->jnt_bodyid[i], 0);
        printf("Joint %d global position: %f %f %f\n", i, global_joint_pos[0], global_joint_pos[1], global_joint_pos[2]);
    }

    

    // const int number_of_sensors = 7;
    
    // for (int i = 1; i < number_of_sensors+1; i++) {
       
    //     // construct the sensor name using the pattern
    //     char sensor_name[100];
    //     sprintf(sensor_name, "hinge_position_sensor%d", i);

    //     // Get joint position from sensor data array
    //     int sensorId = mj_name2id(m, mjOBJ_SENSOR, sensor_name);
        
    //     if(sensorId >= 0) {
    //         // Access x, y, z coordinates from sensordata
    //         mjtNum x = d->sensordata[sensorId];
    //         mjtNum y = d->sensordata[sensorId + 1];
    //         mjtNum z = d->sensordata[sensorId + 2];
 


    //         printf("Sensor %d: x=%f, y=%f, z=%f\n", i, x, y, z);

           
            
    //     }
    //     else 
    //     {
    //         printf("Sensor %d: not found\n", i);
    //     }
    // }




    // for (int i = 0; i < m->njnt; i++) {
    //     printf("Joint %d: type=%d, pos=[%f %f %f], axis=[%f %f %f]\n", 
    //            i, m->jnt_type[i], d->xpos[m->jnt_qposadr[i]],d->xpos[m->jnt_qposadr[i]+1],d->xpos[m->jnt_qposadr[i]+2],
    //            m->jnt_axis[3*i], m->jnt_axis[3*i+1], m->jnt_axis[3*i+2]);
    //     // Access limits if m->jnt_limited[i] is true
    //     if (m->jnt_limited[i]) {
    //         printf("Range: [%f %f]\n", m->jnt_range[2*i], m->jnt_range[2*i+1]); 
    //     }
    // }
}


void get_kinematic_parameters(const mjModel* m, mjData* d) {
    get_joint_information();
    std::shared_ptr<mjtNum[]> link_length_array = calculate_joint_distances();
    
    
}

void cleanup_mujoco() {
    // Free the allocated memory for actuator IDs
    if (actuator_ids != NULL) {
        free(actuator_ids);
    }
    
    mj_deleteData(d);
    mj_deleteModel(m);
}

void apply_force() {
    if (isPushing && grabbedBodyId >= 0) {
        d->xfrc_applied[6 * grabbedBodyId] = pushForce[0];
        d->xfrc_applied[6 * grabbedBodyId + 1] = pushForce[1];
        d->xfrc_applied[6 * grabbedBodyId + 2] = pushForce[2];
    }
}

void init_actuator_ids() {
    // Get the number of actuators
    int num_actuators = m->nu;

    // Allocate memory for the actuator ID array
    actuator_ids = (int*)malloc(num_actuators * sizeof(int));
    if (actuator_ids == NULL) {
        mju_error("Could not allocate memory for actuator IDs");
    }

    // Populate the array with actuator IDs
    for (int i = 0; i < num_actuators; i++) {
        const char* actuatorName = mj_id2name(m, mjOBJ_ACTUATOR, i);
        actuator_ids[i] = i; // Assuming actuator IDs are sequential from 0 to num_actuators-1
        DEBUG_PRINT("Actuator ID: %d, Name: %s\n", actuator_ids[i], actuatorName);
    }
}

void init_control() {
    // double T[7] = {2.5, 3.0, 4.2, 1.8, 5.5, 2.1, 3.7}; // Example periods

    // reasonable period for the actuators
    double T[7] = {13, 17, 19, 23, 29, 31, 37};

    // Get the current simulation time
    mjtNum t = d->time;

    // init actuators position
    for (int i = 0; i < m->nu; i++) {
        double angle = m->jnt_range[2*i];
        d->ctrl[i] = abs(angle) * sin(2 * M_PI / T[i] * t);
        // print angle range
        DEBUG_PRINT("Actuator ID: %d, Angle range: %f\n", i, angle);
        // print angle range min
        DEBUG_PRINT("Actuator ID: %d, Control: %f\n", i, d->qpos[m->jnt_qposadr[i]]);
    }

    
}

void update_control(const mjModel* m, mjData* d) {
    // init_control();
    get_kinematic_parameters(m,d);
    mr::verifyForwardKinematics(m, d);
    // print_sensor_data();
}

void print_sensor_data() {
    // Get sensor IDs
    int mocap_velocity_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "mocap_vel_sens");
    int mocap_position_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "mocap_pos_sens");
    int ee_pos_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "ee_position_sensor");
    int ee_vel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "ee_velocity_sensor");

    // Get position and velocity from sensor data
    mjtNum position[3];
    mjtNum velocity[3];
    mjtNum ee_position[3];
    mjtNum ee_velocity[3];

    // Get position
    mju_copy(position, &d->sensordata[3 * mocap_position_sensor_id], 3);
    // Get velocity
    mju_copy(velocity, &d->sensordata[3 * mocap_velocity_sensor_id], 3);

    // Get position
    mju_copy(ee_position, &d->sensordata[3 * ee_pos_sensor_id], 3);
    // Get velocity
 
    mju_copy(ee_velocity, &d->sensordata[3 * ee_vel_sensor_id], 3);

    // Print the position and velocity
    DEBUG_PRINT("Mocap Position: [%f %f %f]\n", position[0], position[1], position[2]);
    DEBUG_PRINT("Mocap Velocity: [%f %f %f]\n", velocity[0], velocity[1], velocity[2]);
    DEBUG_PRINT("End Effector Position: [%f %f %f]\n", ee_position[0], ee_position[1], ee_position[2]);
    DEBUG_PRINT("End Effector Velocity: [%f %f %f]\n", ee_velocity[0], ee_velocity[1], ee_velocity[2]);
    }