#include <simulation/bridge/model_adapter.hpp>
#include <string>
#include <cstring>
#include <cmath>

namespace robotlib::sim {

SimVehicleParams ModelAdapter::extractVehicleParams(const mjModel* m) {
    SimVehicleParams params;

    // Try to find robot body by name
    int robotId = mj_name2id(m, mjOBJ_BODY, "robot_chassis");
    if (robotId >= 0) {
        params.bodyId = robotId;
        params.mass = m->body_mass[robotId];
    } else {
        // Fallback: search for body with "robot" or "chassis" in name
        for (int i = 1; i < m->nbody; ++i) {
            const char* name = mj_id2name(m, mjOBJ_BODY, i);
            if (name && (std::strstr(name, "robot") || std::strstr(name, "chassis"))) {
                params.bodyId = i;
                params.mass = m->body_mass[i];
                break;
            }
        }
    }

    // Try to extract wheel info from geom sizes
    // For mjGEOM_CYLINDER: geom_size[3*i+0] = radius, geom_size[3*i+1] = half-length
    for (int i = 0; i < m->ngeom; ++i) {
        const char* name = mj_id2name(m, mjOBJ_GEOM, i);
        if (name && std::strstr(name, "wheel")) {
            if (m->geom_type[i] == mjGEOM_CYLINDER) {
                params.wheelRadius = m->geom_size[3 * i + 0];  // cylinder radius
            }
        }
    }

    // Extract track width from wheel body positions
    int leftWheelBody = mj_name2id(m, mjOBJ_BODY, "left_wheel");
    int rightWheelBody = mj_name2id(m, mjOBJ_BODY, "right_wheel");
    if (leftWheelBody >= 0 && rightWheelBody >= 0) {
        double ly = m->body_pos[3 * leftWheelBody + 1];
        double ry = m->body_pos[3 * rightWheelBody + 1];
        params.trackWidth = std::abs(ly - ry);
    }

    // Extract actuator IDs by name
    int leftMotor = mj_name2id(m, mjOBJ_ACTUATOR, "left_motor");
    int rightMotor = mj_name2id(m, mjOBJ_ACTUATOR, "right_motor");
    if (leftMotor >= 0) params.leftWheelActuator = leftMotor;
    if (rightMotor >= 0) params.rightWheelActuator = rightMotor;

    return params;
}

}  // namespace robotlib::sim
