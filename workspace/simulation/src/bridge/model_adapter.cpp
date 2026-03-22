#include <simulation/bridge/model_adapter.hpp>
#include <string>
#include <cstring>

namespace robotlib::sim {

SimVehicleParams ModelAdapter::extractVehicleParams(const mjModel* m) {
    SimVehicleParams params;

    // Try to find robot body
    for (int i = 1; i < m->nbody; ++i) {
        const char* name = mj_id2name(m, mjOBJ_BODY, i);
        if (name && (std::strstr(name, "robot") || std::strstr(name, "chassis"))) {
            params.bodyId = i;
            params.mass = m->body_mass[i];
            break;
        }
    }

    // Try to extract wheel info from geom sizes
    for (int i = 0; i < m->ngeom; ++i) {
        const char* name = mj_id2name(m, mjOBJ_GEOM, i);
        if (name && std::strstr(name, "wheel")) {
            if (m->geom_type[i] == mjGEOM_CYLINDER) {
                params.wheelRadius = m->geom_size[3 * i + 1];  // cylinder half-height = radius
            }
        }
    }

    return params;
}

}  // namespace robotlib::sim
