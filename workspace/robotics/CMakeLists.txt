# Robotics domain aggregator — adds all domain sub-directories.
# No library target is produced here; each leaf module produces its own.

add_subdirectory(common)
add_subdirectory(control)
add_subdirectory(perception)
add_subdirectory(state_estimation)
add_subdirectory(motion_planning)
add_subdirectory(fleet_management)
