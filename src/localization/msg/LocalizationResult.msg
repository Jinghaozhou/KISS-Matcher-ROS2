# Result of one localization attempt or acceptance decision.
builtin_interfaces/Time completed_stamp
builtin_interfaces/Time local_map_stamp
builtin_interfaces/Time last_success_stamp
bool matched
bool applied
bool has_previous
# True when the old and candidate map->base_link poses could be compared.
bool delta_valid
string reason
geometry_msgs/TransformStamped candidate_map_to_odom
# Difference between the candidate and last applied map->base_link pose,
# evaluated using the same latest odom->base_link transform.
# These values are meaningful only when delta_valid is true.
float64 translation_delta_m
float64 rotation_delta_deg
uint32 inliers
float64 overlap_percent
float64 registration_ms
# Nonempty only when a successful result is pending acceptance.
string candidate_id
