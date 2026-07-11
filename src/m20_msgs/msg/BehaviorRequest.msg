# Version: 0.1
# Request execution of a named behavior. m20_behaviors is the only consumer.
std_msgs/Header header
string behavior_name      # registry name, e.g. "jump_pipe", "self_right", "camera_scan"
string requester
string params_json        # behavior-specific parameters (target pose, scan sector, ...)
