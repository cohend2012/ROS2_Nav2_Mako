# Version: 0.1
# One mission primitive. Consumed by m20_missions, executed via Nav2.
uint8 TYPE_GOTO=0
uint8 TYPE_WAIT=1
uint8 TYPE_SET_GAIT=2
uint8 TYPE_INSPECT=3

uint8 type
geometry_msgs/PoseStamped goal    # for TYPE_GOTO / TYPE_INSPECT
float32 wait_seconds              # for TYPE_WAIT
uint8 gait                        # for TYPE_SET_GAIT (GaitRequest constants)
string label
