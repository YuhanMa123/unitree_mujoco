#include "g1_parkour_policy.h"
namespace g1_visual_policy = g1_parkour;
#define G1_VISUAL_DEFAULT_CONFIG G1_PARKOUR_DEFAULT_CONFIG
#define G1_VISUAL_DEFAULT_MODELS G1_PARKOUR_DEFAULT_MODELS
#define G1_VISUAL_NETWORK_DESCRIPTION "ONNX ready: depth [1,8,18,32] -> [1,128]; actor [1,896] -> [1,29]."
#define G1_VISUAL_POLICY_NAME "Parkour"
#include "g1_visual_controller.h"
