#include "g1_ca_pe_amp_vae_policy.h"
namespace g1_visual_policy = g1_ca_pe_amp_vae;
#define G1_VISUAL_DEFAULT_CONFIG G1_CA_PE_DEFAULT_CONFIG
#define G1_VISUAL_DEFAULT_MODELS G1_CA_PE_DEFAULT_MODELS
#define G1_VISUAL_NETWORK_DESCRIPTION "ONNX ready: depth [1,8,32,32] -> tokens [1,64,64]; actor (96,480,tokens) -> [1,29]."
#define G1_VISUAL_POLICY_NAME "CA PE AMP VAE"
#include "g1_visual_controller.h"
