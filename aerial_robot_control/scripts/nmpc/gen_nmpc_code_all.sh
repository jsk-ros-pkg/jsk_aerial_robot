#!/bin/bash

# ===== avoid jobserver warning when "catkin build", given by ChatGPT o3
set -e
unset MAKEFLAGS
# =====

MODELS=(
#    NMPCFixQdAngvelOut
#    NMPCFixQdThrustOut
   #NMPCTiltQdNoServo
   #NMPCTiltQdServo
   #NMPCTiltQdServoDist
   #NMPCTiltQdServoImpedance
   #NMPCTiltQdServoThrustDist
   #NMPCTiltQdServoThrustImpedance
#    NMPCTiltTriServo
#    NMPCTiltBiServo
#    NMPCTiltBi2OrdServo
   #MHEWrenchEstAccMom
)

for model in "${MODELS[@]}"
do
    echo "Generating NMPC code for model: $model"
    python3 gen_nmpc_code.py -m "$model"
done
