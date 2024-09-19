#!/bin/bash

# Function to display script usage
usage() {
    echo "Usage: $0 <num_iterations>"
    echo "Example: $0 10"
    exit 1
}

# Check if the number of arguments is correct
if [ $# -ne 1 ]; then
    usage
fi

# Extracting the number of iterations from command line arguments
num_iterations=$1

# Loop for the specified number of iterations
for ((i=1; i<=$num_iterations; i++))
do
    echo "Iteration $i:"
    rostopic pub -1 /servo/target_states spinal/ServoControlCmd "{index: [0], angles: [1024]}"
    rostopic pub -1 /servo/target_states spinal/ServoControlCmd "{index: [0], angles: [-1024]}"
done

