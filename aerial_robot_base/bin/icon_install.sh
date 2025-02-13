#!/bin/bash

# Get the package path using rospack
SRC_DIR=$(rospack find aerial_robot_base)/images

# Destination directory for symbolic links
DEST_DIR="$HOME/.local/share/icons"

# Create destination directory if it does not exist
mkdir -p "$DEST_DIR"

# Loop through all .png files in the source directory
for file in "$SRC_DIR"/*.png; do

    # Determine the symbolic link name (use the original filename)
    link_name="$DEST_DIR/$(basename "$file")"

    # Force create symbolic link (overwrite if it exists)
    ln -sf "$file" "$link_name"
done
