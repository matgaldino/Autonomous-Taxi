#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$ROOT_DIR/ros2_ws"
THIRD_PARTY_DIR="$WS_DIR/src/third_party"
SLLIDAR_DIR="$THIRD_PARTY_DIR/sllidar_ros2"
PATCH_FILE="$ROOT_DIR/patches/sllidar_ros2-lyrical.patch"
REPOS_FILE="$ROOT_DIR/dependencies.repos"

command -v vcs >/dev/null 2>&1 || {
    echo "Error: vcstool is not installed."
    echo "Install it with: sudo apt install python3-vcstool"
    exit 1
}

command -v rosdep >/dev/null 2>&1 || {
    echo "Error: rosdep is not installed."
    exit 1
}

mkdir -p "$THIRD_PARTY_DIR"

if [ ! -d "$SLLIDAR_DIR/.git" ]; then
    echo "Importing external dependencies..."
    vcs import "$THIRD_PARTY_DIR" < "$REPOS_FILE"
else
    echo "sllidar_ros2 already present."
fi

EXPECTED_COMMIT="$(awk '$1 == "version:" {print $2; exit}' "$REPOS_FILE")"
CURRENT_COMMIT="$(git -C "$SLLIDAR_DIR" rev-parse HEAD)"

if [ "$CURRENT_COMMIT" != "$EXPECTED_COMMIT" ]; then
    echo "Error: sllidar_ros2 is not at the expected commit."
    echo "Expected: $EXPECTED_COMMIT"
    echo "Current:  $CURRENT_COMMIT"
    exit 1
fi

if git -C "$SLLIDAR_DIR" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1; then
    echo "Lyrical compatibility patch already applied."

elif git -C "$SLLIDAR_DIR" apply --check "$PATCH_FILE" >/dev/null 2>&1; then
    echo "Applying Lyrical compatibility patch..."
    git -C "$SLLIDAR_DIR" apply "$PATCH_FILE"

else
    echo "Error: patch cannot be applied cleanly."
    exit 1
fi

echo "Installing ROS dependencies..."
rosdep install \
    --from-paths "$WS_DIR/src" \
    --ignore-src \
    -r \
    -y

echo
echo "Dependencies successfully configured."