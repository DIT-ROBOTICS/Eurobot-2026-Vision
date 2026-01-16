#!/bin/bash
set -e

if [ -z "$ULTRALYTICS" ]; then
    echo "Skipping Ultralytics installation as ULTRALYTICS is not set"
    exit 0
fi

if [ "$ULTRALYTICS" = "YES" ]; then
    echo "Installing python and ultralytics"

    sudo apt-get update && sudo apt-get install -y \
        python3 python3-pip \
        libgl1 \
        libglib2.0-0 \
        libgtk-3-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        && sudo rm -rf /var/lib/apt/lists/*

    python3 -m pip install --upgrade pip \
        && python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 \
        && python3 -m pip install tqdm \
        && python3 -m pip install 'numpy<2' \
        && python3 -m pip install 'ultralytics>=8.0.0' \
        && python3 -m pip install roboflow \
        && python3 -m pip install pillow

    # Ensure OpenCV has HighGUI support (imshow). Some dependency chains pull in headless wheels.
    python3 -m pip uninstall -y opencv-python-headless opencv-contrib-python-headless || true
    python3 -m pip install --upgrade --force-reinstall opencv-python

    python3 -m pip install --force-reinstall 'numpy<2'
fi

echo "Ultralytics installation completed successfully!"
