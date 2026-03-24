#!/usr/bin/env bash

IMAGES_DIR=./images
BUILDER_NAME=container-builder

while [[ $# -gt 0 ]]; do
  case $1 in
    --images_dir)
      IMAGES_DIR="$2"
      shift
      shift
      ;;
    --builder_name)
      BUILDER_NAME="$2"
      shift
      shift
      ;;
    --dockerfile)
      CUSTOM_DOCKERFILE="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Compute absolute path to top-level AIC directory (two levels up from flowstate/scripts)
AIC_TOP_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
  DOCKERFILE="$CUSTOM_DOCKERFILE"
else
  echo "ERROR: No Dockerfile provided."
  exit 1
fi

# Use absolute path for Dockerfile to avoid context mismatches
DOCKERFILE_ABS=$(realpath "$DOCKERFILE")

# 1. Build the base image (loads to host daemon)
docker buildx build -t aic_model:latest \
  --load \
  --file "$DOCKERFILE_ABS" \
  "$AIC_TOP_DIR"

# 2. Build the flowstate service image on top of aic_model:latest
SERVICE_DIR="$AIC_TOP_DIR/flowstate/services/aic_model"
DOCKERFILE_SERVICE="$SERVICE_DIR/Dockerfile.service"

docker buildx build -t flowstate:aic_model \
  --load \
  --file "$DOCKERFILE_SERVICE" \
  "$SERVICE_DIR"

# 3. Export the service image to a .tar bundle
echo "INFO: Exporting image to tar file..."
mkdir -p ./images/aic_model
docker save -o ./images/aic_model/aic_model.tar flowstate:aic_model
chmod 644 images/aic_model/aic_model.tar

# 4. Bundle the service using inbuild
# Parse the SDK_VERSION from sdk_version.json
SDK_VERSION_FILE="$SCRIPT_DIR/../../../sdk-ros/intrinsic_sdk_cmake/cmake/sdk_version.json"
SDK_VERSION=$(grep -oP '"sdk_version": "\K[^"]+' "$SDK_VERSION_FILE")

# Download the 'inbuild' tool if it doesn't exist
if [ ! -f ./inbuild ]; then
  echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
  wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
  && chmod +x inbuild
fi

echo "INFO: Bundling service using inbuild..."
./inbuild -v 5 service bundle \
  --manifest "$SERVICE_DIR/aic_model.manifest.textproto" \
  --oci_image "./images/aic_model/aic_model.tar" \
  --output "./images/aic_model/aic_model.bundle.tar"