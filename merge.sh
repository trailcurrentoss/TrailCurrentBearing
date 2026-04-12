#!/bin/bash
# Create a merged binary (bootloader + partition table + OTA data + app)
# that can be flashed at offset 0x0 for the web flasher.
#
# Output: build/bearing_merged.bin
#
# The original build/bearing.bin (app-only) is left untouched — Headwaters
# OTA needs the app-only binary because esp_ota_write targets a single app
# partition and validates an app image header. A merged binary would fail
# that validation.
#
# Both files should be attached to a GitHub release:
#   bearing.bin         — OTA updates (Headwaters deploy.sh / ota.js)
#   bearing_merged.bin  — Web flasher (full flash at 0x0)
set -e

BUILD_DIR="build"

echo "Creating merged firmware binary..."
esptool.py --chip esp32s3 merge_bin -o "$BUILD_DIR/bearing_merged.bin" \
    --flash_mode dio --flash_size 4MB \
    0x0 "$BUILD_DIR/bootloader/bootloader.bin" \
    0x8000 "$BUILD_DIR/partition_table/partition-table.bin" \
    0xe000 "$BUILD_DIR/ota_data_initial.bin" \
    0x10000 "$BUILD_DIR/bearing.bin"

echo ""
echo "Build artifacts:"
ls -lh "$BUILD_DIR/bearing.bin" "$BUILD_DIR/bearing_merged.bin"
echo ""
echo "Attach BOTH files to the GitHub release:"
echo "  bearing.bin         — for OTA updates via Headwaters"
echo "  bearing_merged.bin  — for web flasher (full flash at 0x0)"
