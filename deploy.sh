#!/bin/bash
scp .pioenvs/nodemcuv2/firmware.bin shibuya:/etc/openhab2/html/fw_dev.bin && \
sed -e 's/const int FW_VERSION = \(..\);/\1/' -e 't' -e 'd'  src/aprilbrother_soil.ino | ssh shibuya "cat - > /etc/openhab2/html/fw_dev.version"