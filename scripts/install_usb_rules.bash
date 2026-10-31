#!/usr/bin/env bash
set -e
sudo groupadd -f plugdev
sudo usermod -a -G plugdev "$USER"
sudo tee /etc/udev/rules.d/99-safe-redcho-bitcraze.rules >/dev/null <<'RULES'
# Crazyradio normal operation and bootloader; Crazyflie USB.
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger
printf '%s\n' 'Log out/in for group membership to update, then unplug/replug Crazyradio.'
