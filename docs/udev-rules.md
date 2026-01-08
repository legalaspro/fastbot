# Udev Rules for Fastbot

Udev rules create stable device symlinks so USB devices always have the same path regardless of plug order.

## Quick Setup

```bash
# Create the rules file
sudo nano /etc/udev/rules.d/99-fastbot.rules
```

Paste these rules (adjust vendor/product IDs for your hardware):

```bash
# Lslidar N10 (CP2102 USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lslidar", MODE="0666"

# Arduino Nano (CH340 USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino", MODE="0666"
```

Apply the rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Finding Your Device IDs

Plug in ONE device at a time and run:

```bash
# Find which ttyUSB it is
ls /dev/ttyUSB*

# Get device info
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
```

Example output:
```
ATTRS{idVendor}=="10c4"
ATTRS{idProduct}=="ea60"
```

## Verify

After applying rules, check symlinks exist:

```bash
ls -la /dev/lslidar /dev/arduino
# Should show something like:
# lrwxrwxrwx 1 root root 7 Jan  6 12:00 /dev/arduino -> ttyUSB0
# lrwxrwxrwx 1 root root 7 Jan  6 12:00 /dev/lslidar -> ttyUSB1
```

## Common Vendor/Product IDs

| Device | Chip | idVendor | idProduct |
|--------|------|----------|-----------|
| Lslidar N10 | CP2102 | 10c4 | ea60 |
| Arduino Nano (clone) | CH340 | 1a86 | 7523 |
| Arduino Nano (original) | FTDI | 0403 | 6001 |
| Arduino Uno | ATmega16U2 | 2341 | 0043 |

## Troubleshooting

**Device not appearing?**
```bash
# Check kernel messages
dmesg | tail -20
```

**Permission denied?**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and back in
```

