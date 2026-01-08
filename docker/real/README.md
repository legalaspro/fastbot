# Fastbot Real Robot - Raspberry Pi Setup

Quick guide to run Fastbot on Raspberry Pi with Docker.

## Contents

- [Prerequisites](#prerequisites)
- [USB Device Setup](#usb-device-setup)
- [Running the Robot](#running-the-robot)
- [Adding SLAM After Robot is Running](#adding-slam-after-robot-is-running)
- [Rebuilding Images](#rebuilding-images)
- [Auto-Start on Boot](#auto-start-on-boot-optional)
- [Device Mapping](#device-mapping)
- [Troubleshooting](#troubleshooting)

> **Remote Access:** To control the robot from a development machine (Mac/Linux/Windows), see [README-REMOTE.md](README-REMOTE.md).

## Prerequisites

### Install Docker

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in for group changes
```

### Install Docker Compose Plugin

Download the plugin directly (recommended for Raspberry Pi):

```bash
# Create cli-plugins directory
mkdir -p ~/.docker/cli-plugins/

# Download docker-compose (check for latest version at https://github.com/docker/compose/releases)
curl -SL https://github.com/docker/compose/releases/download/v2.32.4/docker-compose-linux-aarch64 -o ~/.docker/cli-plugins/docker-compose

# Make executable
chmod +x ~/.docker/cli-plugins/docker-compose
```

Verify installation:

```bash
docker compose version
```

## USB Device Setup

Docker doesn't pass through udev symlinks, so we map actual device paths directly.

### Identify Your Devices

Plug in devices one at a time and check:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -10
```

Typically:

- **Lidar (CP2102)** → `/dev/ttyACM0`
- **Arduino (CH340)** → `/dev/ttyUSB0`

### Update docker-compose.yaml

Edit the device mappings to match your setup:

```yaml
devices:
  - /dev/ttyACM0:/dev/lslidar # Lidar: host device -> container path
  - /dev/ttyUSB0:/dev/arduino_nano # Arduino: host device -> container path
  - /dev/video0:/dev/video0 # Camera
```

> **Note:** For running ROS2 directly on the host (without Docker), see `docs/udev` for udev rules setup.

## Running the Robot

### Start Robot Only

```bash
cd ~/fastbot/docker/real
docker compose up -d robot
```

### Start Robot + SLAM

```bash
docker compose up -d
```

The SLAM container waits for the robot to be healthy (lidar publishing) before starting.

### View Logs

```bash
docker compose logs -f          # All services
docker compose logs -f robot    # Robot only
docker compose logs -f slam     # SLAM only
```

### Check Status

```bash
docker ps
```

Expected output when healthy:

```
CONTAINER ID   IMAGE                                       COMMAND                  CREATED         STATUS                    NAMES
45bccb2327ca   legalaspro/fastbot:fastbot-ros2-real        "/ros_entrypoint.sh …"   5 minutes ago   Up 29 seconds (healthy)   fastbot-ros2-real
a1b2c3d4e5f6   legalaspro/fastbot:fastbot-ros2-slam-real   "/ros_entrypoint.sh …"   5 minutes ago   Up 25 seconds             fastbot-ros2-slam-real
```

### Stop All Services

```bash
docker compose down
```

## Adding SLAM After Robot is Running

If robot is already running and you want to add SLAM:

```bash
docker compose up -d slam
```

## Rebuilding Images

After code changes, rebuild using the Makefile (from repo root):

```bash
# Pull latest code
git pull

# Rebuild robot image
make build-real

# Rebuild SLAM image
make build-real-slam

# Restart with new images
cd docker/real
docker compose down
docker compose up -d
```

## Auto-Start on Boot (Optional)

To automatically start the robot when the Pi boots:

```bash
sudo nano /etc/systemd/system/fastbot.service
```

Add this content:

```ini
[Unit]
Description=Fastbot ROS2 Robot
Requires=docker.service
After=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/fastbot/fastbot/docker/real
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable fastbot.service
sudo systemctl start fastbot.service
```

Check status:

```bash
sudo systemctl status fastbot.service
```

> **Note:** Adjust `WorkingDirectory` path if your repo is in a different location.

## Device Mapping

Docker maps host devices to container paths:

| Host Device  | Container Path    | Description              |
| ------------ | ----------------- | ------------------------ |
| /dev/ttyACM0 | /dev/lslidar      | Lidar serial             |
| /dev/ttyUSB0 | /dev/arduino_nano | Arduino motor controller |
| /dev/video0  | /dev/video0       | Camera                   |

> **Note:** Adjust `ttyACM0`/`ttyUSB0` in `docker-compose.yaml` if your devices enumerate differently.

## Troubleshooting

### Container not starting?

```bash
docker compose logs robot
```

### Device permission denied?

```bash
# Check device permissions
ls -la /dev/ttyUSB0 /dev/ttyACM0

# Should show rw-rw-rw- (666)
# If not, check udev rules have MODE="0666"
```

### ROS topics not visible between containers?

Both containers use `network_mode: "host"` and same `ROS_DOMAIN_ID=1`.

### Lidar/Arduino not detected?

Check which tty they're on:

```bash
dmesg | tail -20
ls /dev/ttyUSB* /dev/ttyACM*
```

Update `docker-compose.yaml` device mappings accordingly.
