# Fastbot Remote Access - VPN Setup

Connect to your Fastbot robot from a development machine (Mac/Linux/Windows) over VPN.

## Quick Start (Tailscale - Recommended)

```bash
# 1. Pi: Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4  # Note this IP (e.g., 100.88.1.19)

# 2. Dev: Create .env with auth key
echo "TAILSCALE_AUTHKEY=tskey-auth-YOUR_KEY" > .devcontainer/real/.env

# 3. Dev: Open in VS Code → "Reopen in Container" → "Fastbot Real Robot (Remote)"

# 4. In devcontainer: Get dev IP and update cyclonedds.xml
tailscale ip -4  # Note this IP (e.g., 100.73.231.76)
# Edit docker/real/cyclonedds.xml with both IPs

# 5. Pi: Start robot
cd ~/fastbot/docker/real && docker compose up -d robot

# 6. Dev: Start services and verify
./services.sh start
ros2 topic list  # Should see /fastbot/scan, /odom, etc.
```

---

## Contents

- [Architecture](#architecture)
- [Option A: Tailscale](#option-a-tailscale-recommended)
- [Option B: Husarnet](#option-b-husarnet)
- [Services & Ports](#services--ports)
- [Comparison](#comparison)
- [Troubleshooting](#troubleshooting)

---

## Architecture

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│  Raspberry Pi (Robot)       │         │  Dev Machine (Mac/Win/Linux)│
│                             │         │                             │
│  ┌───────────────────────┐  │         │  ┌───────────────────────┐  │
│  │ Docker: robot + slam  │  │         │  │ VS Code Devcontainer  │  │
│  │ - Lidar driver        │  │         │  │ - RViz2               │  │
│  │ - Motor driver        │  │         │  │ - Nav2                │  │
│  │ - Cartographer        │  │   VPN   │  │ - ROSBridge (:9090)   │  │
│  │ - CycloneDDS ─────────┼──┼─────────┼──┼─ CycloneDDS           │  │
│  └───────────────────────┘  │         │  │ - WebVideo (:11315)   │  │
│                             │         │  │ - WebApp (:8000)      │  │
│  Tailscale: 100.x.x.x       │         │  └───────────────────────┘  │
│  (or Husarnet: fc94:...)    │         │                             │
└─────────────────────────────┘         │  Tailscale: 100.y.y.y       │
                                        │  noVNC Desktop: :6080       │
                                        └─────────────────────────────┘
```

**Why VPN?** ROS2 DDS uses multicast for discovery, which doesn't work across networks. VPN creates a virtual network where both machines can see each other via unicast peer discovery.

---

## Option A: Tailscale (Recommended)

Tailscale is a zero-config mesh VPN. Works seamlessly with CycloneDDS.

### 1. Install Tailscale on Pi

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

Note the Tailscale IP:

```bash
tailscale ip -4
# Example: 100.88.1.19
```

### 2. Get Auth Key for Devcontainer

Generate a reusable auth key from [Tailscale Admin Console](https://login.tailscale.com/admin/settings/keys):

1. Click **"Generate auth key"**
2. Enable **"Reusable"** and **"Ephemeral"**
3. Copy the key (starts with `tskey-auth-`)

### 3. Configure Devcontainer

Create `.devcontainer/real/.env`:

```bash
TAILSCALE_AUTHKEY=tskey-auth-xxxxxxxxxxxxx
```

> **Note:** This file is gitignored. Never commit auth keys.

### 4. Start Devcontainer

In VS Code:

1. Open Command Palette (Cmd+Shift+P)
2. Select **"Dev Containers: Reopen in Container"**
3. Choose **"Fastbot Real Robot (Remote)"**

The container will:

- Start Tailscale daemon automatically
- Connect using your auth key
- Persist state in `tailscale-state` volume

### 5. Get Devcontainer Tailscale IP

In the devcontainer terminal:

```bash
tailscale ip -4
# Example: 100.73.231.76
```

### 6. Update CycloneDDS Config

Edit `docker/real/cyclonedds.xml` with both IPs:

```xml
<Peers>
  <Peer address="100.88.1.19"/>   <!-- Pi Tailscale IP -->
  <Peer address="100.73.231.76"/> <!-- Devcontainer Tailscale IP -->
</Peers>
```

> **Important:** This file is used by BOTH Pi and devcontainer. Update it once, both sides read it.

### 7. Start Robot on Pi

```bash
cd ~/fastbot/docker/real
docker compose up -d robot

# Optional: Start SLAM
docker compose up -d slam
```

### 8. Start Services in Devcontainer

```bash
# Start all web services
./services.sh start

# Verify ROS2 connectivity
ros2 topic list
# Should see: /fastbot/scan, /odom, /tf, /map (if SLAM running)
```

### 9. Access Applications

| Service       | URL                    | Description          |
| ------------- | ---------------------- | -------------------- |
| noVNC Desktop | http://localhost:6080  | Run RViz2 here       |
| WebApp        | http://localhost:8000  | Robot control UI     |
| ROSBridge     | ws://localhost:9090    | WebSocket for webapp |
| Web Video     | http://localhost:11315 | Camera stream        |

---

## Option B: Husarnet

Husarnet is a P2P VPN designed for robotics. Uses IPv6 with hostname-based discovery.

> **Note:** Husarnet requires a sidecar container on Mac/Windows since Docker can't access host VPN interfaces. There is no Husarnet-specific devcontainer - use docker-compose only.

### 1. Create Husarnet Account

1. Sign up at [app.husarnet.com](https://app.husarnet.com)
2. Create a network (e.g., "fastbot-network")
3. Copy the **Join Code**

### 2. Install Husarnet on Pi (Host, not Docker)

```bash
curl -fsSL https://install.husarnet.com/install.sh | sudo bash
sudo systemctl enable husarnet
sudo husarnet join <YOUR_JOIN_CODE> fastbot-pi
```

Verify:

```bash
husarnet status
# Should show: fastbot-pi is connected
```

### 3. Configure Pi to Use Husarnet CycloneDDS

Edit `docker-compose.yaml` on Pi:

```yaml
volumes:
  - ./cyclonedds-husarnet.xml:/ros2_ws/cyclonedds.xml:ro
```

### 4. Start Robot on Pi

```bash
cd ~/fastbot/docker/real
docker compose up -d robot
```

### 5. Start Remote Container (Dev Machine)

Create `.env` in `docker/real/`:

```bash
echo "HUSARNET_JOIN_CODE=<your-join-code>" > docker/real/.env
```

Start with docker-compose (sidecar pattern):

```bash
cd docker/real
docker compose -f docker-compose.remote-husarnet.yaml up -d
```

This starts:

- `husarnet` container: Provides `hnet0` VPN interface
- `fastbot-remote` container: Shares husarnet's network namespace

### 6. Verify Connection

```bash
docker exec husarnet husarnet status
# Should show both fastbot-pi and fastbot-dev as "active"
```

### 7. Access Shell

```bash
docker exec -it fastbot-remote bash
ros2 topic list
```

---

## Services & Ports

The devcontainer includes a service manager script:

```bash
# Start all services (ROSBridge, WebVideo, TF2Web, HTTP Server)
./services.sh start

# Check status
./services.sh status

# View logs
./services.sh logs

# Stop all
./services.sh stop

# Save map from Cartographer
./services.sh save-map my_room
```

### Port Reference

| Port  | Service                 | Protocol  |
| ----- | ----------------------- | --------- |
| 6080  | noVNC (browser desktop) | HTTP      |
| 5901  | VNC Direct              | VNC       |
| 8000  | WebApp                  | HTTP      |
| 9090  | ROSBridge               | WebSocket |
| 11315 | Web Video Server        | HTTP      |

---

## Comparison

| Feature                  | Tailscale             | Husarnet                 |
| ------------------------ | --------------------- | ------------------------ |
| **Setup difficulty**     | ⭐ Easy               | ⭐⭐ Moderate            |
| **IP type**              | IPv4 (100.x.x.x)      | IPv6 (fc94:...)          |
| **CycloneDDS**           | ✅ Excellent          | ⚠️ Needs hostname config |
| **Mac/Windows Docker**   | ✅ Works in container | ⚠️ Requires sidecar      |
| **Devcontainer support** | ✅ Yes                | ❌ Docker-compose only   |
| **Self-hosted option**   | ❌ No                 | ✅ Yes                   |
| **Robotics features**    | ❌ General VPN        | ✅ Built for ROS         |
| **Free tier**            | 100 devices           | 5 devices                |

**Recommendation:** Use **Tailscale** unless you need self-hosting or have specific Husarnet requirements.

---

## File Reference

```
.devcontainer/real/
├── devcontainer.json              # Main devcontainer config
├── devcontainer.tailscale-feature.json  # Reference: official Tailscale feature
├── .env                           # TAILSCALE_AUTHKEY (gitignored)
├── services.sh                    # Service manager script
└── setup-tailscale.sh             # Tailscale startup script

docker/real/
├── cyclonedds.xml                 # Tailscale peer config
├── cyclonedds-husarnet.xml        # Husarnet peer config
├── docker-compose.yaml            # Pi: robot + slam
├── docker-compose.remote.yaml     # Dev: Tailscale (reference)
├── docker-compose.remote-husarnet.yaml  # Dev: Husarnet sidecar
├── Dockerfile.remote              # Dev container image
├── Dockerfile.robot               # Pi robot image
└── Dockerfile.slam                # Pi SLAM image
```

---

## Troubleshooting

### ROS2 topics not visible?

1. **Check VPN connection:**

   ```bash
   # Tailscale
   tailscale status

   # Husarnet
   husarnet status
   ```

2. **Ping the other machine:**

   ```bash
   ping 100.x.x.x  # Pi's Tailscale IP
   ```

3. **Check CycloneDDS config:**
   ```bash
   echo $CYCLONEDDS_URI
   cat /ros2_ws/cyclonedds.xml
   ```

### Tailscale not connecting?

```bash
# Check daemon status
pgrep tailscaled

# Manually start
bash /ros2_ws/src/.devcontainer/real/setup-tailscale.sh

# Check logs
tail -f /var/log/tailscaled.log
```

### noVNC not working?

```bash
# Restart desktop service
/usr/local/share/desktop-init.sh
```

### Transform timeout errors?

Clock drift between Pi and devcontainer. Options:

1. **Enable NTP on Pi:** `sudo timedatectl set-ntp true`
2. **Increase tolerance:** Edit `transform_tolerance` in config files (temporary fix)

### WebApp can't connect to ROSBridge?

```bash
# Check ROSBridge is running
./services.sh status

# Restart services
./services.sh restart
```

---

## Notes

- **Auth keys expire.** Generate new ones if connection fails after weeks/months.
- **Tailscale state persists** in Docker volume `tailscale-state`. First connection requires auth key, subsequent starts auto-connect.
- **Maps saved** via `./services.sh save-map` go to `/ros2_ws/maps/` (Docker volume `fastbot-maps`).
