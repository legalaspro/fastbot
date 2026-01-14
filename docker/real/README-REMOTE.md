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

# 5. Pi: Update docker-compose.yaml to use Tailscale config
#    Change: cyclonedds-local.xml → cyclonedds.xml
cd ~/fastbot/docker/real
nano docker-compose.yaml  # Change volume mount to ./cyclonedds.xml

# 6. Pi: Start robot
docker compose up -d robot

# 7. Dev: Start services and verify
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

First, update `docker-compose.yaml` to use the Tailscale CycloneDDS config:

```yaml
volumes:
  # Change from cyclonedds-local.xml to cyclonedds.xml for remote access
  - ./cyclonedds.xml:/ros2_ws/cyclonedds.xml:ro
```

Then start the robot:

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

> **Note:** Husarnet uses a sidecar container pattern on Mac/Windows since Docker can't access host VPN interfaces directly.

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

### 5. Configure Devcontainer (Dev Machine)

Copy the example env file:

```bash
cp .devcontainer/real-husarnet/.env.example .devcontainer/real-husarnet/.env
```

Edit `.devcontainer/real-husarnet/.env`:

```bash
HUSARNET_JOIN_CODE=fc94:xxxx:xxxx/xxxxxxxx
```

### 6. Start Devcontainer

In VS Code:

1. Open Command Palette (Cmd+Shift+P)
2. Select **"Dev Containers: Reopen in Container"**
3. Choose **"Fastbot Real Robot (Husarnet)"**

The devcontainer uses a sidecar pattern:

- `husarnet` container: Provides `hnet0` VPN interface
- `devcontainer` container: Shares husarnet's network namespace

### 7. Verify Connection

In devcontainer terminal:

```bash
# Check Husarnet status (run in husarnet container)
docker exec husarnet-devcontainer husarnet status
# Should show both fastbot-pi and fastbot-dev as "active"

# Check ROS2 topics
ros2 topic list
```

### Alternative: Docker Compose (without VS Code)

```bash
cd docker/real
echo "HUSARNET_JOIN_CODE=<your-join-code>" > .env
docker compose -f docker-compose.remote-husarnet.yaml up -d
docker exec -it fastbot-remote bash
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

| Feature                  | Tailscale             | Husarnet           |
| ------------------------ | --------------------- | ------------------ |
| **Setup difficulty**     | ⭐ Easy               | ⭐⭐ Moderate      |
| **IP type**              | IPv4 (100.x.x.x)      | IPv6 (fc94:...)    |
| **CycloneDDS**           | ✅ Excellent          | ✅ Hostname-based  |
| **Mac/Windows Docker**   | ✅ Works in container | ✅ Sidecar pattern |
| **Devcontainer support** | ✅ Yes                | ✅ Yes (sidecar)   |
| **Self-hosted option**   | ❌ No                 | ✅ Yes             |
| **Robotics features**    | ❌ General VPN        | ✅ Built for ROS   |
| **Free tier**            | 100 devices           | 5 devices          |

**Recommendation:** Use **Tailscale** for simpler setup, or **Husarnet** if you need self-hosting or robotics-specific features.

---

## File Reference

```
.devcontainer/
├── real/                              # Tailscale devcontainer
│   ├── devcontainer.json              # Main config (Tailscale)
│   ├── .env.example                   # Template for auth key
│   ├── .env                           # TAILSCALE_AUTHKEY (gitignored)
│   ├── services.sh                    # Service manager script
│   └── setup-tailscale.sh             # Tailscale startup script
│
└── real-husarnet/                     # Husarnet devcontainer
    ├── devcontainer.json              # Main config (Husarnet sidecar)
    ├── docker-compose.devcontainer.yaml  # Sidecar compose
    ├── .env.example                   # Template for join code
    └── .env                           # HUSARNET_JOIN_CODE (gitignored)

docker/real/
├── cyclonedds-local.xml               # Local config (no VPN, default)
├── cyclonedds.xml                     # Tailscale peer config
├── cyclonedds-husarnet.xml            # Husarnet peer config (hostnames)
├── docker-compose.yaml                # Pi: robot + slam
├── docker-compose.remote.yaml         # Dev: Tailscale (docker-compose alt)
├── docker-compose.remote-husarnet.yaml  # Dev: Husarnet (docker-compose alt)
├── Dockerfile.remote                  # Dev container image
├── Dockerfile.robot                   # Pi robot image
└── Dockerfile.slam                    # Pi SLAM image
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
