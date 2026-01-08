# Fastbot Remote Access - VPN Setup

Connect to your Fastbot robot from a development machine (Mac/Linux/Windows) using VPN.

## Contents

- [Overview](#overview)
- [Option A: Tailscale](#option-a-tailscale)
- [Option B: Husarnet](#option-b-husarnet)
- [Comparison](#comparison)

## Overview

To control the robot remotely, you need a VPN to connect your dev machine and the Raspberry Pi. This enables:

- **ROS2 topic discovery** across networks
- **WebApp** for robot control (port 8000)
- **ROSBridge** for web-based ROS communication (port 9090)
- **Web Video Server** for camera streaming (port 11315)
- **RViz2** for visualization
- **Nav2** for autonomous navigation

Choose either **Tailscale** (easier setup, IPv4) or **Husarnet** (robotics-focused, IPv6).

---

## Option A: Tailscale

Tailscale is a zero-config VPN that works well with ROS2/CycloneDDS.

### 1. Install Tailscale

**On Raspberry Pi:**

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

**On Mac (Devcontainer):**
The devcontainer installs and runs Tailscale automatically. You just need to provide an auth key.

### 2. Get Tailscale Auth Key

Generate a reusable auth key from [login.tailscale.com/admin/settings/keys](https://login.tailscale.com/admin/settings/keys):

- Click "Create auth key"
- Check "Reusable" and "Ephemeral"
- Copy the key

### 3. Configure Devcontainer

In `.devcontainer/real/.env`:

```env
TAILSCALE_AUTHKEY=tskey-auth-YOUR_KEY_HERE
```

Replace `YOUR_KEY_HERE` with your auth key from step 2.

### 4. Get Tailscale IPs

**On Pi:**

```bash
tailscale ip -4
```

**In Devcontainer:**
Open devcontainer and run:

```bash
tailscale ip -4
```

### 5. Update CycloneDDS Config

On the Pi, edit `cyclonedds.xml` with both IPs:

```xml
<Peers>
  <Peer address="100.x.x.x"/>  <!-- Pi Tailscale IP -->
  <Peer address="100.y.y.y"/>  <!-- Devcontainer Tailscale IP -->
</Peers>
```

### 6. Run Robot with Tailscale Config

```bash
# On Pi - uses cyclonedds.xml (Tailscale)
cd ~/fastbot/docker/real
docker compose up -d robot
```

### 7. Run Remote Devcontainer (Dev Machine)

Open the repo in VS Code and select "Reopen in Container" → "Fastbot Real Robot (Remote)"

**Alternative: Docker Compose**

If using Docker Compose instead of devcontainer, use `docker-compose.remote.yaml` with additional steps.

### 8. Access Services

- **WebApp:** http://localhost:8000
- **ROSBridge:** ws://localhost:9090
- **Video:** http://localhost:11315

---

## Option B: Husarnet

Husarnet is a P2P VPN designed for robotics with native IPv6 support.

### 1. Create Husarnet Account

1. Sign up at [app.husarnet.com](https://app.husarnet.com)
2. Create a network (e.g., "fastbot-network")
3. Copy the **Join Code**

### 2. Install Husarnet on Raspberry Pi

```bash
curl -fsSL https://install.husarnet.com/install.sh | sudo bash
sudo systemctl enable husarnet
sudo husarnet join <YOUR_JOIN_CODE> fastbot-pi
```

Verify:

```bash
husarnet status
```

### 3. Update CycloneDDS Config

The robot uses `cyclonedds-husarnet.xml`. Husarnet adds hostnames to `/etc/hosts`, so peer discovery works via hostnames.

### 4. Run Robot with Husarnet Config

Edit `docker-compose.yaml` to use Husarnet config:

```yaml
volumes:
  - ./cyclonedds-husarnet.xml:/ros2_ws/cyclonedds.xml:ro
```

Then start:

```bash
cd ~/fastbot/docker/real
docker compose up -d robot
```

### 5. Run Remote Container (Dev Machine)

On Mac/Windows, Docker containers can't access host VPN interfaces directly. Use the Husarnet sidecar pattern:

**Option A: VS Code Devcontainer (Recommended)**

1. Create `.devcontainer/real-husarnet/.env`:

   ```
   HUSARNET_JOIN_CODE=<your-join-code>
   ```

2. Open repo in VS Code → "Reopen in Container" → "Fastbot Real Robot (Husarnet)"

**Option B: Docker Compose**

1. Create `.env` file:

   ```bash
   cd docker/real
   echo "HUSARNET_JOIN_CODE=<your-join-code>" > .env
   ```

2. Start:
   ```bash
   docker compose -f docker-compose.remote-husarnet.yaml up -d
   ```

### 6. Verify Connection

```bash
# Check Husarnet status in container
docker exec husarnet husarnet status

# Should show both peers as "active"
```

### 7. Access Services

- **WebApp:** http://localhost:8000
- **ROSBridge:** ws://localhost:9090
- **Video:** http://localhost:11315

---

## Comparison

| Feature                  | Tailscale                 | Husarnet                        |
| ------------------------ | ------------------------- | ------------------------------- |
| IP type                  | IPv4 (100.x.x.x)          | IPv6 (fc94:...)                 |
| CycloneDDS compatibility | ✅ Excellent              | ⚠️ Requires hostname resolution |
| Mac Docker support       | ✅ We installed Tailscale | ⚠️ Needs sidecar container      |
| Robotics-focused         | ❌ General VPN            | ✅ Built for ROS                |
| Self-hosted option       | ❌ No                     | ✅ Yes                          |

**Recommendation:** Use **Tailscale** for simpler setup, or **Husarnet** if you need robotics-specific features or self-hosting.

---

## Troubleshooting

### ROS topics not visible?

Check VPN connection:

```bash
# Tailscale
tailscale status

# Husarnet
husarnet status
```

### CycloneDDS not discovering peers?

Verify the correct `cyclonedds.xml` is mounted and IPs/hostnames are correct.
