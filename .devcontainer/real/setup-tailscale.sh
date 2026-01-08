#!/bin/bash
# =============================================================================
# Start Tailscale in devcontainer
#
# First time:  TAILSCALE_AUTHKEY=tskey-auth-xxx bash setup-tailscale.sh
# After that:  Auto-connects using saved state
#
# Get auth key: https://login.tailscale.com/admin/settings/keys
# =============================================================================

# Create TUN device (needed for privileged container)
mkdir -p /dev/net
mknod /dev/net/tun c 10 200 2>/dev/null || true

# Start daemon if not running (nohup keeps it alive)
if ! pgrep -x tailscaled >/dev/null; then
    nohup tailscaled --state=/var/lib/tailscale/tailscaled.state &>/var/log/tailscaled.log &
    sleep 3
fi

# Connect (uses saved state or auth key)
if tailscale status &>/dev/null; then
    echo "Tailscale: $(tailscale ip -4)"
elif [ -n "$TAILSCALE_AUTHKEY" ]; then
    tailscale up --authkey="$TAILSCALE_AUTHKEY" --hostname="fastbot-dev"
else
    tailscale up --hostname="fastbot-dev"
fi

