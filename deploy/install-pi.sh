#!/usr/bin/env bash
# Install Orbitaly as a system service on a Raspberry Pi.
#
#   sudo ./deploy/install-pi.sh
#
# Idempotent: safe to re-run to upgrade an existing install.
set -euo pipefail

PREFIX=${PREFIX:-/opt/orbitaly}
CONFIG_DIR=${CONFIG_DIR:-/etc/orbitaly}
SERVICE_USER=${SERVICE_USER:-orbitaly}
SOURCE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [[ $EUID -ne 0 ]]; then
    echo "Run this with sudo." >&2
    exit 1
fi

model=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "unknown")
echo "Installing Orbitaly on: ${model}"
case "${model}" in
    *"Raspberry Pi 5"*|*"Raspberry Pi 500"*)
        echo "  Pi 5 detected — using lgpio (RPi.GPIO and pigpio cannot work on RP1)." ;;
    *"Raspberry Pi"*)
        echo "  Using lgpio via the kernel gpiochip interface." ;;
    *)
        echo "  Not a Raspberry Pi: Orbitaly will run with the simulated rotator." ;;
esac

if ! id "${SERVICE_USER}" &>/dev/null; then
    echo "Creating service user ${SERVICE_USER}"
    useradd --system --home-dir "${PREFIX}" --shell /usr/sbin/nologin "${SERVICE_USER}"
fi
if getent group gpio >/dev/null; then
    usermod -aG gpio "${SERVICE_USER}"
else
    echo "WARNING: no 'gpio' group on this system; check /dev/gpiochip* permissions yourself." >&2
fi

echo "Installing to ${PREFIX}"
mkdir -p "${PREFIX}" "${CONFIG_DIR}"
cp -r "${SOURCE_DIR}/orbitaly" "${SOURCE_DIR}/pyproject.toml" "${SOURCE_DIR}/README.md" "${PREFIX}/"

if [[ ! -d "${PREFIX}/.venv" ]]; then
    python3 -m venv "${PREFIX}/.venv" 2>/dev/null || {
        echo "python3-venv is missing. Install it with: apt install python3-venv" >&2
        exit 1
    }
fi
"${PREFIX}/.venv/bin/pip" install --upgrade pip >/dev/null
if [[ "${model}" == *"Raspberry Pi"* ]]; then
    "${PREFIX}/.venv/bin/pip" install "${PREFIX}[pi]"
else
    "${PREFIX}/.venv/bin/pip" install "${PREFIX}"
fi

if [[ ! -f "${CONFIG_DIR}/config.yaml" ]]; then
    cp "${SOURCE_DIR}/config.example.yaml" "${CONFIG_DIR}/config.yaml"
    echo "Wrote ${CONFIG_DIR}/config.yaml — set your station coordinates before tracking."
fi
chown -R "${SERVICE_USER}:${SERVICE_USER}" "${PREFIX}" "${CONFIG_DIR}"

install -m 644 "${SOURCE_DIR}/deploy/orbitaly.service" /etc/systemd/system/orbitaly.service
systemctl daemon-reload
systemctl enable orbitaly.service

echo
echo "Installed. Before starting, check the machine over:"
echo "    sudo -u ${SERVICE_USER} ${PREFIX}/.venv/bin/orbitaly doctor -c ${CONFIG_DIR}/config.yaml"
echo
echo "Then:  systemctl start orbitaly   ·   http://$(hostname -I | awk '{print $1}'):8000"
echo "Commissioning steps are in docs/HARDWARE.md — read them before connecting motors."
