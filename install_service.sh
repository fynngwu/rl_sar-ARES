#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="ares_rl.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_POLICY="dream_waq/dream_waq"

case "${1:-}" in
    --help|-h)
        echo "Usage:"
        echo "  sudo ./install_service.sh                     # show current config"
        echo "  sudo ./install_service.sh [policy]            # install / update"
        echo "  sudo ./install_service.sh --remove            # remove service"
        echo ""
        echo "Example:"
        echo "  sudo ./install_service.sh ${DEFAULT_POLICY}"
        exit 0
        ;;
    --remove)
        if [[ "${EUID}" -ne 0 ]]; then
            echo "This script must be run as root (sudo)." >&2
            exit 1
        fi
        systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
        systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
        rm -f "${SERVICE_PATH}"
        systemctl daemon-reload
        echo "[install_service] Removed ${SERVICE_NAME}"
        exit 0
        ;;
esac

if [[ -z "${1:-}" ]]; then
    if [[ -f "${SERVICE_PATH}" ]]; then
        echo "[install_service] Current config (${SERVICE_NAME}):"
        grep -E "ExecStart=|Description=" "${SERVICE_PATH}" | sed 's/^[[:space:]]*//'
        echo ""
        echo "Status: $(systemctl is-enabled "${SERVICE_NAME}" 2>/dev/null || echo 'unknown')"
        echo ""
        echo "To change:  sudo $0 [policy]"
        echo "To remove:  sudo $0 --remove"
    else
        echo "[install_service] ${SERVICE_NAME} is not installed."
        echo "  sudo $0 ${DEFAULT_POLICY}"
    fi
    exit 0
fi

POLICY="${1:-$DEFAULT_POLICY}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "This script must be run as root (sudo)." >&2
    exit 1
fi

cat > "${SERVICE_PATH}" <<SERVICE_EOF
[Unit]
Description=ARES RL Robot Controller (${POLICY})
After=network.target network-online.target

[Service]
Type=simple
ExecStart=${PROJECT_ROOT}/run.sh ${POLICY}
User=ares
Environment=HOME=/home/ares
Restart=no
WorkingDirectory=${PROJECT_ROOT}

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "[install_service] Installed ${SERVICE_NAME}"
echo "    ExecStart: ${PROJECT_ROOT}/run.sh ${POLICY}"
echo ""
echo "Start now:  systemctl start ${SERVICE_NAME}"
echo "Stop:       systemctl stop ${SERVICE_NAME}"
echo "Status:     systemctl status ${SERVICE_NAME}"
echo "Logs:       journalctl -u ${SERVICE_NAME} -f"
echo ""
echo "To change later, re-run with different args:"
echo "  sudo $0 ${DEFAULT_POLICY}"
echo "  sudo $0 --remove"
