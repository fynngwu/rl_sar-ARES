#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="ares_rl.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
DEFAULT_POLICY="dream_waq/dream_waq"
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME="$(eval echo "~${REAL_USER}")"
BASHRC="${REAL_HOME}/.bashrc"
ALIAS_MARKER_START="# >>> ARES RL Service Aliases >>>"
ALIAS_MARKER_END="# <<< ARES RL Service Aliases <<<"

ALIAS_BLOCK_FILE="$(mktemp)"
chmod 644 "${ALIAS_BLOCK_FILE}"
trap 'rm -f "${ALIAS_BLOCK_FILE}"' EXIT

cat > "${ALIAS_BLOCK_FILE}" <<ALIASES_EOF
${ALIAS_MARKER_START}
alias ares-start='sudo systemctl start ${SERVICE_NAME}'
alias ares-stop='sudo systemctl stop ${SERVICE_NAME}'
alias ares-restart='sudo systemctl restart ${SERVICE_NAME}'
alias ares-status='sudo systemctl status ${SERVICE_NAME}'
alias ares-logs='journalctl -u ${SERVICE_NAME} -f -n 50'
alias ares-enable='sudo systemctl enable ${SERVICE_NAME}'
alias ares-disable='sudo systemctl disable ${SERVICE_NAME}'
${ALIAS_MARKER_END}
ALIASES_EOF

install_aliases() {
    sudo -u "${REAL_USER}" bash -c "
        if grep -qF '${ALIAS_MARKER_START}' '${BASHRC}' 2>/dev/null; then
            tmp=\$(mktemp)
            awk -v start='${ALIAS_MARKER_START}' -v end='${ALIAS_MARKER_END}' '
                \$0 == start { skip=1; next }
                \$0 == end   { skip=0; next }
                !skip { print }
            ' '${BASHRC}' > \"\${tmp}\"
            mv \"\${tmp}\" '${BASHRC}'
        fi
        cat '${ALIAS_BLOCK_FILE}' >> '${BASHRC}'
    "
    echo "[install_service] Aliases added to ${BASHRC}"
    echo "    Run 'source ${BASHRC}' or open a new terminal to use them."
}

remove_aliases() {
    sudo -u "${REAL_USER}" bash -c "
        if grep -qF '${ALIAS_MARKER_START}' '${BASHRC}' 2>/dev/null; then
            tmp=\$(mktemp)
            awk -v start='${ALIAS_MARKER_START}' -v end='${ALIAS_MARKER_END}' '
                \$0 == start { skip=1; next }
                \$0 == end   { skip=0; next }
                !skip { print }
            ' '${BASHRC}' > \"\${tmp}\"
            mv \"\${tmp}\" '${BASHRC}'
        fi
    "
}

usage() {
    echo "Usage:"
    echo "  sudo ./install_service.sh                     # show current config"
    echo "  sudo ./install_service.sh [policy]            # install / update"
    echo "  sudo ./install_service.sh --remove            # remove service + aliases"
    echo ""
    echo "Example:"
    echo "  sudo ./install_service.sh ${DEFAULT_POLICY}"
    exit 0
}

case "${1:-}" in
    --help|-h) usage ;;
    --remove)
        if [[ "${EUID}" -ne 0 ]]; then
            echo "This script must be run as root (sudo)." >&2
            exit 1
        fi
        systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
        systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
        rm -f "${SERVICE_PATH}"
        systemctl daemon-reload
        remove_aliases
        echo "[install_service] Removed ${SERVICE_NAME} and aliases"
        exit 0
        ;;
esac

if [[ -z "${1:-}" ]]; then
    if [[ -f "${SERVICE_PATH}" ]]; then
        echo "[install_service] Current config (${SERVICE_NAME}):"
        grep -E "ExecStart=|ExecStartPre=|Description=" "${SERVICE_PATH}" | sed 's/^[[:space:]]*//'
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
ExecStartPre=${REAL_HOME}/.local/bin/start
ExecStart=${PROJECT_ROOT}/run.sh ${POLICY}
User=${REAL_USER}
Environment=HOME=${REAL_HOME}
Restart=no
WorkingDirectory=${PROJECT_ROOT}

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

install_aliases "${POLICY}"

echo ""
echo "[install_service] Installed ${SERVICE_NAME}"
echo "    ExecStart: ${PROJECT_ROOT}/run.sh ${POLICY}"
echo "    ExecStartPre: ${HOME}/.local/bin/start"
echo ""
echo "Start now:  sudo systemctl start ${SERVICE_NAME}"
echo "Stop:       sudo systemctl stop ${SERVICE_NAME}"
echo "Status:     sudo systemctl status ${SERVICE_NAME}"
echo "Logs:       journalctl -u ${SERVICE_NAME} -f"
echo ""
echo "Aliases installed: ares-start, ares-stop, ares-restart, ares-status, ares-logs"
echo ""
echo "To change later, re-run with different args:"
echo "  sudo $0 ${DEFAULT_POLICY}"
echo "  sudo $0 --remove"
