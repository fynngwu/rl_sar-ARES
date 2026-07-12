#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="ares_rl.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
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
    echo "  sudo ./install_service.sh                     # install / update"
    echo "  sudo ./install_service.sh --remove            # remove service + aliases"
    exit 0
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
fi

if [[ "${1:-}" == "--remove" ]]; then
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
fi

if [[ "${EUID}" -ne 0 ]]; then
    echo "This script must be run as root (sudo)." >&2
    exit 1
fi

cat > "${SERVICE_PATH}" <<SERVICE_EOF
[Unit]
Description=ARES RL Robot Controller
After=network.target network-online.target

[Service]
Type=simple
ExecStartPre=${REAL_HOME}/.local/bin/start
ExecStart=${PROJECT_ROOT}/run.sh
User=${REAL_USER}
Environment=HOME=${REAL_HOME}
Restart=no
WorkingDirectory=${PROJECT_ROOT}

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

install_aliases

echo ""
echo "[install_service] Installed ${SERVICE_NAME}"
echo "    ExecStart: ${PROJECT_ROOT}/run.sh"
echo "    ExecStartPre: ${HOME}/.local/bin/start"
echo ""
echo "Start now:  sudo systemctl start ${SERVICE_NAME}"
echo "Stop:       sudo systemctl stop ${SERVICE_NAME}"
echo "Status:     sudo systemctl status ${SERVICE_NAME}"
echo "Logs:       journalctl -u ${SERVICE_NAME} -f"
echo ""
echo "Aliases installed: ares-start, ares-stop, ares-restart, ares-status, ares-logs"
