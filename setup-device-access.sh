#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ "$EUID" -ne 0 || -z "${SUDO_USER:-}" ]]; then
  echo 'Run with sudo from your normal login account.' >&2
  exit 1
fi
rules_tmp=$(mktemp)
trap 'rm -f -- "$rules_tmp"' EXIT
python3 device_rules.py --user "$SUDO_USER" > "$rules_tmp"
install -m 0644 "$rules_tmp" /etc/udev/rules.d/99-panthera-local.rules
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=tty
udevadm settle
echo 'Panthera USB access configured.'
