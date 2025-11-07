#!/usr/bin/env bash
set -euo pipefail
sudo systemctl start olad
sleep 1
if ! sudo grep -q 'universe {\s*id: 0' /var/lib/ola/conf/ola-universe.conf 2>/dev/null; then
  DEV_ID=$(ola_dev_info | awk '/DMXking/{print id}{id=$2}')
  sudo ola_patch -d "${DEV_ID:-10}" -p 0 -u 0
fi
echo "---- ola-universe.conf ----"
sudo sed -n '1,120p' /var/lib/ola/conf/ola-universe.conf || true
