#!/bin/sh
set -eu

LOGFILE="/var/log/wifi-bringup.log"
exec >>"$LOGFILE" 2>&1
echo "==== $(date -Iseconds) wifi-bringup start ===="

IFACE=""
i=0
while [ "$i" -lt 30 ]; do
  IFACE="$(ls /sys/class/net 2>/dev/null | grep -E '^wl' | head -n1 || true)"
  [ -n "$IFACE" ] && break
  i=$((i + 1))
  sleep 1
done

[ -n "$IFACE" ] || exit 0
echo "Detected Wi-Fi interface: $IFACE"

# Match wpa_supplicant@<iface>.service expectations.
CONF_SRC="/etc/wpa_supplicant/wpa_supplicant-wlan0.conf"
CONF_IF="/etc/wpa_supplicant/wpa_supplicant-${IFACE}.conf"
if [ -f "$CONF_SRC" ] && [ ! -f "$CONF_IF" ]; then
  cp "$CONF_SRC" "$CONF_IF"
fi

/usr/sbin/rfkill unblock wifi || true
/bin/systemctl enable "wpa_supplicant@${IFACE}.service" >/dev/null 2>&1 || true
/bin/systemctl restart "wpa_supplicant@${IFACE}.service" || true
/bin/networkctl reconfigure "$IFACE" || true
/bin/systemctl restart systemd-networkd || true
echo "==== $(date -Iseconds) wifi-bringup done ===="
