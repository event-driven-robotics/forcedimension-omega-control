SRC_FILE="$REPO_ROOT/udev/$RULE_FILE"
DST_FILE="/etc/udev/rules.d/$RULE_FILE"

echo "Installing Force Dimension udev rules..."

if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo:"
  echo "  sudo $0"
  exit 1
fi

if [[ ! -f "$SRC_FILE" ]]; then
  echo "ERROR: udev rule not found at:"
  echo "  $SRC_FILE"
  exit 1
fi

if [[ -f "$DST_FILE" ]]; then
  echo "Rule already installed at $DST_FILE, overwriting."
fi

cp "$SRC_FILE" "$DST_FILE"
chmod 644 "$DST_FILE"

udevadm control --reload-rules
udevadm trigger
