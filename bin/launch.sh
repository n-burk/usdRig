#!/bin/bash
# Launch the animated arm example, a supplied stage, or --blank.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
case "${1:-}" in
  -h|--help)
    echo "Usage: bin/launch.sh [stage.usda | --blank] [usdview flags...]"
    exit 0
    ;;
  --blank) shift ;;
  "") set -- "$SCRIPT_DIR/../examples/ArmShotAnim.usda" ;;
esac
exec "$SCRIPT_DIR/usdview.sh" "$@"
