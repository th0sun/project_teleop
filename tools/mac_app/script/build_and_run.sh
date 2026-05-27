#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-run}"
APP_NAME="ProjectTeleopMac"
PROJECT="ProjectTeleopMac.xcodeproj"
SCHEME="ProjectTeleopMac"
CONFIGURATION="${CONFIGURATION:-Debug}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DERIVED_DATA="$ROOT_DIR/build/XcodeDerivedData"
APP_BUNDLE="$DERIVED_DATA/Build/Products/${CONFIGURATION}/${APP_NAME}.app"

cd "$ROOT_DIR"

stop_running_app() {
  pkill -x "$APP_NAME" >/dev/null 2>&1 || true
}

build_app() {
  xcodebuild \
    -project "$PROJECT" \
    -scheme "$SCHEME" \
    -configuration "$CONFIGURATION" \
    -derivedDataPath "$DERIVED_DATA" \
    CODE_SIGNING_ALLOWED=NO \
    build
}

open_app() {
  /usr/bin/open -n "$APP_BUNDLE"
}

case "$MODE" in
  run)
    stop_running_app
    build_app
    open_app
    ;;
  --verify|verify)
    stop_running_app
    build_app
    open_app
    sleep 2
    pgrep -x "$APP_NAME" >/dev/null
    echo "${APP_NAME} launched"
    ;;
  --logs|logs)
    stop_running_app
    build_app
    open_app
    /usr/bin/log stream --info --style compact --predicate "process == \"${APP_NAME}\""
    ;;
  --debug|debug)
    stop_running_app
    build_app
    lldb -- "$APP_BUNDLE/Contents/MacOS/$APP_NAME"
    ;;
  *)
    echo "usage: $0 [run|--verify|--logs|--debug]" >&2
    exit 2
    ;;
esac
