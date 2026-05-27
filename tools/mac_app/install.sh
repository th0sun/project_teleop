#!/usr/bin/env bash
# Build ProjectTeleopMac, embed icon, install into /Applications.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HERE}/build"
DERIVED="${BUILD_DIR}/XcodeDerivedData"
ICON_SRC="${HERE}/resources/ProjectTeleopMac.icns"
PRODUCT="ProjectTeleopMac.app"
INSTALL_DIR="/Applications"
INSTALL_PATH="${INSTALL_DIR}/${PRODUCT}"

cd "${HERE}"

if [[ ! -f "${ICON_SRC}" ]]; then
  echo "[install] generating icon"
  python3 "${HERE}/resources/make_icon.py"
fi

echo "[install] xcodebuild Release"
xcodebuild \
  -project ProjectTeleopMac.xcodeproj \
  -scheme ProjectTeleopMac \
  -configuration Release \
  -destination 'platform=macOS' \
  -derivedDataPath "${DERIVED}" \
  build >/dev/null

BUILT_APP="${DERIVED}/Build/Products/Release/${PRODUCT}"
if [[ ! -d "${BUILT_APP}" ]]; then
  echo "[install] missing built app at ${BUILT_APP}" >&2
  exit 1
fi

STAGE="${BUILD_DIR}/${PRODUCT}"
rm -rf "${STAGE}"
ditto "${BUILT_APP}" "${STAGE}"

mkdir -p "${STAGE}/Contents/Resources"
cp "${ICON_SRC}" "${STAGE}/Contents/Resources/ProjectTeleopMac.icns"

# Ensure Info.plist references the icon (idempotent)
PLIST="${STAGE}/Contents/Info.plist"
/usr/libexec/PlistBuddy -c "Set :CFBundleIconFile ProjectTeleopMac" "${PLIST}" 2>/dev/null \
  || /usr/libexec/PlistBuddy -c "Add :CFBundleIconFile string ProjectTeleopMac" "${PLIST}"
/usr/libexec/PlistBuddy -c "Set :CFBundleIconName ProjectTeleopMac" "${PLIST}" 2>/dev/null \
  || /usr/libexec/PlistBuddy -c "Add :CFBundleIconName string ProjectTeleopMac" "${PLIST}"

# Re-sign after modifying bundle
codesign --force --deep --sign - "${STAGE}" >/dev/null

echo "[install] copying to ${INSTALL_PATH}"
if [[ -d "${INSTALL_PATH}" ]]; then
  if pgrep -f "${INSTALL_PATH}/Contents/MacOS/ProjectTeleopMac" >/dev/null; then
    echo "[install] stopping running instance"
    pkill -f "${INSTALL_PATH}/Contents/MacOS/ProjectTeleopMac" || true
    sleep 1
  fi
  rm -rf "${INSTALL_PATH}"
fi
ditto "${STAGE}" "${INSTALL_PATH}"
rm -rf "${STAGE}"

# Refresh Finder/LaunchServices icon cache for this bundle
/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/LaunchServices.framework/Versions/A/Support/lsregister \
  -f -R -trusted "${INSTALL_PATH}" >/dev/null 2>&1 || true
touch "${INSTALL_PATH}"

echo "[install] done -> ${INSTALL_PATH}"
