#!/bin/sh
# Builds a double-clickable macOS .app around the PC executable:
#   bundle_app.sh <exe> <out.app> <app name> <version> [icon image]
# Non-system dylibs (libfpv, Homebrew/MacPorts SDL2, ...) are copied into
# Contents/Frameworks so the bundle runs without them installed. The icon can
# be any image sips reads (.ico, .png, ...). The bundle is ad-hoc signed, so it
# runs on the Mac that built it; elsewhere Gatekeeper wants right-click > Open.
set -e

EXE=$1
APP=$2
NAME=$3
VERSION=$4
ICON=$5
HERE=$(cd "$(dirname "$0")" && pwd)
EXE_NAME=$(basename "$EXE")
EXE_DIR=$(dirname "$EXE")

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Frameworks" "$APP/Contents/Resources"
cp "$EXE" "$APP/Contents/MacOS/$EXE_NAME"
codesign --remove-signature "$APP/Contents/MacOS/$EXE_NAME"

# minimum macOS = what the executable was built for
MIN_MACOS=$(otool -l "$EXE" | awk '/LC_BUILD_VERSION/ { f = 1 } f && $1 == "minos" { print $2; exit }')
sed -e "s|@APP_NAME@|$NAME|g" -e "s|@EXECUTABLE@|$EXE_NAME|g" \
    -e "s|@VERSION@|$VERSION|g" -e "s|@MIN_MACOS@|${MIN_MACOS:-11.0}|g" \
    "$HERE/Info.plist.in" > "$APP/Contents/Info.plist"
printf 'APPL????' > "$APP/Contents/PkgInfo"

# copy every non-system dylib (recursively) into Frameworks and point the
# references at @rpath; @rpath/ ones are looked up next to the executable
FW="$APP/Contents/Frameworks"
install_name_tool -add_rpath @executable_path/../Frameworks "$APP/Contents/MacOS/$EXE_NAME"
todo="$APP/Contents/MacOS/$EXE_NAME"
while [ -n "$todo" ]; do
    bin=${todo%%:*}
    case "$todo" in *:*) todo=${todo#*:} ;; *) todo= ;; esac
    for dep in $(otool -L "$bin" | tail -n +2 | awk '{ print $1 }'); do
        case "$dep" in
            /usr/lib/*|/System/*) continue ;;
            @rpath/*) src="$EXE_DIR/${dep#@rpath/}" ;;
            @*) continue ;;
            *) src=$dep ;;
        esac
        lib=$(basename "$dep")
        [ "$(basename "$bin")" = "$lib" ] && continue # the library's own id
        if [ ! -f "$FW/$lib" ]; then
            [ -f "$src" ] || { echo "bundle_app: can't find $dep" >&2; exit 1; }
            cp "$src" "$FW/$lib"
            chmod u+w "$FW/$lib"
            codesign --remove-signature "$FW/$lib"
            install_name_tool -id "@rpath/$lib" "$FW/$lib"
            todo="${todo:+$todo:}$FW/$lib"
        fi
        [ "$dep" = "@rpath/$lib" ] || install_name_tool -change "$dep" "@rpath/$lib" "$bin"
    done
done
# Frameworks dylibs find each other through the executable's rpath

# icon: scale the artwork to ~80% of the canvas, then build the .icns
if [ -n "$ICON" ] && [ -f "$ICON" ]; then
    TMP=$(mktemp -d)
    SET="$TMP/AppIcon.iconset"
    mkdir "$SET"
    sips -s format png "$ICON" --out "$TMP/src.png" >/dev/null
    sips -z 824 824 "$TMP/src.png" --out "$TMP/scaled.png" >/dev/null
    sips -p 1024 1024 "$TMP/scaled.png" --out "$TMP/1024.png" >/dev/null
    for s in 16 32 128 256 512; do
        sips -z $s $s "$TMP/1024.png" --out "$SET/icon_${s}x${s}.png" >/dev/null
        sips -z $((s * 2)) $((s * 2)) "$TMP/1024.png" --out "$SET/icon_${s}x${s}@2x.png" >/dev/null
    done
    iconutil -c icns "$SET" -o "$APP/Contents/Resources/AppIcon.icns"
    rm -rf "$TMP"
else
    echo "bundle_app: no icon (set APP_ICON=<image>)"
fi

# sign last: any change to the bundle afterwards breaks the signature
for lib in "$FW"/*; do
    [ -f "$lib" ] && codesign --sign - "$lib"
done
codesign --sign - "$APP"
echo "bundle_app: built $APP"
