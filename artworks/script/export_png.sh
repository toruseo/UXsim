#!/bin/bash
# Exports every uxsim-*.svg in the project root to png/ at 1x and 2x with headless Chrome.
# Usage (Git Bash): bash script/export_png.sh
# The background is transparent unless the SVG paints one; the Chrome profile is kept in tmp/ so the user's profile is not touched.
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd -W 2>/dev/null || pwd)
C="/c/Program Files/Google/Chrome/Application/chrome.exe"
mkdir -p png tmp
for f in uxsim-*.svg; do
    stem=${f%.svg}
    w=$(head -1 "$f" | grep -o 'width="[0-9]*"' | grep -o '[0-9]*')
    h=$(head -1 "$f" | grep -o 'height="[0-9]*"' | grep -o '[0-9]*')
    for s in 1 2; do
        suf=""; [ $s = 2 ] && suf="@2x"
        "$C" --headless=new --disable-gpu --hide-scrollbars --no-first-run --no-default-browser-check --user-data-dir="$ROOT/tmp/chrome-profile" --default-background-color=00000000 --force-device-scale-factor=$s --window-size=$w,$h --screenshot="$ROOT/png/${stem}${suf}.png" "file:///$ROOT/$f" 2>&1 | grep -o 'bytes written to file.*'
    done
done
