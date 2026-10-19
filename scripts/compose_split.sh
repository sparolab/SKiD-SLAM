#!/bin/bash
# Composes two run_bag.sh videos side by side, split by a diagonal, and exports an mp4 + gif.
#
#   scripts/compose_split.sh left.mp4 right.mp4 out_dir [name] [style]
#
# style: line (thin crisp line, default) | gap (dark gap) | blend (soft cross-fade, no line)
#
# Each video's center is placed in the middle of its half. The shorter clip holds its last frame.
set -e
[[ $# -lt 3 ]] && { sed -n '2,5p' "$0"; exit 1; }
LEFT=$1; RIGHT=$2; OUT=$3; NAME=${4:-skid_slam}; STYLE=${5:-line}
mkdir -p "$OUT"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

W=1920; H=1080
TOP=0.58; BOT=0.42      # x position of the diagonal at the top / bottom (fraction of width)
SHIFT=384               # each clip is shifted this many pixels towards its side
BG=0x111318             # RViz background

dur() { ffprobe -v error -show_entries format=duration -of csv=p=0 "$1"; }
DL=$(dur "$LEFT"); DR=$(dur "$RIGHT")
DUR=$(printf "%.3f" "$(echo "if ($DL > $DR) $DL else $DR" | bc -l)")
PADL=$(printf "%.3f" "$(echo "$DUR - $DL" | bc -l)"); PADR=$(printf "%.3f" "$(echo "$DUR - $DR" | bc -l)")

# static masks: left-half alpha and the separator (signed pixel distance SD to the diagonal)
LINE_X="($W*$TOP-$W*($TOP-$BOT)*Y/$H)"
SD="((X-$LINE_X)*$H/sqrt($H*$H+($W*($TOP-$BOT))^2))"
case $STYLE in
  blend) MASK="255*clip(0.5-$SD/120,0,1)"; SEP_A="0"; SEP_C=white ;;
  gap)   MASK="if(lt($SD,0),255,0)";        SEP_A="255*clip(6-abs($SD),0,1)"; SEP_C=0x0B0C10 ;;
  *)     MASK="if(lt($SD,0),255,0)";        SEP_A="200*clip(1.5-abs($SD),0,1)"; SEP_C=white ;;
esac
ffmpeg -loglevel error -y -f lavfi -i "color=black:s=${W}x${H},format=gray" -frames:v 1 \
  -vf "geq=lum='$MASK'" "$TMP/mask.png"
ffmpeg -loglevel error -y -f lavfi -i "color=$SEP_C:s=${W}x${H},format=rgba" -frames:v 1 \
  -vf "geq=r='r(X,Y)':g='g(X,Y)':b='b(X,Y)':a='$SEP_A'" "$TMP/line.png"

ffmpeg -loglevel error -y -i "$LEFT" -i "$RIGHT" -loop 1 -i "$TMP/mask.png" -loop 1 -i "$TMP/line.png" -filter_complex "
  [0:v]tpad=stop_mode=clone:stop_duration=$PADL,crop=$((W-SHIFT)):$H:$SHIFT:0,pad=$W:$H:0:0:color=$BG[l];
  [1:v]tpad=stop_mode=clone:stop_duration=$PADR,crop=$((W-SHIFT)):$H:0:0,pad=$W:$H:$SHIFT:0:color=$BG[r];
  [2:v]format=gray[m];
  [l]format=rgba[lr];[lr][m]alphamerge[la];
  [r][la]overlay=shortest=1[s];
  [s][3:v]overlay=shortest=1,format=yuv420p[v]" \
  -map "[v]" -t "$DUR" -r 30 -c:v libx264 -preset slow -crf 18 -movflags +faststart "$OUT/$NAME.mp4"

# gif: 960 px, 15 fps, per-video palette
ffmpeg -loglevel error -y -i "$OUT/$NAME.mp4" -vf "fps=15,scale=960:-1:flags=lanczos,palettegen=stats_mode=diff" "$TMP/palette.png"
ffmpeg -loglevel error -y -i "$OUT/$NAME.mp4" -i "$TMP/palette.png" \
  -lavfi "fps=15,scale=960:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=sierra2_4a:diff_mode=rectangle" "$OUT/$NAME.gif"

echo "$OUT/$NAME.mp4"
echo "$OUT/$NAME.gif"
