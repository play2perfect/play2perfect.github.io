#!/bin/bash
#
# Build static/videos/highlight_reel.mp4 — the precise-assembly highlight reel.
#
# Each of the 3 segments is a per-task Blender render (left, white-padded panel)
# hstacked with the corresponding real-robot clip (right). The three segments are
# concatenated and a "1x speed" badge is burned into the top-right corner.
#
# Output is exactly 1280x720 (16:9): left panel padded to 484px + right clip 796px.
# The right clips are bottom-cropped (ih*100/1080) to trim a status bar, then
# scaled to a FIXED 796px wide x 720 tall. Forcing the width (rather than keeping
# aspect) guarantees every segment is identically sized regardless of whether the
# source is 720x720 or 1080x1080 (those round to 796 vs 794 otherwise), so the
# concat is clean and the total is pinned to exactly 1280. The ~0.25% horizontal
# nudge on the 1080x1080 clips is imperceptible and uniform across segments.
#
# Source clips + render PNGs live in static/videos/highlight_videos/, which is
# gitignored (local-only build sources). Requires ffmpeg on PATH.
#
# Usage: scripts/build_highlight_reel.sh
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
DIR="$REPO/static/videos/highlight_videos"
OUT="$REPO/static/videos/highlight_reel.mp4"
FONT="/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
cd "$DIR"

# make_seg idx video image [silent]
# Pass "silent" if the clip has no audio track, to attach a silent stereo track
# so every segment shares identical audio params for a clean concat.
make_seg () {
  local idx="$1" vid="$2" img="$3" silent="$4"
  local audio_in audio_map
  if [ "$silent" = "silent" ]; then
    audio_in=(-f lavfi -t 60 -i anullsrc=channel_layout=stereo:sample_rate=48000)
    audio_map="2:a"
  else
    audio_in=()
    audio_map="1:a"
  fi
  ffmpeg -y -loglevel error \
    -loop 1 -framerate 30 -i "$img" \
    -i "$vid" \
    "${audio_in[@]}" \
    -filter_complex "\
[0:v]scale=440:720:force_original_aspect_ratio=decrease,pad=440:720:(ow-iw)/2:(oh-ih)/2:color=white,pad=484:720:0:0:color=white,setsar=1[L];\
[1:v]crop=in_w:ih-ih*100/1080:0:0,scale=796:720,setsar=1,fps=30[R];\
[L][R]hstack=inputs=2:shortest=1[v]" \
    -map "[v]" -map "$audio_map" \
    -c:v libx264 -profile:v high -pix_fmt yuv420p -crf 23 -preset slow \
    -c:a aac -ar 48000 -ac 2 -b:a 128k \
    -shortest "$TMP/seg_${idx}.mp4"
  echo "built seg_${idx}.mp4"
}

# Order: screwing -> multi-part assembly -> tight insertion
make_seg 1 screwing_final.mp4 Screwing_Blender_Renders.png
make_seg 2 multi_part_assembl.mp4 MultiPartAssemblyFull_Blender_Renders.png
make_seg 3 tight_insertion_05mm_with_audio.mp4 Tight_Insertion_Blender_Renders.png

cat > "$TMP/concat_list.txt" <<EOF
file '$TMP/seg_1.mp4'
file '$TMP/seg_2.mp4'
file '$TMP/seg_3.mp4'
EOF

ffmpeg -y -loglevel error -f concat -safe 0 -i "$TMP/concat_list.txt" \
  -vf "drawtext=fontfile='$FONT':text='1× speed':fontcolor=black:fontsize=44:box=1:boxcolor=white:boxborderw=16:x=w-text_w-16:y=16" \
  -c:v libx264 -profile:v high -pix_fmt yuv420p -crf 23 -preset slow \
  -c:a aac -ar 48000 -ac 2 -b:a 128k -movflags +faststart \
  "$OUT"

echo "=== DONE ==="
ls -la "$OUT"
ffprobe -v error -show_entries format=duration -show_entries stream=width,height,codec_type -of default=noprint_wrappers=1 "$OUT"
