#!/bin/bash
#
# Remove background human voice from a video while keeping mechanical/robot
# sounds, using Demucs AI source separation (NOT an EQ/filter — voice and
# robot sound overlap in frequency, so only timbre-based separation works).
#
# It splits the audio into stems, drops the `vocals` stem, and muxes the
# remaining audio back onto the original (untouched, stream-copied) video.
#
# Model note: mdx_extra retained the most robot sound for the precise-assembly
# clips and is the default. htdemucs_ft removes voice most cleanly but strips
# more mechanical sound. Override via the 3rd arg.
#
# Optional add-back: some robot sound bleeds into the vocals stem and is lost.
# Set ADDBACK=0.20 (0..1) to mix that fraction of the discarded vocals stem
# back in — recovers robot sound at the cost of a faint voice ghost.
#
# Requires: ffmpeg on PATH, and a Demucs venv. Point DEMUCS_PY at its python
# (defaults to the local Thesis_Defense venv used to build these assets).
#
# Usage:   scripts/remove_voice.sh INPUT.mp4 [OUTPUT.mp4] [model]
# Example: scripts/remove_voice.sh in.mp4 out_novox.mp4 mdx_extra
set -e

SRC="$1"
OUT="${2:-${SRC%.*}_novox.mp4}"
MODEL="${3:-mdx_extra}"
ADDBACK="${ADDBACK:-0}"
DEMUCS_PY="${DEMUCS_PY:-/Users/kushalkedia/Thesis_Defense/.venv_demucs/bin/python}"

if [ -z "$SRC" ]; then echo "usage: $0 INPUT.mp4 [OUTPUT.mp4] [model]" >&2; exit 1; fi
SEP="$(cd "$(dirname "$0")" && pwd)/demucs_separate.py"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# 1. Extract audio to WAV (44.1 kHz stereo) for Demucs
ffmpeg -y -loglevel error -i "$SRC" -vn -acodec pcm_s16le -ar 44100 -ac 2 "$TMP/audio.wav"

# 2. Separate stems (writes no_vocals.wav + vocals.wav)
"$DEMUCS_PY" "$SEP" "$TMP/audio.wav" "$TMP" "$MODEL"

# 3. Mux voice-less audio back onto the original video (video copied untouched)
if [ "$ADDBACK" != "0" ]; then
  ffmpeg -y -loglevel error -i "$SRC" -i "$TMP/no_vocals.wav" -i "$TMP/vocals.wav" \
    -filter_complex "[2:a]volume=${ADDBACK}[v];[1:a][v]amix=inputs=2:normalize=0[a]" \
    -map 0:v:0 -map "[a]" -c:v copy -c:a aac -b:a 192k -shortest "$OUT"
else
  ffmpeg -y -loglevel error -i "$SRC" -i "$TMP/no_vocals.wav" \
    -map 0:v:0 -map 1:a:0 -c:v copy -c:a aac -b:a 192k -shortest "$OUT"
fi

echo "=== DONE: $OUT (model=$MODEL addback=$ADDBACK) ==="
ls -la "$OUT"
