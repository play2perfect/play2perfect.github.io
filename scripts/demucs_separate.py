#!/usr/bin/env python3
"""Demucs source separation -> writes no_vocals.wav (kept) + vocals.wav (discarded).

Drives Demucs through its Python API (the CLI now needs torchcodec to write
stems, which is awkward to install) and writes stems with soundfile.

Usage: demucs_separate.py in.wav out_dir [model_name]
  model_name defaults to mdx_extra (best robot-sound retention for the
  precise-assembly clips; htdemucs_ft gives the cleanest pop-vocal removal).
"""
import sys
import numpy as np
import soundfile as sf
import torch
from demucs.pretrained import get_model
from demucs.apply import apply_model

in_wav, out_dir = sys.argv[1], sys.argv[2]
model_name = sys.argv[3] if len(sys.argv) > 3 else 'mdx_extra'

model = get_model(model_name)
model.cpu().eval()
sr = model.samplerate
ch = model.audio_channels

data, file_sr = sf.read(in_wav, dtype='float32', always_2d=True)
assert file_sr == sr, f"sample rate {file_sr} != model {sr}"
if data.shape[1] == 1:
    data = np.repeat(data, ch, axis=1)
wav = torch.from_numpy(data.T).contiguous()

ref = wav.mean(0)
wav = (wav - ref.mean()) / (ref.std() + 1e-8)

with torch.no_grad():
    sources = apply_model(model, wav[None], device='cpu', split=True,
                          overlap=0.25, progress=True)[0]
sources = sources * ref.std() + ref.mean()

names = model.sources                      # e.g. ['drums','bass','other','vocals']
vidx = names.index('vocals')
no_vocals = sum(sources[i] for i in range(len(names)) if i != vidx)
vocals = sources[vidx]

sf.write(f"{out_dir}/no_vocals.wav", no_vocals.T.numpy(), sr)
sf.write(f"{out_dir}/vocals.wav", vocals.T.numpy(), sr)
print("WROTE no_vocals + vocals to", out_dir, "model:", model_name, "stems:", names)
