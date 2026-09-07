"""Limit the vendored ORT 1.24.3 memory reservation; WASM binary is unchanged."""
from pathlib import Path
import hashlib

p=Path(__file__).resolve().parents[1]/'interactive/vendor/ort-wasm-simd-threaded.mjs'
s=p.read_text()
old='new WebAssembly.Memory({initial:256,maximum:65536,shared:!0})'
new='new WebAssembly.Memory({initial:256,maximum:16384,shared:!0})'
assert s.count(old)==1, 'Expected unmodified ORT 1.24.3 memory constructor'
print('Original SHA256:',hashlib.sha256(s.encode()).hexdigest())
s='// Local modification: cap memory growth at 1 GiB instead of 4 GiB; initial allocation remains 16 MiB.\n'+s.replace(old,new)
p.write_text(s)
print('Modified SHA256:',hashlib.sha256(s.encode()).hexdigest())
