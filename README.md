# Play2Perfect Website

Static project page for `https://play2perfect.github.io`.

<!-- Maintained by the Play2Perfect team. -->

## Local Preview

Open `index.html` directly in a browser. No build step is required.

## Videos

The page uses placeholder paths under `static/videos/`. Add the final videos using the filenames listed in `static/videos/README.md`.

## Google Analytics

Google Analytics is active in `index.html` with measurement ID `G-4GHT8PVEGV`.

After deployment:

1. Open `https://play2perfect.github.io`.
2. In Google Analytics, open `Reports` -> `Realtime`.
3. Confirm the visit appears for the Play2Perfect property.

## Interactive assembly demo

The demo appears before Key Idea and loads assets only when opened. MuJoCo
3.8.1 WASM runs CPU physics, ONNX Runtime Web1.24.3 runs exported policies,
and Three.js renders the scene. Select screwing, tight insertion, or Fabrica.
Faster rendering reduces robot visual detail without changing simulation.

Serve this checkout with `python3 -m http.server 8767` and open `/interactive/`.
All required runtime files and four exported policies are included. The optional
`/diagnostics/browser-check.html` runs fixed-start checks and saves a local report.
No automatic report upload. Ordinary-laptop and macOS Safari coverage is pending.

### Inference memory limit

The vendored ONNX Runtime1.24.3 loader has a documented local modification:
its WebAssembly memory maximum is1GiB instead of4GiB (initial allocation remains
16MiB). The WASM binary and policies are unchanged. Current task inference heaps
measure about98–142MiB. `scripts/limit_ort_memory.py` reapplies this exact edit to
an unmodified loader after regeneration. Worker runtime URLs include a version
query to avoid retaining the old loader in browser caches. Compiled MuJoCo
models release source-file buffers, and page navigation terminates old workers;
restoring a cached page reloads its simulation.
