# png-squeeze-01-run-20261127

[![make.fun · png-squeeze-01-run-20261127](https://make.fun/badge/png-squeeze-01-run-20261127.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![rank](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/rank.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![reward](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/reward.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)

A deterministic lossless PNG squeezer for desktop screenshots, written for the make.fun bounty
[png-squeeze-01-run-20261127](https://make.fun/bounty/png-squeeze-01-run-20261127). Node 22, standard library only,
no external dependencies or prebuilt binaries.

## How it works

1. **Colour form** (`squeeze.mjs`, `png.mjs`): palette (colour type 3, ordered by frequency) when the image has at most
   256 colours and is not grey; else grey (0), grey + alpha (4), RGB (2) or RGBA (6), with a tRNS key replacing alpha
   when a single fully transparent value allows it. Bit depth stays 8 and only IHDR, PLTE, tRNS, IDAT and IEND are written.
2. **Filters** (`squeeze.mjs`, `png.mjs`): 5-state dictionary-carrying Strip Viterbi DP over 8-row and 16-row bands
   (switching to `Z_FILTERED` strategy on literal-heavy photo images), fast 2-stage 1-row transition snapping around
   filter changes, SmartUp exact-repeat row overlay (preserving all-zero/flat runs), and per-row brute force on grey.
   Candidates are screened with zlib levels 6 and 8 (plus `Z_FILTERED` for photo regimes); on 1-bpp images, a close
   distinct runner-up is also DEFLATE-encoded and the smaller PNG kept.
3. **DEFLATE** (`deflate.mjs`, `wasm-core.mjs`): self-assembled inline WebAssembly BT3 match finder and O(1) RMQ
   Exact-258 shortest-path parser with exact multi-block deep-run skipping, dual-basin (fixed + empirical literal-heavy)
   iterated cost models with integer Huffman-length polish, incremental-histogram bottom-up agglomerative block merge
   and boundary sliding, exact Package-Merge length-limited Huffman codes, and shortest-path DP header RLE with count
   smoothing. Every stream is inflated back and verified byte-for-byte against the filtered buffer before use.

All computation depends strictly on input bytes (no clock, no randomness), guaranteeing identical bytes across runs,
with a deterministic byte-counted budget to stay well inside the 60 s wall limit on heavy sets.

`main.ts` launches `squeeze.mjs` in a plain `.mjs` child process so Node 22's TypeScript stripper does not slow the
hot loop.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

## License

MIT
