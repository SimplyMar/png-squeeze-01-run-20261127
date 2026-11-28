# png-squeeze-01-run-20261127

[![make.fun · png-squeeze-01-run-20261127](https://make.fun/badge/png-squeeze-01-run-20261127.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![rank](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/rank.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![reward](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/reward.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)

A deterministic lossless PNG squeezer for desktop screenshots, written for the make.fun bounty
[png-squeeze-01-run-20261127](https://make.fun/bounty/png-squeeze-01-run-20261127). Node 22, standard library only,
no external dependencies or prebuilt binaries.

## How it works

1. **Colour form** (`squeeze.mjs`, `png.mjs`): palette (colour type 3, frequency-ordered with pure filter-invariant
   exact-repeat `f=0/2` candidates on flat UI screenshots so Post-Zopfli Palette RLE and monotonic block re-splitting
   always fire, plus 2-opt co-occurrence TSP when spatial filters beat filter 0) when the image has at most 256 colours
   and is not grey; on opaque greyscale images with spare palette slots, also evaluates an auto-detected Smooth-CC
   protected, orientation-aligned Affine Anti-Aliased Multi-Hub Shadow-PLTE (colour type 3) with flat-component boundary
   classification, chain-balanced geometric-mean ramp discovery, dead-canonical PLTE trimming, and `f0`-Row Revert back
   to canonical indices on unfiltered bands; else grey (0), grey + alpha (4), RGB (2) or RGBA (6), with a tRNS key
   replacing alpha when a single fully transparent value allows it. Bit depth stays 8 and only IHDR, PLTE, tRNS, IDAT
   and IEND are written.
2. **Filters** (`squeeze.mjs`, `png.mjs`): top-2 dictionary-carrying Strip Viterbi DP over 1-, 4-, 8-, and 16-row bands
   (`memLevel=5` Linux-safe workspace with dynamic dictionary sizing) with untrapped dual-strategy
   (`min(Z_DEFAULT, Z_FILTERED)`) evaluation and dual-plan Viterbi recombination (`recombine2`), fast 2-stage 1-row
   transition snapping, SmartUp exact-repeat row overlay, per-row brute force on photo grey, fast 1-bpp WASM DEFLATE
   pre-screening on non-flat hybrids, and a Centered L5 Transition + Row Post-Filter before final encoding.
3. **DEFLATE** (`deflate.mjs`, `wasm-core.mjs`): self-assembled inline WebAssembly SIMD128 Dual-Hash (BT3 + inline HC3
   + HC4) All-30 dSym match finder with explicit 2D-stride (`bpp`, `rowLen`, `rowLen±bpp`, `k*rowLen`) match injection
   and dominance compaction, exact Pareto-frontier O(1) RMQ Exact-258 shortest-path DP with optimal `skipDist` (1..4)
   deep-run jumping, iterative distance-basin re-optimization, automatic photo-hybrid `litheavy` basin selection plus
   multi-basin (`fixed` + `pixdist` + `litheavy`) iterated cost models with integer Huffman-length polish,
   single-pass agglomerative + Exact O(M^2) DP + incremental-histogram boundary sliding block splitting, exact
   Package-Merge length-limited Huffman codes, Kraft-preserving 0-hole/glitch length polish, and shortest-path DP header
   RLE with independent LL/Dist smoothing. Every stream is inflated back and verified byte-for-byte against the filtered
   buffer before use.

All computation depends strictly on input bytes (no clock, no randomness), guaranteeing identical bytes across runs,
with a deterministic symbol-aware work budget to stay safely inside the 60 s wall limit on heavy sets.

`main.ts` launches `squeeze.mjs` in a plain `.mjs` child process so Node 22's TypeScript stripper does not slow the
hot loop.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

## License

MIT
