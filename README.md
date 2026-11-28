# png-squeeze-01-run-20261127

[![make.fun · png-squeeze-01-run-20261127](https://make.fun/badge/png-squeeze-01-run-20261127.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![rank](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/rank.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![reward](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/reward.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)

A deterministic lossless PNG squeezer for desktop screenshots, written for the make.fun bounty
[png-squeeze-01-run-20261127](https://make.fun/bounty/png-squeeze-01-run-20261127). Node 22, standard library only,
no external dependencies or prebuilt binaries.

## How it works

1. **Colour form** (`squeeze.mjs`, `png.mjs`): palette (colour type 3, frequency-ordered plus 2-opt co-occurrence TSP
   when filtered rows beat filter 0, followed by Post-Zopfli Palette RLE permutation of filter-invariant indices and
   monotonic post-permutation block re-splitting) when the image has at most 256 colours and is not grey; on opaque
   greyscale images with spare palette slots, also evaluates an auto-detected photo-safe multi-ramp duplicate-palette
   colour type 3 form that flood-fills flat UI text windows into contiguous 5-level anti-aliased ramps without touching
   smooth gradients; else grey (0), grey + alpha (4), RGB (2) or RGBA (6), with a tRNS key replacing alpha when a single
   fully transparent value allows it. Bit depth stays 8 and only IHDR, PLTE, tRNS, IDAT and IEND are written.
2. **Filters** (`squeeze.mjs`, `png.mjs`): top-2 dictionary-carrying Strip Viterbi DP over 2-, 4-, 8-, and 16-row bands
   with untrapped dual-strategy (`min(Z_DEFAULT, Z_FILTERED)`) photo evaluation and dual-plan Viterbi recombination
   (`recombine2` + natural dup-run lookahead `recombine2Cuts`), fast 2-stage 1-row transition snapping, SmartUp
   exact-repeat row overlay, per-row brute force on grey, fast 1-bpp WASM DEFLATE pre-screening, and a Centered L6
   Transition + Row Post-Filter that evaluates downstream 32 KB dictionary impact before final encoding.
3. **DEFLATE** (`deflate.mjs`, `wasm-core.mjs`): self-assembled inline WebAssembly SIMD128 Dual-Hash (BT3 + inline HC3
   + HC4) All-30 dSym match finder with explicit 2D-stride (`bpp`, `rowLen`, `rowLen±bpp`, `k*rowLen`) match injection
   and dominance compaction, exact Pareto-frontier O(1) RMQ Exact-258 shortest-path DP with optimal `skipDist` (1..4)
   deep-run jumping, iterative distance-basin re-optimization, multi-basin (`fixed` + `pixdist` + `litheavy`) iterated
   cost models with integer Huffman-length polish, filter-transition-aware hybrid Bottom-Up + Exact O(M^2) DP +
   2-sweep incremental-histogram boundary sliding block splitting, exact Package-Merge length-limited Huffman codes,
   Kraft-preserving 0-hole/glitch length polish, and shortest-path DP header RLE with independent LL/Dist smoothing.
   Every stream is inflated back and verified byte-for-byte against the filtered buffer before use.

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
