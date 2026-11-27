# png-squeeze-01-run-20261127

[![make.fun · png-squeeze-01-run-20261127](https://make.fun/badge/png-squeeze-01-run-20261127.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![rank](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/rank.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![reward](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/reward.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)

A lossless PNG squeezer for screenshots, written for the make.fun bounty
[png-squeeze-01-run-20261127](https://make.fun/bounty/png-squeeze-01-run-20261127). Node 22, standard library only,
no dependencies.

## How it works

1. **Colour form** (`squeeze.mjs`): a palette (colour type 3) when the image has at most 256 colours, else grey (0),
   grey + alpha (4), RGB (2) or RGBA (6), with a tRNS key instead of an alpha channel when a single fully transparent
   value allows it. Bit depth stays 8 and only IHDR, PLTE, tRNS, IDAT and IEND are written.
2. **Filters** (`squeeze.mjs`, `png.mjs`): filter 0, and a filter per band of 16 rows chosen by zlib with the
   32 KiB before the band as dictionary (a filter kept over many rows lets LZ match rows against the rows above them);
   grey images also try a per-row brute force, palettes filter 0 and the minimum sum. Candidates are screened with
   zlib level 6; when zlib prefers the filtered palette, the unfiltered one is written too and the smaller kept.
3. **DEFLATE** (`deflate.mjs`, written for this entry): binary-tree match finder over the 32 KiB window (a node already
   known to match from the position before skips its compare), an optimal parse under an iterated entropy cost model
   started from the data's own byte statistics, where a match that merely continues one from the position before is
   relaxed only at short lengths and the ends of length codes, block splitting with each block re-parsed under its own
   model (grey images get more rounds), length-limited Huffman codes and the smallest RLE of the code-length header. The stream is inflated back and
   compared before it is used; zlib's stream is the fallback.

The work done depends only on the input (no clock, no randomness), so two runs write identical bytes.

Local practice set: **797,389 / 1,000,000**. The work is sized well inside the grader's 60 s wall on one core: a
deterministic budget (counted in bytes, never the clock) lightens the costliest images of an unusually heavy set.

`main.ts` only starts `squeeze.mjs` in a child Node process: on Node 22 a `.ts` entry loads the TypeScript stripper,
which slows all later JavaScript in that process by about a half.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

## License

MIT
