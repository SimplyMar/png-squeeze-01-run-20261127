# png-squeeze-01-run-20261127

[![make.fun · png-squeeze-01-run-20261127](https://make.fun/badge/png-squeeze-01-run-20261127.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![rank](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/rank.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)
[![reward](https://make.fun/badge/png-squeeze-01-run-20261127/SimplyMar/png-squeeze-01-run-20261127/reward.svg)](https://make.fun/bounty/png-squeeze-01-run-20261127?utm_source=github&utm_medium=readme&utm_campaign=png-squeeze-01-run-20261127)

A lossless PNG squeezer for screenshots, written for the make.fun bounty
[png-squeeze-01-run-20261127](https://make.fun/bounty/png-squeeze-01-run-20261127). Node 22, standard library only,
no dependencies.

## How it works

1. **Colour form** (`main.ts`): a palette (colour type 3) when the image has at most 256 colours, else grey (0),
   grey + alpha (4), RGB (2) or RGBA (6), with a tRNS key instead of an alpha channel when a single fully transparent
   value allows it. Bit depth stays 8 and only IHDR, PLTE, tRNS, IDAT and IEND are written.
2. **Filters** (`main.ts`, `png.mjs`): filter 0, a per-row entropy heuristic and a per-row brute force (each row
   compressed after the 32 KiB already chosen before it) are screened with zlib level 6; the smallest wins.
3. **DEFLATE** (`deflate.mjs`, written for this entry): binary-tree match finder over the 32 KiB window, a shortest-path
   (optimal) parse under an iterated entropy cost model, greedy block splitting with each
   block re-parsed under its own model, length-limited Huffman codes and the smallest RLE of the code-length header.
   The stream is inflated back and compared before it is used; zlib's stream is the fallback.

The work done depends only on the input (no clock, no randomness), so two runs write identical bytes.

Local practice set: **775,429 / 1,000,000**. The work is sized well inside the grader's 60 s wall on one core: a
deterministic budget (counted in bytes, never the clock) lightens the costliest images of an unusually heavy set.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

## License

MIT
