// png-squeeze: lossless PNG squeezer for desktop screenshots (main.ts runs this file in a plain .mjs child).
//   1. Colour form: palette (CT 3, sorted by frequency) when <= 256 colours and not grey; else grey (0),
//      grey+alpha (4), RGB (2) or RGBA (6), using a tRNS key in place of alpha when one transparent value allows it.
//   2. Filters: 5-state dictionary-carrying Strip Viterbi DP (8-row and 16-row bands, using Z_FILTERED strategy
//      when the image is literal-heavy/photo-like), fast 2-stage 1-row transition snapping, SmartUp exact-repeat
//      overlay (skipping all-zero/flat rows), per-row brute force on grey, and Z6->Z8/Z8F screening. On 1-bpp
//      images, a close runner-up is also DEFLATE-encoded and the smaller PNG kept.
//   3. DEFLATE (deflate.mjs + wasm-core.mjs): self-assembled inline WebAssembly BT3 match finder and O(1) RMQ
//      Exact-258 DP with exact multi-block deep-run skipping, dual-basin (fixed + literal-heavy) optimal parsing
//      with integer Huffman-length polish, incremental-histogram bottom-up agglomerative block merge + boundary
//      slide, exact Package-Merge Huffman lengths, and shortest-path DP header RLE with count smoothing.
// Zero clock reads, zero randomness: two runs on the same set always write byte-for-byte identical outputs.

import { copyFileSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { deflateSync, inflateRawSync } from 'node:zlib';
import { decode, analyse, transparentKey, samplesOf, paletteForm, filterRow, pngOf, chunkOverhead } from './png.mjs';
import { deflateRaw, zlibWrap } from './deflate.mjs';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const Z6 = { level: 6, memLevel: 9 };
const Z6F = { level: 6, memLevel: 9, strategy: 1 };
const Z8 = { level: 8, memLevel: 9 };
const Z8F = { level: 8, memLevel: 9, strategy: 1 };

function precomputeFilteredRows(samples, width, height, bpp, allowed = [0, 1, 2, 3, 4]) {
  const stride = width * bpp;
  const rowLen = stride + 1;
  const fRows = new Array(5);
  for (const t of allowed) fRows[t] = new Uint8Array(height * rowLen);
  for (let y = 0; y < height; y++) {
    const cur = samples.subarray(y * stride, (y + 1) * stride);
    const prev = y === 0 ? null : samples.subarray((y - 1) * stride, y * stride);
    const at = y * rowLen;
    for (const t of allowed) {
      fRows[t][at] = t;
      filterRow(t, cur, prev, bpp, fRows[t].subarray(at + 1, at + rowLen));
    }
  }
  return fRows;
}

function assembleFromFRows(fRows, height, rowLen, types) {
  const raw = new Uint8Array(height * rowLen);
  for (let y = 0; y < height; y++) {
    const at = y * rowLen;
    raw.set(fRows[types[y]].subarray(at, at + rowLen), at);
  }
  return raw;
}

function analyzeRows(samples, width, height, bpp) {
  const stride = width * bpp;
  const dup = new Uint8Array(height);
  const trans = new Uint16Array(height);
  const allZero = new Uint8Array(height);
  for (let y = 0; y < height; y++) {
    const o = y * stride;
    let nz = 0, tr = 0;
    for (let i = 0; i < stride; i++) {
      if (samples[o + i] !== 0) nz = 1;
      if (i >= bpp && samples[o + i] !== samples[o + i - bpp]) tr++;
    }
    allZero[y] = nz === 0 ? 1 : 0;
    trans[y] = tr;
    if (y > 0) {
      const p = o - stride;
      let eq = 1;
      for (let i = 0; i < stride; i++) if (samples[o + i] !== samples[p + i]) { eq = 0; break; }
      dup[y] = eq;
    }
  }
  return { dup, trans, allZero };
}

/** SmartUp: apply filter 2 (Up) to exact-duplicate rows that have non-trivial horizontal complexity. */
function smartUpOverlay(types, info, minTrans = 2) {
  const out = types.slice();
  const { dup, trans, allZero } = info;
  for (let y = 1; y < out.length; y++) {
    if (dup[y] && !allZero[y] && trans[y] >= minTrans && out[y] !== 2 && out[y] !== 4) out[y] = 2;
  }
  return out;
}

function bruteRow(fRows, height, rowLen, level, bias) {
  const raw = new Uint8Array(height * rowLen);
  const types = new Uint8Array(height);
  for (let y = 0; y < height; y++) {
    const at = y * rowLen;
    const dict = raw.subarray(Math.max(0, at - 32768), at);
    let best = Infinity, bt = 0;
    for (let t = 0; t < 5; t++) {
      const row = fRows[t].subarray(at, at + rowLen);
      let c = deflateSync(row, dict.length ? { level, memLevel: 9, dictionary: dict } : { level, memLevel: 9 }).length;
      if (bias && y > 0 && t === types[y - 1]) c -= bias;
      if (c < best) { best = c; bt = t; }
    }
    types[y] = bt;
    raw.set(fRows[bt].subarray(at, at + rowLen), at);
  }
  return types;
}

/** 5-state dictionary-carrying Viterbi DP over row strips. */
function stripViterbiDict(fRows, height, rowLen, cuts, level, strategy = 0, allowed = [0, 1, 2, 3, 4]) {
  const K = allowed.length;
  const nStrips = cuts.length - 1;
  const dicts = Array.from({ length: K }, () => new Uint8Array(32768));
  const dictLen = new Int32Array(K);
  const nextDicts = Array.from({ length: K }, () => new Uint8Array(32768));
  const nextDictLen = new Int32Array(K);
  const dp = new Float64Array(K);
  const nextDp = new Float64Array(K);
  const parent = new Uint8Array(nStrips * K);

  const appendDict = (src, srcLen, chunk, dst) => {
    const cl = chunk.length;
    if (cl >= 32768) { dst.set(chunk.subarray(cl - 32768)); return 32768; }
    const keep = Math.min(srcLen, 32768 - cl);
    dst.set(src.subarray(srcLen - keep, srcLen), 0);
    dst.set(chunk, keep);
    return keep + cl;
  };

  for (let si = 0; si < nStrips; si++) {
    const y0 = cuts[si], y1 = cuts[si + 1];
    const at0 = y0 * rowLen, at1 = y1 * rowLen;
    if (si === 0) {
      for (let ai = 0; ai < K; ai++) {
        const view = fRows[allowed[ai]].subarray(at0, at1);
        dp[ai] = deflateSync(view, { level, memLevel: 9, strategy }).length;
        dictLen[ai] = appendDict(dicts[ai], 0, view, dicts[ai]);
        parent[ai] = ai;
      }
      continue;
    }
    let bestPrev = 0;
    for (let pi = 1; pi < K; pi++) if (dp[pi] < dp[bestPrev]) bestPrev = pi;

    for (let ai = 0; ai < K; ai++) {
      const view = fRows[allowed[ai]].subarray(at0, at1);
      const dSame = dicts[ai].subarray(0, dictLen[ai]);
      let bestC = dp[ai] + deflateSync(view, { level, memLevel: 9, strategy, dictionary: dSame }).length;
      let bp = ai;
      if (bestPrev !== ai) {
        const dSw = dicts[bestPrev].subarray(0, dictLen[bestPrev]);
        const cSw = dp[bestPrev] + deflateSync(view, { level, memLevel: 9, strategy, dictionary: dSw }).length;
        if (cSw < bestC) { bestC = cSw; bp = bestPrev; }
      }
      nextDp[ai] = bestC;
      parent[si * K + ai] = bp;
      nextDictLen[ai] = appendDict(dicts[bp], dictLen[bp], view, nextDicts[ai]);
    }
    for (let ai = 0; ai < K; ai++) {
      dp[ai] = nextDp[ai];
      dicts[ai].set(nextDicts[ai].subarray(0, nextDictLen[ai]));
      dictLen[ai] = nextDictLen[ai];
    }
  }

  let cur = 0;
  for (let ai = 1; ai < K; ai++) if (dp[ai] < dp[cur]) cur = ai;
  const types = new Uint8Array(height);
  for (let si = nStrips - 1; si >= 0; si--) {
    const t = allowed[cur];
    for (let y = cuts[si]; y < cuts[si + 1]; y++) types[y] = t;
    cur = parent[si * K + cur];
  }
  return types;
}

/** 2-stage 1-row transition snapping (step-2 coarse + ±1 fine) around every filter change boundary. */
function snapTransitions(fRows, height, rowLen, types, level = 4, strategy = 0) {
  const out = types.slice();
  const edges = [];
  for (let y = 1; y < height; y++) if (out[y] !== out[y - 1]) edges.push(y);
  if (edges.length === 0) return out;

  const dictRows = Math.min(height, Math.ceil(32768 / rowLen) + 1);
  for (let ei = 0; ei < edges.length; ei++) {
    const e = edges[ei];
    const tL = out[e - 1], tR = out[e];
    if (tL === tR) continue;
    const minY = Math.max(ei > 0 ? edges[ei - 1] + 1 : 1, e - 8);
    const maxY = Math.min(ei + 1 < edges.length ? edges[ei + 1] - 1 : height - 1, e + 8);
    if (maxY <= minY) continue;
    const winStart = Math.max(0, minY - 2);
    const winEnd = Math.min(height, maxY + 6);
    const dictStart = Math.max(0, winStart - dictRows);
    const dictBuf = new Uint8Array((winStart - dictStart) * rowLen);
    for (let y = dictStart; y < winStart; y++) dictBuf.set(fRows[out[y]].subarray(y * rowLen, (y + 1) * rowLen), (y - dictStart) * rowLen);
    const dict = dictBuf.length > 32768 ? dictBuf.subarray(dictBuf.length - 32768) : dictBuf;
    const winBuf = new Uint8Array((winEnd - winStart) * rowLen);
    const opt = dict.length ? { level, memLevel: 9, strategy, dictionary: dict } : { level, memLevel: 9, strategy };

    const evalCand = (cand) => {
      for (let y = winStart; y < winEnd; y++) winBuf.set(fRows[y < cand ? tL : tR].subarray(y * rowLen, (y + 1) * rowLen), (y - winStart) * rowLen);
      return deflateSync(winBuf, opt).length;
    };

    let bestY = e, bestC = evalCand(e);
    for (let cand = minY; cand <= maxY; cand += 2) {
      if (cand === e) continue;
      const c = evalCand(cand);
      if (c < bestC) { bestC = c; bestY = cand; }
    }
    for (const d of [-1, 1]) {
      const cand = bestY + d;
      if (cand >= minY && cand <= maxY && cand !== e) {
        const c = evalCand(cand);
        if (c < bestC) { bestC = c; bestY = cand; }
      }
    }
    if (bestY !== e) {
      if (bestY < e) for (let y = bestY; y < e; y++) out[y] = tR;
      else for (let y = e; y < bestY; y++) out[y] = tL;
      edges[ei] = bestY;
    }
  }
  return out;
}

function fixedCuts(height, band) {
  const c = [0];
  for (let y = band; y < height; y += band) c.push(y);
  c.push(height);
  return c;
}

function screen(rgba, width, height) {
  const { opaque, grey, colours } = analyse(rgba);
  const rawCands = [];
  const push = (name, form, fRows, rowLen, types, isPhoto = false) => {
    const raw = assembleFromFRows(fRows, height, rowLen, types);
    const z0 = deflateSync(raw, Z6);
    let z = z0, sz = z0.length;
    if (isPhoto) {
      const z1 = deflateSync(raw, Z6F);
      if (z1.length < sz) { z = z1; sz = z1.length; }
    }
    rawCands.push({
      name,
      form: { colourType: form.colourType, bpp: form.bpp, plte: form.plte, trns: form.trns },
      raw,
      z,
      size: sz + chunkOverhead(form),
      isPhoto,
    });
  };

  if (colours !== null && !grey) {
    const byCount = [...colours].sort((x, y) => y.count - x.count || x.word - y.word);
    const form = paletteForm(rgba, byCount);
    const rowLen = width + 1;
    const fRows = precomputeFilteredRows(form.samples, width, height, 1, [0, 2]);
    const info = analyzeRows(form.samples, width, height, 1);
    const f0 = new Uint8Array(height).fill(0);
    push('f0', form, fRows, rowLen, f0);
    push('f0+su2', form, fRows, rowLen, smartUpOverlay(f0, info, 2));
    const v02 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 6, 0, [0, 2]);
    const v02s = snapTransitions(fRows, height, rowLen, v02, 4, 0);
    push('v02_8+snap', form, fRows, rowLen, v02s);
    push('v02_8+snap+su', form, fRows, rowLen, smartUpOverlay(v02s, info, 2));
  } else {
    const key = opaque ? null : transparentKey(rgba, grey);
    const ct = grey ? (opaque || key !== null ? 0 : 4) : (opaque || key !== null ? 2 : 6);
    const trns = key === null ? null : grey ? Uint8Array.of(0, key) : Uint8Array.of(0, key >>> 16, 0, (key >>> 8) & 0xff, 0, key & 0xff);
    const bpp = { 0: 1, 2: 3, 4: 2, 6: 4 }[ct];
    const form = { colourType: ct, bpp, samples: samplesOf(rgba, ct), plte: null, trns };
    const rowLen = width * bpp + 1;
    const fRows = precomputeFilteredRows(form.samples, width, height, bpp);
    const info = analyzeRows(form.samples, width, height, bpp);

    const z4def = deflateSync(fRows[4], Z6).length;
    const z4flt = deflateSync(fRows[4], Z6F).length;
    const isPhoto = z4flt < z4def * 0.99;
    const heavy = Math.min(z4def, z4flt) > 180000;
    const judgeStrat = isPhoto ? 1 : 0;

    const f0 = new Uint8Array(height).fill(0);
    push('f0', form, fRows, rowLen, f0, isPhoto);
    push('f0+su2', form, fRows, rowLen, smartUpOverlay(f0, info, 2), isPhoto);

    if (grey) {
      const br = bruteRow(fRows, height, rowLen, heavy ? 3 : 4, 4);
      push('brute4', form, fRows, rowLen, br, isPhoto);
      push('brute4+su2', form, fRows, rowLen, smartUpOverlay(br, info, 2), isPhoto);
    }

    const v16 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 16), 6, judgeStrat);
    const v8 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), heavy ? 5 : 6, judgeStrat);
    const z16 = deflateSync(assembleFromFRows(fRows, height, rowLen, v16), isPhoto ? Z6F : Z6).length;
    const z8 = deflateSync(assembleFromFRows(fRows, height, rowLen, v8), isPhoto ? Z6F : Z6).length;

    if (z16 <= z8) {
      const v16s = snapTransitions(fRows, height, rowLen, v16, 4, judgeStrat);
      push('vit16+snap', form, fRows, rowLen, v16s, isPhoto);
      push('vit16+snap+su', form, fRows, rowLen, smartUpOverlay(v16s, info, 2), isPhoto);
      if (!heavy && (bpp === 1 || z8 <= z16 * 1.018)) {
        const v8s = snapTransitions(fRows, height, rowLen, v8, 4, judgeStrat);
        push('vit8+snap', form, fRows, rowLen, v8s, isPhoto);
        push('vit8+snap+su', form, fRows, rowLen, smartUpOverlay(v8s, info, 2), isPhoto);
      }
    } else {
      const v8s = snapTransitions(fRows, height, rowLen, v8, 4, judgeStrat);
      push('vit8+snap', form, fRows, rowLen, v8s, isPhoto);
      push('vit8+snap+su', form, fRows, rowLen, smartUpOverlay(v8s, info, 2), isPhoto);
      if (!heavy && (bpp === 1 || z16 <= z8 * 1.018)) {
        const v16s = snapTransitions(fRows, height, rowLen, v16, 4, judgeStrat);
        push('vit16+snap', form, fRows, rowLen, v16s, isPhoto);
        push('vit16+snap+su', form, fRows, rowLen, smartUpOverlay(v16s, info, 2), isPhoto);
      }
    }
  }

  rawCands.sort((a, b) => a.size - b.size);
  const topK = [];
  const seenSig = new Set();
  for (const c of rawCands) {
    const sig = c.size + ':' + c.raw[0] + ':' + c.raw[c.raw.length >> 1] + ':' + c.raw[c.raw.length - 1];
    if (seenSig.has(sig)) continue;
    seenSig.add(sig);
    const z0 = deflateSync(c.raw, Z8);
    let z = z0, sz = z0.length;
    if (c.isPhoto) {
      const z1 = deflateSync(c.raw, Z8F);
      if (z1.length < sz) { z = z1; sz = z1.length; }
    }
    c.z = z;
    c.size = sz + chunkOverhead(c.form);
    topK.push(c);
    if (topK.length >= 3) break;
  }
  topK.sort((a, b) => a.size - b.size);

  const best = topK[0];
  let second = null;
  if (best.form.bpp === 1 && topK.length > 1) {
    const baseOf = (n) => n.replace('+su2', '').replace('+su', '');
    for (let i = 1; i < topK.length; i++) {
      const cand = topK[i];
      if (cand.size <= best.size * 1.03 && baseOf(cand.name) !== baseOf(best.name)) {
        second = cand;
        break;
      }
    }
    if (!second && topK[1].size <= best.size * 1.02) second = topK[1];
  }
  return { best, second, width, height };
}

function encodeCand(c, width, height, settings) {
  let idat = new Uint8Array(c.z.buffer, c.z.byteOffset, c.z.length);
  try {
    const raw = deflateRaw(c.raw, settings);
    const back = inflateRawSync(raw);
    if (Buffer.compare(back, Buffer.from(c.raw.buffer, c.raw.byteOffset, c.raw.length)) === 0 && raw.length + 6 < idat.length) {
      idat = zlibWrap(raw, c.raw);
    }
  } catch {
    // Keep verified zlib stream.
  }
  return pngOf(width, height, c.form, idat);
}

// Deterministic byte-counted work budget (never reads wall clock).
const DEF_1BPP  = { cut: 320, nice: 259, iters: 3, blockIters: 3, huffIters: 2, patience: 1, passes: 2, initChunks: 160, basins: ['fixed', 'litheavy'], blockAltBasin: false };
const DEF_2ND   = { cut: 256, nice: 259, iters: 2, blockIters: 2, huffIters: 2, patience: 1, passes: 1, initChunks: 140, basins: ['fixed', 'litheavy'], blockAltBasin: false };
const DEF_NORM  = { cut: 256, nice: 259, iters: 2, blockIters: 3, huffIters: 2, patience: 1, passes: 2, initChunks: 150, basins: ['fixed', 'litheavy'], blockAltBasin: false };
const DEF_LIGHT = { cut: 160, nice: 259, iters: 2, blockIters: 2, huffIters: 1, patience: 1, passes: 1, initChunks: 120, basins: ['fixed', 'litheavy'], blockAltBasin: false };
const BUDGET = 20000;
const oneCost = (c) => 0.9 * (c.raw.length / 1024) + 5.5 * (c.size / 1024);
const costOf = (s) => oneCost(s.best) + (s.second ? oneCost(s.second) * 0.5 : 0);

// Phase 1: screen every image and immediately write verified zlib PNG so every instance holds a valid answer.
const ids = readdirSync(I).sort();
const screened = new Map();
for (const id of ids) {
  const src = join(I, id, 'image.png');
  mkdirSync(join(O, id), { recursive: true });
  try {
    const img = decode(new Uint8Array(readFileSync(src)));
    const s = screen(img.rgba, img.width, img.height);
    const zView = new Uint8Array(s.best.z.buffer, s.best.z.byteOffset, s.best.z.length);
    writeFileSync(join(O, id, 'image.png'), pngOf(s.width, s.height, s.best.form, zView));
    screened.set(id, s);
  } catch {
    copyFileSync(src, join(O, id, 'image.png'));
  }
}

// Phase 2: plan deterministic work budget, then encode each image with WASM optimal DEFLATE.
const light = new Set();
let total = 0;
for (const s of screened.values()) total += costOf(s);
for (const [id, s] of [...screened].sort((a, b) => costOf(b[1]) - costOf(a[1]) || (a[0] < b[0] ? -1 : 1))) {
  if (total <= BUDGET) break;
  light.add(id);
  total -= costOf(s) * 0.45;
}

for (const [id, s] of screened) {
  try {
    const isLight = light.has(id);
    const settings = isLight ? DEF_LIGHT : (s.best.form.bpp === 1 ? DEF_1BPP : DEF_NORM);
    let bestPng = encodeCand(s.best, s.width, s.height, settings);
    if (!isLight && s.second) {
      const altPng = encodeCand(s.second, s.width, s.height, DEF_2ND);
      if (altPng.length < bestPng.length) bestPng = altPng;
    }
    writeFileSync(join(O, id, 'image.png'), bestPng);
  } catch {
    // Verified Phase 1 zlib PNG stands.
  }
}
