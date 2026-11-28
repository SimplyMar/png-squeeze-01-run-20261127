// png-squeeze: lossless PNG squeezer for desktop screenshots (main.ts runs this file in a plain .mjs child).
//   1. Colour form: palette (CT 3, frequency-sorted and 2-opt co-occurrence TSP when filtered) when <= 256 colours
//      and not grey; else grey (0), grey+alpha (4), RGB (2) or RGBA (6), using a tRNS key when one transparent value
//      allows it.
//   2. Filters: top-2 dictionary-carrying Strip Viterbi DP (4/8/16-row bands, using Z_FILTERED on photo-like images),
//      2-stage 1-row transition snapping, SmartUp exact-repeat overlay (minTrans 1 & 2), per-row brute force on grey,
//      and fast WASM DEFLATE pre-screening over diverse filter candidates to eliminate zlib block-splitting bias.
//   3. DEFLATE (deflate.mjs + wasm-core.mjs): self-assembled inline WebAssembly BT3 + explicit pixel/row-stride match
//      injection, exact non-monotonic distCost O(1) RMQ Exact-258 DP with optimal skipDist (1..4) deep-run jump,
//      distance-basin re-optimization, multi-basin optimal parsing, filter-transition-aware hybrid Bottom-Up + Exact
//      O(M^2) DP + incremental-histogram boundary slide block splitting, exact Package-Merge Huffman, and DP header RLE.
// Zero clock reads, zero randomness: two runs on the same set always write byte-for-byte identical outputs.

import { copyFileSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { deflateSync, inflateRawSync } from 'node:zlib';
import { decode, analyse, transparentKey, samplesOf, paletteForm, filterRow, pngOf, chunkOverhead } from './png.mjs';
import { deflateRaw, estimateDeflateBytes, zlibWrap } from './deflate.mjs';

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

function smartUpOverlay(types, info, minTrans = 2) {
  const out = types.slice();
  const { dup, trans, allZero } = info;
  for (let y = 1; y < out.length; y++) {
    if (dup[y] && !allZero[y] && trans[y] >= minTrans && out[y] !== 2 && out[y] !== 4) out[y] = 2;
  }
  return out;
}

function tspPaletteOrder(rgba, width, height, byCount) {
  const nC = byCount.length;
  if (nC <= 3) return byCount;
  const baseForm = paletteForm(rgba, byCount);
  const s = baseForm.samples;
  const co = Array.from({ length: nC }, () => new Int32Array(nC));
  for (let y = 0; y < height; y++) {
    const o = y * width;
    for (let x = 1; x < width; x++) {
      const u = s[o + x - 1], v = s[o + x];
      if (u !== v) { co[u][v]++; co[v][u]++; }
    }
    if (y > 0) {
      for (let x = 0; x < width; x++) {
        const u = s[o - width + x], v = s[o + x];
        if (u !== v) { co[u][v]++; co[v][u]++; }
      }
    }
  }
  const used = new Uint8Array(nC);
  const idx = [0];
  used[0] = 1;
  for (let k = 1; k < nC; k++) {
    const last = idx[idx.length - 1];
    let best = -1, bw = -1;
    for (let j = 0; j < nC; j++) {
      if (!used[j] && co[last][j] > bw) { bw = co[last][j]; best = j; }
    }
    idx.push(best);
    used[best] = 1;
  }
  for (let it = 0; it < 40; it++) {
    let imp = false;
    for (let i = 1; i < nC - 1; i++) {
      for (let j = i + 1; j < nC; j++) {
        const curW = co[idx[i - 1]][idx[i]] + (j + 1 < nC ? co[idx[j]][idx[j + 1]] : 0);
        const newW = co[idx[i - 1]][idx[j]] + (j + 1 < nC ? co[idx[i]][idx[j + 1]] : 0);
        if (newW > curW) {
          for (let l = i, r = j; l < r; l++, r--) { const t = idx[l]; idx[l] = idx[r]; idx[r] = t; }
          imp = true;
        }
      }
    }
    if (!imp) break;
  }
  return idx.map((i) => byCount[i]);
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

/** Top-2 dictionary-carrying Viterbi DP over row strips with optional switch penalty. */
function stripViterbiDict(fRows, height, rowLen, cuts, level, strategy = 0, allowed = [0, 1, 2, 3, 4], switchPenalty = 0) {
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
    // Find best and second-best previous states
    let b0 = 0, b1 = -1;
    for (let pi = 1; pi < K; pi++) {
      if (dp[pi] < dp[b0]) { b1 = b0; b0 = pi; }
      else if (b1 < 0 || dp[pi] < dp[b1]) b1 = pi;
    }

    for (let ai = 0; ai < K; ai++) {
      const view = fRows[allowed[ai]].subarray(at0, at1);
      const dSame = dicts[ai].subarray(0, dictLen[ai]);
      let bestC = dp[ai] + deflateSync(view, { level, memLevel: 9, strategy, dictionary: dSame }).length;
      let bp = ai;
      if (b0 !== ai) {
        const dSw = dicts[b0].subarray(0, dictLen[b0]);
        const cSw = dp[b0] + deflateSync(view, { level, memLevel: 9, strategy, dictionary: dSw }).length + switchPenalty;
        if (cSw < bestC) { bestC = cSw; bp = b0; }
      }
      if (b1 >= 0 && b1 !== ai && b1 !== b0 && dp[b1] <= dp[b0] + 250) {
        const dSw2 = dicts[b1].subarray(0, dictLen[b1]);
        const cSw2 = dp[b1] + deflateSync(view, { level, memLevel: 9, strategy, dictionary: dSw2 }).length + switchPenalty;
        if (cSw2 < bestC) { bestC = cSw2; bp = b1; }
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

/** 2-stage 1-row transition snapping (step-2 coarse + +-1 fine) around every filter change boundary. */
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
    const dict = dictBuf.subarray(Math.max(0, dictBuf.length - 32768));

    const winBuf = new Uint8Array((winEnd - winStart) * rowLen);
    const evalAt = (sp) => {
      for (let y = winStart; y < winEnd; y++) {
        const ft = y < sp ? (y < minY ? out[y] : tL) : (y >= maxY ? out[y] : tR);
        winBuf.set(fRows[ft].subarray(y * rowLen, (y + 1) * rowLen), (y - winStart) * rowLen);
      }
      return deflateSync(winBuf, dict.length ? { level, memLevel: 9, strategy, dictionary: dict } : { level, memLevel: 9, strategy }).length;
    };

    let bestY = e, bestS = evalAt(e);
    for (let sp = minY; sp <= maxY; sp += 2) {
      const s = evalAt(sp);
      if (s < bestS) { bestS = s; bestY = sp; }
    }
    for (const sp of [bestY - 1, bestY + 1]) {
      if (sp >= minY && sp <= maxY) {
        const s = evalAt(sp);
        if (s < bestS) { bestS = s; bestY = sp; }
      }
    }
    for (let y = minY; y < maxY; y++) out[y] = y < bestY ? tL : tR;
    edges[ei] = bestY;
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
      rowLen,
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
    const fRows = precomputeFilteredRows(form.samples, width, height, 1, [0, 1, 2, 3, 4]);
    const info = analyzeRows(form.samples, width, height, 1);
    const f0 = new Uint8Array(height).fill(0);
    push('f0', form, fRows, rowLen, f0);
    push('f0+su1', form, fRows, rowLen, smartUpOverlay(f0, info, 1));
    push('f0+su2', form, fRows, rowLen, smartUpOverlay(f0, info, 2));
    const v02 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 6, 0, [0, 2]);
    const v02s = snapTransitions(fRows, height, rowLen, v02, 4, 0);
    push('v02_8+snap', form, fRows, rowLen, v02s);
    push('v02_8+snap+su1', form, fRows, rowLen, smartUpOverlay(v02s, info, 1));

    // Also allow all 5 filters + TSP palette reordering when filtered rows beat F0!
    const z0 = deflateSync(fRows[0], Z6).length;
    const z4 = deflateSync(fRows[4], Z6).length;
    const z2 = deflateSync(fRows[2], Z6).length;
    if (Math.min(z4, z2) < z0 * 1.08) {
      const vAll = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 5, 0, [0, 1, 2, 3, 4]);
      const vAllS = snapTransitions(fRows, height, rowLen, vAll, 3, 0);
      push('vall_8+snap', form, fRows, rowLen, vAllS);
      push('vall_8+snap+su1', form, fRows, rowLen, smartUpOverlay(vAllS, info, 1));

      if (z4 < z0 * 0.96) {
        const tspOrd = tspPaletteOrder(rgba, width, height, byCount);
        const tspForm = paletteForm(rgba, tspOrd);
        const tspRows = precomputeFilteredRows(tspForm.samples, width, height, 1, [0, 1, 2, 3, 4]);
        const tspInfo = analyzeRows(tspForm.samples, width, height, 1);
        const tvAll = stripViterbiDict(tspRows, height, rowLen, fixedCuts(height, 8), 5, 0, [0, 1, 2, 3, 4]);
        const tvAllS = snapTransitions(tspRows, height, rowLen, tvAll, 3, 0);
        push('tsp_vall+snap', tspForm, tspRows, rowLen, tvAllS);
        push('tsp_vall+snap+su1', tspForm, tspRows, rowLen, smartUpOverlay(tvAllS, tspInfo, 1));
      }
    }
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
    const heavy = Math.min(z4def, z4flt) > 300000;
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

    const v16s = snapTransitions(fRows, height, rowLen, v16, 4, judgeStrat);
    push('vit16+snap', form, fRows, rowLen, v16s, isPhoto);
    push('vit16+snap+su', form, fRows, rowLen, smartUpOverlay(v16s, info, 2), isPhoto);

    if (!heavy || z8 <= z16 * 1.02) {
      const v8s = snapTransitions(fRows, height, rowLen, v8, 4, judgeStrat);
      push('vit8+snap', form, fRows, rowLen, v8s, isPhoto);
      push('vit8+snap+su', form, fRows, rowLen, smartUpOverlay(v8s, info, 2), isPhoto);
    }

    // On grey images where Z_DEFAULT won, also probe Z_FILTERED Viterbi (level 5 lazy matching catches subtle gradients!)
    if (grey && !isPhoto && !heavy) {
      const v8f = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 5, 1);
      const v8fs = snapTransitions(fRows, height, rowLen, v8f, 4, 1);
      push('vit8f+snap', form, fRows, rowLen, v8fs, false);
      const v4f = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 4), 5, 1, [0, 1, 2, 3, 4], 15);
      const v4fs = snapTransitions(fRows, height, rowLen, v4f, 4, 1);
      push('vit4f+snap', form, fRows, rowLen, v4fs, false);
    }

    // Add finer 4-row Viterbi with switch penalty on photo-like images when vit8 beats vit16 (e.g. gallery layouts)
    if (isPhoto && !heavy && z8 <= z16 * 1.01) {
      const v4 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 4), 5, judgeStrat, [0, 1, 2, 3, 4], 15);
      const v4s = snapTransitions(fRows, height, rowLen, v4, 4, judgeStrat);
      push('vit4+snap', form, fRows, rowLen, v4s, isPhoto);
      push('vit4+snap+su', form, fRows, rowLen, smartUpOverlay(v4s, info, 2), isPhoto);
    }
  }

  rawCands.sort((a, b) => a.size - b.size);
  const topK = [];
  const seenSig = new Set();
  const baseOf = (n) => n.replace('+su2', '').replace('+su1', '').replace('+su', '');
  const seenFamily = new Set();

  const addCand = (c) => {
    const sig = c.size + ':' + c.raw[0] + ':' + c.raw[c.raw.length >> 1] + ':' + c.raw[c.raw.length - 1];
    if (seenSig.has(sig)) return false;
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
    return true;
  };

  // Keep top diverse candidates (always preserving v02_8+snap & f0+su1 on CT3, and vit4f+snap on non-photo grey!)
  const bestZ6 = rawCands[0].size;
  const maxK = rawCands[0].form.bpp === 1 ? 3 : 2;
  for (const c of rawCands) {
    if (topK.length >= 2 && c.size > bestZ6 * 1.04) break;
    const fam = baseOf(c.name);
    if (topK.length >= 2 && seenFamily.has(fam)) continue;
    if (addCand(c)) {
      seenFamily.add(fam);
      if (topK.length >= maxK) break;
    }
  }
  if (rawCands[0].form.colourType === 3) {
    for (const must of ['v02_8+snap', 'f0+su1']) {
      if (!seenFamily.has(baseOf(must))) {
        const found = rawCands.find((c) => c.name === must);
        if (found) addCand(found);
      }
    }
  } else if (rawCands[0].form.colourType === 0 && !rawCands[0].isPhoto) {
    const v4f = rawCands.find((c) => c.name === 'vit4f+snap');
    if (v4f && !seenFamily.has('vit4f+snap')) addCand(v4f);
  }
  topK.sort((a, b) => a.size - b.size);
  return { cands: topK, best: topK[0], width, height };
}

function encodeCand(c, width, height, settings) {
  let idat = new Uint8Array(c.z.buffer, c.z.byteOffset, c.z.length);
  try {
    const raw = deflateRaw(c.raw, { ...settings, bpp: c.form.bpp, rowLen: c.rowLen });
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
const DEF_1BPP  = { cut: 512, nice: 259, iters: 3, blockIters: 4, huffIters: 2, patience: 2, passes: 2, initChunks: 175, dpCap: 54, blockAltBasin: true };
const DEF_2ND   = { cut: 320, nice: 259, iters: 2, blockIters: 3, huffIters: 2, patience: 1, passes: 1, initChunks: 155, dpCap: 44, blockAltBasin: false };
const DEF_NORM  = { cut: 256, nice: 259, iters: 2, blockIters: 3, huffIters: 2, patience: 1, passes: 2, initChunks: 165, dpCap: 48, blockAltBasin: true };
const DEF_LIGHT = { cut: 160, nice: 259, iters: 2, blockIters: 2, huffIters: 1, patience: 1, passes: 1, initChunks: 125, dpCap: 0, blockAltBasin: false };

const BUDGET = 15200;
const oneCost = (c) => 0.35 * (c.raw.length / 1024) + 12.0 * (c.size / 1024);

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
for (const s of screened.values()) total += oneCost(s.best);
for (const [id, s] of [...screened].sort((a, b) => oneCost(b[1].best) - oneCost(a[1].best) || (a[0] < b[0] ? -1 : 1))) {
  if (total <= BUDGET) break;
  light.add(id);
  total -= oneCost(s.best) * 0.45;
}

for (const [id, s] of screened) {
  try {
    const isLight = light.has(id);
    const bpp = s.best.form.bpp;

    // Fast WASM pre-screen on 1-bpp images (and ultra-close multi-bpp ties) to eliminate zlib block-splitting bias
    let chosen = s.best;
    let runnerUp = null;
    if (!isLight && s.cands.length > 1) {
      const pool = bpp === 1
        ? s.cands
        : s.cands.filter((c, i) => i === 0 || (i < 2 && c.size <= s.best.size * 1.004));
      if (pool.length > 1) {
        const ranked = pool.map((c) => ({
          c,
          est: estimateDeflateBytes(c.raw, c.form.bpp, c.rowLen, c.isPhoto) + chunkOverhead(c.form),
        }));
        ranked.sort((a, b) => a.est - b.est);
        chosen = ranked[0].c;
        if (bpp === 1 && ranked.length > 1 && ranked[1].est <= ranked[0].est * 1.005) {
          runnerUp = ranked[1].c;
        }
      }
    }

    const settings = isLight ? DEF_LIGHT : (bpp === 1 ? DEF_1BPP : DEF_NORM);
    let bestPng = encodeCand(chosen, s.width, s.height, settings);
    if (!isLight && runnerUp) {
      const altPng = encodeCand(runnerUp, s.width, s.height, DEF_2ND);
      if (altPng.length < bestPng.length) bestPng = altPng;
    }
    writeFileSync(join(O, id, 'image.png'), bestPng);
  } catch {
    // Verified Phase 1 zlib PNG stands.
  }
}
