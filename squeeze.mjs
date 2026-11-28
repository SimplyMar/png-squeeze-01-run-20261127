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

function smartUpOverlay(types, info, minTrans = 2, minRun = 1) {
  const out = types.slice();
  const { dup, trans, allZero } = info;
  const h = out.length;
  let y = 1;
  while (y < h) {
    if (dup[y] && !allZero[y] && trans[y] >= minTrans && out[y] !== 2 && out[y] !== 4) {
      let end = y + 1;
      while (end < h && dup[end] && !allZero[end] && trans[end] >= minTrans && out[end] !== 2 && out[end] !== 4) end++;
      if (end - y >= minRun) {
        for (let k = y; k < end; k++) out[k] = 2;
      }
      y = end;
    } else {
      y++;
    }
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

/** Top-2 dictionary-carrying Viterbi DP over row strips with optional switch penalty (strat=-1 uses min(def, flt)). */
function stripViterbiDict(fRows, height, rowLen, cuts, level, strategy = 0, allowed = [0, 1, 2, 3, 4], switchPenalty = 0) {
  const K = allowed.length;
  const nStrips = cuts.length - 1;
  const maxDict = 32768;
  const dicts = Array.from({ length: K }, () => new Uint8Array(maxDict));
  const dictLen = new Int32Array(K);
  const nextDicts = Array.from({ length: K }, () => new Uint8Array(maxDict));
  const nextDictLen = new Int32Array(K);
  const dp = new Float64Array(K);
  const nextDp = new Float64Array(K);
  const parent = new Uint8Array(nStrips * K);

  const appendDict = (src, srcLen, chunk, dst) => {
    const cl = chunk.length;
    if (cl >= maxDict) { dst.set(chunk.subarray(cl - maxDict)); return maxDict; }
    const keep = Math.min(srcLen, maxDict - cl);
    dst.set(src.subarray(srcLen - keep, srcLen), 0);
    dst.set(chunk, keep);
    return keep + cl;
  };

  const scoreView = (view, dict) => {
    const base = dict && dict.length ? { level, memLevel: 9, dictionary: dict } : { level, memLevel: 9 };
    if (strategy !== -1) return deflateSync(view, { ...base, strategy }).length;
    return Math.min(
      deflateSync(view, { ...base, strategy: 0 }).length,
      deflateSync(view, { ...base, strategy: 1 }).length,
    );
  };

  for (let si = 0; si < nStrips; si++) {
    const y0 = cuts[si], y1 = cuts[si + 1];
    const at0 = y0 * rowLen, at1 = y1 * rowLen;
    if (si === 0) {
      for (let ai = 0; ai < K; ai++) {
        const view = fRows[allowed[ai]].subarray(at0, at1);
        dp[ai] = scoreView(view, null);
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
      let bestC = dp[ai] + scoreView(view, dSame);
      let bp = ai;
      if (b0 !== ai) {
        const dSw = dicts[b0].subarray(0, dictLen[b0]);
        const cSw = dp[b0] + scoreView(view, dSw) + switchPenalty;
        if (cSw < bestC) { bestC = cSw; bp = b0; }
      }
      const slack = (cuts.length > 2 && cuts[1] - cuts[0] >= 8) ? 250 : 60;
      if (b1 >= 0 && b1 !== ai && b1 !== b0 && dp[b1] <= dp[b0] + slack) {
        const dSw2 = dicts[b1].subarray(0, dictLen[b1]);
        const cSw2 = dp[b1] + scoreView(view, dSw2) + switchPenalty;
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
      const base = dict.length ? { level, memLevel: 9, dictionary: dict } : { level, memLevel: 9 };
      if (strategy !== -1) return deflateSync(winBuf, { ...base, strategy }).length;
      return Math.min(
        deflateSync(winBuf, { ...base, strategy: 0 }).length,
        deflateSync(winBuf, { ...base, strategy: 1 }).length,
      );
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

/** Recombine two row-filter plans over arbitrary cuts (with optional lookahead rows) using 2-state dictionary Viterbi DP. */
function recombine2Cuts(fRows, height, rowLen, planA, planB, cuts, level = 5, strat = 0, swPen = 0, lookRows = 0) {
  const plans = [planA, planB];
  const nS = cuts.length - 1;
  const dicts = [new Uint8Array(32768), new Uint8Array(32768)];
  const dLen = new Int32Array(2);
  const nDicts = [new Uint8Array(32768), new Uint8Array(32768)];
  const nDLen = new Int32Array(2);
  const dp = new Float64Array(2), nDp = new Float64Array(2);
  const par = new Uint8Array(nS * 2);
  const getChunk = (p, y0, y1) => {
    const ch = new Uint8Array((y1 - y0) * rowLen);
    for (let y = y0; y < y1; y++) ch.set(fRows[p[y]].subarray(y * rowLen, (y + 1) * rowLen), (y - y0) * rowLen);
    return ch;
  };
  const app = (src, sl, ch, dst) => {
    const cl = ch.length;
    if (cl >= 32768) { dst.set(ch.subarray(cl - 32768)); return 32768; }
    const keep = Math.min(sl, 32768 - cl);
    dst.set(src.subarray(sl - keep, sl), 0);
    dst.set(ch, keep);
    return keep + cl;
  };
  const scoreChunk = (ch, dict, lk) => {
    let target = ch;
    if (lk && lk.length) {
      target = new Uint8Array(ch.length + lk.length);
      target.set(ch, 0);
      target.set(lk, ch.length);
    }
    const base = dict && dict.length ? { level, memLevel: 9, dictionary: dict } : { level, memLevel: 9 };
    if (strat !== -1) return deflateSync(target, { ...base, strategy: strat }).length;
    return Math.min(
      deflateSync(target, { ...base, strategy: 0 }).length,
      deflateSync(target, { ...base, strategy: 1 }).length,
    );
  };
  for (let si = 0; si < nS; si++) {
    const y0 = cuts[si], y1 = cuts[si + 1];
    const chs = [getChunk(planA, y0, y1), getChunk(planB, y0, y1)];
    const y2 = lookRows > 0 ? Math.min(height, y1 + lookRows) : y1;
    const lk = y2 > y1 ? getChunk(planA, y1, y2) : null;
    if (si === 0) {
      for (let a = 0; a < 2; a++) {
        dp[a] = scoreChunk(chs[a], null, lk);
        dLen[a] = app(dicts[a], 0, chs[a], dicts[a]);
        par[a] = a;
      }
      continue;
    }
    for (let a = 0; a < 2; a++) {
      let bestC = Infinity, bp = a;
      for (let p = 0; p < 2; p++) {
        const c = dp[p] + scoreChunk(chs[a], dicts[p].subarray(0, dLen[p]), lk) + (p !== a ? swPen : 0);
        if (c < bestC) { bestC = c; bp = p; }
      }
      nDp[a] = bestC;
      par[si * 2 + a] = bp;
      nDLen[a] = app(dicts[bp], dLen[bp], chs[a], nDicts[a]);
    }
    for (let a = 0; a < 2; a++) {
      dp[a] = nDp[a];
      dicts[a].set(nDicts[a].subarray(0, nDLen[a]));
      dLen[a] = nDLen[a];
    }
  }
  let cur = dp[1] < dp[0] ? 1 : 0;
  const out = new Uint8Array(height);
  for (let si = nS - 1; si >= 0; si--) {
    const p = plans[cur];
    for (let y = cuts[si]; y < cuts[si + 1]; y++) out[y] = p[y];
    cur = par[si * 2 + cur];
  }
  return out;
}

function recombine2(fRows, height, rowLen, planA, planB, band = 16, level = 5, strat = 0, swPen = 0) {
  return recombine2Cuts(fRows, height, rowLen, planA, planB, fixedCuts(height, band), level, strat, swPen, 0);
}

function fixedCuts(height, band) {
  const c = [0];
  for (let y = band; y < height; y += band) c.push(y);
  c.push(height);
  return c;
}

/** Centered L6 Transition + Row Post-Filter (evaluates downstream dictionary impact at level 6). */
function centeredPostFilterL6(fRows, height, rowLen, types, bpp, info) {
  const out = types.slice();
  const maxWin = Math.max(4, Math.min(20, Math.floor(32768 / rowLen)));
  const backRows = maxWin;
  const fwdRows = bpp === 1 ? Math.min(16, maxWin) : maxWin;
  const buf = new Uint8Array((backRows + fwdRows + 4) * rowLen);

  const scoreRange = (yA, yB, fCand) => {
    const y0 = Math.max(0, yA - backRows);
    const y1 = Math.min(height, yB + fwdRows);
    const dRows = yA - y0;
    for (let y = y0; y < y1; y++) {
      const f = (y >= yA && y < yB && fCand >= 0) ? fCand : out[y];
      buf.set(fRows[f].subarray(y * rowLen, (y + 1) * rowLen), (y - y0) * rowLen);
    }
    const dict = dRows > 0 ? buf.subarray(0, dRows * rowLen) : null;
    const body = buf.subarray(dRows * rowLen, (y1 - y0) * rowLen);
    const base = dict ? { level: 6, memLevel: 9, strategy: 0, dictionary: dict } : { level: 6, memLevel: 9, strategy: 0 };
    return deflateSync(body, base).length;
  };

  // Pass 1: Transition L6 Polish (up to +-10 rows)
  for (let pass = 0; pass < 2; pass++) {
    let mv = 0;
    for (let y = 1; y < height; y++) {
      if (out[y] === out[y - 1]) continue;
      const fL = out[y - 1], fR = out[y];
      let cur = y, steps = 0;
      while (cur < height && out[cur] === fR && steps < 10) {
        if (scoreRange(cur, cur + 1, fL) < scoreRange(cur, cur + 1, fR)) { out[cur] = fL; cur++; steps++; mv++; }
        else break;
      }
      if (steps > 0) { y = cur; continue; }
      cur = y - 1; steps = 0;
      while (cur >= 0 && out[cur] === fL && steps < 10) {
        if (scoreRange(cur, cur + 1, fR) < scoreRange(cur, cur + 1, fL)) { out[cur] = fR; cur--; steps++; mv++; }
        else break;
      }
    }
    if (!mv) break;
  }

  // Pass 2: 1-pass Centered L6 Row Post-Filter (skips all-zero or exact-dup rows already inside a uniform f=0 band)
  const margin = bpp === 1 ? 1 : 0;
  const allZero = info ? info.allZero : null;
  const dup = info ? info.dup : null;
  for (let y = 0; y < height; y++) {
    const curF = out[y];
    if (curF === 0 && y > 0 && y + 1 < height && out[y - 1] === 0 && out[y + 1] === 0 && ((allZero && allZero[y]) || (bpp > 1 && dup && dup[y] && info.trans[y] < 2))) continue;
    let bestF = curF, bestS = scoreRange(y, y + 1, curF) - margin;
    for (let f = 0; f < 5; f++) {
      if (f === curF) continue;
      const s = scoreRange(y, y + 1, f);
      if (s < bestS) { bestS = s; bestF = f; }
    }
    if (bestF !== curF) out[y] = bestF;
  }
  return out;
}

function screen(rgba, width, height) {
  const { opaque, grey, colours } = analyse(rgba);
  const rawCands = [];
  const push = (name, form, fRows, rowLen, types, isPhoto = false, info = null) => {
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
      fRows,
      types,
      info,
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
    const f0su1 = smartUpOverlay(f0, info, 1, 1);
    const f0su1r2 = smartUpOverlay(f0, info, 1, 2);
    push('f0', form, fRows, rowLen, f0);
    push('f0+su1', form, fRows, rowLen, f0su1);
    push('f0+su1r2', form, fRows, rowLen, f0su1r2);
    push('rec_f0_su1', form, fRows, rowLen, recombine2(fRows, height, rowLen, f0, f0su1, 16, 6, 0, 0));
    push('rec_f0_su1_8', form, fRows, rowLen, recombine2(fRows, height, rowLen, f0, f0su1, 8, 6, 0, 0));
    // Natural dup-run boundary recombination with 8-row lookahead (captures dictionary impact on subsequent UI rows!)
    const natCuts = [0];
    for (let y = 1; y < height; y++) if (f0su1[y] === 2 && f0su1[y - 1] === 0) natCuts.push(y);
    natCuts.push(height);
    if (natCuts.length >= 4 && natCuts.length <= 160) {
      push('nat_lk8', form, fRows, rowLen, recombine2Cuts(fRows, height, rowLen, f0, f0su1, natCuts, 6, 0, 0, 8));
    }
    const v02 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 6, 0, [0, 2]);
    const v02s = snapTransitions(fRows, height, rowLen, v02, 3, 0);
    push('v02_8+snap', form, fRows, rowLen, v02s);
    push('v02_8+snap+su1', form, fRows, rowLen, smartUpOverlay(v02s, info, 1, 1));
    push('rec_su1_v02', form, fRows, rowLen, recombine2(fRows, height, rowLen, f0su1, v02s, 16, 6, 0, 0));

    // Allow all 5 filters + recombination with f0su1 (captures charts/gradients embedded in flat UI like i005!)
    const z0 = deflateSync(fRows[0], Z6).length;
    const z4 = deflateSync(fRows[4], Z6).length;
    const z2 = deflateSync(fRows[2], Z6).length;
    if (Math.min(z4, z2) < z0 * 1.25) {
      const vAll = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), 5, 0, [0, 1, 2, 3, 4]);
      const vAllS = snapTransitions(fRows, height, rowLen, vAll, 3, 0);
      push('vall_8+snap', form, fRows, rowLen, vAllS);
      push('vall_8+snap+su1', form, fRows, rowLen, smartUpOverlay(vAllS, info, 1, 1));
      const recAll = recombine2(fRows, height, rowLen, f0su1, vAllS, 16, 6, 0, 0);
      const recAllS = snapTransitions(fRows, height, rowLen, recAll, 3, 0);
      push('rec_su1_vall', form, fRows, rowLen, recAllS);
      push('rec_su1_vall+su1', form, fRows, rowLen, smartUpOverlay(recAllS, info, 1, 1));

      if (z4 < z0 * 0.96) {
        const tspOrd = tspPaletteOrder(rgba, width, height, byCount);
        const tspForm = paletteForm(rgba, tspOrd);
        const tspRows = precomputeFilteredRows(tspForm.samples, width, height, 1, [0, 1, 2, 3, 4]);
        const tspInfo = analyzeRows(tspForm.samples, width, height, 1);
        const tvAll = stripViterbiDict(tspRows, height, rowLen, fixedCuts(height, 8), 5, 0, [0, 1, 2, 3, 4]);
        const tvAllS = snapTransitions(tspRows, height, rowLen, tvAll, 3, 0);
        push('tsp_vall+snap', tspForm, tspRows, rowLen, tvAllS);
        push('tsp_vall+snap+su1', tspForm, tspRows, rowLen, smartUpOverlay(tvAllS, tspInfo, 1, 1));
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
    const z4min = Math.min(z4def, z4flt);
    const isPhoto = z4flt < z4def * 0.99;
    const heavy = z4min > 300000;
    const judgeStrat = isPhoto ? 1 : 0;

    const f0 = new Uint8Array(height).fill(0);
    push('f0', form, fRows, rowLen, f0, isPhoto, info);
    push('f0+su2', form, fRows, rowLen, smartUpOverlay(f0, info, 2, 1), isPhoto, info);

    let br = null;
    if (grey) {
      br = bruteRow(fRows, height, rowLen, z4min > 125000 ? 3 : 4, isPhoto ? 2 : 4);
      push('brute4', form, fRows, rowLen, br, isPhoto, info);
      push('brute4+su2', form, fRows, rowLen, smartUpOverlay(br, info, 2, 1), isPhoto, info);
    }

    const v16 = isPhoto ? null : stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 16), 6, judgeStrat);
    const v8Strat = (!grey && isPhoto) ? -1 : judgeStrat;
    const v8 = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 8), (heavy || (!grey && isPhoto)) ? 5 : 6, v8Strat, [0, 1, 2, 3, 4], isPhoto ? 15 : 0);
    const z16 = v16 ? deflateSync(assembleFromFRows(fRows, height, rowLen, v16), isPhoto ? Z6F : Z6).length : Infinity;
    const z8 = deflateSync(assembleFromFRows(fRows, height, rowLen, v8), isPhoto ? Z6F : Z6).length;

    let v16s = null;
    if (v16) {
      v16s = snapTransitions(fRows, height, rowLen, v16, 4, judgeStrat);
      push('vit16+snap', form, fRows, rowLen, v16s, isPhoto, info);
      push('vit16+snap+su', form, fRows, rowLen, smartUpOverlay(v16s, info, 2, 1), isPhoto, info);
    }

    let v8s = null;
    if (!heavy || z8 <= z16 * 1.02) {
      v8s = snapTransitions(fRows, height, rowLen, v8, (!grey && isPhoto) ? 5 : 4, v8Strat);
      push((!grey && isPhoto) ? 'vdual8+snap' : 'vit8+snap', form, fRows, rowLen, v8s, isPhoto, info);
      push((!grey && isPhoto) ? 'vdual8+snap+su' : 'vit8+snap+su', form, fRows, rowLen, smartUpOverlay(v8s, info, 2, 1), isPhoto, info);
    }

    if (grey && !isPhoto && !heavy) {
      const v4f = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 4), 5, 1, [0, 1, 2, 3, 4], 15);
      const v4fs = snapTransitions(fRows, height, rowLen, v4f, 4, 1);
      push('vit4f+snap', form, fRows, rowLen, v4fs, false, info);
      const v2f = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 2), 5, 1, [0, 1, 2, 3, 4], 15);
      const v2fs = snapTransitions(fRows, height, rowLen, v2f, 4, 1);
      push('vit2f+snap', form, fRows, rowLen, v2fs, false, info);
      // Untrapped dual-plan recombination for non-photo grey hybrids (e.g. i008)
      const baseDef = z16 <= z8 ? v16s : (v8s || v16s);
      const recG = recombine2(fRows, height, rowLen, baseDef, v4fs, 16, 5, -1, 0);
      const recGS = snapTransitions(fRows, height, rowLen, recG, 4, 0);
      push('rec_g_def_v4f+snap', form, fRows, rowLen, recGS, false, info);
    } else if (grey && isPhoto && br) {
      if (v8s) {
        const recBr = recombine2(fRows, height, rowLen, br, v8s, 16, z4min > 120000 ? 4 : 5, -1, 0);
        push('rec_br_v8', form, fRows, rowLen, recBr, isPhoto, info);
      }
    }

    // Universal untrapped dual-plan recombination for non-heavy multi-bpp (both photo and UI+photo hybrids!)
    if (!grey && !heavy) {
      const v4f = stripViterbiDict(fRows, height, rowLen, fixedCuts(height, 4), isPhoto ? 5 : 4, 1, [0, 1, 2, 3, 4], 15);
      const v4fs = snapTransitions(fRows, height, rowLen, v4f, 4, isPhoto ? -1 : 1);
      if (isPhoto) {
        push('vit4+snap', form, fRows, rowLen, v4fs, true, info);
        push('vit4+snap+su', form, fRows, rowLen, smartUpOverlay(v4fs, info, 2, 1), true, info);
        const recP = recombine2(fRows, height, rowLen, v8s || v16s, v4fs, 16, 5, -1, 0);
        const recPS = snapTransitions(fRows, height, rowLen, recP, 5, -1);
        push('rec_dual8_v4f+snap', form, fRows, rowLen, recPS, true, info);
        push('rec_dual8_v4f+snap+su', form, fRows, rowLen, smartUpOverlay(recPS, info, 2, 1), true, info);
      } else {
        const baseDef = v8s || v16s;
        const rec = recombine2(fRows, height, rowLen, baseDef, v4fs, 16, 5, -1, 0);
        const recS = snapTransitions(fRows, height, rowLen, rec, 4, 0);
        push('rec_v8_v4f+snap', form, fRows, rowLen, recS, false, info);
        push('rec_v8_v4f+snap+su', form, fRows, rowLen, smartUpOverlay(recS, info, 2, 1), false, info);
      }
    }
  }

  rawCands.sort((a, b) => a.size - b.size);
  const topK = [];
  const baseOf = (n) => n.replace('+su2', '').replace('+su1r2', '').replace('+su1', '').replace('+su', '');
  const seenFamily = new Set();

  const isDupRaw = (raw) => {
    for (let i = 0; i < topK.length; i++) {
      if (topK[i].raw.length === raw.length && Buffer.compare(topK[i].raw, raw) === 0) return true;
    }
    return false;
  };

  const addCand = (c) => {
    if (isDupRaw(c.raw)) return false;
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

  // Keep top diverse candidates (preserving high-value recombination & 1-bpp candidates for WASM pre-screen!)
  const bestZ6 = rawCands[0].size;
  const maxK = rawCands[0].form.bpp === 1 ? 4 : 3;
  for (const c of rawCands) {
    if (topK.length >= 2 && c.size > bestZ6 * 1.04) break;
    const fam = baseOf(c.name);
    if (topK.length >= 1 && seenFamily.has(fam)) continue;
    if (addCand(c)) {
      seenFamily.add(fam);
      if (topK.length >= maxK) break;
    }
  }
  if (rawCands[0].form.colourType === 3) {
    for (const must of ['nat_lk8', 'rec_f0_su1', 'rec_f0_su1_8', 'rec_su1_vall+su1', 'f0+su1r2', 'rec_su1_v02']) {
      if (!seenFamily.has(baseOf(must))) {
        const found = rawCands.find((c) => c.name === must);
        if (found && addCand(found)) seenFamily.add(baseOf(must));
      }
    }
  } else if (rawCands[0].form.colourType === 0) {
    const musts = rawCands[0].isPhoto ? ['dualvit1', 'rec_br_v1d', 'rec_br_v8'] : ['rec_g_def_v4f+snap', 'vit4f+snap', 'vit2f+snap'];
    for (const must of musts) {
      const found = rawCands.find((c) => c.name === must);
      if (found && !seenFamily.has(baseOf(must)) && addCand(found)) seenFamily.add(baseOf(must));
    }
  } else {
    const musts = rawCands[0].isPhoto ? ['vdual8+snap', 'rec_dual8_v4f+snap', 'rec_dual8_v4f+snap+su'] : ['rec_v8_v4f+snap'];
    for (const must of musts) {
      const found = rawCands.find((c) => c.name === must);
      if (found && !seenFamily.has(baseOf(must)) && addCand(found)) seenFamily.add(baseOf(must));
    }
  }
  topK.sort((a, b) => a.size - b.size);
  return { cands: topK, best: topK[0], width, height };
}

function encodeCand(c, width, height, settings) {
  let idat = new Uint8Array(c.z.buffer, c.z.byteOffset, c.z.length);
  let outForm = c.form;
  try {
    let palInv = null;
    let verifyRaw = c.raw;
    const palSize = c.form.colourType === 3 && c.form.plte ? (c.form.plte.length / 3) : 0;
    const raw = deflateRaw(c.raw, {
      ...settings,
      bpp: c.form.bpp,
      rowLen: c.rowLen,
      isPhoto: c.isPhoto,
      paletteSize: palSize,
      onPalPerm: palSize >= 4 ? (inv, _fwd, remapped) => { palInv = inv; verifyRaw = remapped; } : null,
    });
    const back = inflateRawSync(raw);
    if (Buffer.compare(back, Buffer.from(verifyRaw.buffer, verifyRaw.byteOffset, verifyRaw.length)) === 0 && raw.length + 6 < idat.length) {
      idat = zlibWrap(raw, verifyRaw);
      if (palInv) {
        const K = palInv.length;
        const nPlte = new Uint8Array(K * 3);
        for (let ni = 0; ni < K; ni++) {
          const oi = palInv[ni];
          nPlte[ni * 3] = c.form.plte[oi * 3];
          nPlte[ni * 3 + 1] = c.form.plte[oi * 3 + 1];
          nPlte[ni * 3 + 2] = c.form.plte[oi * 3 + 2];
        }
        let nTrns = null;
        if (c.form.trns) {
          const oldT = c.form.trns;
          const tmpT = new Uint8Array(K);
          let lastNonOpaque = -1;
          for (let ni = 0; ni < K; ni++) {
            const oi = palInv[ni];
            const a = oi < oldT.length ? oldT[oi] : 255;
            tmpT[ni] = a;
            if (a !== 255) lastNonOpaque = ni;
          }
          if (lastNonOpaque >= 0) nTrns = tmpT.subarray(0, lastNonOpaque + 1);
        }
        outForm = { ...c.form, plte: nPlte, trns: nTrns };
      }
    }
  } catch {
    // Keep verified zlib stream.
  }
  return pngOf(width, height, outForm, idat);
}

// Deterministic byte-counted work budget (never reads wall clock).
const DEF_1BPP_SM = { cut: 512, hc3Cut: 48, hc4Cut: 128, nice: 259, iters: 4, blockIters: 5, huffIters: 2, patience: 2, passes: 2, minSyms: 64,  initChunks: 200, dpCap: 70, blockAltBasin: true };
const DEF_1BPP    = { cut: 256, hc3Cut: 32, hc4Cut: 80,  nice: 259, iters: 3, blockIters: 5, huffIters: 2, patience: 2, passes: 2, minSyms: 128, initChunks: 180, dpCap: 60, blockAltBasin: true };
const DEF_2ND     = { cut: 192, hc3Cut: 16, hc4Cut: 48,  nice: 259, iters: 2, blockIters: 2, huffIters: 1, patience: 1, passes: 1, minSyms: 192, initChunks: 140, dpCap: 40, blockAltBasin: false };
const DEF_NORM    = { cut: 256, hc3Cut: 0,  hc4Cut: 8,   nice: 259, iters: 2, blockIters: 3, huffIters: 2, patience: 1, passes: 2, minSyms: 256, initChunks: 165, dpCap: 56, blockAltBasin: true };
const DEF_PHOTO   = { cut: 224, hc3Cut: 0,  hc4Cut: 16,  nice: 259, iters: 2, blockIters: 2, huffIters: 2, patience: 1, passes: 2, minSyms: 256, initChunks: 165, dpCap: 56, blockAltBasin: true };
const DEF_LIGHT   = { cut: 160, hc3Cut: 0,  hc4Cut: 16,  nice: 259, iters: 2, blockIters: 2, huffIters: 1, patience: 1, passes: 2, minSyms: 256, initChunks: 150, dpCap: 48, blockAltBasin: true };

const BUDGET = 14800;
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

    // Fast WASM pre-screen on 1-bpp non-photo images (eliminates zlib bias on palette & grey hybrids without wasting multi-bpp time)
    let chosen = s.best;
    let runnerUp = null;
    if (!isLight && bpp === 1 && !s.best.isPhoto && s.cands.length > 1) {
      const maxPool = s.best.form.colourType === 3 ? 6 : 4;
      const pool = s.cands.filter((c, i) => i === 0 || (i < maxPool && c.size <= s.best.size * 1.04));
      if (pool.length > 1) {
        const ranked = pool.map((c) => ({
          c,
          est: estimateDeflateBytes(c.raw, c.form.bpp, c.rowLen, c.isPhoto) + chunkOverhead(c.form),
        }));
        ranked.sort((a, b) => a.est - b.est);
        chosen = ranked[0].c;
        const runRatio = chosen.size < 25000 ? 1.0035 : 1.0012;
        if (chosen.size < 45000 && ranked.length > 1 && ranked[1].est <= ranked[0].est * runRatio) {
          runnerUp = ranked[1].c;
        }
      }
    }

    // Centered L6 Post-Filter on the winning non-palette non-photo candidate (one pass, guarded by global z8 check)
    if (!isLight && !chosen.isPhoto && chosen.form.colourType !== 3 && chosen.fRows && chosen.types) {
      const polTypes = centeredPostFilterL6(chosen.fRows, s.height, chosen.rowLen, chosen.types, chosen.form.bpp, chosen.info);
      const polRaw = assembleFromFRows(chosen.fRows, s.height, chosen.rowLen, polTypes);
      const zP = deflateSync(polRaw, Z8);
      if (zP.length + chunkOverhead(chosen.form) < chosen.size) {
        chosen = { ...chosen, raw: polRaw, types: polTypes, z: zP, size: zP.length + chunkOverhead(chosen.form) };
      }
    }

    const settings = isLight
      ? DEF_LIGHT
      : (bpp === 1 ? (chosen.size < 35000 ? DEF_1BPP_SM : DEF_1BPP) : (chosen.isPhoto ? DEF_PHOTO : DEF_NORM));
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
