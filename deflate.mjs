// Full-powered deterministic DEFLATE encoder powered by inline WebAssembly BT3 + O(1) RMQ Exact-258 DP.
// Features:
//  - Deep WASM BT3 match finder + O(1) RMQ Exact-258 DP with exact deep-run fast path
//  - Exact Package-Merge length-limited Huffman (15-bit LL/Dist, 7-bit CL)
//  - Exact shortest-path DP for header RLE (codes 16/17/18) + RLE-aware count smoothing
//  - Dual-basin cost model starts (match-heavy fixed + empirical literal-heavy) + integer Huffman-length polish
//  - Fast O(K) prefix-histogram Bottom-Up Agglomerative Block Merge + Boundary Slide + Changed-Block Pass 2

import { getWasmEngine } from './wasm-core.mjs';

const LEN_BASE = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
const LEN_EXTRA = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DIST_BASE = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
const DIST_EXTRA = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];
const CL_ORDER = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];

const LSYM = new Uint8Array(259);
const LEXT = new Uint8Array(259);
for (let c = 0; c < 29; c++) {
  const end = c < 28 ? LEN_BASE[c + 1] : 259;
  for (let l = LEN_BASE[c]; l < end; l++) {
    LSYM[l] = c;
    LEXT[l] = LEN_EXTRA[c];
  }
}
const DSYM = new Uint8Array(32769);
for (let c = 0; c < 30; c++) {
  const end = c < 29 ? DIST_BASE[c + 1] : 32769;
  for (let d = DIST_BASE[c]; d < end; d++) DSYM[d] = c;
}

const FIXED_LL = new Uint8Array(288);
for (let i = 0; i <= 143; i++) FIXED_LL[i] = 8;
for (let i = 144; i <= 255; i++) FIXED_LL[i] = 9;
for (let i = 256; i <= 279; i++) FIXED_LL[i] = 7;
for (let i = 280; i <= 287; i++) FIXED_LL[i] = 8;
const FIXED_D = new Uint8Array(30).fill(5);

// ---------------------------------------------------------------------------------------------------------------
// Exact Package-Merge length-limited Huffman code lengths.

const pmSyms = new Int32Array(320);
const pmLeafW = new Float64Array(320);
const pmParent = new Int32Array(640);
const pmDepth = new Int32Array(640);
const pmHuffW = new Float64Array(640);
const pmChainW = Array.from({ length: 16 }, () => new Float64Array(600));
const pmChainCnt = Array.from({ length: 16 }, () => new Int32Array(600));
const pmLen = new Int32Array(16);
const pmLeafUses = new Int32Array(320);

export function codeLengths(counts, nsym, maxBits, out) {
  let k = 0;
  for (let s = 0; s < nsym; s++) {
    out[s] = 0;
    if (counts[s] > 0) pmSyms[k++] = s;
  }
  if (k === 0) return;
  if (k === 1) {
    out[pmSyms[0]] = 1;
    out[pmSyms[0] === 0 ? 1 : 0] = 1;
    return;
  }
  for (let a = 1; a < k; a++) {
    const s = pmSyms[a];
    const c = counts[s];
    let b = a - 1;
    while (b >= 0 && (counts[pmSyms[b]] > c || (counts[pmSyms[b]] === c && pmSyms[b] > s))) {
      pmSyms[b + 1] = pmSyms[b];
      b--;
    }
    pmSyms[b + 1] = s;
  }

  for (let a = 0; a < k; a++) pmHuffW[a] = counts[pmSyms[a]];
  let li = 0, ii = k, next = k;
  for (let t = 0; t < k - 1; t++) {
    const x = li < k && (ii >= next || pmHuffW[li] <= pmHuffW[ii]) ? li++ : ii++;
    const y = li < k && (ii >= next || pmHuffW[li] <= pmHuffW[ii]) ? li++ : ii++;
    pmHuffW[next] = pmHuffW[x] + pmHuffW[y];
    pmParent[x] = next;
    pmParent[y] = next;
    next++;
  }
  const root = next - 1;
  pmDepth[root] = 0;
  for (let t = root - 1; t >= k; t--) pmDepth[t] = pmDepth[pmParent[t]] + 1;
  let maxD = 0;
  for (let a = 0; a < k; a++) {
    const d = pmDepth[pmParent[a]] + 1;
    pmDepth[a] = d;
    if (d > maxD) maxD = d;
  }
  if (maxD <= maxBits) {
    for (let a = 0; a < k; a++) out[pmSyms[a]] = pmDepth[a];
    return;
  }

  for (let a = 0; a < k; a++) {
    const w = counts[pmSyms[a]];
    pmLeafW[a] = w;
    pmChainW[0][a] = w;
    pmChainCnt[0][a] = 1;
  }
  pmLen[0] = k;

  for (let lv = 1; lv < maxBits; lv++) {
    const pw = pmChainW[lv - 1];
    const pPairs = pmLen[lv - 1] >>> 1;
    const cw = pmChainW[lv];
    const cc = pmChainCnt[lv];
    let iLeaf = 0, iPkg = 0, outIdx = 0;
    while (iLeaf < k || iPkg < pPairs) {
      const pkgW = iPkg < pPairs ? pw[iPkg * 2] + pw[iPkg * 2 + 1] : Infinity;
      if (iLeaf < k && pmLeafW[iLeaf] <= pkgW) {
        cw[outIdx] = pmLeafW[iLeaf++];
        cc[outIdx++] = 1;
      } else {
        cw[outIdx] = pkgW;
        cc[outIdx++] = 0;
        iPkg++;
      }
    }
    pmLen[lv] = outIdx;
  }

  let take = 2 * k - 2;
  pmLeafUses.fill(0, 0, k);
  for (let lv = maxBits - 1; lv >= 0; lv--) {
    const cc = pmChainCnt[lv];
    let leavesTaken = 0, pkgsTaken = 0;
    for (let i = 0; i < take; i++) {
      if (cc[i] === 1) leavesTaken++;
      else pkgsTaken++;
    }
    for (let i = 0; i < leavesTaken; i++) pmLeafUses[i]++;
    take = pkgsTaken * 2;
  }
  for (let a = 0; a < k; a++) out[pmSyms[a]] = pmLeafUses[a];
}

// ---------------------------------------------------------------------------------------------------------------
// Exact RLE header planner (DP over RLE codes 16/17/18 + 8 variant checks) & RLE-aware count smoothing.

const rleSym = new Uint8Array(320);
const rleExt = new Uint8Array(320);
const clCounts = new Int32Array(19);
const clLens = new Uint8Array(19);
const allLens = new Uint8Array(320);
const bestRleSym = new Uint8Array(320);
const bestRleExt = new Uint8Array(320);

function rleGreedy(lens, n, use16, use17, use18, symOut, extOut) {
  let m = 0, i = 0;
  while (i < n) {
    const v = lens[i];
    let run = 1;
    while (i + run < n && lens[i + run] === v) run++;
    let r = run;
    if (v === 0 && (use17 || use18)) {
      while (r >= 11 && use18) {
        const t = r > 138 ? 138 : r;
        symOut[m] = 18; extOut[m++] = t - 11; r -= t;
      }
      while (r >= 3 && use17) {
        const t = r > 10 ? 10 : r;
        symOut[m] = 17; extOut[m++] = t - 3; r -= t;
      }
      while (r > 0) { symOut[m++] = 0; r--; }
    } else if (use16 && r >= 4) {
      symOut[m++] = v; r--;
      while (r >= 3) {
        const t = r > 6 ? 6 : r;
        symOut[m] = 16; extOut[m++] = t - 3; r -= t;
      }
      while (r > 0) { symOut[m++] = v; r--; }
    } else {
      while (r > 0) { symOut[m++] = v; r--; }
    }
    i += run;
  }
  return m;
}

const rdpCost = new Float64Array(321);
const rdpStep = new Uint8Array(321);
const rdpSym = new Uint8Array(321);
const rdpExt = new Uint8Array(321);

function rleDP(lens, n, clCost, symOut, extOut) {
  rdpCost[0] = 0;
  for (let i = 1; i <= n; i++) rdpCost[i] = Infinity;
  for (let i = 0; i < n; i++) {
    const c = rdpCost[i];
    if (c === Infinity) continue;
    const v = lens[i];
    const cl = c + clCost[v];
    if (cl < rdpCost[i + 1]) {
      rdpCost[i + 1] = cl; rdpStep[i + 1] = 1; rdpSym[i + 1] = v; rdpExt[i + 1] = 0;
    }
    if (i > 0 && lens[i - 1] === v) {
      const c16 = c + clCost[16] + 2;
      let run = 0;
      while (run < 6 && i + run < n && lens[i + run] === v) run++;
      for (let t = 3; t <= run; t++) {
        if (c16 < rdpCost[i + t]) {
          rdpCost[i + t] = c16; rdpStep[i + t] = t; rdpSym[i + t] = 16; rdpExt[i + t] = t - 3;
        }
      }
    }
    if (v === 0) {
      let run = 0;
      while (run < 138 && i + run < n && lens[i + run] === 0) run++;
      if (run >= 3) {
        const c17 = c + clCost[17] + 3;
        const max17 = run < 10 ? run : 10;
        for (let t = 3; t <= max17; t++) {
          if (c17 < rdpCost[i + t]) {
            rdpCost[i + t] = c17; rdpStep[i + t] = t; rdpSym[i + t] = 17; rdpExt[i + t] = t - 3;
          }
        }
      }
      if (run >= 11) {
        const c18 = c + clCost[18] + 7;
        for (let t = 11; t <= run; t++) {
          if (c18 < rdpCost[i + t]) {
            rdpCost[i + t] = c18; rdpStep[i + t] = t; rdpSym[i + t] = 18; rdpExt[i + t] = t - 11;
          }
        }
      }
    }
  }
  let cnt = 0;
  for (let i = n; i > 0; i -= rdpStep[i]) cnt++;
  let j = cnt;
  for (let i = n; i > 0; ) {
    j--;
    symOut[j] = rdpSym[i];
    extOut[j] = rdpExt[i];
    i -= rdpStep[i];
  }
  return cnt;
}

function scoreRle(sym, m) {
  clCounts.fill(0);
  for (let j = 0; j < m; j++) clCounts[sym[j]]++;
  codeLengths(clCounts, 19, 7, clLens);
  let hclen = 19;
  while (hclen > 4 && clLens[CL_ORDER[hclen - 1]] === 0) hclen--;
  let bits = 14 + 3 * hclen;
  for (let s = 0; s < 19; s++) bits += clCounts[s] * clLens[s];
  bits += clCounts[16] * 2 + clCounts[17] * 3 + clCounts[18] * 7;
  return bits;
}

const clCostBuf = new Float64Array(19);
const tmpRleSym = new Uint8Array(320);
const tmpRleExt = new Uint8Array(320);

function headerPlan(llLens, dLens, full = false) {
  let hlit = 286;
  while (hlit > 257 && llLens[hlit - 1] === 0) hlit--;
  let hdist = 30;
  while (hdist > 1 && dLens[hdist - 1] === 0) hdist--;
  for (let i = 0; i < hlit; i++) allLens[i] = llLens[i];
  for (let i = 0; i < hdist; i++) allLens[hlit + i] = dLens[i];
  const n = hlit + hdist;

  let bestBits = Infinity;
  let bestM = 0;
  for (let v = 0; v < 8; v++) {
    const m = rleGreedy(allLens, n, v & 1, v & 2, v & 4, rleSym, rleExt);
    const bits = scoreRle(rleSym, m);
    if (bits < bestBits) {
      bestBits = bits;
      bestM = m;
      if (full) {
        bestRleSym.set(rleSym.subarray(0, m));
        bestRleExt.set(rleExt.subarray(0, m));
      }
    }
  }
  if (full) {
    scoreRle(bestRleSym, bestM);
    for (let it = 0; it < 2; it++) {
      for (let s = 0; s < 19; s++) clCostBuf[s] = clLens[s] > 0 ? clLens[s] : 8;
      const m = rleDP(allLens, n, clCostBuf, tmpRleSym, tmpRleExt);
      const bits = scoreRle(tmpRleSym, m);
      if (bits < bestBits) {
        bestBits = bits;
        bestM = m;
        bestRleSym.set(tmpRleSym.subarray(0, m));
        bestRleExt.set(tmpRleExt.subarray(0, m));
      } else break;
    }
  }
  return { bits: bestBits, hlit, hdist, m: bestM };
}

function optimizeCountsForRle(counts, length, out) {
  out.set(counts.subarray(0, length));
  while (length > 0 && out[length - 1] === 0) length--;
  if (length === 0) return;
  const good = new Uint8Array(length);
  let sym = out[0], stride = 0;
  for (let i = 0; i < length + 1; i++) {
    if (i === length || out[i] !== sym) {
      if ((sym === 0 && stride >= 5) || (sym !== 0 && stride >= 7)) {
        for (let k = 0; k < stride; k++) good[i - k - 1] = 1;
      }
      stride = 1;
      if (i !== length) sym = out[i];
    } else stride++;
  }
  stride = 0;
  let limit = out[0], sum = 0;
  for (let i = 0; i < length + 1; i++) {
    if (i === length || good[i] || (out[i] === 0 && limit !== 0) || (out[i] !== 0 && limit === 0) || Math.abs(out[i] - limit) >= 4) {
      if (stride >= 4 || (stride >= 3 && sum === 0)) {
        let count = Math.floor((sum + stride / 2) / stride);
        if (count < 1) count = 1;
        if (sum === 0) count = 0;
        for (let k = 0; k < stride; k++) out[i - k - 1] = count;
      }
      stride = 0;
      sum = 0;
      if (i < length - 3) limit = Math.floor((out[i] + out[i + 1] + out[i + 2] + out[i + 3] + 2) / 4);
      else if (i < length) limit = out[i];
      else limit = 0;
    }
    stride++;
    if (i !== length) {
      sum += out[i];
      if (stride >= 4) limit = Math.floor((sum + stride / 2) / stride);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Symbol histograms & exact block bit calculation.

const llC = new Int32Array(288);
const dC = new Int32Array(30);
const llL = new Uint8Array(288);
const dL = new Uint8Array(30);
const smLL = new Int32Array(288);
const smD = new Int32Array(30);
const smLLL = new Uint8Array(288);
const smDL = new Uint8Array(30);

function histogram(lit, dist, a, b, llOut = llC, dOut = dC) {
  llOut.fill(0);
  dOut.fill(0);
  let extra = 0;
  for (let i = a; i < b; i++) {
    const d = dist[i];
    if (d === 0) llOut[lit[i]]++;
    else {
      const l = lit[i];
      llOut[257 + LSYM[l]]++;
      extra += LEXT[l];
      const dc = DSYM[d];
      dOut[dc]++;
      extra += DIST_EXTRA[dc];
    }
  }
  llOut[256] = 1;
  return extra;
}

function fixDistLens(dl) {
  let used = 0;
  for (let i = 0; i < 30; i++) if (dl[i] !== 0) used++;
  if (used === 0) { dl[0] = 1; dl[1] = 1; }
  else if (used === 1) { if (dl[0] === 0) dl[0] = 1; else dl[1] = 1; }
}

function evalDynamicBits(llCounts, dCounts, llLensOut, dLensOut, extra, fullHeader) {
  let bits = 3 + extra;
  for (let s = 0; s < 288; s++) bits += llCounts[s] * llLensOut[s];
  for (let s = 0; s < 30; s++) bits += dCounts[s] * dLensOut[s];
  return bits + headerPlan(llLensOut, dLensOut, fullHeader).bits;
}

function evalFixedBits(llCounts, dCounts, extra) {
  let bits = 3 + extra;
  for (let s = 0; s < 288; s++) bits += llCounts[s] * FIXED_LL[s];
  for (let s = 0; s < 30; s++) bits += dCounts[s] * 5;
  return bits;
}

function bitsFromHist(llCounts, dCounts, extra, trySmooth = false) {
  codeLengths(llCounts, 288, 15, llL);
  codeLengths(dCounts, 30, 15, dL);
  fixDistLens(dL);
  let best = evalDynamicBits(llCounts, dCounts, llL, dL, extra, false);
  const fx = evalFixedBits(llCounts, dCounts, extra);
  if (fx < best) best = fx;
  if (trySmooth) {
    optimizeCountsForRle(llCounts, 286, smLL);
    optimizeCountsForRle(dCounts, 30, smD);
    smLL[256] = 1;
    codeLengths(smLL, 288, 15, smLLL);
    codeLengths(smD, 30, 15, smDL);
    fixDistLens(smDL);
    let ok = true;
    for (let s = 0; s < 288; s++) if (llCounts[s] > 0 && smLLL[s] === 0) { ok = false; break; }
    if (ok) for (let s = 0; s < 30; s++) if (dCounts[s] > 0 && smDL[s] === 0) { ok = false; break; }
    if (ok) {
      const sb = evalDynamicBits(llCounts, dCounts, smLLL, smDL, extra, false);
      if (sb < best) best = sb;
    }
  }
  return best;
}

export function blockBits(lit, dist, a, b, trySmooth = false) {
  const extra = histogram(lit, dist, a, b);
  return bitsFromHist(llC, dC, extra, trySmooth);
}

// ---------------------------------------------------------------------------------------------------------------
// Cost models (1/1024-bit integer scaled for WASM parseRMQ).

const SCALE = 1024;
const litCostI32 = new Int32Array(256);
const lenCostI32 = new Int32Array(32);
const distCostI32 = new Int32Array(32);

function costsFromCounts(ll, d) {
  let tl = 0;
  for (let s = 0; s < 286; s++) tl += ll[s];
  let td = 0;
  for (let s = 0; s < 30; s++) td += d[s];
  const lgl = tl > 0 ? Math.log2(tl) : 8;
  const lgd = td > 0 ? Math.log2(td) : 5;
  for (let s = 0; s < 256; s++) {
    const bits = Math.min(15.25, ll[s] > 0 ? lgl - Math.log2(ll[s]) : lgl + 1.25);
    litCostI32[s] = Math.max(1, Math.round(bits * SCALE));
  }
  for (let c = 0; c < 29; c++) {
    const cnt = ll[257 + c];
    const bits = Math.min(15.25, cnt > 0 ? lgl - Math.log2(cnt) : lgl + 1.25) + LEN_EXTRA[c];
    lenCostI32[c] = Math.max(1, Math.round(bits * SCALE));
  }
  for (let c = 0; c < 30; c++) {
    const cnt = d[c];
    const bits = Math.min(15.25, cnt > 0 ? lgd - Math.log2(cnt) : lgd + 1.25) + DIST_EXTRA[c];
    distCostI32[c] = Math.max(1, Math.round(bits * SCALE));
  }
}

function costsFromHuffCounts(ll, d) {
  codeLengths(ll, 288, 15, llL);
  codeLengths(d, 30, 15, dL);
  fixDistLens(dL);
  for (let s = 0; s < 256; s++) litCostI32[s] = (llL[s] > 0 ? llL[s] : 15) * SCALE;
  for (let c = 0; c < 29; c++) lenCostI32[c] = ((llL[257 + c] > 0 ? llL[257 + c] : 15) + LEN_EXTRA[c]) * SCALE;
  for (let c = 0; c < 30; c++) distCostI32[c] = ((dL[c] > 0 ? dL[c] : 15) + DIST_EXTRA[c]) * SCALE;
}

function costsFromFixed() {
  for (let s = 0; s < 256; s++) litCostI32[s] = FIXED_LL[s] * SCALE;
  for (let c = 0; c < 29; c++) lenCostI32[c] = (FIXED_LL[257 + c] + LEN_EXTRA[c]) * SCALE;
  for (let c = 0; c < 30; c++) distCostI32[c] = (5 + DIST_EXTRA[c]) * SCALE;
}

function costsFromLitHeavy(data, s, e) {
  const bc = new Int32Array(256);
  for (let i = s; i < e; i++) bc[data[i]]++;
  const nonZ = Math.max(1, (e - s) - bc[0]);
  const lg = Math.log2(nonZ);
  for (let b = 0; b < 256; b++) {
    const bits = b === 0 ? 2 : (bc[b] > 0 ? Math.min(15, lg - Math.log2(bc[b]) + 1) : 15);
    litCostI32[b] = Math.max(1, Math.round(bits * SCALE));
  }
  for (let c = 0; c < 29; c++) lenCostI32[c] = (12 + LEN_EXTRA[c]) * SCALE;
  for (let c = 0; c < 30; c++) distCostI32[c] = (8 + DIST_EXTRA[c]) * SCALE;
}

// ---------------------------------------------------------------------------------------------------------------
// Fast Bottom-Up Agglomerative Block Merge + Boundary Slide (solves periodic A-B-A-B layouts in ~15ms!).

function bytesOf(lit, dist, a, b) {
  let n = 0;
  for (let i = a; i < b; i++) n += dist[i] === 0 ? 1 : lit[i];
  return n;
}

const combLL = new Int32Array(288);
const combD = new Int32Array(30);

function combinedHistBits(s1, s2) {
  for (let k = 0; k < 288; k++) combLL[k] = s1.ll[k] + s2.ll[k];
  combLL[256] = 1;
  for (let k = 0; k < 30; k++) combD[k] = s1.d[k] + s2.d[k];
  return bitsFromHist(combLL, combD, s1.extra + s2.extra, false);
}

function splitBottomUp(lit, dist, m, minSyms = 256, maxBlocks = 200, initChunks = 180) {
  if (m < 2 * minSyms) return [];
  const K = Math.min(initChunks, Math.max(2, Math.floor(m / minSyms)));
  const segs = [];
  for (let i = 0; i < K; i++) {
    const a = Math.round((i * m) / K);
    const b = Math.round(((i + 1) * m) / K);
    if (b <= a) continue;
    const ll = new Int32Array(288);
    const d = new Int32Array(30);
    const extra = histogram(lit, dist, a, b, ll, d);
    const bits = bitsFromHist(ll, d, extra, false);
    segs.push({ a, b, ll, d, extra, bits });
  }

  // O(K) histogram-based agglomerative merge!
  const gains = [];
  for (let i = 0; i + 1 < segs.length; i++) {
    gains.push(segs[i].bits + segs[i + 1].bits - combinedHistBits(segs[i], segs[i + 1]));
  }
  while (segs.length > 1) {
    let bestI = -1, bestG = -Infinity;
    for (let i = 0; i < gains.length; i++) {
      if (gains[i] > bestG) { bestG = gains[i]; bestI = i; }
    }
    if (bestI < 0 || (bestG <= 0 && segs.length <= maxBlocks)) break;
    const s1 = segs[bestI], s2 = segs[bestI + 1];
    for (let k = 0; k < 288; k++) s1.ll[k] += s2.ll[k];
    s1.ll[256] = 1;
    for (let k = 0; k < 30; k++) s1.d[k] += s2.d[k];
    s1.extra += s2.extra;
    s1.b = s2.b;
    s1.bits = s1.bits + s2.bits - bestG;
    segs.splice(bestI + 1, 1);
    gains.splice(bestI, 1);
    if (bestI > 0) gains[bestI - 1] = segs[bestI - 1].bits + segs[bestI].bits - combinedHistBits(segs[bestI - 1], segs[bestI]);
    if (bestI < segs.length - 1) gains[bestI] = segs[bestI].bits + segs[bestI + 1].bits - combinedHistBits(segs[bestI], segs[bestI + 1]);
  }

  // Fast incremental-histogram boundary sliding (1 linear scan per narrowing round instead of 18!)
  const lLL = new Int32Array(288), lD = new Int32Array(30);
  const rLL = new Int32Array(288), rD = new Int32Array(30);
  const sLL = new Int32Array(288), sD = new Int32Array(30);

  for (let i = 0; i + 1 < segs.length; i++) {
    const x = segs[i].a, y = segs[i + 1].b;
    let lo = x + minSyms, hi = y - minSyms;
    if (hi <= lo) continue;
    let bestP = segs[i].b;
    let bestF = segs[i].bits + segs[i + 1].bits;
    // Build base left [x, lo) and right [lo, y) histograms once!
    let baseLEx = histogram(lit, dist, x, lo, lLL, lD);
    let baseREx = histogram(lit, dist, lo, y, rLL, rD);
    let baseLo = lo;

    while (hi - lo > 0) {
      // Advance baseLo -> lo incrementally if lo moved right
      while (baseLo < lo) {
        const d = dist[baseLo];
        if (d === 0) { const v = lit[baseLo]; lLL[v]++; rLL[v]--; }
        else {
          const l = lit[baseLo], lc = LSYM[l], dc = DSYM[d], ex = LEXT[l] + DIST_EXTRA[dc];
          lLL[257 + lc]++; rLL[257 + lc]--; lD[dc]++; rD[dc]--;
          baseLEx += ex; baseREx -= ex;
        }
        baseLo++;
      }
      // Copy running histograms for this monotonic sweep lo -> hi
      sLL.set(lLL); sD.set(lD);
      const tRLL = new Int32Array(rLL), tRD = new Int32Array(rD);
      let curLEx = baseLEx, curREx = baseREx;
      let curP = lo;
      let localBest = bestP, localF = bestF;
      const step = (hi - lo) / 10;
      for (let t = 1; t <= 9; t++) {
        const p = Math.round(lo + step * t);
        if (p <= curP || p >= y) continue;
        while (curP < p) {
          const d = dist[curP];
          if (d === 0) { const v = lit[curP]; sLL[v]++; tRLL[v]--; }
          else {
            const l = lit[curP], lc = LSYM[l], dc = DSYM[d], ex = LEXT[l] + DIST_EXTRA[dc];
            sLL[257 + lc]++; tRLL[257 + lc]--; sD[dc]++; tRD[dc]--;
            curLEx += ex; curREx -= ex;
          }
          curP++;
        }
        const f = bitsFromHist(sLL, sD, curLEx, false) + bitsFromHist(tRLL, tRD, curREx, false);
        if (f < localF) { localF = f; localBest = p; }
      }
      if (localF < bestF) { bestF = localF; bestP = localBest; }
      if (step < 1) break;
      const nlo = Math.max(lo, Math.round(localBest - step));
      const nhi = Math.min(hi, Math.round(localBest + step));
      if (nhi - nlo < 4 || (nlo === lo && nhi === hi)) break;
      lo = nlo; hi = nhi;
    }
    if (bestP !== segs[i].b) {
      segs[i].b = bestP; segs[i].bits = blockBits(lit, dist, x, bestP, false);
      segs[i + 1].a = bestP; segs[i + 1].bits = blockBits(lit, dist, bestP, y, false);
    }
  }
  return segs.map((s) => s.a).slice(1);
}

// ---------------------------------------------------------------------------------------------------------------
// Bit writer & block emitter.

class Bits {
  constructor(cap) {
    this.buf = new Uint8Array(cap);
    this.pos = 0;
    this.acc = 0;
    this.n = 0;
  }
  put(v, nb) {
    this.acc |= v << this.n;
    this.n += nb;
    while (this.n >= 8) {
      if (this.pos >= this.buf.length) {
        const nb2 = new Uint8Array(this.buf.length * 2);
        nb2.set(this.buf);
        this.buf = nb2;
      }
      this.buf[this.pos++] = this.acc & 0xff;
      this.acc >>>= 8;
      this.n -= 8;
    }
  }
  alignByte() { if (this.n > 0) this.put(0, 8 - this.n); }
  finish() { this.alignByte(); return this.buf.subarray(0, this.pos); }
}

function canonicalCodes(lens, nsym) {
  const blCount = new Int32Array(16);
  for (let i = 0; i < nsym; i++) blCount[lens[i]]++;
  blCount[0] = 0;
  const nextCode = new Int32Array(16);
  let code = 0;
  for (let b = 1; b <= 15; b++) { code = (code + blCount[b - 1]) << 1; nextCode[b] = code; }
  const rev = new Uint16Array(nsym);
  for (let s = 0; s < nsym; s++) {
    const l = lens[s];
    if (l === 0) continue;
    let c = nextCode[l]++, r = 0;
    for (let i = 0; i < l; i++) { r = (r << 1) | (c & 1); c >>>= 1; }
    rev[s] = r;
  }
  return rev;
}

function writeSymbols(out, lit, dist, a, b, llLens, dLens) {
  const llCode = canonicalCodes(llLens, 288);
  const dCode = canonicalCodes(dLens, 30);
  for (let i = a; i < b; i++) {
    const d = dist[i];
    if (d === 0) {
      const s = lit[i];
      out.put(llCode[s], llLens[s]);
    } else {
      const l = lit[i];
      const lc = LSYM[l];
      out.put(llCode[257 + lc], llLens[257 + lc]);
      if (LEN_EXTRA[lc] > 0) out.put(l - LEN_BASE[lc], LEN_EXTRA[lc]);
      const dc = DSYM[d];
      out.put(dCode[dc], dLens[dc]);
      if (DIST_EXTRA[dc] > 0) out.put(d - DIST_BASE[dc], DIST_EXTRA[dc]);
    }
  }
  out.put(llCode[256], llLens[256]);
}

function writeBlock(out, data, byteS, byteE, lit, dist, a, b, isFinal) {
  const extra = histogram(lit, dist, a, b);
  codeLengths(llC, 288, 15, llL);
  codeLengths(dC, 30, 15, dL);
  fixDistLens(dL);

  let bestMode = 2;
  let bestBits = evalDynamicBits(llC, dC, llL, dL, extra, true);
  let useLL = llL, useD = dL;

  optimizeCountsForRle(llC, 286, smLL);
  optimizeCountsForRle(dC, 30, smD);
  smLL[256] = 1;
  codeLengths(smLL, 288, 15, smLLL);
  codeLengths(smD, 30, 15, smDL);
  fixDistLens(smDL);
  let ok = true;
  for (let s = 0; s < 288; s++) if (llC[s] > 0 && smLLL[s] === 0) { ok = false; break; }
  if (ok) for (let s = 0; s < 30; s++) if (dC[s] > 0 && smDL[s] === 0) { ok = false; break; }
  if (ok) {
    const dynSm = evalDynamicBits(llC, dC, smLLL, smDL, extra, true);
    if (dynSm < bestBits) { bestBits = dynSm; bestMode = 3; useLL = smLLL; useD = smDL; }
  }

  const fixBits = evalFixedBits(llC, dC, extra);
  if (fixBits < bestBits) { bestBits = fixBits; bestMode = 1; }

  const byteLen = byteE - byteS;
  if (byteLen <= 65535) {
    const pad = (8 - ((out.n + 3) & 7)) & 7;
    if (3 + pad + 32 + byteLen * 8 < bestBits) bestMode = 0;
  }

  if (bestMode === 0) {
    out.put(isFinal ? 1 : 0, 1); out.put(0, 2); out.alignByte();
    out.put(byteLen & 0xffff, 16); out.put((~byteLen) & 0xffff, 16);
    for (let i = byteS; i < byteE; i++) out.put(data[i], 8);
    return;
  }
  if (bestMode === 1) {
    out.put(isFinal ? 1 : 0, 1); out.put(1, 2);
    writeSymbols(out, lit, dist, a, b, FIXED_LL, FIXED_D);
    return;
  }

  const plan = headerPlan(useLL, useD, true);
  scoreRle(bestRleSym, plan.m);
  let hclen = 19;
  while (hclen > 4 && clLens[CL_ORDER[hclen - 1]] === 0) hclen--;
  const clCode = canonicalCodes(clLens, 19);

  out.put(isFinal ? 1 : 0, 1); out.put(2, 2);
  out.put(plan.hlit - 257, 5); out.put(plan.hdist - 1, 5); out.put(hclen - 4, 4);
  for (let i = 0; i < hclen; i++) out.put(clLens[CL_ORDER[i]], 3);
  for (let j = 0; j < plan.m; j++) {
    const s = bestRleSym[j];
    out.put(clCode[s], clLens[s]);
    if (s === 16) out.put(bestRleExt[j], 2);
    else if (s === 17) out.put(bestRleExt[j], 3);
    else if (s === 18) out.put(bestRleExt[j], 7);
  }
  writeSymbols(out, lit, dist, a, b, useLL, useD);
}

// ---------------------------------------------------------------------------------------------------------------
// Iterative optimal parsing & multi-basin orchestration.

const mixLL = new Int32Array(288);
const mixD = new Int32Array(30);

function iterateRange(eng, L, s, e, bestL, bestD, bestM, iters, huffIters, patience, tmpA_L, tmpA_D, tmpB_L, tmpB_D) {
  let bestBits = blockBits(bestL, bestD, 0, bestM, false);
  let curL = bestL, curD = bestD, curM = bestM;
  let tL = tmpA_L, tD = tmpA_D, oL = tmpB_L, oD = tmpB_D;
  let fails = 0;

  for (let it = 0; it < iters; it++) {
    histogram(curL, curD, 0, curM);
    if (fails > 0) {
      mixLL.set(llC); mixD.set(dC);
      histogram(bestL, bestD, 0, bestM);
      for (let k = 0; k < 288; k++) mixLL[k] += llC[k];
      for (let k = 0; k < 30; k++) mixD[k] += dC[k];
      costsFromCounts(mixLL, mixD);
    } else {
      costsFromCounts(llC, dC);
    }
    const m2 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, tL, tD, 0);
    const bits = blockBits(tL, tD, 0, m2, false);
    if (bits < bestBits) {
      bestBits = bits;
      bestL.set(tL.subarray(0, m2));
      bestD.set(tD.subarray(0, m2));
      bestM = m2;
      fails = 0;
    } else if (++fails >= patience) {
      break;
    }
    curL = tL; curD = tD; curM = m2;
    [tL, oL] = [oL, tL];
    [tD, oD] = [oD, tD];
  }

  for (let hit = 0; hit < huffIters; hit++) {
    histogram(bestL, bestD, 0, bestM);
    costsFromHuffCounts(llC, dC);
    const m2 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, tL, tD, 0);
    const bits = blockBits(tL, tD, 0, m2, false);
    if (bits < bestBits) {
      bestBits = bits;
      bestL.set(tL.subarray(0, m2));
      bestD.set(tD.subarray(0, m2));
      bestM = m2;
    } else break;
  }
  return { m: bestM, bits: bestBits };
}

function splitAndOptimize(eng, L, data, baseL, baseD, baseM, prevByteMap, minSyms, maxBlocks, initChunks, blockIters, huffIters, patience, blockAltBasin, tmpA_L, tmpA_D, tmpB_L, tmpB_D) {
  const n = data.length;
  const sp = splitBottomUp(baseL, baseD, baseM, minSyms, maxBlocks, initChunks);
  const symB = [0, ...sp, baseM];
  const outL = new Uint16Array(n + 1);
  const outD = new Uint16Array(n + 1);
  const starts = [0];
  const byteStarts = [0];
  const byteMap = new Map();
  let w = 0, pos = 0, totalBits = 0;

  for (let b = 0; b + 1 < symB.length; b++) {
    const a = symB[b], z = symB[b + 1];
    const s = pos;
    const e = s + bytesOf(baseL, baseD, a, z);
    const key = s + ':' + e;
    const cached = prevByteMap ? prevByteMap.get(key) : null;
    let r;
    if (cached) {
      // Unchanged block boundary in pass 2: reuse already-converged optimal parse!
      outL.set(cached.lit, w);
      outD.set(cached.dist, w);
      w += cached.lit.length;
      r = { m: cached.lit.length, bits: cached.bits };
      byteMap.set(key, cached);
    } else {
      const bl = new Uint16Array(e - s + 1);
      const bd = new Uint16Array(e - s + 1);
      bl.set(baseL.subarray(a, z));
      bd.set(baseD.subarray(a, z));
      r = iterateRange(eng, L, s, e, bl, bd, z - a, blockIters, huffIters, patience, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
      // Only probe litheavy alt basin on substantial literal-rich blocks!
      if (blockAltBasin && (z - a) >= 4000) {
        let lits = 0;
        for (let i = 0; i < r.m; i++) if (bd[i] === 0) lits++;
        if (lits > r.m * 0.45) {
          costsFromLitHeavy(data, s, e);
          const bl2 = new Uint16Array(e - s + 1);
          const bd2 = new Uint16Array(e - s + 1);
          const m0 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, bl2, bd2, 0);
          const r2 = iterateRange(eng, L, s, e, bl2, bd2, m0, Math.max(2, blockIters - 1), huffIters, 1, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
          if (r2.bits < r.bits) {
            r = r2;
            bl.set(bl2.subarray(0, r2.m));
            bd.set(bd2.subarray(0, r2.m));
          }
        }
      }
      const cl = bl.slice(0, r.m), cd = bd.slice(0, r.m);
      outL.set(cl, w);
      outD.set(cd, w);
      w += r.m;
      byteMap.set(key, { lit: cl, dist: cd, bits: r.bits });
    }
    starts.push(w);
    byteStarts.push(e);
    totalBits += r.bits;
    pos = e;
  }

  // Final adjacent merge check (with RLE-smoothed dynamic bits)
  let merged = true;
  while (merged && starts.length > 2) {
    merged = false;
    for (let b = 0; b + 2 < starts.length; b++) {
      const a0 = starts[b], a1 = starts[b + 1], a2 = starts[b + 2];
      const sep = blockBits(outL, outD, a0, a1, true) + blockBits(outL, outD, a1, a2, true);
      const comb = blockBits(outL, outD, a0, a2, true);
      if (comb <= sep) {
        starts.splice(b + 1, 1);
        byteStarts.splice(b + 1, 1);
        totalBits -= (sep - comb);
        merged = true;
        break;
      }
    }
  }

  return { lit: outL, dist: outD, starts, byteStarts, bits: totalBits, byteMap };
}

export function deflateRawWasm(data, opts = {}) {
  const n = data.length;
  const cut = opts.cut ?? 384;
  const nice = opts.nice ?? 259;
  const iters = opts.iters ?? 3;
  const blockIters = opts.blockIters ?? 4;
  const huffIters = opts.huffIters ?? 2;
  const patience = opts.patience ?? 1;
  const maxBlocks = opts.maxBlocks ?? 200;
  const minSyms = opts.minSyms ?? 256;
  const initChunks = opts.initChunks ?? 180;
  const passes = opts.passes ?? 2;
  const basins = opts.basins ?? ['fixed', 'litheavy'];
  const blockAltBasin = opts.blockAltBasin ?? true;

  const eng = getWasmEngine();
  const L = eng.prepareMatches(data, cut, nice);

  const gL = new Uint16Array(n + 1);
  const gD = new Uint16Array(n + 1);
  const candL = new Uint16Array(n + 1);
  const candD = new Uint16Array(n + 1);
  const tmpA_L = new Uint16Array(n + 1);
  const tmpA_D = new Uint16Array(n + 1);
  const tmpB_L = new Uint16Array(n + 1);
  const tmpB_D = new Uint16Array(n + 1);

  let bestM = 0;
  let bestGlobalBits = Infinity;

  for (const basin of basins) {
    if (basin === 'fixed') costsFromFixed();
    else if (basin === 'litheavy') costsFromLitHeavy(data, 0, n);
    const m0 = eng.parseRange(L, 0, n, litCostI32, lenCostI32, distCostI32, candL, candD, 0);
    const gHuff = basin === 'litheavy' ? 1 : 0;
    const r = iterateRange(eng, L, 0, n, candL, candD, m0, iters, gHuff, patience, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
    if (r.bits < bestGlobalBits) {
      bestGlobalBits = r.bits;
      bestM = r.m;
      gL.set(candL.subarray(0, r.m));
      gD.set(candD.subarray(0, r.m));
    }
  }

  let best = splitAndOptimize(eng, L, data, gL, gD, bestM, null, minSyms, maxBlocks, initChunks, blockIters, huffIters, patience, blockAltBasin, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
  for (let p = 1; p < passes; p++) {
    const p2Iters = Math.max(1, Math.min(2, blockIters - 1));
    const p2Huff = Math.max(1, huffIters - 1);
    const next = splitAndOptimize(eng, L, data, best.lit, best.dist, best.starts[best.starts.length - 1], best.byteMap, minSyms, maxBlocks, initChunks, p2Iters, p2Huff, 1, false, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
    if (next.bits < best.bits) best = next;
    else break;
  }

  const out = new Bits(Math.max(1024, n >>> 3));
  const st = best.starts;
  const bst = best.byteStarts;
  for (let b = 0; b + 1 < st.length; b++) {
    writeBlock(out, data, bst[b], bst[b + 1], best.lit, best.dist, st[b], st[b + 1], b + 2 === st.length);
  }
  return out.finish();
}

export { deflateRawWasm as deflateRaw };

export function adler32(bytes) {
  let a = 1, b = 0;
  for (let i = 0; i < bytes.length; ) {
    const end = Math.min(bytes.length, i + 5552);
    for (; i < end; i++) { a += bytes[i]; b += a; }
    a %= 65521; b %= 65521;
  }
  return ((b << 16) | a) >>> 0;
}

export function zlibWrap(raw, data) {
  const out = new Uint8Array(raw.length + 6);
  out[0] = 0x78; out[1] = 0xda;
  out.set(raw, 2);
  const ad = adler32(data);
  const at = raw.length + 2;
  out[at] = ad >>> 24;
  out[at + 1] = (ad >>> 16) & 0xff;
  out[at + 2] = (ad >>> 8) & 0xff;
  out[at + 3] = ad & 0xff;
  return out;
}
