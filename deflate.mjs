// Full-powered deterministic DEFLATE encoder powered by inline WebAssembly BT3+Stride + O(1) RMQ Exact-258 DP.
// Features:
//  - Deep WASM BT3 match finder + explicit pixel-stride (bpp) and row-stride (rowLen) match injection
//  - Exact non-monotonic distCost handling in O(1) RMQ parseRMQ + optimal skipDist (1..4) deep-run jump
//  - Iterative distance-basin optimization (collapses dist=1 into dist=bpp / rowLen when cheaper)
//  - Exact Package-Merge length-limited Huffman (15-bit LL/Dist, 7-bit CL)
//  - Exact shortest-path DP for header RLE (codes 16/17/18) + multi-threshold RLE count smoothing
//  - Hybrid Filter-Transition-Aware Bottom-Up -> Exact O(M^2) DP -> Incremental-Histogram Boundary Slide block split

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

  if (!full) {
    const m = rleGreedy(allLens, n, 1, 1, 1, rleSym, rleExt);
    return { bits: scoreRle(rleSym, m), hlit, hdist, m };
  }

  let bestBits = Infinity;
  let bestM = 0;
  for (let v = 0; v < 8; v++) {
    const m = rleGreedy(allLens, n, v & 1, v & 2, v & 4, rleSym, rleExt);
    const bits = scoreRle(rleSym, m);
    if (bits < bestBits) {
      bestBits = bits;
      bestM = m;
      bestRleSym.set(rleSym.subarray(0, m));
      bestRleExt.set(rleExt.subarray(0, m));
    }
  }
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
  return { bits: bestBits, hlit, hdist, m: bestM };
}

function optimizeCountsForRle(counts, length, out, diffThresh = 4) {
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
    if (i === length || good[i] || (out[i] === 0 && limit !== 0) || (out[i] !== 0 && limit === 0) || Math.abs(out[i] - limit) >= diffThresh) {
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
    for (const th of [4, 3, 2, 5]) {
      optimizeCountsForRle(llCounts, 286, smLL, th);
      optimizeCountsForRle(dCounts, 30, smD, th);
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
  }
  return best;
}

export function blockBits(lit, dist, a, b, trySmooth = false) {
  const extra = histogram(lit, dist, a, b);
  return bitsFromHist(llC, dC, extra, trySmooth);
}

// ---------------------------------------------------------------------------------------------------------------
// Distance-basin re-optimization (fast in-place check over pixel/row stride candidates).

function buildTryDists(bpp, rowLen) {
  const arr = [1, 2, 3, 4];
  if (bpp > 1) { arr.push(bpp, bpp * 2); }
  else { arr.push(8); }
  if (rowLen >= 3 && rowLen <= 32768) {
    arr.push(rowLen);
    if (rowLen - bpp >= 1) arr.push(rowLen - bpp);
    if (rowLen + bpp <= 32768) arr.push(rowLen + bpp);
    for (let k = 2; k <= 8; k++) {
      if (rowLen * k <= 32768) arr.push(rowLen * k);
    }
  }
  const uniq = [];
  for (const d of arr) if (d >= 1 && d <= 32768 && !uniq.includes(d)) uniq.push(d);
  return uniq;
}

const optDC = new Int32Array(30);
const optDL = new Uint8Array(30);
const optCost = new Int32Array(30);

function rewriteDistsPass(data, byteStart, lit, dist, a, b, tryDists, forcePreferBpp = 0) {
  for (let pass = 0; pass < 3; pass++) {
    optDC.fill(0);
    for (let i = a; i < b; i++) { const d = dist[i]; if (d > 0) optDC[DSYM[d]]++; }
    codeLengths(optDC, 30, 15, optDL);
    fixDistLens(optDL);
    for (let c = 0; c < 30; c++) optCost[c] = (optDL[c] > 0 ? optDL[c] : 15) + DIST_EXTRA[c];
    if (pass === 0 && forcePreferBpp >= 2 && forcePreferBpp <= 4) {
      optCost[forcePreferBpp - 1] = 0;
    }
    let pos = byteStart;
    let changed = 0;
    for (let i = a; i < b; i++) {
      const d = dist[i];
      if (d === 0) { pos++; continue; }
      const l = lit[i];
      let bestD = d;
      let bestC = optCost[DSYM[d]];
      for (let ti = 0; ti < tryDists.length; ti++) {
        const cd = tryDists[ti];
        if (cd > pos || cd === bestD) continue;
        const cc = optCost[DSYM[cd]];
        if (cc >= bestC) continue;
        let ok = true;
        for (let k = 0; k < l; k++) {
          if (data[pos + k] !== data[pos - cd + k]) { ok = false; break; }
        }
        if (ok) { bestD = cd; bestC = cc; }
      }
      if (bestD !== d) { dist[i] = bestD; changed++; }
      pos += l;
    }
    if (!changed) break;
  }
}

function optimizeBlockDistances(data, byteStart, lit, dist, a, b, tryDists, bpp) {
  if (tryDists.length <= 1 || b <= a) return blockBits(lit, dist, a, b, false);
  const origD = dist.slice(a, b);
  let bestBits = blockBits(lit, dist, a, b, false);

  rewriteDistsPass(data, byteStart, lit, dist, a, b, tryDists, 0);
  let bits1 = blockBits(lit, dist, a, b, false);
  if (bits1 < bestBits) {
    bestBits = bits1;
  } else {
    dist.set(origD, a);
  }

  // On multi-byte pixels (bpp 2..4), also probe collapsing dist=1 -> dist=bpp basin!
  if (bpp >= 2 && bpp <= 4) {
    const savedD = dist.slice(a, b);
    dist.set(origD, a);
    rewriteDistsPass(data, byteStart, lit, dist, a, b, tryDists, bpp);
    const bits2 = blockBits(lit, dist, a, b, false);
    if (bits2 < bestBits) {
      bestBits = bits2;
    } else {
      dist.set(savedD, a);
    }
  }
  return bestBits;
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

function costsFromFixed(preferBpp = 0) {
  for (let s = 0; s < 256; s++) litCostI32[s] = FIXED_LL[s] * SCALE;
  for (let c = 0; c < 29; c++) lenCostI32[c] = (FIXED_LL[257 + c] + LEN_EXTRA[c]) * SCALE;
  for (let c = 0; c < 30; c++) distCostI32[c] = (5 + DIST_EXTRA[c]) * SCALE;
  if (preferBpp >= 2 && preferBpp <= 4) {
    distCostI32[preferBpp - 1] = 2 * SCALE;
    distCostI32[0] = 6 * SCALE;
  }
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

function costsFromRowDist(rowLen) {
  for (let s = 0; s < 256; s++) litCostI32[s] = FIXED_LL[s] * SCALE;
  for (let c = 0; c < 29; c++) lenCostI32[c] = (FIXED_LL[257 + c] + LEN_EXTRA[c]) * SCALE;
  for (let c = 0; c < 30; c++) distCostI32[c] = (7 + DIST_EXTRA[c]) * SCALE;
  distCostI32[0] = 2 * SCALE;
  if (rowLen >= 3 && rowLen <= 32768) {
    const rds = DSYM[rowLen];
    distCostI32[rds] = (2 + DIST_EXTRA[rds]) * SCALE;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Hybrid Block Splitter: Filter-Transition Hints + Agglomerative Coarsening -> Exact O(M^2) DP -> Boundary Slide!

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

function filterTransitionSymPoints(data, rowLen, lit, dist, m, minSyms) {
  if (rowLen < 16 || data.length < rowLen * 4) return [];
  const h = Math.floor(data.length / rowLen);
  const byteCuts = [];
  for (let y = 1; y < h; y++) {
    if (data[y * rowLen] !== data[(y - 1) * rowLen]) byteCuts.push(y * rowLen);
  }
  if (byteCuts.length === 0 || byteCuts.length > 200) return [];
  const symCuts = [];
  let pos = 0, ci = 0;
  for (let i = 0; i < m && ci < byteCuts.length; i++) {
    while (ci < byteCuts.length && pos >= byteCuts[ci]) {
      if (i >= minSyms && m - i >= minSyms) symCuts.push(i);
      ci++;
    }
    pos += dist[i] === 0 ? 1 : lit[i];
  }
  return symCuts;
}

function splitHybridDP(data, rowLen, lit, dist, m, minSyms = 256, maxBlocks = 200, initChunks = 180, dpCap = 68, noSlide = false, prevStarts = null) {
  const effMin = m < 24000 ? Math.min(minSyms, 96) : minSyms;
  if (m < 2 * effMin) return [];
  const K = Math.min(initChunks, Math.max(2, Math.floor(m / effMin)));
  const rawPts = [0];
  for (let i = 1; i < K; i++) rawPts.push(Math.round((i * m) / K));
  const fPts = filterTransitionSymPoints(data, rowLen, lit, dist, m, effMin);
  for (const p of fPts) rawPts.push(p);
  if (prevStarts) {
    for (let i = 1; i + 1 < prevStarts.length; i++) {
      const p = prevStarts[i];
      if (p >= effMin && m - p >= effMin) rawPts.push(p);
    }
  }
  rawPts.push(m);
  rawPts.sort((a, b) => a - b);

  // Deduplicate keeping minimum gap
  const pts = [0];
  const minGap = Math.max(48, effMin >> 1);
  for (let i = 1; i < rawPts.length - 1; i++) {
    if (rawPts[i] - pts[pts.length - 1] >= minGap && m - rawPts[i] >= minGap) pts.push(rawPts[i]);
  }
  pts.push(m);

  const segs = [];
  for (let i = 0; i + 1 < pts.length; i++) {
    const a = pts[i], b = pts[i + 1];
    const ll = new Int32Array(288);
    const d = new Int32Array(30);
    const extra = histogram(lit, dist, a, b, ll, d);
    const bits = bitsFromHist(ll, d, extra, false);
    segs.push({ a, b, ll, d, extra, bits });
  }

  // Stage 1a: Clone segments for pure greedy bottom-up merge (proven safeguard on small/uniform streams)
  const buSegs = segs.map((s) => ({ a: s.a, b: s.b, ll: s.ll.slice(), d: s.d.slice(), extra: s.extra, bits: s.bits }));
  const buGains = [];
  for (let i = 0; i + 1 < buSegs.length; i++) buGains.push(buSegs[i].bits + buSegs[i + 1].bits - combinedHistBits(buSegs[i], buSegs[i + 1]));
  while (buSegs.length > 1) {
    let bestI = -1, bestG = -Infinity;
    for (let i = 0; i < buGains.length; i++) if (buGains[i] > bestG) { bestG = buGains[i]; bestI = i; }
    if (bestI < 0 || (bestG <= 0 && buSegs.length <= maxBlocks)) break;
    const s1 = buSegs[bestI], s2 = buSegs[bestI + 1];
    for (let k = 0; k < 288; k++) s1.ll[k] += s2.ll[k];
    s1.ll[256] = 1;
    for (let k = 0; k < 30; k++) s1.d[k] += s2.d[k];
    s1.extra += s2.extra;
    s1.b = s2.b;
    s1.bits = s1.bits + s2.bits - bestG;
    buSegs.splice(bestI + 1, 1);
    buGains.splice(bestI, 1);
    if (bestI > 0) buGains[bestI - 1] = buSegs[bestI - 1].bits + buSegs[bestI].bits - combinedHistBits(buSegs[bestI - 1], buSegs[bestI]);
    if (bestI < buSegs.length - 1) buGains[bestI] = buSegs[bestI].bits + buSegs[bestI + 1].bits - combinedHistBits(buSegs[bestI], buSegs[bestI + 1]);
  }
  let buBits = 0;
  for (const s of buSegs) buBits += s.bits;

  let chosen;
  if (dpCap <= 0) {
    chosen = buSegs.map((s) => ({ a: s.a, b: s.b, bits: s.bits }));
  } else {
    // Stage 1b: Agglomerative coarsening down to <= dpCap segments, then Exact O(M^2) DP!
    if (segs.length > dpCap) {
      const gains = [];
      for (let i = 0; i + 1 < segs.length; i++) {
        gains.push(segs[i].bits + segs[i + 1].bits - combinedHistBits(segs[i], segs[i + 1]));
      }
      while (segs.length > dpCap) {
        let bestI = -1, bestG = -Infinity;
        for (let i = 0; i < gains.length; i++) {
          if (gains[i] > bestG) { bestG = gains[i]; bestI = i; }
        }
        if (bestI < 0) break;
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
    }

    const M = segs.length;
    const pLL = Array.from({ length: M + 1 }, () => new Int32Array(288));
    const pD = Array.from({ length: M + 1 }, () => new Int32Array(30));
    const pEx = new Int32Array(M + 1);
    for (let i = 0; i < M; i++) {
      const prevL = pLL[i], nextL = pLL[i + 1], sl = segs[i].ll;
      for (let k = 0; k < 288; k++) nextL[k] = prevL[k] + sl[k];
      const prevD = pD[i], nextD = pD[i + 1], sd = segs[i].d;
      for (let k = 0; k < 30; k++) nextD[k] = prevD[k] + sd[k];
      pEx[i + 1] = pEx[i] + segs[i].extra;
    }

    const dp = new Float64Array(M + 1).fill(Infinity);
    const par = new Int32Array(M + 1);
    dp[0] = 0;
    for (let j = 1; j <= M; j++) {
      const jL = pLL[j], jD = pD[j], jEx = pEx[j];
      for (let i = 0; i < j; i++) {
        const iL = pLL[i], iD = pD[i];
        for (let k = 0; k < 288; k++) combLL[k] = jL[k] - iL[k];
        combLL[256] = 1;
        for (let k = 0; k < 30; k++) combD[k] = jD[k] - iD[k];
        const cost = dp[i] + bitsFromHist(combLL, combD, jEx - pEx[i], false);
        if (cost < dp[j]) {
          dp[j] = cost;
          par[j] = i;
        }
      }
    }

    if (buBits <= dp[M]) {
      chosen = buSegs.map((s) => ({ a: s.a, b: s.b, bits: s.bits }));
    } else {
      chosen = [];
      for (let cur = M; cur > 0; cur = par[cur]) {
        const prev = par[cur];
        chosen.push({ a: segs[prev].a, b: segs[cur - 1].b, bits: dp[cur] - dp[prev] });
      }
      chosen.reverse();
    }
  }

  // Enforce maxBlocks if needed
  while (chosen.length > maxBlocks) {
    let bestI = 0, bestG = -Infinity;
    for (let i = 0; i + 1 < chosen.length; i++) {
      const g = chosen[i].bits + chosen[i + 1].bits - blockBits(lit, dist, chosen[i].a, chosen[i + 1].b, false);
      if (g > bestG) { bestG = g; bestI = i; }
    }
    chosen[bestI].b = chosen[bestI + 1].b;
    chosen[bestI].bits = chosen[bestI].bits + chosen[bestI + 1].bits - bestG;
    chosen.splice(bestI + 1, 1);
  }

  if (noSlide) return chosen.map((s) => s.a).slice(1);

  // Stage 3: Iterated 2-sweep incremental-histogram boundary sliding
  const lLL = new Int32Array(288), lD = new Int32Array(30);
  const rLL = new Int32Array(288), rD = new Int32Array(30);
  const sLL = new Int32Array(288), sD = new Int32Array(30);
  const tRLL = new Int32Array(288), tRD = new Int32Array(30);

  for (let sweep = 0; sweep < 2; sweep++) {
    let anyMoved = false;
    for (let i = 0; i + 1 < chosen.length; i++) {
    const x = chosen[i].a, y = chosen[i + 1].b;
    let lo = x + effMin, hi = y - effMin;
    if (hi <= lo) continue;
    let bestP = chosen[i].b;
    let bestF = chosen[i].bits + chosen[i + 1].bits;
    let baseLEx = histogram(lit, dist, x, lo, lLL, lD);
    let baseREx = histogram(lit, dist, lo, y, rLL, rD);
    let baseLo = lo;

    while (hi - lo > 0) {
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
      sLL.set(lLL); sD.set(lD);
      tRLL.set(rLL); tRD.set(rD);
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
    if (bestP !== chosen[i].b) {
      chosen[i].b = bestP; chosen[i].bits = blockBits(lit, dist, x, bestP, false);
      chosen[i + 1].a = bestP; chosen[i + 1].bits = blockBits(lit, dist, bestP, y, false);
      anyMoved = true;
    }
    }
    if (!anyMoved) break;
  }
  return chosen.map((s) => s.a).slice(1);
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

function kraftRebalance(counts, baseLens, nsym, edits) {
  const trial = baseLens.slice();
  const locked = new Uint8Array(nsym);
  for (let ei = 0; ei < edits.length; ei++) {
    const sym = edits[ei][0], nl = edits[ei][1];
    if (nl < 1 || nl > 15) return null;
    trial[sym] = nl;
    locked[sym] = 1;
  }
  let kraft = 0;
  for (let i = 0; i < nsym; i++) if (trial[i] > 0) kraft += 1 << (15 - trial[i]);
  if (kraft > 32768) {
    let rem = kraft - 32768;
    let guard = 64;
    while (rem > 0 && guard-- > 0) {
      let bestT = -1, bestScore = Infinity;
      for (let t = 0; t < nsym; t++) {
        if (locked[t] || trial[t] === 0 || trial[t] >= 15) continue;
        const delta = 1 << (14 - trial[t]);
        if (delta > rem) continue;
        const sc = counts[t] / delta;
        if (sc < bestScore) { bestScore = sc; bestT = t; }
      }
      if (bestT < 0) return null;
      rem -= 1 << (14 - trial[bestT]);
      trial[bestT]++;
    }
    if (rem !== 0) return null;
  } else if (kraft < 32768) {
    let rem = 32768 - kraft;
    let guard = 64;
    while (rem > 0 && guard-- > 0) {
      let bestT = -1, bestScore = -1;
      for (let t = 0; t < nsym; t++) {
        if (locked[t] || trial[t] <= 1) continue;
        const delta = 1 << (15 - trial[t]);
        if (delta > rem) continue;
        const sc = counts[t] / delta;
        if (sc > bestScore) { bestScore = sc; bestT = t; }
      }
      if (bestT < 0) return null;
      rem -= 1 << (15 - trial[bestT]);
      trial[bestT]--;
    }
    if (rem !== 0) return null;
  }
  let check = 0;
  for (let i = 0; i < nsym; i++) {
    if (counts[i] > 0 && (trial[i] < 1 || trial[i] > 15)) return null;
    if (trial[i] > 0) check += 1 << (15 - trial[i]);
  }
  return check === 32768 ? trial : null;
}

function polishKraftLengths(llCounts, dCounts, useLL, useD, extra, bestBits) {
  let improved = true;
  let rounds = 0;
  while (improved && rounds++ < 4) {
    improved = false;
    // Polish LL (1-hole, 2-hole, and 1-symbol glitch equalization)
    let hlit = 286;
    while (hlit > 257 && useLL[hlit - 1] === 0) hlit--;
    for (let i = 1; i + 1 < hlit; i++) {
      const v = useLL[i], L = useLL[i - 1], R = useLL[i + 1];
      if (v === 0 && L > 0 && R > 0) {
        const tryLens = L === R ? [L] : [L, R];
        if (L < 15 && R < 15) tryLens.push(15);
        for (const nl of tryLens) {
          const tr = kraftRebalance(llCounts, useLL, 286, [[i, nl]]);
          if (tr) {
            const b = evalDynamicBits(llCounts, dCounts, tr, useD, extra, true);
            if (b < bestBits) { bestBits = b; useLL.set(tr); improved = true; break; }
          }
        }
      } else if (v === 0 && i + 2 < hlit && useLL[i + 1] === 0 && L > 0 && L === useLL[i + 2]) {
        const tr = kraftRebalance(llCounts, useLL, 286, [[i, L], [i + 1, L]]);
        if (tr) {
          const b = evalDynamicBits(llCounts, dCounts, tr, useD, extra, true);
          if (b < bestBits) { bestBits = b; useLL.set(tr); improved = true; }
        }
      } else if (v > 0 && L > 0 && L === R && v !== L && llCounts[i] <= 8) {
        const tr = kraftRebalance(llCounts, useLL, 286, [[i, L]]);
        if (tr) {
          const b = evalDynamicBits(llCounts, dCounts, tr, useD, extra, true);
          if (b < bestBits) { bestBits = b; useLL.set(tr); improved = true; }
        }
      } else if (v > 0 && i >= 2 && L > 0 && L === useLL[i - 2] && v !== L && llCounts[i] <= 6) {
        const tr = kraftRebalance(llCounts, useLL, 286, [[i, L]]);
        if (tr) {
          const b = evalDynamicBits(llCounts, dCounts, tr, useD, extra, true);
          if (b < bestBits) { bestBits = b; useLL.set(tr); improved = true; }
        }
      }
    }
    // Polish Dist (1-hole and 1-symbol glitch)
    let hdist = 30;
    while (hdist > 1 && useD[hdist - 1] === 0) hdist--;
    for (let i = 1; i + 1 < hdist; i++) {
      const v = useD[i], L = useD[i - 1], R = useD[i + 1];
      if (v === 0 && L > 0 && R > 0) {
        const tryLens = L === R ? [L] : [L, R];
        for (const nl of tryLens) {
          const tr = kraftRebalance(dCounts, useD, 30, [[i, nl]]);
          if (tr) {
            const b = evalDynamicBits(llCounts, dCounts, useLL, tr, extra, true);
            if (b < bestBits) { bestBits = b; useD.set(tr); improved = true; break; }
          }
        }
      } else if (v > 0 && L > 0 && L === R && v !== L && dCounts[i] <= 8) {
        const tr = kraftRebalance(dCounts, useD, 30, [[i, L]]);
        if (tr) {
          const b = evalDynamicBits(llCounts, dCounts, useLL, tr, extra, true);
          if (b < bestBits) { bestBits = b; useD.set(tr); improved = true; }
        }
      }
    }
  }
  return bestBits;
}

function writeBlock(out, data, byteS, byteE, lit, dist, a, b, isFinal) {
  const extra = histogram(lit, dist, a, b);
  codeLengths(llC, 288, 15, llL);
  codeLengths(dC, 30, 15, dL);
  fixDistLens(dL);

  let bestMode = 2;
  let bestBits = evalDynamicBits(llC, dC, llL, dL, extra, true);
  let useLL = llL.slice(), useD = dL.slice();

  // Independent RLE count smoothing for LL and Dist
  const llCands = [llL.slice()];
  const dCands = [dL.slice()];
  for (const th of [2, 3, 4, 5, 6]) {
    optimizeCountsForRle(llC, 286, smLL, th);
    smLL[256] = 1;
    codeLengths(smLL, 288, 15, smLLL);
    let ok = true;
    for (let s = 0; s < 288; s++) if (llC[s] > 0 && smLLL[s] === 0) { ok = false; break; }
    if (ok) llCands.push(smLLL.slice());
  }
  for (const th of [2, 3, 4, 5]) {
    optimizeCountsForRle(dC, 30, smD, th);
    codeLengths(smD, 30, 15, smDL);
    fixDistLens(smDL);
    let ok = true;
    for (let s = 0; s < 30; s++) if (dC[s] > 0 && smDL[s] === 0) { ok = false; break; }
    if (ok) dCands.push(smDL.slice());
  }
  for (let li = 0; li < llCands.length; li++) {
    for (let di = 0; di < dCands.length; di++) {
      if (li === 0 && di === 0) continue;
      const dynSm = evalDynamicBits(llC, dC, llCands[li], dCands[di], extra, true);
      if (dynSm < bestBits) {
        bestBits = dynSm; bestMode = 3;
        useLL = llCands[li].slice(); useD = dCands[di].slice();
      }
    }
  }
  // Exact Kraft-preserving 0-hole fill and isolated-length equalization
  const polished = polishKraftLengths(llC, dC, useLL, useD, extra, bestBits);
  if (polished < bestBits) { bestBits = polished; bestMode = 3; }

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

function iterateRange(eng, L, data, s, e, bestL, bestD, bestM, iters, huffIters, patience, bpp, tryDists, tmpA_L, tmpA_D, tmpB_L, tmpB_D) {
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
    const m2 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, tL, tD, 0, bpp);
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
    const m2 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, tL, tD, 0, bpp);
    const bits = blockBits(tL, tD, 0, m2, false);
    if (bits < bestBits) {
      bestBits = bits;
      bestL.set(tL.subarray(0, m2));
      bestD.set(tD.subarray(0, m2));
      bestM = m2;
    } else break;
  }

  // Final in-place distance polish on converged block parse
  bestBits = optimizeBlockDistances(data, s, bestL, bestD, 0, bestM, tryDists, bpp);
  return { m: bestM, bits: bestBits };
}

function splitAndOptimize(eng, L, data, rowLen, bpp, tryDists, baseL, baseD, baseM, prevByteMap, prevStarts, globalBasin, minSyms, maxBlocks, initChunks, dpCap, blockIters, huffIters, patience, blockAltBasin, tmpA_L, tmpA_D, tmpB_L, tmpB_D) {
  const n = data.length;
  const sp = splitHybridDP(data, rowLen, baseL, baseD, baseM, minSyms, maxBlocks, initChunks, dpCap, false, prevStarts);
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
      r = iterateRange(eng, L, data, s, e, bl, bd, z - a, blockIters, huffIters, patience, bpp, tryDists, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
      if (blockAltBasin && (z - a) >= (bpp === 1 ? 800 : 1800)) {
        let lits = 0;
        for (let i = 0; i < r.m; i++) if (bd[i] === 0) lits++;
        let altKind = null;
        if (globalBasin !== 'litheavy' && lits > r.m * 0.42) altKind = 'litheavy';
        else if (globalBasin === 'litheavy' && lits < r.m * 0.35) altKind = bpp > 1 ? 'pixdist' : 'fixed';
        else if (bpp === 1 && globalBasin !== 'rowdist' && lits < r.m * 0.25 && rowLen >= 3 && rowLen <= 32768) altKind = 'rowdist';
        if (altKind) {
          if (altKind === 'litheavy') costsFromLitHeavy(data, s, e);
          else if (altKind === 'pixdist') costsFromFixed(bpp);
          else if (altKind === 'rowdist') costsFromRowDist(rowLen);
          else costsFromFixed(0);
          const bl2 = new Uint16Array(e - s + 1);
          const bd2 = new Uint16Array(e - s + 1);
          const m0 = eng.parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, bl2, bd2, 0, bpp);
          const r2 = iterateRange(eng, L, data, s, e, bl2, bd2, m0, Math.max(2, blockIters - 1), huffIters, 1, bpp, tryDists, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
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

  // Final adjacent merge check (with RLE-smoothed dynamic bits) + re-parse merged block!
  let merged = true;
  while (merged && starts.length > 2) {
    merged = false;
    for (let b = 0; b + 2 < starts.length; b++) {
      const a0 = starts[b], a1 = starts[b + 1], a2 = starts[b + 2];
      const sep = blockBits(outL, outD, a0, a1, true) + blockBits(outL, outD, a1, a2, true);
      const comb = blockBits(outL, outD, a0, a2, true);
      if (comb <= sep) {
        const bs = byteStarts[b], be = byteStarts[b + 2];
        const mLen = a2 - a0;
        const ml = new Uint16Array(be - bs + 1);
        const md = new Uint16Array(be - bs + 1);
        ml.set(outL.subarray(a0, a2));
        md.set(outD.subarray(a0, a2));
        const rr = iterateRange(eng, L, data, bs, be, ml, md, mLen, 2, 1, 1, bpp, tryDists, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
        const comb2 = blockBits(ml, md, 0, rr.m, true);
        if (comb2 < comb) {
          const tailLen = starts[starts.length - 1] - a2;
          const tailL = outL.slice(a2, a2 + tailLen);
          const tailD = outD.slice(a2, a2 + tailLen);
          outL.set(ml.subarray(0, rr.m), a0);
          outD.set(md.subarray(0, rr.m), a0);
          outL.set(tailL, a0 + rr.m);
          outD.set(tailD, a0 + rr.m);
          const delta = rr.m - mLen;
          starts.splice(b + 1, 1);
          byteStarts.splice(b + 1, 1);
          for (let k = b + 1; k < starts.length; k++) starts[k] += delta;
          totalBits -= (sep - comb2);
        } else {
          starts.splice(b + 1, 1);
          byteStarts.splice(b + 1, 1);
          totalBits -= (sep - comb);
        }
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
  const bpp = opts.bpp ?? 1;
  const rowLen = opts.rowLen ?? 0;
  const iters = opts.iters ?? 3;
  const blockIters = opts.blockIters ?? 4;
  const huffIters = opts.huffIters ?? 2;
  const patience = opts.patience ?? 2;
  const maxBlocks = opts.maxBlocks ?? 200;
  const minSyms = opts.minSyms ?? 256;
  const initChunks = opts.initChunks ?? 180;
  const dpCap = opts.dpCap ?? 60;
  const passes = opts.passes ?? 2;
  const isPhoto = opts.isPhoto ?? false;
  const basins = opts.basins ?? (isPhoto ? ['litheavy', 'fixed'] : ['fixed', 'litheavy']);
  const blockAltBasin = opts.blockAltBasin ?? true;

  const hc3Cut = opts.hc3Cut ?? (bpp === 1 ? 48 : 16);
  const hc4Cut = opts.hc4Cut ?? (bpp === 1 ? 128 : 48);

  const tryDists = buildTryDists(bpp, rowLen);
  const eng = getWasmEngine();
  const L = eng.prepareMatches(data, cut, nice, bpp, rowLen, hc3Cut, hc4Cut);

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
  let bestBasin = basins[0];

  for (const basin of basins) {
    let gIt = (basin === 'pixdist' || (isPhoto && basin === 'fixed')) ? Math.max(1, iters - 1) : iters;
    let gHuff = basin === 'litheavy' ? 1 : 0;
    if (basin === 'litheavy' && bpp === 1 && bestM > 0) {
      let lits = 0;
      for (let i = 0; i < bestM; i++) if (gD[i] === 0) lits++;
      if (lits < bestM * 0.22) continue;
    }
    if (basin === 'fixed') costsFromFixed(0);
    else if (basin === 'pixdist') costsFromFixed(bpp);
    else if (basin === 'rowdist') costsFromRowDist(rowLen);
    else if (basin === 'litheavy') costsFromLitHeavy(data, 0, n);
    const m0 = eng.parseRange(L, 0, n, litCostI32, lenCostI32, distCostI32, candL, candD, 0, bpp);
    const r = iterateRange(eng, L, data, 0, n, candL, candD, m0, gIt, gHuff, patience, bpp, tryDists, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
    if (r.bits < bestGlobalBits) {
      bestGlobalBits = r.bits;
      bestM = r.m;
      bestBasin = basin;
      gL.set(candL.subarray(0, r.m));
      gD.set(candD.subarray(0, r.m));
    }
  }

  let best = splitAndOptimize(eng, L, data, rowLen, bpp, tryDists, gL, gD, bestM, null, null, bestBasin, minSyms, maxBlocks, initChunks, dpCap, blockIters, huffIters, patience, blockAltBasin, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
  const smoothedTotalBits = (res) => {
    let tot = 0;
    for (let b = 0; b + 1 < res.starts.length; b++) tot += blockBits(res.lit, res.dist, res.starts[b], res.starts[b + 1], true);
    return tot;
  };
  let bestSmBits = passes > 1 ? smoothedTotalBits(best) : best.bits;
  for (let p = 1; p < passes; p++) {
    const p2Iters = Math.max(1, Math.min(2, blockIters - 1));
    const p2Huff = Math.max(1, huffIters - 1);
    const next = splitAndOptimize(eng, L, data, rowLen, bpp, tryDists, best.lit, best.dist, best.starts[best.starts.length - 1], best.byteMap, best.starts, bestBasin, minSyms, maxBlocks, initChunks, dpCap, p2Iters, p2Huff, 1, false, tmpA_L, tmpA_D, tmpB_L, tmpB_D);
    const nextSmBits = smoothedTotalBits(next);
    if (nextSmBits < bestSmBits) { best = next; bestSmBits = nextSmBits; }
    else break;
  }

  // Post-Zopfli Palette RLE: when CT 3 uses only f=0 and all-zero dup f=2 rows, permuting indices
  // while keeping filter bytes fixed (0, and 2 if any f=2 row exists) is a 100% pointwise bijection
  // on the entire filtered byte stream (preserving every LZ77 match & filter byte) and shrinks block headers!
  let outData = data;
  if (opts.paletteSize && opts.paletteSize >= 4 && opts.paletteSize <= 256 && opts.onPalPerm && rowLen >= 2 && data.length % rowLen === 0) {
    const K = opts.paletteSize;
    let invariant = true;
    let hasF2 = false;
    for (let off = 0; off < data.length; off += rowLen) {
      const f = data[off];
      if (f === 0) continue;
      if (f === 2) {
        hasF2 = true;
        for (let x = 1; x < rowLen; x++) if (data[off + x] !== 0) { invariant = false; break; }
        if (!invariant) break;
      } else { invariant = false; break; }
    }
    if (invariant) {
      const fixedIdx = (c) => c === 0 || (hasF2 && c === 2);
      const movable = [];
      for (let c = 1; c < K; c++) if (!fixedIdx(c)) movable.push(c);
      const st0 = best.starts;
      const nB = st0.length - 1;
      const bInfo = [];
      for (let b = 0; b < nB; b++) {
        const ll = new Int32Array(288), dd = new Int32Array(30);
        histogram(best.lit, best.dist, st0[b], st0[b + 1], ll, dd);
        const lL = new Uint8Array(288), dL = new Uint8Array(30);
        codeLengths(ll, 288, 15, lL);
        codeLengths(dd, 30, 15, dL);
        fixDistLens(dL);
        bInfo.push({ lL, dL });
      }
      const tmpLL = new Uint8Array(288);
      const evalPerm = (invPerm) => {
        let tot = 0;
        for (let b = 0; b < nB; b++) {
          const bi = bInfo[b];
          tmpLL.set(bi.lL);
          for (let ni = 1; ni < K; ni++) tmpLL[ni] = bi.lL[invPerm[ni]];
          tot += headerPlan(tmpLL, bi.dL, false).bits;
        }
        return tot;
      };
      const inv = new Uint8Array(K);
      for (let i = 0; i < K; i++) inv[i] = i;
      const baseH = evalPerm(inv);

      const fillFromOrder = (ord) => {
        const outInv = new Uint8Array(K);
        for (let i = 0; i < K; i++) outInv[i] = i;
        let k = 0;
        for (let i = 1; i < K; i++) if (!fixedIdx(i)) outInv[i] = ord[k++];
        return outInv;
      };
      const ordLex = [...movable].sort((a, b) => {
        for (let bi = 0; bi < nB; bi++) { const d = bInfo[bi].lL[a] - bInfo[bi].lL[b]; if (d !== 0) return d; }
        return a - b;
      });
      const invLex = fillFromOrder(ordLex);
      const invDesc = fillFromOrder([...ordLex].reverse());
      const ordNN = [];
      const used = new Uint8Array(K);
      let prevC = 0;
      for (let step = 0; step < movable.length; step++) {
        let bestC = -1, bestD = Infinity;
        for (let mi = 0; mi < movable.length; mi++) {
          const c = movable[mi];
          if (used[c]) continue;
          let d = 0;
          for (let bi = 0; bi < nB; bi++) if (bInfo[bi].lL[c] !== bInfo[bi].lL[prevC]) d++;
          if (d < bestD) { bestD = d; bestC = c; }
        }
        ordNN.push(bestC); used[bestC] = 1; prevC = bestC;
      }
      const invNN = fillFromOrder(ordNN);
      let bestInv = inv.slice(), bestH = baseH;
      for (const cand of [invLex, invDesc, invNN]) {
        const h = evalPerm(cand);
        if (h < bestH) { bestH = h; bestInv = cand.slice(); }
      }
      const maxSpan = movable.length <= 64 ? movable.length : 20;
      const maxPasses = movable.length <= 64 ? 3 : 2;
      for (let pass = 0; pass < maxPasses; pass++) {
        let imp = false;
        for (let mi = 0; mi < movable.length; mi++) {
          const i = movable[mi];
          const jLim = Math.min(movable.length, mi + maxSpan);
          for (let mj = mi + 1; mj < jLim; mj++) {
            const j = movable[mj];
            const t = bestInv[i]; bestInv[i] = bestInv[j]; bestInv[j] = t;
            const h = evalPerm(bestInv);
            if (h < bestH) { bestH = h; imp = true; }
            else { bestInv[j] = bestInv[i]; bestInv[i] = t; }
          }
        }
        if (!imp) break;
      }
      if (bestH < baseH) {
        const fwd = new Uint8Array(K);
        for (let ni = 0; ni < K; ni++) fwd[bestInv[ni]] = ni;
        const endM = st0[st0.length - 1];
        for (let i = 0; i < endM; i++) {
          if (best.dist[i] === 0 && best.lit[i] < K) best.lit[i] = fwd[best.lit[i]];
        }
        outData = new Uint8Array(data.length);
        for (let i = 0; i < data.length; i++) {
          const v = data[i];
          outData[i] = v < K ? fwd[v] : v;
        }
        opts.onPalPerm(bestInv, fwd, outData);

        // Monotonic Post-Pal Re-Split: re-evaluate block boundaries under the cheaper permuted headers!
        const spNew = splitHybridDP(outData, rowLen, best.lit, best.dist, endM, minSyms, maxBlocks, initChunks, dpCap, false, best.starts);
        const newStarts = [0, ...spNew, endM];
        let oldTot = 0, newTot = 0;
        for (let b = 0; b + 1 < best.starts.length; b++) oldTot += blockBits(best.lit, best.dist, best.starts[b], best.starts[b + 1], true);
        for (let b = 0; b + 1 < newStarts.length; b++) newTot += blockBits(best.lit, best.dist, newStarts[b], newStarts[b + 1], true);
        if (newTot < oldTot) {
          const newBst = [0];
          let bp = 0;
          for (let b = 0; b + 1 < newStarts.length; b++) {
            bp += bytesOf(best.lit, best.dist, newStarts[b], newStarts[b + 1]);
            newBst.push(bp);
          }
          best = { ...best, starts: newStarts, byteStarts: newBst };
        }
      }
    }
  }

  const out = new Bits(Math.max(1024, n >>> 3));
  const st = best.starts;
  const bst = best.byteStarts;
  for (let b = 0; b + 1 < st.length; b++) {
    writeBlock(out, outData, bst[b], bst[b + 1], best.lit, best.dist, st[b], st[b + 1], b + 2 === st.length);
  }
  return out.finish();
}

/** Ultra-fast WASM DEFLATE size estimator (bytes) for comparing filter candidates without zlib bias. */
export function estimateDeflateBytes(data, bpp = 1, rowLen = 0, isPhoto = false) {
  const n = data.length;
  const eng = getWasmEngine();
  const L = eng.prepareMatches(data, 20, 259, bpp, rowLen, 0, 0);
  const gL = new Uint16Array(n + 1);
  const gD = new Uint16Array(n + 1);

  if (isPhoto) costsFromLitHeavy(data, 0, n);
  else costsFromFixed(0);
  let bestM = eng.parseRange(L, 0, n, litCostI32, lenCostI32, distCostI32, gL, gD, 0, bpp);
  if (bpp === 1) {
    const tL = new Uint16Array(n + 1);
    const tD = new Uint16Array(n + 1);
    let bestB = blockBits(gL, gD, 0, bestM, false);
    histogram(gL, gD, 0, bestM);
    costsFromCounts(llC, dC);
    const m2 = eng.parseRange(L, 0, n, litCostI32, lenCostI32, distCostI32, tL, tD, 0, bpp);
    const b2 = blockBits(tL, tD, 0, m2, false);
    if (b2 < bestB) {
      gL.set(tL.subarray(0, m2));
      gD.set(tD.subarray(0, m2));
      bestM = m2;
    }
  }

  // Fast bottom-up split without boundary slide -> accurate block-split byte estimate!
  const sp = splitHybridDP(data, rowLen, gL, gD, bestM, 256, 200, 56, 0, true);
  const symB = [0, ...sp, bestM];
  let totalBits = 0;
  for (let b = 0; b + 1 < symB.length; b++) {
    totalBits += blockBits(gL, gD, symB[b], symB[b + 1], false);
  }
  return Math.ceil(totalBits / 8) + 6;
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
