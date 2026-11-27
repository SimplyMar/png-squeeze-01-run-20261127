// A DEFLATE encoder written for this entry: cached hash-chain matches, an optimal (shortest-path) parse under an
// iterated entropy cost model, dynamic Huffman blocks with length-limited codes, and greedy block splitting. Everything
// is integer or IEEE arithmetic over the input bytes alone, so the same input always yields the same stream.

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
// NEXT_END[L]: the smallest length >= L that is the last of its length code (258 past the last).
const NEXT_END = new Uint16Array(260);
{
  let end = 258;
  NEXT_END[259] = 259;
  for (let L = 258; L >= 0; L--) {
    if (L >= 3 && (L === 258 || LSYM[L + 1] !== LSYM[L])) end = L;
    NEXT_END[L] = end;
  }
}
const DSYM = new Uint8Array(32769);
for (let c = 0; c < 30; c++) {
  const end = c < 29 ? DIST_BASE[c + 1] : 32769;
  for (let d = DIST_BASE[c]; d < end; d++) DSYM[d] = c;
}

// ---------------------------------------------------------------------------------------------------------------
// Matches: for every position, the strictly increasing lengths reachable and the nearest distance giving each.

/**
 * The same match lists from a binary search tree over the window (each node's left and right children ordered by the
 * bytes that follow it): `cut` bounds the nodes visited a position; a match of `nice` bytes ends the search and the
 * positions it covers are inserted without recording matches.
 */
export function findMatchesBT(data, cut, nice) {
  const n = data.length;
  const HBITS = 24;
  const head = new Int32Array(1 << HBITS).fill(-1);
  // The tree's links in a cyclic buffer one window long (positions older than the window are never followed).
  const CS = 32769;
  const son = new Int32Array(2 * CS);
  const off = new Int32Array(n + 1);
  let cap = Math.max(1024, n);
  let lens = new Uint16Array(cap);
  let dists = new Uint16Array(cap);
  let m = 0;
  let skipTo = 0;
  for (let i = 0; i < n; i++) {
    off[i] = m;
    const lenLimit = n - i < 258 ? n - i : 258;
    const cp = i % CS;
    if (lenLimit < 3) {
      son[2 * cp] = -1;
      son[2 * cp + 1] = -1;
      continue;
    }
    const record = i >= skipTo;
    const h = Math.imul((data[i] << 16) | (data[i + 1] << 8) | data[i + 2], 0x9e3779b1) >>> (32 - HBITS);
    let cur = head[h];
    head[h] = i;
    let ptr0 = 2 * cp + 1;
    let ptr1 = 2 * cp;
    let len0 = 0;
    let len1 = 0;
    let best = 2;
    let cnt = cut;
    for (;;) {
      if (cur < 0 || i - cur > 32768 || cnt-- === 0) {
        son[ptr0] = -1;
        son[ptr1] = -1;
        break;
      }
      const sl = cp - (i - cur);
      const pair = 2 * (sl < 0 ? sl + CS : sl);
      let len = len0 < len1 ? len0 : len1;
      if (data[cur + len] === data[i + len]) {
        while (++len < lenLimit) if (data[cur + len] !== data[i + len]) break;
        if (len > best) {
          best = len;
          if (record) {
            if (m >= cap) {
              cap *= 2;
              const nl = new Uint16Array(cap);
              nl.set(lens);
              lens = nl;
              const nd = new Uint16Array(cap);
              nd.set(dists);
              dists = nd;
            }
            lens[m] = len;
            dists[m] = i - cur;
            m++;
          }
          if (len >= lenLimit) {
            son[ptr1] = son[pair];
            son[ptr0] = son[pair + 1];
            break;
          }
        }
      }
      if (data[cur + len] < data[i + len]) {
        son[ptr1] = cur;
        ptr1 = pair + 1;
        cur = son[ptr1];
        len1 = len;
      } else {
        son[ptr0] = cur;
        ptr0 = pair;
        cur = son[ptr0];
        len0 = len;
      }
    }
    if (record && best >= nice) skipTo = i + best;
  }
  off[n] = m;
  return { off, lens, dists, n };
}

// ---------------------------------------------------------------------------------------------------------------
// Huffman code lengths, limited to maxBits (two-queue Huffman, then the overflow redistributed by the Kraft sum).

const hSyms = new Int32Array(320);
const hW = new Float64Array(640);
const hParent = new Int32Array(640);
const hDepth = new Int32Array(640);
const hNum = new Int32Array(64);

export function codeLengths(counts, nsym, maxBits, out) {
  let k = 0;
  for (let s = 0; s < nsym; s++) {
    out[s] = 0;
    if (counts[s] > 0) hSyms[k++] = s;
  }
  if (k === 0) return;
  if (k === 1) {
    // A complete code needs two symbols: a second one of length 1 (unused) keeps every inflater content.
    out[hSyms[0]] = 1;
    out[hSyms[0] === 0 ? 1 : 0] = 1;
    return;
  }
  // Sort by (count, symbol) ascending: insertion sort is fine for <= 288 entries.
  for (let a = 1; a < k; a++) {
    const s = hSyms[a];
    const c = counts[s];
    let b = a - 1;
    while (b >= 0 && (counts[hSyms[b]] > c || (counts[hSyms[b]] === c && hSyms[b] > s))) {
      hSyms[b + 1] = hSyms[b];
      b--;
    }
    hSyms[b + 1] = s;
  }
  // Leaves 0..k-1, internal nodes k..2k-2.
  for (let a = 0; a < k; a++) hW[a] = counts[hSyms[a]];
  let li = 0;
  let ii = k;
  let next = k;
  for (let t = 0; t < k - 1; t++) {
    let x;
    if (li < k && (ii >= next || hW[li] <= hW[ii])) x = li++;
    else x = ii++;
    let y;
    if (li < k && (ii >= next || hW[li] <= hW[ii])) y = li++;
    else y = ii++;
    hW[next] = hW[x] + hW[y];
    hParent[x] = next;
    hParent[y] = next;
    next++;
  }
  const root = next - 1;
  hDepth[root] = 0;
  for (let t = root - 1; t >= k; t--) hDepth[t] = hDepth[hParent[t]] + 1;
  for (let b = 0; b < 64; b++) hNum[b] = 0;
  let maxD = 0;
  for (let a = 0; a < k; a++) {
    const d = hDepth[hParent[a]] + 1;
    hDepth[a] = d;
    if (d > maxD) maxD = d;
  }
  if (maxD <= maxBits) {
    for (let a = 0; a < k; a++) out[hSyms[a]] = hDepth[a];
    return;
  }
  for (let a = 0; a < k; a++) hNum[Math.min(hDepth[a], maxBits)]++;
  let total = 0;
  for (let b = maxBits; b > 0; b--) total += hNum[b] * 2 ** (maxBits - b);
  const full = 2 ** maxBits;
  while (total > full) {
    hNum[maxBits]--;
    for (let b = maxBits - 1; b > 0; b--) {
      if (hNum[b] !== 0) {
        hNum[b]--;
        hNum[b + 1] += 2;
        break;
      }
    }
    total--;
  }
  // Longest codes to the rarest symbols (hSyms is ascending by count).
  let a = 0;
  for (let b = maxBits; b > 0; b--) for (let c = hNum[b]; c > 0; c--) out[hSyms[a++]] = b;
}

// ---------------------------------------------------------------------------------------------------------------
// The code-length (header) encoding: the RLE of the lengths under one of 8 uses of codes 16/17/18, smallest kept.

const rleSym = new Uint8Array(320);
const rleExt = new Uint8Array(320);
const clCounts = new Int32Array(19);
const clLens = new Uint8Array(19);
const allLens = new Uint8Array(320);

function rleOf(lens, n, use16, use17, use18) {
  let m = 0;
  let i = 0;
  while (i < n) {
    const v = lens[i];
    let run = 1;
    while (i + run < n && lens[i + run] === v) run++;
    let r = run;
    if (v === 0 && (use17 || use18)) {
      while (r >= 11 && use18) {
        const t = r > 138 ? 138 : r;
        rleSym[m] = 18;
        rleExt[m++] = t - 11;
        r -= t;
      }
      while (r >= 3 && use17) {
        const t = r > 10 ? 10 : r;
        rleSym[m] = 17;
        rleExt[m++] = t - 3;
        r -= t;
      }
      while (r > 0) {
        rleSym[m++] = 0;
        r--;
      }
    } else if (use16 && r >= 4) {
      rleSym[m++] = v;
      r--;
      while (r >= 3) {
        const t = r > 6 ? 6 : r;
        rleSym[m] = 16;
        rleExt[m++] = t - 3;
        r -= t;
      }
      while (r > 0) {
        rleSym[m++] = v;
        r--;
      }
    } else {
      while (r > 0) {
        rleSym[m++] = v;
        r--;
      }
    }
    i += run;
  }
  return m;
}

/** The best header for (llLens, dLens): {bits, variant, hlit, hdist}. */
function headerPlan(llLens, dLens) {
  let hlit = 286;
  while (hlit > 257 && llLens[hlit - 1] === 0) hlit--;
  let hdist = 30;
  while (hdist > 1 && dLens[hdist - 1] === 0) hdist--;
  for (let i = 0; i < hlit; i++) allLens[i] = llLens[i];
  for (let i = 0; i < hdist; i++) allLens[hlit + i] = dLens[i];
  const n = hlit + hdist;
  let bestBits = Infinity;
  let bestV = 0;
  for (let v = 0; v < 8; v++) {
    const m = rleOf(allLens, n, v & 1, v & 2, v & 4);
    clCounts.fill(0);
    for (let j = 0; j < m; j++) clCounts[rleSym[j]]++;
    codeLengths(clCounts, 19, 7, clLens);
    let hclen = 19;
    while (hclen > 4 && clLens[CL_ORDER[hclen - 1]] === 0) hclen--;
    let bits = 14 + 3 * hclen;
    for (let s = 0; s < 19; s++) bits += clCounts[s] * clLens[s];
    bits += clCounts[16] * 2 + clCounts[17] * 3 + clCounts[18] * 7;
    if (bits < bestBits) {
      bestBits = bits;
      bestV = v;
    }
  }
  return { bits: bestBits, variant: bestV, hlit, hdist };
}

// ---------------------------------------------------------------------------------------------------------------
// Symbol streams: litlen (0..255 literal, else a length 3..258 when dist > 0) and dist (0 for a literal).

const llC = new Int32Array(288);
const dC = new Int32Array(30);
const llL = new Uint8Array(288);
const dL = new Uint8Array(30);

function histogram(lit, dist, a, b) {
  llC.fill(0);
  dC.fill(0);
  let extra = 0;
  for (let i = a; i < b; i++) {
    const d = dist[i];
    if (d === 0) llC[lit[i]]++;
    else {
      const l = lit[i];
      llC[257 + LSYM[l]]++;
      extra += LEXT[l];
      const dc = DSYM[d];
      dC[dc]++;
      extra += DIST_EXTRA[dc];
    }
  }
  llC[256] = 1;
  return extra;
}

function fixDistLens(dL) {
  let used = 0;
  for (let i = 0; i < 30; i++) if (dL[i] !== 0) used++;
  if (used === 0) {
    dL[0] = 1;
    dL[1] = 1;
  } else if (used === 1) {
    if (dL[0] === 0) dL[0] = 1;
    else dL[1] = 1;
  }
}

/** The bits of a dynamic block holding symbols [a, b). */
export function blockBits(lit, dist, a, b) {
  const extra = histogram(lit, dist, a, b);
  codeLengths(llC, 288, 15, llL);
  codeLengths(dC, 30, 15, dL);
  fixDistLens(dL);
  let bits = 3 + extra;
  for (let s = 0; s < 288; s++) bits += llC[s] * llL[s];
  for (let s = 0; s < 30; s++) bits += dC[s] * dL[s];
  return bits + headerPlan(llL, dL).bits;
}

// ---------------------------------------------------------------------------------------------------------------
// Cost model and the optimal parse.

class Costs {
  constructor() {
    this.lit = new Float64Array(256);
    this.len = new Float64Array(259);
    this.dist = new Float64Array(30);
  }
  /** Entropy costs from symbol counts (llC/dC style arrays). */
  fromCounts(ll, d) {
    let tl = 0;
    for (let s = 0; s < 286; s++) tl += ll[s];
    let td = 0;
    for (let s = 0; s < 30; s++) td += d[s];
    const lgl = Math.log2(tl);
    const lgd = td > 0 ? Math.log2(td) : 0;
    const sc = new Float64Array(286);
    for (let s = 0; s < 286; s++) sc[s] = ll[s] > 0 ? lgl - Math.log2(ll[s]) : lgl + 1;
    for (let s = 0; s < 256; s++) this.lit[s] = sc[s];
    for (let l = 3; l < 259; l++) this.len[l] = sc[257 + LSYM[l]] + LEXT[l];
    for (let c = 0; c < 30; c++) this.dist[c] = (d[c] > 0 ? lgd - Math.log2(d[c]) : lgd + 1) + DIST_EXTRA[c];
  }
  /** Costs from Huffman lengths of the counts (integer bits). */
  fromLengths(ll, d) {
    for (let s = 0; s < 256; s++) this.lit[s] = ll[s] > 0 ? ll[s] : 15;
    for (let l = 3; l < 259; l++) {
      const v = ll[257 + LSYM[l]];
      this.len[l] = (v > 0 ? v : 15) + LEXT[l];
    }
    for (let c = 0; c < 30; c++) this.dist[c] = (d[c] > 0 ? d[c] : 15) + DIST_EXTRA[c];
  }
}

/** Shortest path over bytes [s, e) under `costs`; writes symbols into (lit, dist) from index `at`, returns the end. */
function parse(data, mf, s, e, costs, lit, dist, at, work) {
  const N = e - s;
  const { cost, cLen, cDist } = work;
  const cap = work.cap;
  cost[0] = 0;
  for (let i = 1; i <= N; i++) cost[i] = Infinity;
  const { off, lens, dists } = mf;
  const litC = costs.lit;
  const lenC = costs.len;
  const distC = costs.dist;
  for (let i = 0; i < N; i++) {
    const c = cost[i];
    const pos = s + i;
    const v = c + litC[data[pos]];
    if (v < cost[i + 1]) {
      cost[i + 1] = v;
      cLen[i + 1] = 1;
    }
    const a = off[pos];
    const b = off[pos + 1];
    if (a === b) continue;
    const room = N - i;
    let prevL = 2;
    for (let k = a; k < b; k++) {
      let l = lens[k];
      if (l > room) l = room;
      if (l <= prevL) break;
      const d = dists[k];
      const base = c + distC[DSYM[d]];
      // Every length up to `cap`; past it only the segment's own longest (the positions inside a long match have
      // matches of their own, so the parse still reaches every end point).
      const top = l < cap ? l : cap;
      for (let L = prevL + 1; L <= top; L++) {
        const w = base + lenC[L];
        if (w < cost[i + L]) {
          cost[i + L] = w;
          cLen[i + L] = L;
          cDist[i + L] = d;
        }
      }
      if (l > top) {
        // Past `cap`, the last length of each length code (all lengths of one code cost the same) up to the longest.
        if (work.ends) {
          for (let L = NEXT_END[(prevL > top ? prevL : top) + 1]; L < l; L = NEXT_END[L + 1]) {
            const w = base + lenC[L];
            if (w < cost[i + L]) {
              cost[i + L] = w;
              cLen[i + L] = L;
              cDist[i + L] = d;
            }
          }
        }
        const w = base + lenC[l];
        if (w < cost[i + l]) {
          cost[i + l] = w;
          cLen[i + l] = l;
          cDist[i + l] = d;
        }
      }
      prevL = l;
    }
  }
  // Backtrack.
  let count = 0;
  for (let i = N; i > 0; ) {
    const L = cLen[i];
    i -= L;
    count++;
  }
  let j = at + count;
  const end = j;
  for (let i = N; i > 0; ) {
    const L = cLen[i];
    j--;
    if (L === 1) {
      lit[j] = data[s + i - 1];
      dist[j] = 0;
    } else {
      lit[j] = L;
      dist[j] = cDist[i];
    }
    i -= L;
  }
  return end;
}

/** Greedy parse (longest match) over [s, e): the starting point of the cost iterations. */
function greedy(data, mf, s, e, lit, dist, at) {
  const { off, lens, dists } = mf;
  let j = at;
  for (let i = s; i < e; ) {
    const a = off[i];
    const b = off[i + 1];
    if (b > a) {
      let l = lens[b - 1];
      if (l > e - i) l = e - i;
      if (l >= 3) {
        // the nearest distance reaching l
        let k = a;
        while (lens[k] < l) k++;
        lit[j] = l;
        dist[j] = dists[k];
        j++;
        i += l;
        continue;
      }
    }
    lit[j] = data[i];
    dist[j] = 0;
    j++;
    i++;
  }
  return j;
}

// ---------------------------------------------------------------------------------------------------------------
// Block splitting over a symbol stream.

function bytesOf(lit, dist, a, b) {
  let n = 0;
  for (let i = a; i < b; i++) n += dist[i] === 0 ? 1 : lit[i];
  return n;
}

/** The split points (symbol indices) that lower the total bits, recursively, at most maxBlocks blocks. */
function splitPoints(lit, dist, a, b, minSyms, maxBlocks) {
  const blocks = [[a, b, blockBits(lit, dist, a, b), false]];
  for (;;) {
    if (blocks.length >= maxBlocks) break;
    // the largest unsplit-tried block
    let bi = -1;
    let bestSize = 0;
    for (let i = 0; i < blocks.length; i++) {
      const [x, y, , tried] = blocks[i];
      if (!tried && y - x > bestSize) {
        bestSize = y - x;
        bi = i;
      }
    }
    if (bi < 0 || bestSize < 2 * minSyms) break;
    const [x, y, whole] = blocks[bi];
    blocks[bi][3] = true;
    // Find the minimum of f(p) = bits(x,p) + bits(p,y) by successive narrowing over sampled points.
    let lo = x + minSyms;
    let hi = y - minSyms;
    let bestP = -1;
    let bestF = Infinity;
    const SAMPLES = 9;
    while (hi - lo > 0) {
      let localBest = -1;
      let localF = Infinity;
      const step = (hi - lo) / (SAMPLES + 1);
      for (let t = 1; t <= SAMPLES; t++) {
        const p = Math.round(lo + step * t);
        const f = blockBits(lit, dist, x, p) + blockBits(lit, dist, p, y);
        if (f < localF) {
          localF = f;
          localBest = p;
        }
      }
      if (localF < bestF) {
        bestF = localF;
        bestP = localBest;
      }
      if (step < 1) break;
      const nlo = Math.max(lo, Math.round(localBest - step));
      const nhi = Math.min(hi, Math.round(localBest + step));
      if (nhi - nlo < 4 || (nlo === lo && nhi === hi)) break;
      lo = nlo;
      hi = nhi;
    }
    if (bestP > 0 && bestF + 64 < whole) {
      const l = [x, bestP, blockBits(lit, dist, x, bestP), false];
      const r = [bestP, y, blockBits(lit, dist, bestP, y), false];
      blocks.splice(bi, 1, l, r);
    }
  }
  return blocks.map((bk) => bk[0]).slice(1);
}

// ---------------------------------------------------------------------------------------------------------------
// Bit output.

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
  /** Huffman code (MSB-first value) of nb bits. */
  code(v, nb) {
    let r = 0;
    for (let i = 0; i < nb; i++) {
      r = (r << 1) | (v & 1);
      v >>>= 1;
    }
    this.put(r, nb);
  }
  finish() {
    if (this.n > 0) this.put(0, 8 - this.n);
    return this.buf.subarray(0, this.pos);
  }
}

function canonical(lens, n) {
  const blCount = new Int32Array(16);
  for (let i = 0; i < n; i++) blCount[lens[i]]++;
  blCount[0] = 0;
  const nextCode = new Int32Array(16);
  let code = 0;
  for (let b = 1; b < 16; b++) {
    code = (code + blCount[b - 1]) << 1;
    nextCode[b] = code;
  }
  const codes = new Int32Array(n);
  for (let i = 0; i < n; i++) if (lens[i] !== 0) codes[i] = nextCode[lens[i]]++;
  return codes;
}

function writeBlock(out, lit, dist, a, b, last) {
  const extra = histogram(lit, dist, a, b);
  void extra;
  const ll = new Uint8Array(288);
  const dl = new Uint8Array(30);
  codeLengths(llC, 288, 15, ll);
  codeLengths(dC, 30, 15, dl);
  fixDistLens(dl);
  const plan = headerPlan(ll, dl);
  out.put(last ? 1 : 0, 1);
  out.put(2, 2);
  out.put(plan.hlit - 257, 5);
  out.put(plan.hdist - 1, 5);
  // Rebuild the chosen RLE and its code.
  const lensAll = new Uint8Array(plan.hlit + plan.hdist);
  for (let i = 0; i < plan.hlit; i++) lensAll[i] = ll[i];
  for (let i = 0; i < plan.hdist; i++) lensAll[plan.hlit + i] = dl[i];
  const v = plan.variant;
  const m = rleOf(lensAll, lensAll.length, v & 1, v & 2, v & 4);
  const cc = new Int32Array(19);
  for (let j = 0; j < m; j++) cc[rleSym[j]]++;
  const cl = new Uint8Array(19);
  codeLengths(cc, 19, 7, cl);
  let hclen = 19;
  while (hclen > 4 && cl[CL_ORDER[hclen - 1]] === 0) hclen--;
  out.put(hclen - 4, 4);
  for (let i = 0; i < hclen; i++) out.put(cl[CL_ORDER[i]], 3);
  const clCodes = canonical(cl, 19);
  for (let j = 0; j < m; j++) {
    const s = rleSym[j];
    out.code(clCodes[s], cl[s]);
    if (s === 16) out.put(rleExt[j], 2);
    else if (s === 17) out.put(rleExt[j], 3);
    else if (s === 18) out.put(rleExt[j], 7);
  }
  const llCodes = canonical(ll, 288);
  const dCodes = canonical(dl, 30);
  for (let i = a; i < b; i++) {
    const d = dist[i];
    if (d === 0) {
      const s = lit[i];
      out.code(llCodes[s], ll[s]);
    } else {
      const l = lit[i];
      const ls = LSYM[l];
      out.code(llCodes[257 + ls], ll[257 + ls]);
      if (LEN_EXTRA[ls]) out.put(l - LEN_BASE[ls], LEN_EXTRA[ls]);
      const ds = DSYM[d];
      out.code(dCodes[ds], dl[ds]);
      if (DIST_EXTRA[ds]) out.put(d - DIST_BASE[ds], DIST_EXTRA[ds]);
    }
  }
  out.code(llCodes[256], ll[256]);
}

// ---------------------------------------------------------------------------------------------------------------
// The encoder.

function statsOf(lit, dist, a, b, costs) {
  histogram(lit, dist, a, b);
  costs.fromCounts(llC, dC);
}

/**
 * Raw DEFLATE of `data`. opts: chain (match search depth), nice (a match this long ends the search and skips the
 * positions it covers), iters (global cost iterations), blockIters (per-block iterations), maxBlocks, mf (cached matches).
 */
export function deflateRaw(data, opts = {}) {
  const chain = opts.chain ?? 256;
  const nice = opts.nice ?? 258;
  const iters = opts.iters ?? 4;
  const blockIters = opts.blockIters ?? 4;
  const maxBlocks = opts.maxBlocks ?? 40;
  const patience = opts.patience ?? 2;
  const passes = opts.passes ?? 2;
  const n = data.length;
  const mf = opts.mf ?? findMatchesBT(data, chain, nice);
  const ctx = {
    data,
    mf,
    costs: new Costs(),
    work: { cost: new Float64Array(n + 1), cLen: new Uint16Array(n + 1), cDist: new Uint16Array(n + 1), cap: opts.cap ?? 258, ends: opts.ends ?? false },
    aL: new Uint16Array(n + 1),
    aD: new Uint16Array(n + 1),
    bL: new Uint16Array(n + 1),
    bD: new Uint16Array(n + 1),
    patience,
    mix: opts.mix ?? false,
  };

  // Global: greedy, then iterate the cost model.
  const gL = new Uint16Array(n + 1);
  const gD = new Uint16Array(n + 1);
  let m = greedy(data, mf, 0, n, gL, gD, 0);
  m = iterate(ctx, 0, n, gL, gD, m, iters).m;

  // Split, re-parse each block under its own model; then split that result again, and keep the better.
  const minSyms = opts.minSyms ?? 512;
  let best = splitPass(ctx, gL, gD, m, minSyms, maxBlocks, blockIters);
  for (let p = 1; p < passes; p++) {
    const next = splitPass(ctx, best.lit, best.dist, best.starts[best.starts.length - 1], minSyms, maxBlocks, blockIters);
    if (next.bits < best.bits) best = next;
    else break;
  }

  const out = new Bits(Math.max(1024, n >>> 3));
  const st = best.starts;
  for (let b = 0; b + 1 < st.length; b++) writeBlock(out, best.lit, best.dist, st[b], st[b + 1], b + 2 === st.length);
  return out.finish();
}

/**
 * Iterate the cost model over bytes [s, e), starting from the symbols in (L, D)[0, m): each round parses under the
 * statistics of the round before; the best parse by its block bits is kept in (L, D). Returns {m, bits}.
 */
const mixLL = new Int32Array(288);
const mixD = new Int32Array(30);

function iterate(ctx, s, e, L, D, m, iters) {
  const { data, mf, costs, work } = ctx;
  let bestBits = blockBits(L, D, 0, m);
  let curL = L;
  let curD = D;
  let curM = m;
  let tL = ctx.aL;
  let tD = ctx.aD;
  let oL = ctx.bL;
  let oD = ctx.bD;
  let fails = 0;
  let mix = false;
  let mixed = false;
  for (let it = 0; it < iters; it++) {
    if (mix) {
      // Stuck: the statistics of the best parse and the last one together.
      histogram(L, D, 0, m);
      mixLL.set(llC);
      mixD.set(dC);
      histogram(curL, curD, 0, curM);
      for (let k = 0; k < 288; k++) mixLL[k] += llC[k];
      for (let k = 0; k < 30; k++) mixD[k] += dC[k];
      costs.fromCounts(mixLL, mixD);
      mix = false;
    } else statsOf(curL, curD, 0, curM, costs);
    const m2 = parse(data, mf, s, e, costs, tL, tD, 0, work);
    const bits = blockBits(tL, tD, 0, m2);
    if (bits < bestBits) {
      bestBits = bits;
      L.set(tL.subarray(0, m2));
      D.set(tD.subarray(0, m2));
      m = m2;
      fails = 0;
    } else if (++fails >= ctx.patience) {
      if (mixed || !ctx.mix) break;
      mix = true;
      mixed = true;
      fails = 0;
    }
    curL = tL;
    curD = tD;
    curM = m2;
    [tL, oL] = [oL, tL];
    [tD, oD] = [oD, tD];
  }
  return { m, bits: bestBits };
}

/** Split symbols (L, D)[0, m) into blocks, re-parse each block under its own model: {lit, dist, starts, bits}. */
function splitPass(ctx, L, D, m, minSyms, maxBlocks, blockIters) {
  const n = ctx.data.length;
  const sp = splitPoints(L, D, 0, m, minSyms, maxBlocks);
  const symB = [0, ...sp, m];
  const outL = new Uint16Array(n + 1);
  const outD = new Uint16Array(n + 1);
  const starts = [0];
  let w = 0;
  let pos = 0;
  let bits = 0;
  for (let b = 0; b + 1 < symB.length; b++) {
    const a = symB[b];
    const z = symB[b + 1];
    const s = pos;
    const e = s + bytesOf(L, D, a, z);
    const bl = new Uint16Array(e - s + 1);
    const bd = new Uint16Array(e - s + 1);
    bl.set(L.subarray(a, z));
    bd.set(D.subarray(a, z));
    const r = iterate(ctx, s, e, bl, bd, z - a, blockIters);
    outL.set(bl.subarray(0, r.m), w);
    outD.set(bd.subarray(0, r.m), w);
    w += r.m;
    starts.push(w);
    bits += r.bits;
    pos = e;
  }
  return { lit: outL, dist: outD, starts, bits };
}

export function adler32(bytes) {
  let a = 1;
  let b = 0;
  for (let i = 0; i < bytes.length; ) {
    const end = Math.min(bytes.length, i + 5552);
    for (; i < end; i++) {
      a += bytes[i];
      b += a;
    }
    a %= 65521;
    b %= 65521;
  }
  return ((b << 16) | a) >>> 0;
}

/** A zlib stream around a raw DEFLATE stream of `data`. */
export function zlibWrap(raw, data) {
  const out = new Uint8Array(raw.length + 6);
  out[0] = 0x78;
  out[1] = 0xda;
  out.set(raw, 2);
  const ad = adler32(data);
  const at = raw.length + 2;
  out[at] = ad >>> 24;
  out[at + 1] = (ad >>> 16) & 0xff;
  out[at + 2] = (ad >>> 8) & 0xff;
  out[at + 3] = ad & 0xff;
  return out;
}
