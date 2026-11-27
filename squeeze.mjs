// png-squeeze: each screenshot rewritten as the smallest lossless PNG this program finds (main.ts runs this file).
//   1. Forms: a palette when the image has at most 256 colours (and is not grey); grey, grey+alpha, RGB or RGBA
//      otherwise, with a tRNS key in place of alpha when one fully transparent value allows it.
//   2. Filters: filter 0, and one filter a band of 16 rows chosen by zlib with the 32 KiB before the band as its
//      dictionary (grey images also a per-row brute force; a palette filter 0 and the minimum sum), screened by zlib
//      at level 6. A palette's two candidates are both written, the smaller kept.
//   3. The winner written by this entry's own DEFLATE (deflate.mjs): binary-tree matches, an optimal parse under an
//      iterated cost model started from the data's own byte statistics, and block splitting. The stream is inflated
//      back and compared before it is used.
// No clock, randomness or environment (beyond the input and output directories) is read: the work is fixed by the
// input, so two runs write the same bytes.
import { copyFileSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { deflateSync, inflateRawSync } from 'node:zlib';
import { decode, analyse, transparentKey, samplesOf, paletteForm, applyFilters, heuristicTypes, filterRow, pngOf, chunkOverhead } from './png.mjs';
import { deflateRaw, zlibWrap } from './deflate.mjs';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const SCREEN = { level: 6, memLevel: 9 };
const TOP2 = 0;
const DEFLATE = { chain: 256, nice: 300, cap: 8, ends: true, fresh: true, init: 'static', iters: 2, blockIters: 2, maxBlocks: 40, minSyms: 1024, patience: 1, passes: 1 };

/**
 * Per-row brute force: each row's filter the one whose filtered row zlib (at `level`) codes smallest after the 32 KiB
 * already chosen before it; `bias` bytes are taken off the row above's filter, so runs of one filter are favoured.
 */
function bruteTypes(samples, width, height, bpp, level, bias) {
  const stride = width * bpp;
  const raw = new Uint8Array(height * (stride + 1));
  const types = new Uint8Array(height);
  const trial = [];
  for (let t = 0; t < 5; t++) {
    const r = new Uint8Array(stride + 1);
    r[0] = t;
    trial.push(r);
  }
  for (let y = 0; y < height; y++) {
    const cur = samples.subarray(y * stride, (y + 1) * stride);
    const prev = y === 0 ? null : samples.subarray((y - 1) * stride, y * stride);
    const at = y * (stride + 1);
    const before = raw.subarray(Math.max(0, at - 32768), at);
    let best = Infinity;
    let bt = 0;
    for (let t = 0; t < 5; t++) {
      filterRow(t, cur, prev, bpp, trial[t].subarray(1));
      const opt = before.length ? { level, memLevel: 9, dictionary: before } : { level, memLevel: 9 };
      let c = deflateSync(trial[t], opt).length;
      if (bias && y > 0 && t === types[y - 1]) c -= bias;
      if (c < best) {
        best = c;
        bt = t;
      }
    }
    types[y] = bt;
    raw.set(trial[bt], at);
  }
  return types;
}

/**
 * Per-band brute force: rows in bands of `band`, each band one filter, the one whose filtered band zlib (at `level`)
 * codes smallest after the 32 KiB already chosen before it. A filter kept over many rows lets LZ match a row against
 * the rows above it filtered the same way, which a row-by-row choice cannot see.
 */
function bandTypes(samples, width, height, bpp, band, level) {
  const stride = width * bpp;
  const rowLen = stride + 1;
  const raw = new Uint8Array(height * rowLen);
  const types = new Uint8Array(height);
  const trial = new Uint8Array(band * rowLen);
  const keep = new Uint8Array(band * rowLen);
  for (let y0 = 0; y0 < height; y0 += band) {
    const y1 = Math.min(height, y0 + band);
    const at = y0 * rowLen;
    const len = (y1 - y0) * rowLen;
    const dict = raw.subarray(Math.max(0, at - 32768), at);
    let best = Infinity;
    let bt = 0;
    for (let t = 0; t < 5; t++) {
      for (let y = y0; y < y1; y++) {
        const o = (y - y0) * rowLen;
        trial[o] = t;
        filterRow(t, samples.subarray(y * stride, (y + 1) * stride), y === 0 ? null : samples.subarray((y - 1) * stride, y * stride), bpp, trial.subarray(o + 1, o + rowLen));
      }
      const view = trial.subarray(0, len);
      const c = deflateSync(view, dict.length ? { level, memLevel: 9, dictionary: dict } : { level, memLevel: 9 }).length;
      if (c < best) {
        best = c;
        bt = t;
        keep.set(view);
      }
    }
    raw.set(keep.subarray(0, len), at);
    for (let y = y0; y < y1; y++) types[y] = bt;
  }
  return types;
}
/** The forms an image can be written in, each with the filter strategies to screen it with. */
function formsOf(rgba) {
  const { opaque, grey, colours } = analyse(rgba);
  if (colours !== null && !grey) {
    // With filter 0 a palette's order changes nothing but its chunks; by count is kept, by luminance is filtered.
    const byCount = [...colours].sort((x, y) => y.count - x.count || x.word - y.word);
    const lum = (c) => 299 * c.r + 587 * c.g + 114 * c.b;
    const byLum = [...colours].sort((x, y) => lum(x) - lum(y) || x.word - y.word);
    return [
      { ...paletteForm(rgba, byCount), filters: [0] },
      { ...paletteForm(rgba, byLum), filters: ['minsum'] },
    ];
  }
  const key = opaque ? null : transparentKey(rgba, grey);
  let ct;
  let trns = null;
  if (grey) {
    ct = opaque || key !== null ? 0 : 4;
    if (key !== null) trns = Uint8Array.of(0, key);
  } else {
    ct = opaque || key !== null ? 2 : 6;
    if (key !== null) trns = Uint8Array.of(0, key >>> 16, 0, (key >>> 8) & 0xff, 0, key & 0xff);
  }
  const bpp = { 0: 1, 2: 3, 4: 2, 6: 4 }[ct];
  return [{ colourType: ct, bpp, samples: samplesOf(rgba, ct), plte: null, trns, filters: grey ? [0, 'brute4', 'band16'] : [0, 'band16'] }];
}

function typesOf(form, width, height, f) {
  if (typeof f === 'number') return new Uint8Array(height).fill(f);
  if (f === 'brute4') return bruteTypes(form.samples, width, height, form.bpp, 4, 0);
  if (f === 'band16') return bandTypes(form.samples, width, height, form.bpp, 16, 6);
  if (f === 'band32') return bandTypes(form.samples, width, height, form.bpp, 32, 6);
  return heuristicTypes(form.samples, width, height, form.bpp, f);
}

/** The screen's winner for an image: {width, height, form, raw (filtered data), z (zlib's stream), size}. */
function screen(rgba, width, height) {
  const all = [];
  for (const form of formsOf(rgba)) {
    const extra = chunkOverhead(form);
    for (const f of form.filters) {
      const raw = applyFilters(form.samples, width, height, form.bpp, typesOf(form, width, height, f));
      const z = deflateSync(raw, SCREEN);
      all.push({ size: z.length + extra, raw, form: { colourType: form.colourType, plte: form.plte, trns: form.trns }, z, width, height });
    }
  }
  // The smallest first, the earlier on a tie; the runner-up is written too when zlib puts it within TOP2 of the best.
  const order = all.map((c, k) => k).sort((x, y) => all[x].size - all[y].size || x - y);
  const best = all[order[0]];
  // A palette's two candidates are both written (zlib ranks them unlike this entry's DEFLATE, and they are small).
  best.second = order.length > 1 && (best.form.colourType === 3 || all[order[1]].size <= best.size * TOP2) ? all[order[1]] : null;
  return best;
}

/** The PNG of a screened image with its data in this entry's DEFLATE (checked by inflating it back), else zlib's. */
function encodeOne(best, settings) {
  let idat = new Uint8Array(best.z.buffer, best.z.byteOffset, best.z.length);
  try {
    const raw = deflateRaw(best.raw, settings);
    const back = inflateRawSync(raw);
    if (Buffer.compare(back, Buffer.from(best.raw.buffer, best.raw.byteOffset, best.raw.length)) === 0 && raw.length + 6 < idat.length) {
      idat = zlibWrap(raw, best.raw);
    }
  } catch {
    // zlib's stream stands.
  }
  return pngOf(best.width, best.height, best.form, idat);
}

/** The smaller of the best's PNG and, when there is one, the runner-up's. */
function encode(best, settings, both) {
  const a = encodeOne(best, settings);
  if (!both || best.second === null) return a;
  const b = encodeOne(best.second, settings);
  return b.length < a.length ? b : a;
}

// The work budget, counted in the input's bytes, never the clock: DEFLATE's cost grows with the filtered bytes and,
// more steeply, with the bytes zlib could not squeeze. When the set's estimate passes BUDGET, the costliest images are
// written with LIGHT settings (about half the work) until it fits.
const BUDGET = 19000;
const LIGHT = { ...DEFLATE, iters: 2, blockIters: 1, maxBlocks: 20, cap: 8, ends: false };
const one = (s) => 0.8 * (s.raw.length / 1024) + 7 * (s.size / 1024);
const costOf = (s) => one(s) + (s.second ? one(s.second) : 0);

// Phase 1: screen every image and write zlib's PNG of its winner at once, so every instance holds an answer.
const ids = readdirSync(I).sort();
const screened = new Map();
for (const id of ids) {
  const src = join(I, id, 'image.png');
  mkdirSync(join(O, id), { recursive: true });
  try {
    const img = decode(new Uint8Array(readFileSync(src)));
    const s = screen(img.rgba, img.width, img.height);
    writeFileSync(join(O, id, 'image.png'), pngOf(s.width, s.height, s.form, new Uint8Array(s.z.buffer, s.z.byteOffset, s.z.length)));
    screened.set(id, s);
  } catch {
    copyFileSync(src, join(O, id, 'image.png'));
  }
}

// Phase 2: plan the work, then write each image with this entry's DEFLATE.
const light = new Set();
let total = 0;
for (const s of screened.values()) total += costOf(s);
for (const [id, s] of [...screened].sort((a, b) => costOf(b[1]) - costOf(a[1]) || (a[0] < b[0] ? -1 : 1))) {
  if (total <= BUDGET) break;
  light.add(id);
  total -= costOf(s) / 2;
}
for (const [id, s] of screened) {
  try {
    writeFileSync(join(O, id, 'image.png'), encode(s, light.has(id) ? LIGHT : DEFLATE, !light.has(id)));
  } catch {
    // The zlib answer written in phase 1 stands.
  }
}
