// PNG reading and writing for this entry: a decoder for the input (any 8-bit non-interlaced PNG), the image's
// lossless forms (palette, grey, grey+alpha, RGB, RGBA, tRNS keys), the five filters and per-row filter strategies,
// and the chunk writer.

import { inflateSync } from 'node:zlib';

const CRC = (() => {
  const t = new Int32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c;
  }
  return t;
})();

export function crc32(bytes, start, end) {
  let c = -1;
  for (let i = start; i < end; i++) c = CRC[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ -1) >>> 0;
}

const CH = { 0: 1, 2: 3, 3: 1, 4: 2, 6: 4 };

/** RGBA pixels of a PNG (8-bit, non-interlaced). */
export function decode(bytes) {
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let at = 8;
  let w = 0;
  let h = 0;
  let ct = 6;
  let plte = null;
  let trns = null;
  const idat = [];
  while (at + 12 <= bytes.length) {
    const len = v.getUint32(at);
    const type = String.fromCharCode(bytes[at + 4], bytes[at + 5], bytes[at + 6], bytes[at + 7]);
    const data = bytes.subarray(at + 8, at + 8 + len);
    at += 12 + len;
    if (type === 'IHDR') {
      w = v.getUint32(16);
      h = v.getUint32(20);
      ct = data[9];
    } else if (type === 'PLTE') plte = data;
    else if (type === 'tRNS') trns = data;
    else if (type === 'IDAT') idat.push(data);
    else if (type === 'IEND') break;
  }
  const raw = inflateSync(Buffer.concat(idat));
  const bpp = CH[ct];
  const stride = w * bpp;
  const s = new Uint8Array(h * stride);
  for (let y = 0; y < h; y++) {
    const ft = raw[y * (stride + 1)];
    const src = y * (stride + 1) + 1;
    const o = y * stride;
    const p = o - stride;
    for (let i = 0; i < stride; i++) {
      const x = raw[src + i];
      const a = i >= bpp ? s[o + i - bpp] : 0;
      const b = y > 0 ? s[p + i] : 0;
      const c = y > 0 && i >= bpp ? s[p + i - bpp] : 0;
      let pr = 0;
      if (ft === 1) pr = a;
      else if (ft === 2) pr = b;
      else if (ft === 3) pr = (a + b) >> 1;
      else if (ft === 4) pr = paeth(a, b, c);
      s[o + i] = (x + pr) & 0xff;
    }
  }
  const n = w * h;
  const rgba = new Uint8Array(n * 4);
  for (let i = 0; i < n; i++) {
    const d = i * 4;
    if (ct === 6) {
      rgba[d] = s[d];
      rgba[d + 1] = s[d + 1];
      rgba[d + 2] = s[d + 2];
      rgba[d + 3] = s[d + 3];
    } else if (ct === 2) {
      const r = s[i * 3];
      const g = s[i * 3 + 1];
      const b = s[i * 3 + 2];
      rgba[d] = r;
      rgba[d + 1] = g;
      rgba[d + 2] = b;
      rgba[d + 3] = trns !== null && trns.length >= 6 && r === trns[1] && g === trns[3] && b === trns[5] ? 0 : 255;
    } else if (ct === 0) {
      const g = s[i];
      rgba[d] = rgba[d + 1] = rgba[d + 2] = g;
      rgba[d + 3] = trns !== null && trns.length >= 2 && g === trns[1] ? 0 : 255;
    } else if (ct === 4) {
      rgba[d] = rgba[d + 1] = rgba[d + 2] = s[i * 2];
      rgba[d + 3] = s[i * 2 + 1];
    } else {
      const e = s[i];
      rgba[d] = plte[e * 3];
      rgba[d + 1] = plte[e * 3 + 1];
      rgba[d + 2] = plte[e * 3 + 2];
      rgba[d + 3] = trns !== null && e < trns.length ? trns[e] : 255;
    }
  }
  return { width: w, height: h, rgba };
}

export function paeth(a, b, c) {
  const p = a + b - c;
  const pa = p > a ? p - a : a - p;
  const pb = p > b ? p - b : b - p;
  const pc = p > c ? p - c : c - p;
  if (pa <= pb && pa <= pc) return a;
  return pb <= pc ? b : c;
}

/** Row `cur` filtered by `type` into out (stride bytes). prev is null on the first row. */
export function filterRow(type, cur, prev, bpp, out) {
  const n = cur.length;
  if (type === 0) out.set(cur);
  else if (type === 1) {
    for (let i = 0; i < bpp; i++) out[i] = cur[i];
    for (let i = bpp; i < n; i++) out[i] = cur[i] - cur[i - bpp];
  } else if (type === 2) {
    if (prev === null) out.set(cur);
    else for (let i = 0; i < n; i++) out[i] = cur[i] - prev[i];
  } else if (type === 3) {
    if (prev === null) {
      for (let i = 0; i < bpp; i++) out[i] = cur[i];
      for (let i = bpp; i < n; i++) out[i] = cur[i] - (cur[i - bpp] >> 1);
    } else {
      for (let i = 0; i < bpp; i++) out[i] = cur[i] - (prev[i] >> 1);
      for (let i = bpp; i < n; i++) out[i] = cur[i] - ((cur[i - bpp] + prev[i]) >> 1);
    }
  } else {
    if (prev === null) {
      for (let i = 0; i < bpp; i++) out[i] = cur[i];
      for (let i = bpp; i < n; i++) out[i] = cur[i] - cur[i - bpp];
    } else {
      for (let i = 0; i < bpp; i++) out[i] = cur[i] - prev[i];
      for (let i = bpp; i < n; i++) {
        const a = cur[i - bpp];
        const b = prev[i];
        const c = prev[i - bpp];
        const p = a + b - c;
        const pa = p > a ? p - a : a - p;
        const pb = p > b ? p - b : b - p;
        const pc = p > c ? p - c : c - p;
        out[i] = cur[i] - (pa <= pb && pa <= pc ? a : pb <= pc ? b : c);
      }
    }
  }
}

/** Image analysis: opaque, grey, and the colours (32-bit words with counts) when at most 256. */
export function analyse(rgba) {
  const n = rgba.length >>> 2;
  const u32 = new Uint32Array(rgba.buffer, rgba.byteOffset, n);
  let opaque = true;
  let grey = true;
  const counts = new Map();
  let many = false;
  for (let i = 0, d = 0; i < n; i++, d += 4) {
    if (rgba[d + 3] !== 255) opaque = false;
    if (rgba[d] !== rgba[d + 1] || rgba[d] !== rgba[d + 2]) grey = false;
    if (!many) {
      const c = u32[i];
      const k = counts.get(c);
      if (k === undefined) {
        if (counts.size === 256) many = true;
        else counts.set(c, 1);
      } else counts.set(c, k + 1);
    }
  }
  let colours = null;
  if (!many) {
    colours = [];
    for (const [word, count] of counts) {
      const b = new Uint8Array(Uint32Array.of(word).buffer);
      colours.push({ word, count, r: b[0], g: b[1], b: b[2], a: b[3] });
    }
    colours.sort((x, y) => x.word - y.word);
  }
  return { opaque, grey, colours };
}

/** The single transparent key value (grey or RGB) the image may use instead of an alpha channel, or null. */
export function transparentKey(rgba, grey) {
  let key = -1;
  for (let d = 0; d < rgba.length; d += 4) {
    const a = rgba[d + 3];
    if (a === 255) continue;
    if (a !== 0) return null;
    const v = grey ? rgba[d] : (rgba[d] << 16) | (rgba[d + 1] << 8) | rgba[d + 2];
    if (key === -1) key = v;
    else if (key !== v) return null;
  }
  if (key === -1) return null;
  for (let d = 0; d < rgba.length; d += 4) {
    if (rgba[d + 3] !== 255) continue;
    const v = grey ? rgba[d] : (rgba[d] << 16) | (rgba[d + 1] << 8) | rgba[d + 2];
    if (v === key) return null;
  }
  return key;
}

export function samplesOf(rgba, ct) {
  const n = rgba.length >>> 2;
  if (ct === 6) return rgba;
  const ch = ct === 0 ? 1 : ct === 4 ? 2 : 3;
  const out = new Uint8Array(n * ch);
  for (let i = 0, d = 0, s = 0; i < n; i++, d += 4, s += ch) {
    if (ct === 0) out[s] = rgba[d];
    else if (ct === 4) {
      out[s] = rgba[d];
      out[s + 1] = rgba[d + 3];
    } else {
      out[s] = rgba[d];
      out[s + 1] = rgba[d + 1];
      out[s + 2] = rgba[d + 2];
    }
  }
  return out;
}

/** A palette form from colour entries in the given order: transparent entries are moved first (stable). */
export function paletteForm(rgba, entries) {
  const ordered = [...entries.filter((c) => c.a !== 255), ...entries.filter((c) => c.a === 255)];
  const index = new Map();
  ordered.forEach((c, i) => index.set(c.word, i));
  const plte = new Uint8Array(ordered.length * 3);
  let nt = 0;
  ordered.forEach((c, i) => {
    plte[i * 3] = c.r;
    plte[i * 3 + 1] = c.g;
    plte[i * 3 + 2] = c.b;
    if (c.a !== 255) nt = i + 1;
  });
  const trns = nt === 0 ? null : Uint8Array.from(ordered.slice(0, nt), (c) => c.a);
  const n = rgba.length >>> 2;
  const u32 = new Uint32Array(rgba.buffer, rgba.byteOffset, n);
  const lut = new Map(index);
  const idx = new Uint8Array(n);
  let last = -1;
  let lastI = 0;
  for (let i = 0; i < n; i++) {
    const c = u32[i];
    if (c !== last) {
      last = c;
      lastI = lut.get(c);
    }
    idx[i] = lastI;
  }
  return { colourType: 3, bpp: 1, samples: idx, plte, trns };
}

// ---------------------------------------------------------------------------------------------------------------
// Filtering strategies.

function signedSum(row) {
  let s = 0;
  for (let i = 0; i < row.length; i++) {
    const v = row[i];
    s += v < 128 ? v : 256 - v;
  }
  return s;
}

const entCount = new Int32Array(256);
const LOG2 = (() => {
  const t = new Float64Array(8193);
  for (let i = 1; i <= 8192; i++) t[i] = i * Math.log2(i);
  return t;
})();
function rowEntropy(row) {
  entCount.fill(0);
  for (let i = 0; i < row.length; i++) entCount[row[i]]++;
  let e = 0;
  for (let i = 0; i < 256; i++) {
    const c = entCount[i];
    if (c > 1) e -= c <= 8192 ? LOG2[c] : c * Math.log2(c);
  }
  return e;
}

/** Filter the image with one fixed filter per row given by `types` (Uint8Array of height). */
export function applyFilters(samples, width, height, bpp, types) {
  const stride = width * bpp;
  const raw = new Uint8Array(height * (stride + 1));
  for (let y = 0; y < height; y++) {
    const cur = samples.subarray(y * stride, (y + 1) * stride);
    const prev = y === 0 ? null : samples.subarray((y - 1) * stride, y * stride);
    const at = y * (stride + 1);
    raw[at] = types[y];
    filterRow(types[y], cur, prev, bpp, raw.subarray(at + 1, at + 1 + stride));
  }
  return raw;
}

/** Per-row filter types by a heuristic: 'minsum' or 'entropy'. */
export function heuristicTypes(samples, width, height, bpp, kind) {
  const stride = width * bpp;
  const types = new Uint8Array(height);
  const trial = new Uint8Array(stride);
  for (let y = 0; y < height; y++) {
    const cur = samples.subarray(y * stride, (y + 1) * stride);
    const prev = y === 0 ? null : samples.subarray((y - 1) * stride, y * stride);
    let best = Infinity;
    let bt = 0;
    for (let t = 0; t < 5; t++) {
      filterRow(t, cur, prev, bpp, trial);
      const c = kind === 'minsum' ? signedSum(trial) : rowEntropy(trial);
      if (c < best) {
        best = c;
        bt = t;
      }
    }
    types[y] = bt;
  }
  return types;
}

// ---------------------------------------------------------------------------------------------------------------
// Writing.

const ascii = (s) => Uint8Array.from(s, (ch) => ch.charCodeAt(0));

export function pngOf(width, height, form, idat) {
  const ihdr = new Uint8Array(13);
  const dv = new DataView(ihdr.buffer);
  dv.setUint32(0, width);
  dv.setUint32(4, height);
  ihdr[8] = 8;
  ihdr[9] = form.colourType;
  const chunks = [['IHDR', ihdr]];
  if (form.plte) chunks.push(['PLTE', form.plte]);
  if (form.trns) chunks.push(['tRNS', form.trns]);
  chunks.push(['IDAT', idat]);
  chunks.push(['IEND', new Uint8Array(0)]);
  let size = 8;
  for (const [, d] of chunks) size += 12 + d.length;
  const out = new Uint8Array(size);
  const v = new DataView(out.buffer);
  out.set([137, 80, 78, 71, 13, 10, 26, 10], 0);
  let at = 8;
  for (const [type, d] of chunks) {
    v.setUint32(at, d.length);
    out.set(ascii(type), at + 4);
    out.set(d, at + 8);
    v.setUint32(at + 8 + d.length, crc32(out, at + 4, at + 8 + d.length));
    at += 12 + d.length;
  }
  return out;
}

export function chunkOverhead(form) {
  return (form.plte ? 12 + form.plte.length : 0) + (form.trns ? 12 + form.trns.length : 0);
}
