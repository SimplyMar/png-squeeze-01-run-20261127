// Self-assembled inline WebAssembly engine for BT3+2D-Stride match finding and Exact Pareto-Frontier O(1) RMQ DP.
// Features:
//  - Fast 3-byte Binary Search Tree (BT3) + explicit 2D-stride (bpp, rowLen, rowLen+-bpp, 2*rowLen) insertion-sorted by length
//  - Exact Prefix-Dominance / Pareto-Frontier pruning in O(1) RMQ parseRMQ (every length evaluated at most once even
//    with arbitrary non-monotonic distCost)
//  - Exact multi-block deep-run skipping supporting optimal skipDist in 1..4
// Pure deterministic integer arithmetic, zero external dependencies, zero prebuilt binary blobs.

const uleb = (v) => {
  const out = [];
  do {
    let b = v & 0x7f;
    v >>>= 7;
    if (v !== 0) b |= 0x80;
    out.push(b);
  } while (v !== 0);
  return out;
};

const sleb = (v) => {
  const out = [];
  let more = true;
  while (more) {
    let b = v & 0x7f;
    v >>= 7;
    if ((v === 0 && (b & 0x40) === 0) || (v === -1 && (b & 0x40) !== 0)) more = false;
    else b |= 0x80;
    out.push(b);
  }
  return out;
};

const sleb64 = (v) => {
  let x = BigInt(v);
  const out = [];
  let more = true;
  while (more) {
    let b = Number(x & 0x7fn);
    x >>= 7n;
    if ((x === 0n && (b & 0x40) === 0) || (x === -1n && (b & 0x40) !== 0)) more = false;
    else b |= 0x80;
    out.push(b);
  }
  return out;
};

const sec = (id, body) => [id, ...uleb(body.length), ...body];
const vec = (items) => [...uleb(items.length), ...items.flat()];
const str = (s) => [...uleb(s.length), ...Array.from(s, (c) => c.charCodeAt(0))];

const I32 = 0x7f;
const I64 = 0x7e;
const VOID = 0x40;

const OP = {
  block: 0x02, loop: 0x03, if: 0x04, else: 0x05, end: 0x0b,
  br: 0x0c, br_if: 0x0d, ret: 0x0f, select: 0x1b,
  lget: 0x20, lset: 0x21, ltee: 0x22,
  i32_load: 0x28, i64_load: 0x29, i32_load8_u: 0x2d, i32_load16_u: 0x2f,
  i32_store: 0x36, i64_store: 0x37, i32_store16: 0x3b,
  i32_const: 0x41, i64_const: 0x42,
  i32_eqz: 0x45, i32_eq: 0x46, i32_ne: 0x47, i32_lt_s: 0x48, i32_lt_u: 0x49,
  i32_gt_s: 0x4a, i32_gt_u: 0x4b, i32_le_s: 0x4c, i32_le_u: 0x4d, i32_ge_s: 0x4e, i32_ge_u: 0x4f,
  i64_lt_u: 0x53,
  i32_add: 0x6a, i32_sub: 0x6b, i32_mul: 0x6c, i32_div_u: 0x6e,
  i32_and: 0x71, i32_or: 0x72, i32_shl: 0x74, i32_shr_u: 0x76,
  i64_add: 0x7c, i64_mul: 0x7e, i64_and: 0x83, i64_or: 0x84, i64_shl: 0x86, i64_shr_u: 0x88,
  i32_wrap_i64: 0xa7, i64_extend_i32_u: 0xad,
};

const emit = () => {
  const b = [];
  const p = (...xs) => { for (const x of xs) b.push(x); };
  const c32 = (v) => { p(OP.i32_const, ...sleb(v)); };
  const c64 = (v) => { p(OP.i64_const, ...sleb64(v)); };
  const lg = (i) => { p(OP.lget, ...uleb(i)); };
  const ls = (i) => { p(OP.lset, ...uleb(i)); };
  const lt = (i) => { p(OP.ltee, ...uleb(i)); };
  const ld32 = (off = 0) => { p(OP.i32_load, 2, ...uleb(off)); };
  const ld32u = (off = 0) => { p(OP.i32_load, 0, ...uleb(off)); };
  const ld64 = (off = 0) => { p(OP.i64_load, 3, ...uleb(off)); };
  const ld8u = (off = 0) => { p(OP.i32_load8_u, 0, ...uleb(off)); };
  const ld16u = (off = 0) => { p(OP.i32_load16_u, 1, ...uleb(off)); };
  const st32 = (off = 0) => { p(OP.i32_store, 2, ...uleb(off)); };
  const st64 = (off = 0) => { p(OP.i64_store, 3, ...uleb(off)); };
  const st16 = (off = 0) => { p(OP.i32_store16, 1, ...uleb(off)); };
  return { b, p, c32, c64, lg, ls, lt, ld32, ld32u, ld64, ld8u, ld16u, st32, st64, st16 };
};

export const OFF_DSYM = 0;            // 32769 bytes (u8)
export const OFF_LSYM = 32776;        // 259 bytes (u8)
export const OFF_LCODE_END = 33040;   // 32 * 2 bytes (u16)
export const OFF_LOG2 = 33104;        // 36 bytes (u8)
export const OFF_LIT_COST = 33152;    // 256 * 4 bytes (i32)
export const OFF_LEN_COST = 34176;    // 32 * 4 bytes (i32)
export const OFF_DIST_COST = 34304;   // 32 * 4 bytes (i32)
export const OFF_RMQ = 34432;         // 6 * 512 * 8 = 24576 bytes (i64) -> ends 59008
export const OFF_SCRATCH = 59008;     // 320 * 8 = 2560 bytes (i64) -> ends 61568
export const OFF_PARETO_PK = 61568;   // 320 * 4 = 1280 bytes (i32: (dist<<9)|len) -> ends 62848
export const OFF_PARETO_DC = 62848;   // 320 * 8 = 2560 bytes (i64: distCost) -> ends 65408
export const OFF_HEAD = 65536;        // (1 << 19) * 4 = 2097152 bytes (i32) -> ends 2162688
export const OFF_SON = 2162688;       // 2 * 65536 * 4 = 524288 bytes (i32) -> ends 2686976
export const OFF_DYN = 2700000;

const HBUCKETS = 1 << 19;

// ---------------------------------------------------------------------------------------------------------------
// Function 0: bt3(dataPtr, n, offPtr, matchPtr, matchCap, cut, nice, bpp, rowLen) -> m (or -1 on overflow)
function buildBT3() {
  const e = emit();
  const { p, c32, lg, ls, lt, ld32, ld32u, ld8u, st32 } = e;
  const P_DATA = 0, P_N = 1, P_OFF = 2, P_MATCH = 3, P_CAP = 4, P_CUT = 5, P_NICE = 6, P_BPP = 7, P_ROW = 8;
  const L_I = 9, L_M = 10, L_SKIP = 11, L_KD = 12, L_KL = 13, L_LIM = 14, L_CP = 15, L_REC = 16;
  const L_H = 17, L_CUR = 18, L_P0 = 19, L_P1 = 20, L_L0 = 21, L_L1 = 22, L_BEST = 23, L_BD = 24;
  const L_CNT = 25, L_PAIR = 26, L_LEN = 27, L_B1 = 28, L_B2 = 29, L_AI = 30, L_AC = 31;
  const L_SFLAG = 32, L_K = 33, L_A0 = 34, L_MS = 35, L_TD = 36, L_VAL = 37, L_J = 38, L_B3 = 39;

  // Helper: test explicit distance in L_TD (1..32768) and emit if match >= 3
  // mode 0: always emit; mode 1: emit if len > best || bd > 4; mode 2: emit only if len > best
  const emitExplicitDist = (mode) => {
    lg(L_I); lg(L_TD); p(OP.i32_ge_u);
    lg(L_BD); lg(L_TD); p(OP.i32_ne); p(OP.i32_and);
    p(OP.if, VOID);
      lg(L_AI); lg(L_TD); p(OP.i32_sub); ls(L_AC);
      lg(L_AC); ld8u(0); lg(L_AI); ld8u(0); p(OP.i32_eq);
      lg(L_AC); ld8u(1); lg(L_AI); ld8u(1); p(OP.i32_eq); p(OP.i32_and);
      lg(L_AC); ld8u(2); lg(L_AI); ld8u(2); p(OP.i32_eq); p(OP.i32_and);
      p(OP.if, VOID);
        c32(3); ls(L_LEN);
        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_LEN); c32(4); p(OP.i32_add); lg(L_LIM); p(OP.i32_gt_u); p(OP.br_if, 1);
          lg(L_AC); lg(L_LEN); p(OP.i32_add); ld32u();
          lg(L_AI); lg(L_LEN); p(OP.i32_add); ld32u();
          p(OP.i32_ne); p(OP.br_if, 1);
          lg(L_LEN); c32(4); p(OP.i32_add); ls(L_LEN);
          p(OP.br, 0);
        p(OP.end);
        p(OP.end);
        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_LEN); lg(L_LIM); p(OP.i32_ge_u); p(OP.br_if, 1);
          lg(L_AC); lg(L_LEN); p(OP.i32_add); ld8u();
          lg(L_AI); lg(L_LEN); p(OP.i32_add); ld8u();
          p(OP.i32_ne); p(OP.br_if, 1);
          lg(L_LEN); c32(1); p(OP.i32_add); ls(L_LEN);
          p(OP.br, 0);
        p(OP.end);
        p(OP.end);

        if (mode === 0) {
          lg(L_M); lg(P_CAP); p(OP.i32_ge_u);
          p(OP.if, VOID); c32(-1); p(OP.ret); p(OP.end);
          lg(P_MATCH); lg(L_M); c32(2); p(OP.i32_shl); p(OP.i32_add);
          lg(L_TD); c32(9); p(OP.i32_shl); lg(L_LEN); p(OP.i32_or);
          st32();
          lg(L_M); c32(1); p(OP.i32_add); ls(L_M);
        } else {
          if (mode === 2) {
            lg(L_LEN); lg(L_BEST); p(OP.i32_gt_u);
          } else {
            lg(L_LEN); lg(L_BEST); p(OP.i32_gt_u);
            lg(L_BD); c32(4); p(OP.i32_gt_u); p(OP.i32_or);
          }
          p(OP.if, VOID);
            lg(L_M); lg(P_CAP); p(OP.i32_ge_u);
            p(OP.if, VOID); c32(-1); p(OP.ret); p(OP.end);
            lg(P_MATCH); lg(L_M); c32(2); p(OP.i32_shl); p(OP.i32_add);
            lg(L_TD); c32(9); p(OP.i32_shl); lg(L_LEN); p(OP.i32_or);
            st32();
            lg(L_M); c32(1); p(OP.i32_add); ls(L_M);
          p(OP.end);
        }
      p(OP.end);
    p(OP.end);
  };

  c32(0); ls(L_I);
  p(OP.loop, VOID);
    lg(L_I); c32(2); p(OP.i32_shl); c32(OFF_HEAD); p(OP.i32_add); c32(-1); st32();
    lg(L_I); c32(1); p(OP.i32_add); lt(L_I); c32(HBUCKETS); p(OP.i32_lt_u); p(OP.br_if, 0);
  p(OP.end);

  c32(0); ls(L_I);
  c32(0); ls(L_M);
  c32(0); ls(L_SKIP);
  c32(0); ls(L_KD);
  c32(0); ls(L_KL);

  p(OP.block, VOID);
  p(OP.loop, VOID);
    lg(L_I); lg(P_N); p(OP.i32_ge_u); p(OP.br_if, 1);

    lg(L_M); ls(L_A0);
    lg(P_OFF); lg(L_I); c32(2); p(OP.i32_shl); p(OP.i32_add); lg(L_M); st32();

    lg(P_N); lg(L_I); p(OP.i32_sub); lt(L_LIM);
    c32(258); p(OP.i32_gt_u);
    p(OP.if, VOID); c32(258); ls(L_LIM); p(OP.end);

    lg(L_I); c32(65535); p(OP.i32_and); c32(1); p(OP.i32_shl); ls(L_CP);

    lg(L_LIM); c32(3); p(OP.i32_lt_u);
    p(OP.if, VOID);
      c32(0); ls(L_KL);
      c32(OFF_SON); lg(L_CP); c32(2); p(OP.i32_shl); p(OP.i32_add); lt(L_P1); c32(-1); st32(0);
      lg(L_P1); c32(-1); st32(4);
    p(OP.else);
      lg(L_I); lg(L_SKIP); p(OP.i32_ge_u); ls(L_REC);
      lg(P_DATA); lg(L_I); p(OP.i32_add); ls(L_AI);

      // Fast path for continuing a dist=1, len=258 constant run
      c32(0); ls(L_CNT);
      lg(L_KD); c32(1); p(OP.i32_eq);
      lg(L_KL); c32(257); p(OP.i32_eq); p(OP.i32_and);
      lg(L_LIM); c32(258); p(OP.i32_eq); p(OP.i32_and);
      p(OP.if, VOID);
        lg(L_AI); ld8u(257);
        lg(L_AI); c32(1); p(OP.i32_sub); ld8u();
        p(OP.i32_eq);
        p(OP.if, VOID); c32(1); ls(L_CNT); p(OP.end);
      p(OP.end);

      lg(L_CNT);
      p(OP.if, VOID);
        lg(L_AI); ld8u(0); ls(L_B1);
        lg(L_REC);
        p(OP.if, VOID);
          lg(L_M); c32(2); p(OP.i32_add); lg(P_CAP); p(OP.i32_gt_u);
          p(OP.if, VOID); c32(-1); p(OP.ret); p(OP.end);
          lg(P_MATCH); lg(L_M); c32(2); p(OP.i32_shl); p(OP.i32_add);
          c32((1 << 9) | 258); st32();
          lg(L_M); c32(1); p(OP.i32_add); ls(L_M);

          // If bpp > 1 and the previous bpp bytes also equal b1, emit (bpp, 258) too
          lg(P_BPP); c32(1); p(OP.i32_gt_u);
          lg(L_I); lg(P_BPP); p(OP.i32_ge_u); p(OP.i32_and);
          p(OP.if, VOID);
            c32(1); ls(L_K);
            c32(1); ls(L_SFLAG);
            p(OP.block, VOID);
            p(OP.loop, VOID);
              lg(L_K); lg(P_BPP); p(OP.i32_gt_u); p(OP.br_if, 1);
              lg(L_AI); lg(L_K); p(OP.i32_sub); ld8u(); lg(L_B1); p(OP.i32_ne);
              p(OP.if, VOID); c32(0); ls(L_SFLAG); p(OP.br, 2); p(OP.end);
              lg(L_K); c32(1); p(OP.i32_add); ls(L_K);
              p(OP.br, 0);
            p(OP.end);
            p(OP.end);
            lg(L_SFLAG);
            p(OP.if, VOID);
              lg(P_MATCH); lg(L_M); c32(2); p(OP.i32_shl); p(OP.i32_add);
              lg(P_BPP); c32(9); p(OP.i32_shl); c32(258); p(OP.i32_or); st32();
              lg(L_M); c32(1); p(OP.i32_add); ls(L_M);
            p(OP.end);
          p(OP.end);
        p(OP.end);

        lg(L_B1); c32(16); p(OP.i32_shl);
        lg(L_B1); c32(8); p(OP.i32_shl); p(OP.i32_or);
        lg(L_B1); p(OP.i32_or);
        c32(-1640531535); p(OP.i32_mul);
        c32(13); p(OP.i32_shr_u);
        c32(2); p(OP.i32_shl); c32(OFF_HEAD); p(OP.i32_add); lg(L_I); st32();

        c32(OFF_SON); lg(L_I); c32(1); p(OP.i32_sub); c32(65535); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add); ls(L_PAIR);
        c32(OFF_SON); lg(L_CP); c32(2); p(OP.i32_shl); p(OP.i32_add); lt(L_P1);
        lg(L_PAIR); ld32(0); st32(0);
        lg(L_P1); lg(L_PAIR); ld32(4); st32(4);
      p(OP.else);
        lg(L_AI); ld8u(0); c32(16); p(OP.i32_shl);
        lg(L_AI); ld8u(1); c32(8); p(OP.i32_shl); p(OP.i32_or);
        lg(L_AI); ld8u(2); p(OP.i32_or);
        c32(-1640531535); p(OP.i32_mul);
        c32(13); p(OP.i32_shr_u);
        c32(2); p(OP.i32_shl); c32(OFF_HEAD); p(OP.i32_add); lt(L_H);
        ld32(); ls(L_CUR);
        lg(L_H); lg(L_I); st32();

        c32(OFF_SON); lg(L_CP); c32(2); p(OP.i32_shl); p(OP.i32_add); lt(L_P1);
        c32(4); p(OP.i32_add); ls(L_P0);

        c32(0); ls(L_L0);
        c32(0); ls(L_L1);
        c32(2); ls(L_BEST);
        c32(0); ls(L_BD);
        c32(0); ls(L_SFLAG);
        lg(P_CUT); ls(L_CNT);

        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_CUR); c32(0); p(OP.i32_lt_s);
          p(OP.if, VOID);
            lg(L_P0); c32(-1); st32();
            lg(L_P1); c32(-1); st32();
            p(OP.br, 2);
          p(OP.end);
          lg(L_I); lg(L_CUR); p(OP.i32_sub); c32(32768); p(OP.i32_gt_u);
          lg(L_CNT); p(OP.i32_eqz); p(OP.i32_or);
          p(OP.if, VOID);
            lg(L_P0); c32(-1); st32();
            lg(L_P1); c32(-1); st32();
            p(OP.br, 2);
          p(OP.end);
          lg(L_CNT); c32(1); p(OP.i32_sub); ls(L_CNT);

          c32(OFF_SON); lg(L_CUR); c32(65535); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add); ls(L_PAIR);
          lg(L_L0); lg(L_L1); lg(L_L0); lg(L_L1); p(OP.i32_lt_u); p(OP.select); ls(L_LEN);

          lg(L_KL); lg(L_LEN); p(OP.i32_gt_u);
          p(OP.if, VOID);
            lg(L_I); lg(L_CUR); p(OP.i32_sub); lg(L_KD); p(OP.i32_eq);
            p(OP.if, VOID);
              lg(L_KL); lg(L_LIM); lg(L_KL); lg(L_LIM); p(OP.i32_lt_u); p(OP.select); ls(L_LEN);
            p(OP.end);
          p(OP.end);

          lg(P_DATA); lg(L_CUR); p(OP.i32_add); ls(L_AC);

          p(OP.block, VOID);
          p(OP.loop, VOID);
            lg(L_LEN); c32(4); p(OP.i32_add); lg(L_LIM); p(OP.i32_gt_u); p(OP.br_if, 1);
            lg(L_AC); lg(L_LEN); p(OP.i32_add); ld32u();
            lg(L_AI); lg(L_LEN); p(OP.i32_add); ld32u();
            p(OP.i32_ne); p(OP.br_if, 1);
            lg(L_LEN); c32(4); p(OP.i32_add); ls(L_LEN);
            p(OP.br, 0);
          p(OP.end);
          p(OP.end);

          p(OP.block, VOID);
          p(OP.loop, VOID);
            lg(L_LEN); lg(L_LIM); p(OP.i32_ge_u); p(OP.br_if, 1);
            lg(L_AC); lg(L_LEN); p(OP.i32_add); ld8u(); lt(L_B1);
            lg(L_AI); lg(L_LEN); p(OP.i32_add); ld8u(); lt(L_B2);
            p(OP.i32_ne); p(OP.br_if, 1);
            lg(L_LEN); c32(1); p(OP.i32_add); ls(L_LEN);
            p(OP.br, 0);
          p(OP.end);
          p(OP.end);

          lg(L_LEN); lg(L_BEST); p(OP.i32_gt_u);
          p(OP.if, VOID);
            lg(L_LEN); ls(L_BEST);
            lg(L_I); lg(L_CUR); p(OP.i32_sub); ls(L_BD);
            lg(L_REC);
            p(OP.if, VOID);
              lg(L_M); lg(P_CAP); p(OP.i32_ge_u);
              p(OP.if, VOID); c32(-1); p(OP.ret); p(OP.end);
              lg(P_MATCH); lg(L_M); c32(2); p(OP.i32_shl); p(OP.i32_add);
              lg(L_BD); c32(9); p(OP.i32_shl); lg(L_LEN); p(OP.i32_or);
              st32();
              lg(L_M); c32(1); p(OP.i32_add); ls(L_M);
              lg(L_BD); lg(P_BPP); p(OP.i32_eq);
              p(OP.if, VOID); lg(L_SFLAG); c32(1); p(OP.i32_or); ls(L_SFLAG); p(OP.end);
              lg(L_BD); lg(P_ROW); p(OP.i32_eq);
              p(OP.if, VOID); lg(L_SFLAG); c32(2); p(OP.i32_or); ls(L_SFLAG); p(OP.end);
            p(OP.end);
            lg(L_LEN); lg(L_LIM); p(OP.i32_ge_u);
            p(OP.if, VOID);
              lg(L_P1); lg(L_PAIR); ld32(0); st32();
              lg(L_P0); lg(L_PAIR); ld32(4); st32();
              p(OP.br, 3);
            p(OP.end);
          p(OP.end);

          lg(L_B1); lg(L_B2); p(OP.i32_lt_u);
          p(OP.if, VOID);
            lg(L_P1); lg(L_CUR); st32();
            lg(L_PAIR); c32(4); p(OP.i32_add); lt(L_P1); ld32(); ls(L_CUR);
            lg(L_LEN); ls(L_L1);
          p(OP.else);
            lg(L_P0); lg(L_CUR); st32();
            lg(L_PAIR); lt(L_P0); ld32(); ls(L_CUR);
            lg(L_LEN); ls(L_L0);
          p(OP.end);

          p(OP.br, 0);
        p(OP.end);
        p(OP.end);

        // Append explicit 2D-stride matches & insertion-sort by ascending length for exact Pareto-frontier parseRMQ
        lg(L_REC);
        p(OP.if, VOID);
          lg(L_M); ls(L_MS);

          lg(P_BPP); c32(1); p(OP.i32_gt_u);
          lg(L_SFLAG); c32(1); p(OP.i32_and); p(OP.i32_eqz); p(OP.i32_and);
          p(OP.if, VOID);
            lg(P_BPP); ls(L_TD);
            emitExplicitDist(0);
          p(OP.end);

          lg(P_ROW); c32(3); p(OP.i32_ge_u);
          lg(P_ROW); c32(32768); p(OP.i32_le_u); p(OP.i32_and);
          p(OP.if, VOID);
            lg(L_SFLAG); c32(2); p(OP.i32_and); p(OP.i32_eqz);
            p(OP.if, VOID);
              lg(P_ROW); ls(L_TD);
              emitExplicitDist(1);
            p(OP.end);

            lg(P_ROW); lg(P_BPP); p(OP.i32_sub); lt(L_TD); c32(3); p(OP.i32_ge_u);
            p(OP.if, VOID); emitExplicitDist(2); p(OP.end);

            lg(P_ROW); lg(P_BPP); p(OP.i32_add); lt(L_TD); c32(32768); p(OP.i32_le_u);
            p(OP.if, VOID); emitExplicitDist(2); p(OP.end);

            lg(P_ROW); c32(1); p(OP.i32_shl); lt(L_TD); c32(32768); p(OP.i32_le_u);
            p(OP.if, VOID); emitExplicitDist(2); p(OP.end);
          p(OP.end);

          // Insertion-sort appended stride matches into [L_A0 .. L_M) by ascending length
          lg(L_M); lg(L_MS); p(OP.i32_gt_u);
          lg(L_M); lg(L_A0); c32(1); p(OP.i32_add); p(OP.i32_gt_u); p(OP.i32_and);
          p(OP.if, VOID);
            lg(L_MS); ls(L_K);
            p(OP.block, VOID);
            p(OP.loop, VOID);
              lg(L_K); lg(L_M); p(OP.i32_ge_u); p(OP.br_if, 1);
              lg(P_MATCH); lg(L_K); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); lt(L_VAL);
              c32(511); p(OP.i32_and); ls(L_LEN);
              lg(L_K); ls(L_J);
              p(OP.block, VOID);
              p(OP.loop, VOID);
                lg(L_J); lg(L_A0); p(OP.i32_le_u); p(OP.br_if, 1);
                lg(P_MATCH); lg(L_J); c32(1); p(OP.i32_sub); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); lt(L_B3);
                c32(511); p(OP.i32_and); lg(L_LEN); p(OP.i32_le_u); p(OP.br_if, 1);
                lg(P_MATCH); lg(L_J); c32(2); p(OP.i32_shl); p(OP.i32_add); lg(L_B3); st32();
                lg(L_J); c32(1); p(OP.i32_sub); ls(L_J);
                p(OP.br, 0);
              p(OP.end);
              p(OP.end);
              lg(P_MATCH); lg(L_J); c32(2); p(OP.i32_shl); p(OP.i32_add); lg(L_VAL); st32();
              lg(L_K); c32(1); p(OP.i32_add); ls(L_K);
              p(OP.br, 0);
            p(OP.end);
            p(OP.end);
          p(OP.end);
        p(OP.end);

        lg(L_BD); ls(L_KD);
        lg(L_BEST); c32(3); p(OP.i32_ge_u);
        p(OP.if, VOID); lg(L_BEST); c32(1); p(OP.i32_sub); ls(L_KL);
        p(OP.else); c32(0); ls(L_KL); p(OP.end);

        lg(L_REC);
        p(OP.if, VOID);
          lg(L_BEST); lg(P_NICE); p(OP.i32_ge_u);
          p(OP.if, VOID); lg(L_I); lg(L_BEST); p(OP.i32_add); ls(L_SKIP); p(OP.end);
        p(OP.end);
      p(OP.end);
    p(OP.end);

    lg(L_I); c32(1); p(OP.i32_add); ls(L_I);
    p(OP.br, 0);
  p(OP.end);
  p(OP.end);

  lg(P_OFF); lg(P_N); c32(2); p(OP.i32_shl); p(OP.i32_add); lg(L_M); st32();
  lg(L_M);
  p(OP.end);

  return {
    params: [I32, I32, I32, I32, I32, I32, I32, I32, I32],
    results: [I32],
    locals: [[31, I32]],
    body: e.b,
  };
}

// ---------------------------------------------------------------------------------------------------------------
// Function 1: parseRMQ(dataPtr, offPtr, matchPtr, cLenPtr, cDistPtr, litOutPtr, distOutPtr, s, e, skipDist) -> m
// Exact Prefix-Dominance / Pareto-Frontier pruning: every length in 3..L_max evaluated at most once!
function buildParseRMQ() {
  const em = emit();
  const { p, c32, c64, lg, ls, lt, ld32, ld64, ld8u, ld16u, st32, st64, st16 } = em;
  const P_DATA = 0, P_OFF = 1, P_MATCH = 2, P_CLEN = 3, P_CDIST = 4, P_LOUT = 5, P_DOUT = 6, P_S = 7, P_E = 8, P_SKIP = 9;
  const L_N = 10, L_I = 11, L_POS = 12, L_A = 13, L_B = 14, L_K = 15, L_ROOM = 16, L_PREVL = 17;
  const L_PACK = 18, L_L = 19, L_D = 20, L_LO = 21, L_HI = 22, L_C = 23, L_BLEN = 24, L_BDIST = 25;
  const L_LVL = 26, L_IDX = 27, L_M = 28, L_PBYTE = 29, L_SRUN = 30, L_CBYTE = 31, L_LRUN = 32, L_BLKS = 33, L_NEWI = 34;
  const L_TOP = 35, L_LASTL = 36;
  const Q_BCOST = 37, Q_DCOST = 38, Q_EM = 39, Q_E1 = 40, Q_E2 = 41, Q_CAND = 42, Q_C258 = 43, Q_ADD = 44, Q_MINDC = 45;

  const INF = 0x3fffffffffffffffn;

  const emitRmqUpdate = (posVar, costVar) => {
    lg(posVar); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); c32(OFF_RMQ); p(OP.i32_add); ls(L_IDX);
    lg(costVar); c64(22); p(OP.i64_shl); lg(posVar); p(OP.i64_extend_i32_u); p(OP.i64_or); ls(Q_EM);
    lg(L_IDX); lg(Q_EM); st64(0);
    for (let lv = 1; lv < 6; lv++) {
      const step = 1 << (lv - 1);
      const prevBase = OFF_RMQ + (lv - 1) * 4096;
      c32(prevBase); lg(posVar); c32(step); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
      ld64(); ls(Q_E2);
      lg(Q_EM); lg(Q_E2); lg(Q_EM); lg(Q_E2); p(OP.i64_lt_u); p(OP.select); ls(Q_EM);
      lg(L_IDX); lg(Q_EM); st64(lv * 4096);
    }
  };

  lg(P_E); lg(P_S); p(OP.i32_sub); ls(L_N);
  lg(L_N); p(OP.i32_eqz);
  p(OP.if, VOID); c32(0); p(OP.ret); p(OP.end);

  c32(OFF_LEN_COST + 28 * 4); ld32();
  lg(P_SKIP); c32(0); p(OP.i32_gt_s);
  p(OP.if, I32);
    c32(OFF_DIST_COST); lg(P_SKIP); c32(1); p(OP.i32_sub); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32();
  p(OP.else);
    c32(OFF_DIST_COST); ld32();
  p(OP.end);
  p(OP.i32_add); p(OP.i64_extend_i32_u); ls(Q_C258);

  c32(0); ls(L_I);
  p(OP.loop, VOID);
    c32(OFF_RMQ); lg(L_I); c32(3); p(OP.i32_shl); p(OP.i32_add); c64(INF); st64();
    lg(L_I); c32(1); p(OP.i32_add); lt(L_I); c32(3072); p(OP.i32_lt_u); p(OP.br_if, 0);
  p(OP.end);

  lg(L_N); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); c32(OFF_RMQ); p(OP.i32_add); ls(L_IDX);
  lg(L_N); p(OP.i64_extend_i32_u); ls(Q_EM);
  for (let lv = 0; lv < 6; lv++) {
    lg(L_IDX); lg(Q_EM); st64(lv * 4096);
  }

  c32(-1); ls(L_PBYTE);
  c32(0); ls(L_SRUN);

  lg(L_N); c32(1); p(OP.i32_sub); ls(L_I);
  p(OP.block, VOID);
  p(OP.loop, VOID);
    lg(L_I); c32(0); p(OP.i32_lt_s); p(OP.br_if, 1);

    lg(P_S); lg(L_I); p(OP.i32_add); ls(L_POS);
    lg(P_DATA); lg(L_POS); p(OP.i32_add); ld8u(); ls(L_CBYTE);

    lg(L_CBYTE); lg(L_PBYTE); p(OP.i32_eq);
    p(OP.if, VOID);
      lg(L_SRUN); c32(1); p(OP.i32_add); ls(L_SRUN);
    p(OP.else);
      lg(L_CBYTE); ls(L_PBYTE);
      c32(1); ls(L_SRUN);
    p(OP.end);

    // Exact multi-block jump over deep constant runs using P_SKIP (1..4)
    lg(P_SKIP);
    p(OP.if, VOID);
      lg(L_SRUN); c32(292); p(OP.i32_ge_u);
      lg(L_I); c32(262); p(OP.i32_ge_s); p(OP.i32_and);
      p(OP.if, VOID);
        c32(0); ls(L_LRUN);
        lg(P_DATA); lg(L_POS); c32(1); p(OP.i32_sub); p(OP.i32_add); ld8u(); lg(L_CBYTE); p(OP.i32_eq);
        lg(P_DATA); lg(L_POS); c32(258); p(OP.i32_sub); p(OP.i32_add); ld8u(); lg(L_CBYTE); p(OP.i32_eq); p(OP.i32_and);
        p(OP.if, VOID);
          p(OP.block, VOID);
          p(OP.loop, VOID);
            lg(L_LRUN); lg(L_I); p(OP.i32_ge_u); p(OP.br_if, 1);
            lg(P_DATA); lg(L_POS); c32(1); p(OP.i32_sub); lg(L_LRUN); p(OP.i32_sub); p(OP.i32_add); ld8u();
            lg(L_CBYTE); p(OP.i32_ne); p(OP.br_if, 1);
            lg(L_LRUN); c32(1); p(OP.i32_add); ls(L_LRUN);
            p(OP.br, 0);
          p(OP.end);
          p(OP.end);
        p(OP.end);

        lg(L_LRUN); c32(264); p(OP.i32_ge_u);
        p(OP.if, VOID);
          lg(L_LRUN); c32(6); p(OP.i32_sub); c32(258); p(OP.i32_div_u); ls(L_BLKS);
          lg(L_I); lg(L_BLKS); c32(258); p(OP.i32_mul); p(OP.i32_sub); ls(L_NEWI);
          lg(L_BLKS); p(OP.i64_extend_i32_u); lg(Q_C258); p(OP.i64_mul); ls(Q_ADD);

          c32(1); ls(L_K);
          p(OP.loop, VOID);
            c32(OFF_SCRATCH); lg(L_K); c32(3); p(OP.i32_shl); p(OP.i32_add);
            c32(OFF_RMQ); lg(L_I); lg(L_K); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
            ld64(); c64(22); p(OP.i64_shr_u); st64();
            lg(L_K); c32(1); p(OP.i32_add); lt(L_K); c32(290); p(OP.i32_le_u); p(OP.br_if, 0);
          p(OP.end);

          lg(L_NEWI); c32(1); p(OP.i32_add); ls(L_K);
          p(OP.loop, VOID);
            lg(P_CLEN); lg(L_K); c32(1); p(OP.i32_shl); p(OP.i32_add); c32(258); st16();
            lg(P_CDIST); lg(L_K); c32(1); p(OP.i32_shl); p(OP.i32_add); lg(P_SKIP); st16();
            lg(L_K); c32(1); p(OP.i32_add); lt(L_K); lg(L_I); p(OP.i32_le_u); p(OP.br_if, 0);
          p(OP.end);

          c32(290); ls(L_K);
          p(OP.loop, VOID);
            lg(L_NEWI); lg(L_K); p(OP.i32_add); ls(L_A);
            c32(OFF_SCRATCH); lg(L_K); c32(3); p(OP.i32_shl); p(OP.i32_add); ld64();
            lg(Q_ADD); p(OP.i64_add); ls(Q_BCOST);
            emitRmqUpdate(L_A, Q_BCOST);
            lg(L_K); c32(1); p(OP.i32_sub); lt(L_K); c32(1); p(OP.i32_ge_s); p(OP.br_if, 0);
          p(OP.end);

          lg(L_SRUN); lg(L_I); lg(L_NEWI); p(OP.i32_sub); p(OP.i32_add); ls(L_SRUN);
          lg(L_NEWI); ls(L_I);
          p(OP.br, 3);
        p(OP.end);
      p(OP.end);
    p(OP.end);

    // Single-step exact deep-run fast path for skipDist in 1..4
    c32(0); ls(L_IDX);
    lg(P_SKIP);
    p(OP.if, VOID);
      lg(L_SRUN); c32(261); p(OP.i32_ge_u);
      lg(L_POS); lg(P_SKIP); p(OP.i32_ge_u); p(OP.i32_and);
      p(OP.if, VOID);
        c32(1); ls(L_K);
        c32(1); ls(L_IDX);
        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_K); lg(P_SKIP); p(OP.i32_gt_u); p(OP.br_if, 1);
          lg(P_DATA); lg(L_POS); lg(L_K); p(OP.i32_sub); p(OP.i32_add); ld8u();
          lg(L_CBYTE); p(OP.i32_ne);
          p(OP.if, VOID); c32(0); ls(L_IDX); p(OP.br, 2); p(OP.end);
          lg(L_K); c32(1); p(OP.i32_add); ls(L_K);
          p(OP.br, 0);
        p(OP.end);
        p(OP.end);
      p(OP.end);
    p(OP.end);

    lg(L_IDX);
    p(OP.if, VOID);
      c32(OFF_RMQ); lg(L_I); c32(258); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
      ld64(); c64(22); p(OP.i64_shr_u); lg(Q_C258); p(OP.i64_add); ls(Q_BCOST);
      c32(258); ls(L_BLEN);
      lg(P_SKIP); ls(L_BDIST);
    p(OP.else);
      // Literal
      c32(OFF_RMQ); lg(L_I); c32(1); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
      ld64(); c64(22); p(OP.i64_shr_u);
      c32(OFF_LIT_COST); lg(L_CBYTE); c32(2); p(OP.i32_shl); p(OP.i32_add);
      ld32(); p(OP.i64_extend_i32_u);
      p(OP.i64_add); ls(Q_BCOST);
      c32(1); ls(L_BLEN);
      c32(0); ls(L_BDIST);

      lg(P_OFF); lg(L_POS); c32(2); p(OP.i32_shl); p(OP.i32_add); lt(L_IDX);
      ld32(0); ls(L_A);
      lg(L_IDX); ld32(4); ls(L_B);

      lg(L_A); lg(L_B); p(OP.i32_lt_u);
      p(OP.if, VOID);
        lg(L_N); lg(L_I); p(OP.i32_sub); ls(L_ROOM);

        // Pass 1 (backward from B-1 down to A): extract exact Pareto frontier (strictly decreasing L & strictly decreasing distCost)
        c64(INF); ls(Q_MINDC);
        c32(0); ls(L_TOP);
        c32(0); ls(L_LASTL);
        lg(L_B); c32(1); p(OP.i32_sub); ls(L_K);

        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_K); lg(L_A); p(OP.i32_lt_s); p(OP.br_if, 1);

          lg(P_MATCH); lg(L_K); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); lt(L_PACK);
          c32(511); p(OP.i32_and); ls(L_L);
          lg(L_L); lg(L_ROOM); p(OP.i32_gt_u);
          p(OP.if, VOID); lg(L_ROOM); ls(L_L); p(OP.end);

          lg(L_L); c32(3); p(OP.i32_ge_u);
          p(OP.if, VOID);
            lg(L_PACK); c32(9); p(OP.i32_shr_u); ls(L_D);
            c32(OFF_DIST_COST);
            c32(OFF_DSYM); lg(L_D); p(OP.i32_add); ld8u();
            c32(2); p(OP.i32_shl); p(OP.i32_add); ld32();
            p(OP.i64_extend_i32_u); ls(Q_DCOST);

            lg(Q_DCOST); lg(Q_MINDC); p(OP.i64_lt_u);
            p(OP.if, VOID);
              lg(Q_DCOST); ls(Q_MINDC);
              lg(L_TOP); c32(0); p(OP.i32_gt_u);
              lg(L_L); lg(L_LASTL); p(OP.i32_eq); p(OP.i32_and);
              p(OP.if, VOID);
                lg(L_TOP); c32(1); p(OP.i32_sub); ls(L_IDX);
                c32(OFF_PARETO_PK); lg(L_IDX); c32(2); p(OP.i32_shl); p(OP.i32_add);
                lg(L_D); c32(9); p(OP.i32_shl); lg(L_L); p(OP.i32_or); st32();
                c32(OFF_PARETO_DC); lg(L_IDX); c32(3); p(OP.i32_shl); p(OP.i32_add);
                lg(Q_DCOST); st64();
              p(OP.else);
                c32(OFF_PARETO_PK); lg(L_TOP); c32(2); p(OP.i32_shl); p(OP.i32_add);
                lg(L_D); c32(9); p(OP.i32_shl); lg(L_L); p(OP.i32_or); st32();
                c32(OFF_PARETO_DC); lg(L_TOP); c32(3); p(OP.i32_shl); p(OP.i32_add);
                lg(Q_DCOST); st64();
                lg(L_TOP); c32(1); p(OP.i32_add); ls(L_TOP);
                lg(L_L); ls(L_LASTL);
              p(OP.end);
            p(OP.end);
          p(OP.end);

          lg(L_K); c32(1); p(OP.i32_sub); ls(L_K);
          p(OP.br, 0);
        p(OP.end);
        p(OP.end);

        // Pass 2 (pop Pareto stack top-1 down to 0): strictly increasing L, non-overlapping disjoint intervals!
        c32(2); ls(L_PREVL);
        lg(L_TOP); c32(1); p(OP.i32_sub); ls(L_K);
        p(OP.block, VOID);
        p(OP.loop, VOID);
          lg(L_K); c32(0); p(OP.i32_lt_s); p(OP.br_if, 1);

          c32(OFF_PARETO_PK); lg(L_K); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); lt(L_PACK);
          c32(511); p(OP.i32_and); ls(L_L);
          lg(L_PACK); c32(9); p(OP.i32_shr_u); ls(L_D);
          c32(OFF_PARETO_DC); lg(L_K); c32(3); p(OP.i32_shl); p(OP.i32_add); ld64(); ls(Q_DCOST);

          lg(L_PREVL); c32(1); p(OP.i32_add); ls(L_LO);
          lg(L_L); ls(L_PREVL);

          // Fast width-1 loop for lo <= min(l, 10)
          lg(L_LO); c32(10); p(OP.i32_le_u);
          p(OP.if, VOID);
            lg(L_L); c32(10); lg(L_L); c32(10); p(OP.i32_lt_u); p(OP.select); ls(L_HI);
            p(OP.loop, VOID);
              c32(OFF_RMQ); lg(L_I); lg(L_LO); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
              ld64(); c64(22); p(OP.i64_shr_u);
              lg(Q_DCOST); p(OP.i64_add);
              c32(OFF_LEN_COST - 12); lg(L_LO); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); p(OP.i64_extend_i32_u);
              p(OP.i64_add); lt(Q_CAND);
              lg(Q_BCOST); p(OP.i64_lt_u);
              p(OP.if, VOID);
                lg(Q_CAND); ls(Q_BCOST);
                lg(L_LO); ls(L_BLEN);
                lg(L_D); ls(L_BDIST);
              p(OP.end);
              lg(L_LO); c32(1); p(OP.i32_add); lt(L_LO); lg(L_HI); p(OP.i32_le_u); p(OP.br_if, 0);
            p(OP.end);
          p(OP.end);

          // RMQ loop for lo >= 11
          lg(L_LO); lg(L_L); p(OP.i32_le_u);
          p(OP.if, VOID);
            c32(OFF_LSYM); lg(L_LO); p(OP.i32_add); ld8u(); ls(L_C);
            p(OP.block, VOID);
            p(OP.loop, VOID);
              lg(L_LO); lg(L_L); p(OP.i32_gt_u); p(OP.br_if, 1);

              c32(OFF_LCODE_END); lg(L_C); c32(1); p(OP.i32_shl); p(OP.i32_add); ld16u(); ls(L_HI);
              lg(L_HI); lg(L_L); p(OP.i32_gt_u);
              p(OP.if, VOID); lg(L_L); ls(L_HI); p(OP.end);

              lg(L_LO); lg(L_HI); p(OP.i32_eq);
              p(OP.if, VOID);
                c32(OFF_RMQ); lg(L_I); lg(L_LO); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
                ld64(); ls(Q_EM);
              p(OP.else);
                c32(OFF_LOG2); lg(L_HI); lg(L_LO); p(OP.i32_sub); c32(1); p(OP.i32_add); p(OP.i32_add); ld8u(); ls(L_LVL);
                c32(OFF_RMQ); lg(L_LVL); c32(12); p(OP.i32_shl); p(OP.i32_add); ls(L_IDX);
                lg(L_IDX); lg(L_I); lg(L_LO); p(OP.i32_add); c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
                ld64(); ls(Q_E1);
                lg(L_IDX);
                lg(L_I); lg(L_HI); p(OP.i32_add); c32(1); lg(L_LVL); p(OP.i32_shl); p(OP.i32_sub); c32(1); p(OP.i32_add);
                c32(511); p(OP.i32_and); c32(3); p(OP.i32_shl); p(OP.i32_add);
                ld64(); ls(Q_E2);
                lg(Q_E1); lg(Q_E2); lg(Q_E1); lg(Q_E2); p(OP.i64_lt_u); p(OP.select); ls(Q_EM);
              p(OP.end);

              lg(Q_EM); c64(22); p(OP.i64_shr_u);
              lg(Q_DCOST); p(OP.i64_add);
              c32(OFF_LEN_COST); lg(L_C); c32(2); p(OP.i32_shl); p(OP.i32_add); ld32(); p(OP.i64_extend_i32_u);
              p(OP.i64_add); lt(Q_CAND);
              lg(Q_BCOST); p(OP.i64_lt_u);
              p(OP.if, VOID);
                lg(Q_CAND); ls(Q_BCOST);
                lg(Q_EM); c64(0x3fffffn); p(OP.i64_and); p(OP.i32_wrap_i64); lg(L_I); p(OP.i32_sub); ls(L_BLEN);
                lg(L_D); ls(L_BDIST);
              p(OP.end);

              lg(L_HI); c32(1); p(OP.i32_add); ls(L_LO);
              lg(L_C); c32(1); p(OP.i32_add); ls(L_C);
              p(OP.br, 0);
            p(OP.end);
            p(OP.end);
          p(OP.end);

          lg(L_K); c32(1); p(OP.i32_sub); ls(L_K);
          p(OP.br, 0);
        p(OP.end);
        p(OP.end);
      p(OP.end);
    p(OP.end);

    lg(P_CLEN); lg(L_I); c32(1); p(OP.i32_shl); p(OP.i32_add); lg(L_BLEN); st16();
    lg(P_CDIST); lg(L_I); c32(1); p(OP.i32_shl); p(OP.i32_add); lg(L_BDIST); st16();
    emitRmqUpdate(L_I, Q_BCOST);

    lg(L_I); c32(1); p(OP.i32_sub); ls(L_I);
    p(OP.br, 0);
  p(OP.end);
  p(OP.end);

  // Forward backtrack
  c32(0); ls(L_I);
  c32(0); ls(L_M);
  p(OP.block, VOID);
  p(OP.loop, VOID);
    lg(L_I); lg(L_N); p(OP.i32_ge_u); p(OP.br_if, 1);
    lg(P_CLEN); lg(L_I); c32(1); p(OP.i32_shl); p(OP.i32_add); ld16u(); ls(L_L);
    lg(L_L); c32(1); p(OP.i32_eq);
    p(OP.if, VOID);
      lg(P_LOUT); lg(L_M); c32(1); p(OP.i32_shl); p(OP.i32_add);
      lg(P_DATA); lg(P_S); lg(L_I); p(OP.i32_add); p(OP.i32_add); ld8u(); st16();
      lg(P_DOUT); lg(L_M); c32(1); p(OP.i32_shl); p(OP.i32_add); c32(0); st16();
    p(OP.else);
      lg(P_LOUT); lg(L_M); c32(1); p(OP.i32_shl); p(OP.i32_add); lg(L_L); st16();
      lg(P_DOUT); lg(L_M); c32(1); p(OP.i32_shl); p(OP.i32_add);
      lg(P_CDIST); lg(L_I); c32(1); p(OP.i32_shl); p(OP.i32_add); ld16u(); st16();
    p(OP.end);
    lg(L_M); c32(1); p(OP.i32_add); ls(L_M);
    lg(L_I); lg(L_L); p(OP.i32_add); ls(L_I);
    p(OP.br, 0);
  p(OP.end);
  p(OP.end);

  lg(L_M);
  p(OP.end);

  return {
    params: [I32, I32, I32, I32, I32, I32, I32, I32, I32, I32],
    results: [I32],
    locals: [[27, I32], [9, I64]],
    body: em.b,
  };
}

function assembleModule() {
  const fns = [buildBT3(), buildParseRMQ()];
  const types = fns.map((f) => [0x60, ...vec(f.params), ...vec(f.results)]);
  const typeSec = sec(1, vec(types));
  const importSec = sec(2, vec([
    [...str('env'), ...str('mem'), 0x02, 0x01, ...uleb(512), ...uleb(5120)],
  ]));
  const funcSec = sec(3, vec(fns.map((_, i) => uleb(i))));
  const exportSec = sec(7, vec([
    [...str('bt3'), 0x00, ...uleb(0)],
    [...str('parseRMQ'), 0x00, ...uleb(1)],
  ]));
  const codes = fns.map((f) => {
    const loc = vec(f.locals.map(([n, t]) => [...uleb(n), t]));
    const body = [...loc, ...f.body];
    return [...uleb(body.length), ...body];
  });
  const codeSec = sec(10, vec(codes));
  return new Uint8Array([0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00, ...typeSec, ...importSec, ...funcSec, ...exportSec, ...codeSec]);
}

const LEN_BASE = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
const LCODE_END = [3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 22, 26, 30, 34, 42, 50, 58, 66, 82, 98, 114, 130, 162, 194, 226, 257, 258];
const DIST_BASE = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];

let cached = null;

export function getWasmEngine() {
  if (cached) return cached;
  const memory = new WebAssembly.Memory({ initial: 512, maximum: 5120 });
  const mod = new WebAssembly.Module(assembleModule());
  const inst = new WebAssembly.Instance(mod, { env: { mem: memory } });

  const u8 = new Uint8Array(memory.buffer);
  const u16 = new Uint16Array(memory.buffer);
  for (let c = 0; c < 30; c++) {
    const end = c < 29 ? DIST_BASE[c + 1] : 32769;
    for (let d = DIST_BASE[c]; d < end; d++) u8[OFF_DSYM + d] = c;
  }
  for (let c = 0; c < 29; c++) {
    const end = c < 28 ? LEN_BASE[c + 1] : 259;
    for (let l = LEN_BASE[c]; l < end; l++) u8[OFF_LSYM + l] = c;
    u16[(OFF_LCODE_END >>> 1) + c] = end - 1;
  }
  for (let w = 1; w <= 32; w++) {
    let lv = 0;
    while ((2 << lv) <= w) lv++;
    u8[OFF_LOG2 + w] = lv;
  }

  function ensureBytes(totalBytes) {
    const cur = memory.buffer.byteLength;
    if (totalBytes <= cur) return;
    memory.grow(Math.ceil((totalBytes - cur) / 65536));
  }

  const align8 = (x) => (x + 7) & ~7;

  function layout(n, matchCap) {
    let p = OFF_DYN;
    const dataPtr = p; p = align8(p + n + 8);
    const offPtr = p; p = align8(p + (n + 1) * 4);
    const cLenPtr = p; p = align8(p + (n + 1) * 2);
    const cDistPtr = p; p = align8(p + (n + 1) * 2);
    const litOutPtr = p; p = align8(p + (n + 1) * 2);
    const distOutPtr = p; p = align8(p + (n + 1) * 2);
    const matchPtr = p; p = align8(p + matchCap * 4);
    ensureBytes(p);
    return { dataPtr, offPtr, cLenPtr, cDistPtr, litOutPtr, distOutPtr, matchPtr, matchCap, n };
  }

  function prepareMatches(data, cut = 256, nice = 259, bpp = 1, rowLen = 0) {
    const n = data.length;
    let matchCap = Math.max(2_000_000, n * 4);
    for (;;) {
      const L = layout(n, matchCap);
      new Uint8Array(memory.buffer, L.dataPtr, n).set(data);
      const m = inst.exports.bt3(L.dataPtr, n, L.offPtr, L.matchPtr, matchCap, cut, nice, bpp, rowLen);
      if (m >= 0) {
        L.matchCount = m;
        return L;
      }
      if (matchCap >= 48_000_000) throw new Error('match buffer overflow');
      matchCap = Math.min(48_000_000, matchCap * 2);
    }
  }

  function checkCanSkip258(litC, lenC, distC, bpp) {
    let bestD = 1;
    let minC = distC[0];
    if (bpp >= 2 && bpp <= 4 && distC[bpp - 1] < minC) {
      minC = distC[bpp - 1];
      bestD = bpp;
    }
    for (let c = 0; c < 30; c++) if (distC[c] < minC) return 0;
    const c258 = lenC[28] + minC;
    for (let c = 0; c < 28; c++) {
      if (c258 * LCODE_END[c] > (lenC[c] + minC) * 258) return 0;
    }
    let minLit = litC[0];
    for (let s = 1; s < 256; s++) if (litC[s] < minLit) minLit = litC[s];
    if (c258 > minLit * 258) return 0;
    return bestD;
  }

  function parseRange(L, s, e, litCostI32, lenCostI32, distCostI32, litDest, distDest, destAt = 0, bpp = 1) {
    const i32 = new Int32Array(memory.buffer);
    i32.set(litCostI32, OFF_LIT_COST >>> 2);
    i32.set(lenCostI32, OFF_LEN_COST >>> 2);
    i32.set(distCostI32, OFF_DIST_COST >>> 2);
    const skipDist = checkCanSkip258(litCostI32, lenCostI32, distCostI32, bpp);
    const m = inst.exports.parseRMQ(L.dataPtr, L.offPtr, L.matchPtr, L.cLenPtr, L.cDistPtr, L.litOutPtr, L.distOutPtr, s, e, skipDist);
    litDest.set(new Uint16Array(memory.buffer, L.litOutPtr, m), destAt);
    distDest.set(new Uint16Array(memory.buffer, L.distOutPtr, m), destAt);
    return destAt + m;
  }

  cached = { memory, inst, prepareMatches, parseRange };
  return cached;
}
