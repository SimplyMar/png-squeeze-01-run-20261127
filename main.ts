// Entry point. The work lives in squeeze.mjs and runs in a child Node process: on Node 22, loading the TypeScript
// stripper (which a .ts entry needs) slows every later line of JavaScript in the same process by about a half, so the
// child, started on the plain .mjs file, runs at full speed. Same input and output directories (inherited
// environment), same bytes written. Should the child not start, the same module runs here instead.
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const script = fileURLToPath(new URL('./squeeze.mjs', import.meta.url));
const child = spawnSync(process.execPath, [script], { stdio: 'inherit' });
if (child.error) await import('./squeeze.mjs');
else if (child.status !== 0) process.exitCode = child.status ?? 1;
