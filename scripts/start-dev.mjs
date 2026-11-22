// Starts the Vite dev server fully detached from the calling shell,
// so it keeps running after this script exits. Prints the PID.
import { spawn } from 'node:child_process'

const child = spawn('npx', ['vite', '--port', '5173', '--strictPort', '--host', '127.0.0.1'], {
  detached: true, // new process group/session — survives parent exit
  stdio: 'ignore',
  cwd: process.cwd(),
})
child.unref()
console.log(`dev server spawned (pid ${child.pid}) — check http://127.0.0.1:5173`)
