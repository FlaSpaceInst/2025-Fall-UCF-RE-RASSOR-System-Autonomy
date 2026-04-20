const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('api', {
  // ── Rover discovery ──────────────────────────────────────────────────────
  scanPotatoes: () => ipcRenderer.send('scan-potatoes'),

  onPotatoFound: (callback) => {
    ipcRenderer.on('potato-found', (event, potatoInfo) => {
      callback(potatoInfo);
    });
  },

  // ── Demo simulator ───────────────────────────────────────────────────────
  launchDemoSim:    () => ipcRenderer.send('launch-demo-sim'),
  onDemoSimStatus:  (cb) => ipcRenderer.on('demo-sim-status', (_e, status) => cb(status)),

  // ── SSH terminal ─────────────────────────────────────────────────────────
  sshConnect:          (target) => ipcRenderer.send('ssh-connect', { target }),
  sshInput:            (data)   => ipcRenderer.send('ssh-input', data),
  sshDisconnect:       ()       => ipcRenderer.send('ssh-disconnect'),

  onSshData:   (cb) => ipcRenderer.on('ssh-data',   (_e, d)    => cb(d)),
  onSshClosed: (cb) => ipcRenderer.on('ssh-closed', (_e, info) => cb(info)),

  removeSshListeners: () => {
    ipcRenderer.removeAllListeners('ssh-data');
    ipcRenderer.removeAllListeners('ssh-closed');
  },
});
