const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('api', {
  // ── Rover discovery ──────────────────────────────────────────────────────
  scanPotatoes: () => ipcRenderer.send('scan-potatoes'),

  onPotatoFound: (callback) => {
    ipcRenderer.on('potato-found', (event, potatoInfo) => {
      callback(potatoInfo);
    });
  },

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
