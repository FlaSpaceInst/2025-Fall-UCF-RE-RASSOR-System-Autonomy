const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('api', {
  scanPotatoes: () => ipcRenderer.send('scan-potatoes'),

  onPotatoFound: (callback) => {
    ipcRenderer.on('potato-found', (event, potatoInfo) => {
      callback(potatoInfo);
    });
  }
});
