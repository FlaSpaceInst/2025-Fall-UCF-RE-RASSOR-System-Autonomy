const { app, BrowserWindow, Menu, ipcMain } = require('electron');
const path = require('path');
const dnssd = require('dnssd');

function createWindow() {
  const win = new BrowserWindow({
    width: 1200,
    height: 800,
    title: "RE-RASSOR Rover Commander",
    icon: path.join(__dirname, 'assets', 'icon.ico'), // Updated path to the icon
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js'),
    }
  });

  win.loadFile(path.join(__dirname, 'dist', 'index.html'));
  win.webContents.openDevTools();
  // Optional: Set up application menu
  const menu = Menu.buildFromTemplate([
    {
      label: 'File',
      submenu: [
        { role: 'quit' }
      ]
    }
  ]);
  Menu.setApplicationMenu(menu);
}

app.whenReady().then(() => {
  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

// ipcMain.on('scan-potatoes', async () => {
//   const { scanPotatoes } = require('./scan');
//   scanPotatoes(); // Will log to console
// });

ipcMain.on('scan-potatoes', (event) => {
  const browser = new dnssd.Browser(dnssd.tcp('potato'));
  console.log("servicing potatos");
  browser.on('serviceUp', service => {
    console.log('Service found:', service); // Log the service to check its details
    const potatoInfo = {
      name: service.name,
      ip: service.addresses[0],
      port: 5000,
    };
    
    console.log('Sending potato info to renderer:', potatoInfo); // Log potato info
    // Send to renderer
    event.sender.send('potato-found', potatoInfo);
  });

  browser.start();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
