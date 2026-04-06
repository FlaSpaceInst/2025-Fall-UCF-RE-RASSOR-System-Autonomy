const { app, BrowserWindow, Menu, ipcMain } = require('electron');
const path = require('path');
const express = require('express');
const http = require('http');
const { spawn } = require('child_process');

const PORT = 34756;

function startFileServer() {
    const srv = express();
    srv.use(express.static(path.join(__dirname, 'dist')));
    return new Promise((resolve) => {
        http.createServer(srv).listen(PORT, '127.0.0.1', () => resolve());
    });
}

// Pick the right icon format per platform.
// Electron Packager can also auto-detect from `./assets/icon` (no extension).
function getIconPath() {
    const base = path.join(__dirname, 'assets', 'icon');
    if (process.platform === 'darwin') return `${base}.icns`;
    if (process.platform === 'linux')  return `${base}.png`;
    return `${base}.ico`;
}

function createWindow() {
    const iconPath = getIconPath();
    const win = new BrowserWindow({
        width:  1280,
        height: 860,
        title:  'RE-RASSOR Controller',
        // Only set the icon if the file is likely to exist (avoids startup crash
        // on platforms where the icon asset hasn't been built yet).
        ...(require('fs').existsSync(iconPath) ? { icon: iconPath } : {}),
        webPreferences: {
            nodeIntegration:  false,
            contextIsolation: true,
            preload:          path.join(__dirname, 'preload.js'),
        },
    });

    win.loadURL(`http://127.0.0.1:${PORT}`);
    // Comment out the line below to hide DevTools in production builds.
    // win.webContents.openDevTools();

    const menu = Menu.buildFromTemplate([
        { label: 'File', submenu: [{ role: 'quit' }] },
    ]);
    Menu.setApplicationMenu(menu);
}

app.whenReady().then(async () => {
    await startFileServer();
    createWindow();
    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) createWindow();
    });
});

// ── DNS-SD rover discovery ────────────────────────────────────────────────────
// dnssd requires platform-native mDNS support:
//   Windows  → Bonjour for Windows (Apple)
//   macOS    → built-in (works out of the box)
//   Linux    → libavahi-compat-libdnssd-dev   (sudo apt install libavahi-compat-libdnssd-dev)
//
// We wrap in try-catch so a missing native library does not crash the app —
// the scan-potatoes IPC will simply return no results.
ipcMain.on('scan-potatoes', (event) => {
    try {
        const dnssd = require('dnssd');
        const browser = new dnssd.Browser(dnssd.tcp('potato'));

        browser.on('serviceUp', (service) => {
            const potatoInfo = {
                name: service.name,
                ip:   service.addresses[0],
                port: 5000,
            };
            event.sender.send('potato-found', potatoInfo);
        });

        browser.on('error', (err) => {
            console.warn('[dnssd] browser error:', err.message);
        });

        browser.start();
    } catch (err) {
        console.warn(
            '[dnssd] DNS-SD unavailable on this platform — rover scan disabled.\n' +
            '  Windows: install Bonjour for Windows.\n' +
            '  Linux:   sudo apt install libavahi-compat-libdnssd-dev\n' +
            `  Error:   ${err.message}`
        );
        // Send a single event so the renderer knows the scan finished with no results.
        event.sender.send('potato-found', null);
    }
});

// ── SSH terminal ─────────────────────────────────────────────────────────────
let sshProcess  = null;
let sshTarget   = null;

ipcMain.on('ssh-connect', (event, { target }) => {
    if (sshProcess) {
        try { sshProcess.kill('SIGTERM'); } catch (_) {}
        sshProcess = null;
    }
    sshTarget = target;

    const proc = spawn('ssh', [
        '-o', 'StrictHostKeyChecking=no',
        '-o', 'ConnectTimeout=5',
        target,
    ]);
    sshProcess = proc;

    proc.stdout.on('data', (d) => event.sender.send('ssh-data', d.toString()));
    proc.stderr.on('data', (d) => event.sender.send('ssh-data', d.toString()));
    proc.on('close', (code) => {
        event.sender.send('ssh-closed', { code, target: sshTarget });
        if (sshProcess === proc) sshProcess = null;
    });
    proc.on('error', (err) => {
        event.sender.send('ssh-data', `[spawn error] ${err.message}\n`);
        event.sender.send('ssh-closed', { code: 1, target: sshTarget });
        if (sshProcess === proc) sshProcess = null;
    });
});

ipcMain.on('ssh-input', (_event, data) => {
    if (sshProcess) {
        try { sshProcess.stdin.write(data); } catch (_) {}
    }
});

ipcMain.on('ssh-disconnect', () => {
    if (sshProcess) {
        try { sshProcess.kill('SIGTERM'); } catch (_) {}
        sshProcess = null;
    }
});

app.on('window-all-closed', () => {
    if (sshProcess) { try { sshProcess.kill('SIGTERM'); } catch (_) {} }
    if (process.platform !== 'darwin') app.quit();
});
