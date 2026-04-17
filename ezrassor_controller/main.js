const { app, BrowserWindow, Menu, ipcMain } = require('electron');
const path = require('path');
const express = require('express');
const http = require('http');

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
    win.webContents.openDevTools();

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
// Uses the pure-JS `ssh2` library — no OS-level ssh binary required.
// Works on Windows, macOS, and Linux without any extra tools.
let sshConn   = null;
let sshStream = null;

function sshTeardown() {
    sshStream = null;
    if (sshConn) {
        try { sshConn.end(); } catch (_) {}
        sshConn = null;
    }
}

ipcMain.on('ssh-connect', (event, { target }) => {
    sshTeardown();

    // Parse "user@host" from the target string
    const atIdx    = target.lastIndexOf('@');
    const username = atIdx >= 0 ? target.slice(0, atIdx)     : 'ubuntu';
    const host     = atIdx >= 0 ? target.slice(atIdx + 1)    : target;

    let { Client } = require('ssh2');
    const conn = new Client();
    sshConn = conn;

    // Only report ssh-closed once per connection attempt
    let closed = false;
    const reportClosed = (code) => {
        if (closed) return;
        closed = true;
        event.sender.send('ssh-closed', { code, target });
        if (sshConn === conn) sshConn = null;
        sshStream = null;
    };

    conn.on('ready', () => {
        // Request a PTY + interactive shell — this is a protocol-level PTY,
        // so it works on all platforms with no OS pseudo-terminal needed.
        conn.shell({ term: 'xterm-256color', rows: 24, cols: 160 }, (err, stream) => {
            if (err) {
                event.sender.send('ssh-data', `[shell error] ${err.message}\n`);
                reportClosed(1);
                return;
            }
            sshStream = stream;
            stream.on('data',          (d) => event.sender.send('ssh-data', d.toString()));
            stream.stderr.on('data',   (d) => event.sender.send('ssh-data', d.toString()));
            stream.on('close',         ()  => { sshStream = null; reportClosed(0); });
        });
    });

    conn.on('error', (err) => {
        event.sender.send('ssh-data', `[ssh error] ${err.message}\n`);
        reportClosed(1);
    });

    conn.on('close', () => reportClosed(1));

    conn.connect({
        host,
        port:          22,
        username,
        password:      'password',
        readyTimeout:  8000,          // 8 s — gives .local time to resolve or fail
        keepaliveInterval: 15000,
    });
});

ipcMain.on('ssh-input', (_event, data) => {
    if (sshStream) {
        try { sshStream.write(data); } catch (_) {}
    }
});

ipcMain.on('ssh-disconnect', () => sshTeardown());

app.on('window-all-closed', () => {
    sshTeardown();
    if (process.platform !== 'darwin') app.quit();
});
