/**
 * make-mac.js
 *
 * Expo's entry-point resolver reads the "main" field from package.json and
 * passes it straight to Metro as the bundle root.  Since "main" must be
 * main.js for electron-forge to launch the Electron process, we temporarily
 * swap it to index.js (the React entry) for the web export, then restore it
 * before electron-forge packages everything into a DMG.
 *
 * Usage:  node scripts/make-mac.js
 */

'use strict';

const { execSync } = require('child_process');
const fs            = require('fs');
const path          = require('path');

const PKG_PATH  = path.join(__dirname, '..', 'package.json');
const ELECTRON_MAIN = 'main.js';
const EXPO_ENTRY    = 'node_modules/expo/AppEntry.js';

function run(cmd) {
    console.log(`\n> ${cmd}`);
    execSync(cmd, { stdio: 'inherit', cwd: path.join(__dirname, '..') });
}

function setPkgMain(value) {
    const pkg = JSON.parse(fs.readFileSync(PKG_PATH, 'utf-8'));
    pkg.main = value;
    fs.writeFileSync(PKG_PATH, JSON.stringify(pkg, null, 2) + '\n');
    console.log(`[make-mac] package.json "main" → ${value}`);
}

// Always restore on exit (success, error, or Ctrl-C)
process.on('exit', () => setPkgMain(ELECTRON_MAIN));
process.on('SIGINT',  () => process.exit(1));
process.on('SIGTERM', () => process.exit(1));

setPkgMain(EXPO_ENTRY);
run('npx expo export --platform web --clear');

setPkgMain(ELECTRON_MAIN);
run('npx electron-forge make');
