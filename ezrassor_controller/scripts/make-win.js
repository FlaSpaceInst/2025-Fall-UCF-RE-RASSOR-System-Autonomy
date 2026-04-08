/**
 * make-win.js
 *
 * Same main-field swap as make-mac.js, but targets the Squirrel maker for
 * Windows packaging.  Run this on a Windows host.
 *
 * Usage:  node scripts/make-win.js
 */

'use strict';

const { execSync } = require('child_process');
const fs            = require('fs');
const path          = require('path');

const PKG_PATH      = path.join(__dirname, '..', 'package.json');
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
    console.log(`[make-win] package.json "main" → ${value}`);
}

// Always restore on exit (success, error, or Ctrl-C)
process.on('exit', () => setPkgMain(ELECTRON_MAIN));
process.on('SIGINT',  () => process.exit(1));
process.on('SIGTERM', () => process.exit(1));

setPkgMain(EXPO_ENTRY);
run('npx expo export --platform web --clear');

setPkgMain(ELECTRON_MAIN);
run('npx electron-forge make --targets @electron-forge/maker-squirrel');
