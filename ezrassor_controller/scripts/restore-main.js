/**
 * restore-main.js
 *
 * Ensures package.json "main" is set back to main.js before starting the
 * Electron dev server.  Runs automatically via the "prestart" npm hook.
 */

'use strict';

const fs   = require('fs');
const path = require('path');

const PKG_PATH = path.join(__dirname, '..', 'package.json');
const pkg = JSON.parse(fs.readFileSync(PKG_PATH, 'utf-8'));

if (pkg.main !== 'main.js') {
    pkg.main = 'main.js';
    fs.writeFileSync(PKG_PATH, JSON.stringify(pkg, null, 2) + '\n');
    console.log('[restore-main] package.json "main" restored → main.js');
}
