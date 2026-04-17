const { FusesPlugin } = require('@electron-forge/plugin-fuses');
const { FuseV1Options, FuseVersion } = require('@electron/fuses');

module.exports = {
  packagerConfig: {
    asar: true,
    // Specify the icon path without extension — Electron Packager automatically
    // appends the correct extension per platform:
    //   Windows → .ico   macOS → .icns   Linux → .png
    icon: './assets/icon',
    name: 'RE-RASSOR Controller',
    executableName: 'ezrassor-app',
    appVersion: '1.0.0',
  },
  rebuildConfig: {},
  makers: [
    // ── Windows ──────────────────────────────────────────────────────────────
    {
      name: '@electron-forge/maker-squirrel',
      platforms: ['win32'],
      config: {
        name:        'ezrassor_controller',
        description: 'RE-RASSOR rover desktop controller',
        authors:     'UCF RE-RASSOR Team',
      },
    },
    // ── macOS ─────────────────────────────────────────────────────────────────
    {
      name: '@electron-forge/maker-zip',
      platforms: ['darwin'],
    },
    // ── Linux (Debian/Ubuntu) ─────────────────────────────────────────────────
    {
      name: '@electron-forge/maker-deb',
      platforms: ['linux'],
      config: {
        options: {
          name:        'ezrassor-controller',
          productName: 'RE-RASSOR Controller',
          description: 'RE-RASSOR rover desktop controller',
          maintainer:  'UCF RE-RASSOR Team',
          homepage:    'https://github.com/FlaSpaceInst',
          categories:  ['Utility'],
        },
      },
    },
  ],
  plugins: [
    {
      name: '@electron-forge/plugin-auto-unpack-natives',
      config: {},
    },
    new FusesPlugin({
      version: FuseVersion.V1,
      [FuseV1Options.RunAsNode]:                           false,
      [FuseV1Options.EnableCookieEncryption]:              true,
      [FuseV1Options.EnableNodeOptionsEnvironmentVariable]: false,
      [FuseV1Options.EnableNodeCliInspectArguments]:       false,
      [FuseV1Options.EnableEmbeddedAsarIntegrityValidation]: true,
      [FuseV1Options.OnlyLoadAppFromAsar]:                 true,
    }),
  ],
};
