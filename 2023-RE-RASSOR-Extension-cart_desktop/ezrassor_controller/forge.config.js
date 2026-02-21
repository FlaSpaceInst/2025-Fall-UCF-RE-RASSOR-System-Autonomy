const { FusesPlugin } = require('@electron-forge/plugin-fuses');
const { FuseV1Options, FuseVersion } = require('@electron/fuses');

module.exports = {
  packagerConfig: {
    asar: true,
  },
  rebuildConfig: {},
  makers: [
    {
      name: '@electron-forge/maker-squirrel',
      config: {
        name: 'ezrassor-app',
        description: 'Desktop application of the 2024 Re-Rassor controller.',
        authors: 'Christian Vincent, Jose, Madi',
      },
    },
    {
      name: '@electron-forge/maker-zip',
      // Omit the platforms field if it's intended for all platforms or specify 'win32'
    },
    // Remove or omit Linux-specific makers if not needed
    // {
    //   name: '@electron-forge/maker-deb',
    //   platforms: ['linux'],
    // },
    // {
    //   name: '@electron-forge/maker-rpm',
    //   platforms: ['linux'],
    // },
  ],
  plugins: [
    {
      name: '@electron-forge/plugin-auto-unpack-natives',
      config: {},
    },
    // Fuses are used to enable/disable various Electron functionality
    // at package time, before code signing the application
    new FusesPlugin({
      version: FuseVersion.V1,
      [FuseV1Options.RunAsNode]: false,
      [FuseV1Options.EnableCookieEncryption]: true,
      [FuseV1Options.EnableNodeOptionsEnvironmentVariable]: false,
      [FuseV1Options.EnableNodeCliInspectArguments]: false,
      [FuseV1Options.EnableEmbeddedAsarIntegrityValidation]: true,
      [FuseV1Options.OnlyLoadAppFromAsar]: true,
    }),
  ],
};
