const dnssd = require('dnssd');

function scanPotatoes() {
  const browser = new dnssd.Browser(dnssd.tcp('potato'));

  browser.on('serviceUp', service => {
    console.log(`🧅 Found: ${service.name}`);
    console.log(`    IP: ${service.addresses[0]}`);
    console.log(`    Port: ${service.port}`);
  });

  browser.start();
}

module.exports = { scanPotatoes };
