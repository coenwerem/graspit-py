const { setTimeout: sleep } = require('node:timers/promises')

// Use the same run-scoped artifact client as actions/deploy-pages.
async function waitForPagesArtifact(client, artifactId, {
  attempts = 12,
  intervalMs = 5000,
  sleep: pause = sleep,
  log = console.log,
} = {}) {
  if (!/^[1-9]\d*$/.test(String(artifactId))) {
    throw new Error('The Pages upload did not return a valid artifact ID')
  }
  for (let attempt = 1; attempt <= attempts; attempt++) {
    const { artifacts } = await client.listArtifacts()
    const artifact = artifacts.find(item => item.name === 'github-pages' && item.id === Number(artifactId))
    if (artifact) {
      log(`Pages artifact ${artifactId} is visible (attempt ${attempt}/${attempts})`)
      return artifact
    }
    log(`Pages artifact ${artifactId} is not visible (attempt ${attempt}/${attempts})`)
    if (attempt < attempts) await pause(intervalMs)
  }
  throw new Error(`Pages artifact ${artifactId} is not visible after ${attempts} attempts`)
}

module.exports = { waitForPagesArtifact }
