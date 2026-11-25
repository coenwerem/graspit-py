const assert = require('node:assert/strict')
const { test } = require('node:test')
const { waitForPagesArtifact } = require('./wait-for-pages-artifact.cjs')
const artifact = { id: 42, name: 'github-pages', size: 2568503, createdAt: new Date('2026-11-24T03:15:05Z') }

function scenario(responses) {
  let calls = 0
  const delays = []
  return {
    client: { async listArtifacts() {
      const response = responses[calls++]
      if (response instanceof Error) throw response
      return { artifacts: response }
    } },
    options: { attempts: 3, intervalMs: 5, sleep: async delay => delays.push(delay), log: () => {} },
    calls: () => calls,
    delays,
  }
}

test('retries empty discovery until the uploaded artifact becomes visible', async () => {
  const run = scenario([[], [], [artifact]])
  assert.equal(await waitForPagesArtifact(run.client, 42, run.options), artifact)
  assert.equal(run.calls(), 3)
  assert.deepEqual(run.delays, [5, 5])
})

test('returns immediately when the uploaded artifact is visible', async () => {
  const run = scenario([[artifact]])
  assert.equal(await waitForPagesArtifact(run.client, 42, run.options), artifact)
  assert.deepEqual(run.delays, [])
})

test('does not accept an artifact from an earlier upload', async () => {
  const run = scenario([[{ ...artifact, id: 41 }], [artifact]])
  assert.equal(await waitForPagesArtifact(run.client, 42, run.options), artifact)
  assert.equal(run.calls(), 2)
})

test('does not accept a different artifact name', async () => {
  const run = scenario([[{ ...artifact, name: 'other' }], [artifact]])
  assert.equal(await waitForPagesArtifact(run.client, 42, run.options), artifact)
  assert.equal(run.calls(), 2)
})

test('fails after the retry limit without a final sleep', async () => {
  const run = scenario([[], [], []])
  await assert.rejects(waitForPagesArtifact(run.client, 42, run.options), /not visible after 3 attempts/)
  assert.equal(run.calls(), 3)
  assert.deepEqual(run.delays, [5, 5])
})

test('fails promptly on artifact service errors', async () => {
  const run = scenario([new Error('access denied')])
  await assert.rejects(waitForPagesArtifact(run.client, 42, run.options), /access denied/)
  assert.equal(run.calls(), 1)
  assert.deepEqual(run.delays, [])
})

test('rejects a missing upload ID before querying the service', async () => {
  const run = scenario([])
  await assert.rejects(waitForPagesArtifact(run.client, '', run.options), /artifact ID/)
  assert.equal(run.calls(), 0)
})
