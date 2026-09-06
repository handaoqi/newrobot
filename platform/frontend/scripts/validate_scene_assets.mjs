import crypto from 'node:crypto'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const ASSET_DIR = path.resolve(HERE, '../public/scene-assets')
const MAX_ASSET_BYTES = 256 * 1024
const EXPECTED_ASSETS = 18

function sha256(buffer) {
  return crypto.createHash('sha256').update(buffer).digest('hex')
}

function parseGlb(loader, buffer, name) {
  const arrayBuffer = buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength)
  return new Promise((resolve, reject) => loader.parse(arrayBuffer, '', resolve, error => reject(new Error(`${name}: ${error.message}`))))
}

async function main() {
  const catalog = JSON.parse(await fs.readFile(path.join(ASSET_DIR, 'catalog.json'), 'utf8'))
  if (catalog.schema !== 'roamerx.scene-assets.v1') throw new Error('catalog schema mismatch')
  if (catalog.assets.length !== EXPECTED_ASSETS) throw new Error(`expected ${EXPECTED_ASSETS} assets, got ${catalog.assets.length}`)
  const ids = new Set()
  const loader = new GLTFLoader()
  let totalBytes = 0
  for (const entry of catalog.assets) {
    if (ids.has(entry.asset_id)) throw new Error(`duplicate asset id: ${entry.asset_id}`)
    ids.add(entry.asset_id)
    const filename = entry.url.replace('/scene-assets/', '')
    const buffer = await fs.readFile(path.join(ASSET_DIR, filename))
    if (buffer.subarray(0, 4).toString('ascii') !== 'glTF') throw new Error(`${entry.asset_id}: not a GLB`)
    if (buffer.byteLength > MAX_ASSET_BYTES) throw new Error(`${entry.asset_id}: exceeds ${MAX_ASSET_BYTES} bytes`)
    if (sha256(buffer) !== entry.sha256) throw new Error(`${entry.asset_id}: SHA-256 mismatch`)
    const gltf = await parseGlb(loader, buffer, entry.asset_id)
    if (gltf.animations.length) throw new Error(`${entry.asset_id}: animations are not allowed`)
    let triangles = 0
    let textureCount = 0
    gltf.scene.traverse(node => {
      if (node.isMesh) triangles += Math.floor((node.geometry.index?.count || node.geometry.getAttribute('position')?.count || 0) / 3)
      if (node.material?.map) textureCount += 1
    })
    if (textureCount) throw new Error(`${entry.asset_id}: textures are not allowed`)
    if (triangles !== entry.triangle_count) throw new Error(`${entry.asset_id}: triangle count mismatch`)
    totalBytes += buffer.byteLength
  }
  if (totalBytes > 5 * 1024 * 1024) throw new Error('asset package exceeds 5 MiB')
  console.log(`Validated ${catalog.assets.length} GLB assets (${totalBytes} bytes)`)
}

await main()
