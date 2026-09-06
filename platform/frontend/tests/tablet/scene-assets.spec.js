import { expect, test } from '@playwright/test'

test('published scene asset package serves its catalog and every GLB', async ({ request }) => {
  const catalogResponse = await request.get('/scene-assets/catalog.json')
  expect(catalogResponse.ok()).toBe(true)
  const catalog = await catalogResponse.json()
  expect(catalog.schema).toBe('roamerx.scene-assets.v1')
  expect(catalog.assets).toHaveLength(18)

  for (const entry of catalog.assets) {
    const response = await request.get(entry.url)
    expect(response.ok(), entry.asset_id).toBe(true)
    const body = await response.body()
    expect(body.subarray(0, 4), entry.asset_id).toEqual(Buffer.from('glTF'))
    expect(body.byteLength, entry.asset_id).toBe(Number(entry.byte_size))
  }
})
