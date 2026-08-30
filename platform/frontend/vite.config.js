import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [vue()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
      },
      '/media': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
      },
      '/live': {
        target: 'http://127.0.0.1:8080',
        changeOrigin: true,
      },
      // The embedded 3D scene. Proxying rather than linking to the host's own
      // port is the whole point: the replay console reaches into the frame's
      // document, which only works while it is same-origin with the app. In
      // production nginx serves this path from the mounted bundle; in dev it is
      //   robot/script/robot/foxglove_web_serve.py --port 8094 \
      //     --default-layout docs/yuwang/embedded_scene_layout.json
      '/foxglove': {
        target: 'http://127.0.0.1:8094',
        changeOrigin: true,
        ws: true,
        // The host serves the bundle at its own root; nginx mounts that same
        // directory under /foxglove/ with an alias. Strip the prefix here so dev
        // and production agree on the URL the app embeds.
        rewrite: (urlPath) => urlPath.replace(/^\/foxglove/, '') || '/',
      },
    },
  },
})
