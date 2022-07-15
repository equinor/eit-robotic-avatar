import { defineConfig } from 'vite'
import basicSsl from '@vitejs/plugin-basic-ssl'

export default defineConfig({
  root: "client",
  server: {
    https: true
  },
  plugins: [
    basicSsl()
  ]
})