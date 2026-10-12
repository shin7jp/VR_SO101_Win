import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import basicSsl from "@vitejs/plugin-basic-ssl";

// WebXR は HTTPS が必須。HTTPS ページから ws:// は開けない (mixed content) ので、
// /api と /ws は同一オリジンで Vite がバックエンド (http://localhost:8000) へ中継する。
const backend = process.env.VRSO101_BACKEND_URL ?? "http://localhost:8000";
export default defineConfig({
  plugins: [react(), basicSsl()],
  server: {
    host: true,
    port: 5173,
    proxy: {
      "/api": { target: backend, changeOrigin: true },
      "/ws": { target: backend.replace("http", "ws"), ws: true, changeOrigin: true },
    },
  },
  build: { outDir: "dist", chunkSizeWarningLimit: 1500 },
});
