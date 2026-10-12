import { useEffect, useState } from "react";
import { useStore } from "./lib/store";
import { connectWs, disconnectWs } from "./lib/ws";
import { ModeBadge, Toasts, Badge } from "./components/common";
import Dashboard from "./pages/Dashboard";
import Simulation from "./pages/Simulation";
import VrTeleop from "./pages/VrTeleop";
import DataCollection from "./pages/DataCollection";
import DatasetExplorer from "./pages/DatasetExplorer";
import Training from "./pages/Training";
import ModelRegistry from "./pages/ModelRegistry";
import Inference from "./pages/Inference";
import VideoGallery from "./pages/VideoGallery";
import Evaluation from "./pages/Evaluation";
import Settings from "./pages/Settings";

const PAGES: [string, string, () => JSX.Element][] = [
  ["dashboard", "Dashboard", Dashboard], ["simulation", "Simulation", Simulation], ["vr", "VR Teleoperation", VrTeleop],
  ["collect", "Data Collection", DataCollection], ["dataset", "Dataset Explorer", DatasetExplorer], ["train", "ACT Training", Training],
  ["models", "Model Registry", ModelRegistry], ["infer", "Gazebo Inference", Inference], ["videos", "Video Gallery", VideoGallery],
  ["eval", "Evaluation", Evaluation], ["settings", "Settings", Settings],
];

export default function App() {
  const [page, setPage] = useState(() => location.hash.slice(1) || "dashboard");
  const ws = useStore((s) => s.wsState);
  useEffect(() => {
    const h = () => setPage(location.hash.slice(1) || "dashboard");
    window.addEventListener("hashchange", h);
    useStore.getState().loadStatic();
    useStore.getState().refresh();
    connectWs();
    return () => { window.removeEventListener("hashchange", h); disconnectWs(); };
  }, []);
  const Cur = (PAGES.find((p) => p[0] === page) ?? PAGES[0])[2];
  return (
    <div className="app">
      <nav className="nav">
        <div className="brand">VR × SO-101 × ACT</div>
        {PAGES.map(([id, label]) => <a key={id} className={page === id ? "on" : ""} onClick={() => (location.hash = id)}>{label}</a>)}
      </nav>
      <div className="main">
        <div className="topbar">
          <ModeBadge />
          {ws !== "open" && <Badge kind="warn">{ws === "connecting" ? "接続中…" : "バックエンド切断 — 再接続中"}</Badge>}
        </div>
        <Cur />
      </div>
      <Toasts />
    </div>
  );
}
