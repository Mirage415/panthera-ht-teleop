import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { Action, enabled, Status } from "./model";

const CSS = `
.pc {font:13px/1.5 Inter,ui-sans-serif,system-ui,sans-serif;--bg:#fff;--fg:#18181b;--muted:#71717a;--line:#e4e4e7;--sub:#fafafa;box-sizing:border-box;color:var(--fg);background:var(--bg);padding:16px;min-height:100%;}
.pc.dark {--bg:#09090b;--fg:#fafafa;--muted:#a1a1aa;--line:#27272a;--sub:#18181b;}
.pc * {box-sizing:border-box;} .pc h2 {font-size:17px;margin:0;font-weight:600;letter-spacing:-.4px}.pc p {margin:4px 0}.pc .muted {color:var(--muted);font-size:12px}.pc .row {display:flex;align-items:center;gap:8px;flex-wrap:wrap}.pc .heading {justify-content:space-between;margin-bottom:14px}.pc .badge {border:1px solid var(--line);border-radius:6px;padding:3px 8px;font-size:11px;font-weight:600}.pc .good {color:#16a34a}.pc .warn {color:#d97706}.pc .error {color:#ef4444;white-space:pre-wrap;overflow-wrap:anywhere}.pc .box {border:1px solid var(--line);border-radius:9px;padding:12px;margin-bottom:12px}.pc label {font-size:12px;font-weight:500;display:block;margin-bottom:5px}.pc select,.pc input,.pc textarea {width:100%;font:inherit;color:var(--fg);background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px;min-width:0}.pc textarea {resize:vertical;min-height:66px}.pc button {font:inherit;font-weight:500;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--fg);padding:8px 12px;cursor:pointer;white-space:nowrap}.pc button:disabled {opacity:.4;cursor:not-allowed}.pc button:not(:disabled):hover {filter:brightness(.9)}.pc button.primary {background:var(--fg);color:var(--bg)}.pc button.danger {color:#ef4444}.pc button.stop {border-color:#d97706;color:#d97706}.pc .grid {display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px}.pc .metric {font-size:19px;font-weight:600;letter-spacing:-.5px}.pc .field {margin-bottom:10px}.pc .buttons {display:grid;grid-template-columns:repeat(auto-fit,minmax(82px,1fr));gap:8px}.pc .path {overflow-wrap:anywhere;font-family:ui-monospace,monospace;font-size:10px}.pc .steps {font-size:11px;color:var(--muted);margin:10px 0}.pc details {margin-top:8px}.pc summary {cursor:pointer;color:var(--muted);font-size:12px}.pc .form {margin-top:10px}.pc .notice {border-left:2px solid var(--line);padding-left:10px;margin:12px 0}.pc .spacer {flex:1}
`;
const LABELS: Record<string, string> = {
  needs_task: "Select a task",
  ready: "Ready",
  starting: "Preparing",
  recording: "Recording",
  review: "Review episode",
  reset_required: "Reset required",
  resetting: "Resetting",
  stopping: "Stopping",
};
const HINTS: Record<string, string> = {
  needs_task: "Create a task set, then add a subtask with its exact prompt.",
  ready: "Prepare the scene, then press Start.",
  starting: "Hold still. Recording begins when follower feedback is ready.",
  recording: "Save keeps this episode. Discard excludes it from training.",
  review: "Choose Save or Discard before continuing.",
  reset_required: "Reset Leader and Reset Follower independently to this episode’s start pose.",
  resetting: "Returning to the measured start pose. Stop cancels motion.",
  stopping: "Waiting for the robot process to finish.",
};

function Panel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [status, setStatus] = useState<Status>({});
  const [receivedAt, setReceivedAt] = useState(0);
  const [now, setNow] = useState(performance.now());
  const [dark, setDark] = useState(true);
  const [busy, setBusy] = useState("");
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [name, setName] = useState("");
  const [prompt, setPrompt] = useState("");
  const done = useRef<() => void>();
  const mounted = useRef(true);
  const inFlight = useRef(new Set<string>());
  useLayoutEffect(() => {
    mounted.current = true;
    context.onRender = (state, renderDone) => {
      done.current?.();
      done.current = renderDone;
      if (state.colorScheme) {
        setDark(state.colorScheme === "dark");
      }
      for (const event of state.currentFrame ?? []) {
        if (
          event.topic === "/capture/status" &&
          typeof event.message === "object" &&
          event.message != undefined
        ) {
          setStatus(event.message);
          setReceivedAt(performance.now());
        }
      }
      setNow(performance.now());
    };
    context.watch("currentFrame");
    context.watch("colorScheme");
    context.subscribe([{ topic: "/capture/status" }]);
    const timer = window.setInterval(() => {
      setNow(performance.now());
    }, 250);
    return () => {
      mounted.current = false;
      clearInterval(timer);
      done.current?.();
      done.current = undefined;
      context.unsubscribeAll();
      context.onRender = undefined;
    };
  }, [context]);
  useEffect(() => {
    done.current?.();
    done.current = undefined;
  });
  const fresh = receivedAt > 0 && now - receivedAt < 1500;
  const services = context.callService != undefined;
  const phase = status.workflow_phase ?? "needs_task";
  const editable = fresh && services && !busy && (phase === "ready" || phase === "needs_task");
  const catalogs = status.task_catalogs ?? [];
  const catalog = catalogs.find((c) => c.id === status.selected_catalog_id);
  useEffect(() => {
    setPrompt(status.selected_task?.prompt ?? "");
  }, [status.selected_catalog_id, status.selected_task?.task_id, status.selected_task?.prompt]);
  const promptApplied = prompt.trim().length > 0 && prompt.trim() === status.selected_task?.prompt;
  function applyPrompt(): void {
    if (!catalog) {
      return;
    }
    const existing = catalog.tasks.find((task) => task.prompt === prompt.trim());
    void invoke(
      existing ? "task_select" : "task_create",
      existing
        ? { catalog_id: catalog.id, task_id: existing.id }
        : { catalog_id: catalog.id, prompt: prompt.trim() },
    );
  }
  async function invoke(action: string, fields: Record<string, string> = {}): Promise<boolean> {
    if (
      !context.callService ||
      inFlight.current.has(action) ||
      (action !== "stop" && inFlight.current.size > 0)
    ) {
      return false;
    }
    inFlight.current.add(action);
    setBusy(action);
    setError("");
    setNotice("");
    try {
      const response = (await context.callService("/capture/command", {
        action,
        revision: status.workflow_revision ?? "",
        ...fields,
      })) as (Status & { ok?: boolean; message?: string }) | undefined;
      if (response?.ok !== true) {
        throw new Error(response?.message ?? "Command rejected");
      }
      if (mounted.current) {
        // Streaming status is authoritative; never overwrite a newer phase with a late response.
        setNotice(response.message ?? "Done");
      }
      return true;
    } catch (reason) {
      if (mounted.current) {
        setError(reason instanceof Error ? reason.message : String(reason));
      }
      return false;
    } finally {
      inFlight.current.delete(action);
      if (mounted.current) {
        setBusy([...inFlight.current][0] ?? "");
      }
    }
  }
  const metric = (value: number | null | undefined, unit: string): string =>
    fresh && typeof value === "number" && Number.isFinite(value)
      ? `${value.toFixed(1)} ${unit}`
      : "—";
  const robot = !fresh
    ? "Unavailable"
    : status.reset_state === "resetting"
      ? `Resetting ${status.reset_role ?? ""}`
      : status.teleop_state === "running"
        ? status.follower_live === true
          ? "Teleoperating"
          : "Feedback lost"
        : status.teleop_state === "idle"
          ? "Stopped"
          : status.teleop_state === "error"
            ? "Error"
            : (status.teleop_state ?? "Unknown");
  const issue = [
    error,
    ...(fresh
      ? [status.reset_error, status.teleop_error, status.recording_error, status.upload_error]
      : []),
  ].find((value) => value != undefined && value.length > 0);
  return (
    <main className={`pc ${dark ? "dark" : ""}`}>
      <style>{CSS}</style>
      <div className="row heading">
        <h2>Panthera Collection</h2>
        <span className={`badge ${fresh ? (phase === "recording" ? "good" : "") : "warn"}`}>
          {fresh ? (LABELS[phase] ?? phase) : "Disconnected / stale"}
        </span>
      </div>
      <section className="box">
        <div className="field">
          <label htmlFor="pc-catalog">Task set</label>
          <select
            id="pc-catalog"
            disabled={!editable}
            value={catalog?.id ?? ""}
            onChange={(e) =>
              void invoke("task_select", { catalog_id: e.target.value, task_id: "" })
            }
          >
            <option value="" disabled>
              Select a task set
            </option>
            {catalogs.map((c) => (
              <option value={c.id} key={c.id}>
                {c.name}
              </option>
            ))}
          </select>
        </div>
        <div className="field">
          <label htmlFor="pc-prompt">Subtask / prompt</label>
          <div className="row">
            <input
              id="pc-prompt"
              type="text"
              style={{ flex: 1 }}
              disabled={!editable || !catalog}
              maxLength={2000}
              placeholder="e.g. Place the red block in the bowl."
              value={prompt}
              onChange={(e) => {
                setPrompt(e.target.value);
              }}
              onKeyDown={(e) => {
                if (e.key === "Enter" && !e.nativeEvent.isComposing && editable && prompt.trim()) {
                  applyPrompt();
                }
              }}
            />
            <button
              disabled={!editable || !catalog || !prompt.trim() || promptApplied}
              onClick={applyPrompt}
            >
              Use prompt
            </button>
          </div>
          {!promptApplied && prompt.trim() && (
            <p className="muted">Apply this prompt before starting.</p>
          )}
        </div>
        <details>
          <summary>New task set</summary>
          <div className="form">
            <label htmlFor="pc-name">New task set</label>
            <div className="row">
              <input
                id="pc-name"
                style={{ flex: 1 }}
                disabled={!editable}
                maxLength={120}
                placeholder="e.g. Tabletop pick and place"
                value={name}
                onChange={(e) => {
                  setName(e.target.value);
                }}
              />
              <button
                disabled={!editable || !name.trim()}
                onClick={() =>
                  void invoke("catalog_create", { name: name.trim() }).then((ok) => {
                    if (ok) {
                      setName("");
                    }
                  })
                }
              >
                Create set
              </button>
            </div>
          </div>
        </details>
      </section>
      <div className="steps">TASK → RECORD → SAVE / DISCARD → RESET → NEXT EPISODE</div>
      <div className="buttons">
        {(["start", "stop", "save", "discard", "reset_leader", "reset_follower"] as Action[]).map(
          (action) => (
            <button
              key={action}
              className={
                action === "start" || action === "save"
                  ? "primary"
                  : action === "discard"
                    ? "danger"
                    : action === "stop"
                      ? "stop"
                      : ""
              }
              disabled={
                !enabled(action, status, { fresh, services, busy: !!busy }) ||
                inFlight.current.has(action) ||
                (action === "start" && !promptApplied)
              }
              onClick={() => void invoke(action)}
            >
              {busy === action
                ? "Working…"
                : action === "reset_leader"
                  ? "Reset Leader"
                  : action === "reset_follower"
                    ? "Reset Follower"
                    : action.charAt(0).toUpperCase() + action.slice(1)}
            </button>
          ),
        )}
      </div>
      <p className="muted notice">
        {!services
          ? "Connect to the live capture service to enable controls."
          : !fresh
            ? "Waiting for live capture status. Controls resume after reconnecting."
            : HINTS[phase]}
      </p>
      {phase === "reset_required" && (
        <p className="muted">
          Reset completed:{" "}
          {(status.reset_completed_roles ?? []).length > 0
            ? (status.reset_completed_roles ?? []).join(", ")
            : "none"}
          . Both arms must finish before the next episode.
        </p>
      )}
      <div className="grid">
        <section className="box">
          <label>Robot</label>
          <div className="metric">{robot}</div>
          <p className="muted">
            Follower feedback:{" "}
            {fresh && status.follower_live === true
              ? metric((status.follower_age_s ?? 0) * 1000, "ms ago")
              : "not live"}
          </p>
        </section>
        <section className="box">
          <label>Camera latency · estimated</label>
          <div className="metric">{metric(status.camera_latency_ms, "ms")}</div>
          <p className="muted">Sensor → capture host · excludes network / display</p>
          <p className="muted">
            {fresh && status.camera_live === true
              ? `${metric(status.camera_fps, "FPS")} · Last frame ${metric(status.camera_age_ms, "ms ago")}`
              : "Camera unavailable"}
          </p>
        </section>
        <section className="box">
          <label>Episode</label>
          <div className="metric">
            {fresh && status.recording === true
              ? metric(status.elapsed_s, "s")
              : fresh
                ? (status.review_status ?? "Ready")
                : "—"}
          </div>
          <p className="muted">
            This subtask: {status.task_episode_counts?.saved ?? 0} saved ·{" "}
            {status.task_episode_counts?.discarded ?? 0} discarded
          </p>
        </section>
      </div>
      {issue && (
        <p role="alert" className="error">
          {issue}
        </p>
      )}
      {notice && (
        <p role="status" className="muted">
          {notice}
        </p>
      )}
      <details>
        <summary>
          Storage and upload · {fresh ? (status.upload_state ?? "waiting") : "unavailable"}
        </summary>
        <p>{status.upload_repo}</p>
        <p className="muted">
          {status.upload_pending ?? 0} pending · {status.upload_completed ?? 0} uploaded
        </p>
        <p className="path">
          {[status.episode_path, status.last_episode_path].find(
            (value) => value != undefined && value.length > 0,
          ) ?? "No episode yet"}
        </p>
        <p className="muted">
          Save uploads the episode with its exact prompt. Discard keeps local recovery files only.
        </p>
      </details>
    </main>
  );
}
export function initPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<Panel context={context} />);
  return () => {
    root.unmount();
  };
}
