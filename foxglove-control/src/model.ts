export type Task = { id: string; prompt: string };
export type Catalog = { id: string; name: string; tasks: Task[] };
export type Status = {
  workflow_phase?: string;
  workflow_revision?: string;
  selected_catalog_id?: string;
  selected_task?: { catalog_id: string; catalog_name: string; task_id: string; prompt: string };
  task_catalogs?: Catalog[];
  task_episode_counts?: { saved: number; discarded: number };
  reset_available?: boolean;
  reset_state?: string;
  reset_error?: string;
  reset_role?: string;
  reset_completed_roles?: string[];
  teleop_state?: string;
  teleop_error?: string;
  follower_live?: boolean;
  follower_age_s?: number;
  camera_live?: boolean;
  camera_latency_ms?: number | null;
  camera_age_ms?: number | null;
  camera_fps?: number;
  recording?: boolean;
  pending_review?: boolean;
  last_episode_complete?: boolean;
  recording_error?: string;
  elapsed_s?: number;
  episode_path?: string;
  last_episode_path?: string;
  upload_state?: string;
  upload_error?: string;
  upload_repo?: string;
  upload_pending?: number;
  upload_completed?: number;
  review_status?: string;
};
export type Action =
  "start" | "stop" | "save" | "discard" | "reset" | "reset_leader" | "reset_follower";
export function enabled(
  action: Action,
  s: Status,
  { fresh, services, busy }: { fresh: boolean; services: boolean; busy: boolean },
): boolean {
  if (!services) {
    return false;
  }
  // Stop remains reachable while another request is pending or telemetry is stale.
  if (action === "stop") {
    return true;
  }
  if (!fresh || busy || !s.workflow_revision) {
    return false;
  }
  const phase = s.workflow_phase;
  if (action === "start") {
    return phase === "ready" && s.camera_live === true;
  }
  if (action === "reset" || action === "reset_leader" || action === "reset_follower") {
    return (phase === "reset_required" || phase === "ready") && s.reset_available === true;
  }
  if (action === "save") {
    return phase === "recording" || (phase === "review" && s.last_episode_complete === true);
  }
  return phase === "recording" || phase === "review";
}
