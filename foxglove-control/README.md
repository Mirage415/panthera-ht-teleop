# Panthera Collection Console

A Foxglove panel for the existing Panthera capture connection. It uses the current
`/capture/status` topic and `/capture/command` service; it introduces no new robot,
camera, storage, or Hugging Face address.

Task set → exact prompt → Start → Stop (optional) → Save / Discard → Reset → next episode.
Start launches Teleop and recording. Save/Discard also stop an active recording.
Reset Leader / Reset Follower independently return the selected arm and gripper
to its measured first recorded pose. The other arm need not be connected and no
Teleop or recording session is started. Both resets must complete before the next episode. It is a separate explicit action, never triggered by loading this panel.
The scene objects must still be repositioned by the operator.

## Install into your existing page

Download the `.foxe` package, drag it onto Foxglove, then choose
**Add panel → Panthera Collection**. Keep the existing connection, camera and plots.
Organization admins can instead upload the same package in Organization Settings →
Extensions, which distributes it to organization members. No layout JSON is required.
Local extension installation may require a Foxglove developer seat.

The panel disables actions when telemetry is stale; Stop remains available on a
live service connection even while another action is pending. The backend rejects
stale revisioned commands. Task changes are blocked until review and reset finish.
Save queues LeRobot export with the exact prompt stored in episode metadata.

Camera latency is an estimate from RealSense's device timestamp mapped to the host
clock (`global_time`) to the capture process receiving the frame. It excludes JPEG
encoding, network transfer and browser rendering. Unsupported or implausible clock
samples show an em dash, never a fabricated zero. Robot idle does not imply power off.

## Develop

`npm ci`, `npm run typecheck`, `npm run lint`, `npm run package`.
For the browser tests: `npx playwright install chromium`, then `npm test`.
Browser tests run against a mocked Foxglove context and never call the real motors.

The task/review/reset workflow and compact neutral visual style are adapted in
concept from https://github.com/pengyue-polaron/embodied-ops and
https://github.com/pengyue-polaron/galaxea-a1-runtime. Their Galaxea hardware adapter,
addresses, fixed home positions, credentials and organization layouts are not used.
