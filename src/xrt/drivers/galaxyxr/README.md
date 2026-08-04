# Samsung Galaxy XR (SM-I610) on Linux

3DoF Monado support for the Samsung Galaxy XR running desktop Linux: this
driver (IMU + optics) plus the `galaxyxr` compositor backend in
`src/xrt/compositor/main/comp_window_galaxyxr.c` (dual DRM lease direct mode).
Both autodetect via the device-tree model (`SM-I610` / `Samsung XR`).
Camera passthrough (`XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND` via titan-server)
has its own section below.

Quick start (kwin running under sddm owns the panels and grants the leases):

```
sudo -u sddm env XDG_RUNTIME_DIR=/run/user/109 WAYLAND_DISPLAY=/run/user/109/wayland-0 \
    monado-service
sudo -u sddm env XDG_RUNTIME_DIR=/run/user/109 XR_RUNTIME_JSON=<build>/openxr_monado-dev.json \
    <some OpenXR app>
sudo ./gxr_perf.sh on     # optional: pin GPU clocks, see Performance
```


## Display pipeline

The two eye panels are separate DRM devices, both Qualcomm SDE:

| eye   | MDSS MMIO  | panel               | mode         | extra connectors  |
|-------|------------|---------------------|--------------|-------------------|
| left  | `ae00000`  | Sony ECX344A (DSI)  | 3552x3840@90 | DP-1, Virtual-1   |
| right | `15600000` | Sony ECX344A (DSI)  | 3552x3840@90 | none              |

Things the backend has to deal with:

- **DSI-N numbering is not stable across boots** (global probe-order ida in
  the kernel). Eyes are identified by the lessor device sysfs path
  (`ae00000` = left, `15600000` = right), with "the device that also has the
  DP/Virtual connectors is left" as a fallback. Confirmed by physically
  looking into the headset.
- **A panel cannot be fed by one plane.** The SDE feeds each panel with four
  888 px wide SSPP slices side by side. The four planes are picked from the
  lease using the downstream SDE `capabilities` property blob: prefer DMA0-3
  pipes for the 1:1 layer (`max_upscale <= 1`, no `primary_smart_plane_id`),
  ordered by `pipe_idx`; fall back to virtual VIG pipes if DMA is unavailable.
  Vulkan display WSI cannot express any of this, hence the fully custom
  `comp_target`.
- **Leasing:** one wp_drm_lease_v1 lease per DRM device (the protocol forbids
  mixing devices in a request). The backend requests only the panel DSI
  connector from each device; DP-1 and Virtual-1 stay with the compositor.
- **Buffers:** 3 stereo images of 7104x3840, left eye at x 0..3551, right at
  x 3552..7103. Preferred path is **UBWC** (`DRM_FORMAT_MOD_QCOM_COMPRESSED`):
  Turnip lays out and allocates the image, exports a dma-buf, and both lease
  fds get an `AddFB2WithModifiers` framebuffer, validated with a TEST_ONLY
  atomic commit before use. The SDE only scans UBWC in ABGR component
  orders, so this selects `XBGR8888`/`VK_FORMAT_R8G8B8A8_*` (it rejects
  `XRGB8888`+UBWC); Turnip reports one memory plane and supports both
  storage and color-attachment use on it. Fallback is the proven linear
  path: dumb buffers on the left lease fd (dma_heap as second fallback)
  imported into Vulkan as `DRM_FORMAT_MOD_LINEAR`, `XRGB8888` first.
- **Sync:** the render-complete semaphore is turned into a sync file
  (`VK_KHR_external_fence_fd`, note that a SYNC_FD export *resets* the
  fence) and passed to the atomic commits as per-plane `IN_FENCE_FD`, so
  present never blocks on the GPU. How frames flow from here is the
  Rendering section below.


## IMU / SSC

The IMU hangs off the Qualcomm SSC (ADSP sensor core), reached via the SEE
`sns_client` QMI service (id 400) over an `AF_QIPCRTR` socket; QMI is just an
envelope around hand-rolled protobufs (`galaxyxr_ssc.c`, derived from a
standalone tool, no Android blobs needed).

- Flow: QRTR service discovery -> SUID lookup per data type (`accel`,
  `gyro`) -> attribute read -> enable streaming for the instance with
  `hw_id == 0` (there are 5 accel/gyro instances, STMicro LSM6DSV).
- Timestamps are qtimer ticks at 19.2 MHz (`ns = ticks * 625 / 12`), used as
  the fusion timebase.
- Units are already m/s^2 and rad/s. Axis convention is Android-style and
  maps to the OpenXR device frame as identity (user-verified);
  `GALAXYXR_IMU_AXES` remaps without rebuilding.
- Calibration: the raw SSC streams apply none at all (the persist registry
  fac_cal is zero bias + identity for every LSM6DSV instance; the streams
  even report that in the one `sns_cal_event` they emit on enable). The
  driver applies the per-unit factory intrinsics from the efs profile
  `imus{}` entry with ordinal id "0" (key 100, = hw_id 0): per axis
  `true = scale * (raw - bias)`, in the stream frame before
  `GALAXYXR_IMU_AXES`. For the gyro bias it prefers the live estimate from
  the SSC's own online estimator: data_type `gyro_cal`, one on-change
  instance per IMU with no hw_id attr (the `rigid_body` attr names the IMU
  it calibrates), pushing the current TOTAL bias (`sns_cal_event` msg 1022)
  on enable and on change - it replaces the factory `default_gyro_bias`,
  never stacks with it. No live `accel_cal` exists on this unit.
  Misalignment, `gyro_q_accel` and `imu_q_6pts` are parsed but not applied
  (sub-degree effects, conventions unverified). The `/product` devkit
  profile carries another unit's `imus{}` (and no ordinal ids) and is never
  used for IMU cal. `GALAXYXR_IMU_CAL=false` reverts to raw samples,
  `gyro_bias` in the debug GUI shows the bias currently subtracted. The profile also carries VIO-style noise densities and
  full cal + IMU_0 extrinsics for all 5 IMUs (live-verified: each stream
  sits in its extrinsic frame, `v0 = R(q_ext)^T vk`); only hw 0 is consumed
  today, see the sensors tool README for the multi-IMU array analysis.
- Fusion is the x-io AHRS port (`m_imu_xio_ahrs`), orientation only.

The driver also subscribes to the **`ipd`** sensor (AKM AKL algo, on-change,
event msg-id 717 with 14 floats): the first two floats are the per-side
motorized-lens travel in micrometers, and with the 28.5 mm per-side hardware
minimum of the actuator, `ipd_mm = 57.0 + (ipd_L + ipd_R) / 1000` - the same
mapping Android's actuator HAL and OpenPX use. That live value drives the
render eye separation (sanity range 45..85 mm, `GALAXYXR_IPD_MM` overrides).
The 4 `trimag` sensors (AKM ak0997x Hall, raw) are its inputs - each reads a
lens magnet, so they are not a compass. Many more types exist; `--list` in
the original tool enumerates them.


## Headset input

The headset power button is the `KEY_POWER` key on the `pmic_pwrkey` evdev
device. The driver reads `/dev/input/event2` by default; set
`GALAXYXR_POWER_DEVICE` if the event number differs. The Monado service user
must have read access to the device. While the driver is running, it
exclusively grabs this evdev device so that the desktop does not also handle
`KEY_POWER`; closing the driver releases the grab.

OpenXR applications can bind it as `/user/head/input/system/click` under the
core `/interaction_profiles/htc/vive_pro` interaction profile. Monado uses
that profile as a compatibility profile for Galaxy XR because OpenXR has no
generic headset power-button component. The physical input remains named as
a Galaxy XR power click inside the driver.


## Optics

The authoritative model is THIS UNIT's factory calibration,
`/mnt/vendor/efs/device_profile.textproto` (`display_profile_v2`, see the
resource map for the format): per eye a panel-pixel grid of per-color rays
in the display module frame, the module -> eye transform, IMU -> module
extrinsics, the calibrated render fov (`verified_fov`: left -52.9/+39.1 h,
+53.4/-48.9 v; right -37.0/+55.1 h, +53.1/-51.6 v) and the calibration IPD
(62 mm). The driver composes the chain per eye - position and orientation
in the IMU frame, including the per-unit terms (module cants 5.8/6.9 deg
outward, per-eye pitch/roll up to ~1.4 deg, ~1 deg relative rotation
between the eyes, 3 mm vertical eye offset). Those sub-degree relative
terms are exactly what makes the images fuse; the devkit default profile
(v1 under /product, wrong unit: "DK2 ... WIF0126M 2023") cannot provide
them, which showed as a ~2-3 deg per-eye tilt 'V' on distant straight
lines. The per-eye ROTATION is folded into the distortion mapping and fov
(rays mapped module -> IMU frame at load, verified_fov rebounded there)
and the exposed view poses keep identity orientation + calibrated
positions: clients are only ever tested against identity view orientations
(xrgears does its vulkan y-flip by mirroring the world and camera in y,
which silently negates per-eye pitch/roll - with rotated view poses its
gears sat ~1 deg higher in one eye while the compositor-composited
quad/equirect layers fused fine). The same xrgears trick also negates the
vertical frustum asymmetry (fixed in our xrgears checkout, upstreamable);
the per-eye verified fovs are advertised as-is - clients that mishandle
per-eye vertical asymmetry show it as an inter-eye vertical offset and
should be fixed, not papered over. No vergence/cant knobs should be
needed with the efs file. CAC is off by
default (v2 carries exact per-color rays, so `GALAXYXR_DISTORTION=profile`
is worth an eyes-on retry). IPD tracks the SSC `ipd` sensor live (see the
IMU section); the render eye separation follows the motorized lenses.

The earlier reverse-engineering of the qvr svrapi lens tables
(`lens`/`mono` modes, panel/fov/cant guesses) is superseded by the profile
and was removed; see the git history and the resource map entry if those
tables are ever needed again.

## Android-side resource map

Everything useful found on the Android partitions (paths relative to
`/.oldroot`, which is the Android rootfs mounted next to this Linux system).
`/data/vendor/qvr` is uid-1000 owned and not readable as the sddm user;
`/vendor/etc` is world-readable.

Identification:

- `/proc/device-tree/model` (Linux side):
  `Samsung XR VST Gen2 PROJECT - SM-I610_REV07A(board-id,22)` - what the
  driver and backend autodetect on.

Optics and display:

- **`/mnt/vendor/efs/device_profile.textproto`** - THE display calibration:
  this unit's factory file (efs = Samsung per-unit factory partition,
  header carries the serial and calibration date, world readable, 4.2 MB).
  Format `display_profile_v2`, all in one file:
  - `sensor_extrinsics` IMU_0 -> `sensor_id` "1"/"2" (left/right display
    module datum; "3"/"4" are something else). Convention throughout:
    `a_t_b` p = origin of b in a, q rotates *coordinates* from a to b
    (so pose rotation of b in a is conj(q)).
  - Per eye `luts.cells`: a regular grid over panel pixels (u right,
    **v up**, center origin, 50 px step, outermost ring clamped to the
    panel edge at +-1776/+-1920), each cell holding per-color unnormalized
    ray directions (red/green/blue xyz) in the module frame (x right, y up,
    forward -z). The panel center sits ~0.2 deg off the module axis - all
    the cant lives in the extrinsics, unlike v1.
  - `lut_frame_t_eye`: module -> eye camera (CV convention: y down, z
    forward, hence the ~180 deg-about-x in the quaternion). Composed eye
    orientation in IMU = conj(q_imu_mod) * conj(q_mod_eye) * flip_x
    (folded into the mapping, see above), eye position = p_imu_mod +
    rot(conj(q_imu_mod), p_mod_eye). On this unit: modules 5.8/6.9 deg
    outward, eyes near-forward with per-unit pitch/roll, eye separation
    61.5 mm, right eye 3.2 mm higher.
  - `verified_fov` (eye frame, per-side magnitudes) = the render frustum;
    `optical_fov` is slightly larger, `green_fov` equals verified.
  - `device_ipd_in_mm: 62` - the IPD the unit was calibrated at.
  - `camera_extrinsics` with `frame_id: IMU_0` directly locates every
    camera in the same frame as the displays. In particular,
    `rgb-left_curved`/`rgb-right_curved` close the VST camera -> IMU ->
    display-eye chain without interpreting qvr's `Ombc` or anorak axis
    strings.
  - `cameras` entries for `rgb-left_curved`/`rgb-right_curved` are the
    production VST camera models: per-unit 3000x3000 KB4 intrinsics,
    distortion offsets and valid radii, rolling-shutter/timestamp
    metadata, plus the spherical cover-window geometry.
  - This is already Monado's active display-calibration source. After an
    explicit path override, `galaxyxr_hmd.c:load_profile()` tries the efs
    path (and its `/.oldroot` mount) before the build-baked `/product`
    fallback. Passthrough should extend/reuse that loader rather than
    introduce another authoritative profile.
- `/product/etc/device_profile.textproto` - a build-baked **devkit
  default** (v1 `display_profile`, header "DK2 ... calibrated 02/12/2023",
  another unit entirely!), fallback only. v1 is a 77x71 grid over head
  frame directions (tan = id * cell_step / surface_distance, cant baked
  into the directions), cells give the panel position for green plus
  red/blue offsets, `green_fov` the valid crop. `device_profile_dk4` and
  `/vendor/etc/calibration_service/device_profile.textproto` are DK4
  defaults. Using the DK2 file on this unit produced a ~2-3 deg per-eye
  tilt between the eyes' images (the 'V'). This is what the `profile*`
  distortion modes consume (v2 preferred, v1 fallback).
- `/data/vendor/qvr/svrapi_lens_{left,right}.csv` - per-device lens
  distortion + CAC meshes of the qvr stack (unused by this driver, format
  documented in the git history). World-readable defaults:
  `/vendor/etc/qvr/svrapi_lens_{left,right}_default.csv` (identical on this
  unit).
- `/data/vendor/qvr/xr_device_config.json` - fov 89.5, frustum positions
  (+-0.032), sensorHeadOffset [0.03527, 0.02413, 0.05120]. Mostly STALE:
  resolution/refresh describe an older 75 Hz 5088x2544 VST device.
- `/mnt/vendor/persist/display/factory_calib_data_XR2_SONY_ECX344A{,_SUB}.json`
  - per-unit panel color calibration (qdcm); defaults plus other panel
  vendors (BOE VX135KDP, S6E8JN0_AMM130GZ01) under `/vendor/etc/display/`.

Cameras and 6DoF (all the raw material for future inside-out tracking):

- `/data/vendor/qvr/device_calibration.xml` - separate per-unit QVR
  calibration family:
  trackingA/B and ctrl-trackingA/B (640x640 fisheye, FISHEYE_4_PARAMETERS
  intrinsics + rig extrinsics), rgb-left/right (3000x3000, rolling shutter),
  depth (320x240), IMU-camera time alignment, vignetting, and an SFConfig
  block with per-unit IMU biases and noise densities. The generic
  rgb-left/right entries are enough for initial bring-up, but the
  `rgb-*_curved` entries in the efs profile are the VST-specific models
  used by the production stack.
- `/data/vendor/qvr/anorak.txt`, `anorak_camera.txt` - per-device QVR
  service + camera configs; static variants and per-hwid revisions
  (`anorak_camera_hwid_r{0..25}.txt`) under `/vendor/etc/qvr/`. Notable:
  `sensor_orientation = 2 -1 3`, identity `hw_transform = imu eyeL/eyeR`.
- `/vendor/etc/qvr/cfg/{339,356,415,457,549,649}/<n>/<id>/` - per hardware
  revision tracker configs: `anorak_qvr_3dof_tracker_config.txt` (Qualcomm
  AHRS fusion tuning: gains, mag weighting, gyro bias handling) and
  `anorak_6dof_config.xml`.

Sensors (SSC / ADSP, reached over QRTR - see the IMU section):

- QMI service 400 (`sns_client`); accel/gyro x5 instances (LSM6DSV, use
  hw_id 0), `trimag` x4 (AKM ak0997x, raw), `ipd` (AKL algo, event 717,
  14 floats starting ipd_L/ipd_R - the motorized IPD position), and more.
- `/mnt/vendor/persist/sensors/registry/` - the SSC persistent registry
  (per-unit IMU/mag calibration lives here), `sensors_list.txt` next to it.
- `/vendor/etc/sxr/et_camera_config.ini`, `irled_control_config.json` -
  eye tracking camera + IR LED configs.

Hardware nodes (Linux side, gathered along the way):

- `/dev/dri/by-path/platform-ae00000.qcom_mdss_mdp-card` - left eye DRM
  device (also has DP-1 and Virtual-1); `platform-15600000...` - right eye.
- `/sys/class/kgsl/kgsl-3d0/` - Adreno 740v3; only governor is
  msm-adreno-tz, `min_pwrlevel` 0 pins 788 MHz (see gxr_perf.sh);
  `gpu_busy_percentage`, `devfreq/cur_freq` for monitoring.
- `/sys/kernel/debug/dri/{0,1}/debug/core_perf/perf_mode` - display bus
  performance mode; `/sys/kernel/debug/dri/N/planeM/xin_id` - SSPP pipe ids.
- `/sys/class/backlight/panel{0,1}-backlight/brightness` - panel backlight
  (root, panels keep kwin's last setting).
- `/dev/dma_heap/qcom,display` (root only), `/dev/dma_heap/system` (render
  group) - buffer allocation fallbacks.


## Environment variables

| variable | default | meaning |
|---|---|---|
| `GALAXYXR_ENABLE` | true | driver autodetection gate |
| `GALAXYXR_LOG` | info | driver log level |
| `GALAXYXR_IMU_RATE` | 200 | IMU sample rate in Hz |
| `GALAXYXR_IMU_AXES` | `x,y,z` | device axes as signed IMU axes, e.g. `-z,y,+x` |
| `GALAXYXR_IMU_CAL` | true | factory IMU intrinsics + SSC online gyro bias correction |
| `GALAXYXR_DISTORTION` | profile-mono | `profile` (with CAC), `profile-mono`, `none` |
| `GALAXYXR_PROFILE_PATH` | unset | override the device_profile.textproto location |
| `GALAXYXR_POWER_DEVICE` | `/dev/input/event2` | `pmic_pwrkey` evdev device used for the OpenXR system click |
| `XRT_COMPOSITOR_PRESENCE_OFF_DELAY_MS` | 500 | keep displaying this long after the user goes absent |
| `XRT_COMPOSITOR_GALAXYXR` | true | backend autodetection gate |
| `XRT_COMPOSITOR_FORCE_GALAXYXR` | false | force-select the backend |
| `XRT_COMPOSITOR_GALAXYXR_SYNC_FD` | true | IN_FENCE_FD explicit sync |
| `XRT_COMPOSITOR_GALAXYXR_UBWC` | true | UBWC scanout images |
| `XRT_COMPOSITOR_GALAXYXR_DMA_HEAP` | unset | force a dma_heap for buffers |
| `XRT_COMPOSITOR_GALAXYXR_HZ` | 90 | panel mode: 90, 72 or 60 (see Rendering) |
| `XRT_COMPOSITOR_GALAXYXR_LATCH_MARGIN_MS` | 2 | how long before the latch vsync the render fence must signal |
| `XRT_COMPOSITOR_GALAXYXR_EMISSION_LAG_MS` | 0 | panel write-to-emission lag, added to the display time |
| `XRT_COMPOSITOR_GALAXYXR_COMP_TIME_MS` | 8 | initial wake lead of the pacer |
| `XRT_COMPOSITOR_COMPUTE` | false here | compute renderer instead of the mesh one |


## Rendering

How a frame gets from the client to photons. The backend is effectively a
tiny purpose-built WSI: nothing standard (Vulkan display WSI, swapchains)
can express this display, so buffer rotation, fencing, pacing and flip
handling all live here.

### Frame pipeline

Three scanout buffers, at most one commit in the kernel:

```
buffer A: on glass      - being scanned out by the panels right now
buffer B: in the kernel - committed, fence-gated, latches at the next vsync
buffer C: on the GPU    - being rendered, committed next
```

The loop is paced by the pacer's lattice-timed wake, not by blocking:
the previous commit completed at the last vsync, so present() enqueues
both eyes' commits while the kernel is idle and returns in ~0.5 ms. The
per-CRTC kernel worker picks them up immediately, waits for the render
fence in the kthread, programs the planes (~0.3 ms), writes CTL_FLUSH,
and the frame latches at the next vsync. Acquire/present only *harvest*
already-delivered flip events (zero-timeout poll) for the pacer's
vblank reference and the latch matcher.

Per frame, steady state (L = the left-panel vsync this frame latches at):

```
L-9.1   wake: predict, compose (~0.5 ms CPU), submit render
L-8.6   present(): both commits enqueued, kernel worker fence-waits
L-4.0   GPU fence signals; worker programs planes, CTL flush ~L-3.5
L-1.0   flush window closes (kickoffs forbidden in the last 1 ms)
L+0.0   vsync: frame latches; measured latch err vs prediction ~1 us
L+0.13  first active line reaches the panels (47 lines after the vsync)
```

A frame whose fence misses the flush window latches one vsync late, and
the kernel holds a commit from its ioctl until one vsync past its latch:
after a single slip every following commit queues in the ioctl behind
the previous one and the pipeline stays a period deep forever, showing
every frame one period later than predicted (this was the previous
steady state of this backend, costing 11.1 ms of extra latency).
present() therefore watches the commit ioctls: one that both blocked and
returned too late to program and flush before its own vsync means the
queue went deep, and the next commit is dropped to drain it back to
one-deep (one skipped content frame). A block alone is not enough: load
spikes can delay the previous commit's cleanup kthread and stall the
ioctl for a few ms while the frame still latches on time.

The enqueue is also gated to no earlier than one period before the
target vsync: KMS has no notion of a target present time and the render
fence is the only thing holding a commit back, so with a wake lead
beyond one period a quickly signaled fence (a heavy scene clearing)
could otherwise latch a frame a full period early. In practice the
kernel's commit serialization and its stale-vsync wait after idle mask
this in every state we could produce, but neither behaviour is
contractual, so the gate makes the never-early invariant explicit.

Why not "wait for the flip event, then render" (the classic KMS
compositor shape, and what this backend originally did): flip events are
*delivered* 2 ms to nearly a full period after the vsync they timestamp
(the downstream kernel sends them from the commit kthread), so an
event-driven loop inherits that jitter. The flip *timestamps* are exact
hardware stamps, so the pacer instead extrapolates the vsync lattice
from the newest one with the exact mode period.

### The kernel side

Per commit, on a per-CRTC kthread (downstream msm_atomic + sde_encoder):
wait the IN_FENCE (SW) -> program the plane registers (~0.3 ms) -> check
the flush window (the last 1 ms before the vsync is forbidden) -> write
CTL_FLUSH -> the hardware latches it at the next MDP vsync -> vsync IRQ
-> complete the commit, send the flip event and release the pending
slot. The pending slot is held from the *ioctl* until one vsync past the
latch, which is why a commit enqueued behind a pending one adds a full
period of latency.

Flip event timestamps are real hardware stamps: the SDE latches a
19.2 MHz QTimer snapshot at every MDP vsync (`has_precise_vsync_ts`) and
the events carry it as long as the DRM vblank machinery is kept alive -
the `drmCrtcGetSequence` call in the flip handler does exactly that
(without it this kernel reports zero timestamps). Delivery lags the
vsync by 2 ms to nearly a period, so the timestamps are trusted and the
arrival times are not. The MDP vsync leads the panel vsync by VFP-1 = 28
lines of programmable fetch (the whole VFP is consumed as prefill), and
VSW+VBP = 19 more lines pass before the first active line: scanout
begins 47 lines = 134 us (at 90 Hz) after the timestamped vsync.

The two DPUs are ganged (`ctl_op_sync`, auto-enabled by
`qcom,dsi-select-sec-sync-clocks` on the panel DT nodes): the right
display's DSI clocks are slaved to the left's PLL and the master DPU
enables the slave's timing engine. The panels scan out in hardware
lockstep (the R-L flip delta reads 0.00 ms at 90/72 Hz), and a commit
stream to only one device never completes - both eyes must be committed,
back to back, from one thread.

The image does not appear globally at vsync. Both panels scan top-to-bottom
with 3840 active lines and 3888 total lines at every supported refresh rate.
The HMD's `get_compositor_info` callback therefore reports an active scanout
duration of `frame_interval * 3840 / 3888` (10.974 ms at 90 Hz), allowing the
graphics renderer to interpolate timewarp from the beginning to the end of
panel scanout. These timings were read from the active DRM modes on both
CRTCs; the EFS panel JSON files contain color calibration, not mode timing.

### SDE kernel contract

The loop shape leans on specific downstream-kernel behaviour, verified
against the Samsung opensource drop (`vendor/qcom/opensource/
display-drivers/msm/` and `kernel_platform/msm-kernel/drivers/gpu/drm/`;
the running kernel is assumed close) and against live traces. A kernel
or vendor update can invalidate any of it, so each item lists what in
this backend depends on it and how a change would show up.

- **Commit lifetime** (`msm_atomic.c`): the atomic ioctl blocks at entry
  on `pending_crtcs`/`pending_planes` - nonblocking commits too - and
  the bits are held until `commit_destroy()`, the last step of
  `complete_commit()` on the per-CRTC kthread: fence wait -> plane
  programming -> CTL_FLUSH -> wait for the latch vsync -> flip event ->
  cleanup. A commit therefore owns its CRTC from the ioctl until
  ~0.65 ms past its latch vsync, and an ioctl issued while the previous
  commit is pending returns only then. This is the mechanism behind both
  the old always-one-period-deep loop and the drain detector's "blocked
  and returned past the latch deadline" criterion. Shows up as: "commit
  wait" rising in the stats without matching late/dropped counts.

- **Flip events are stashed, not armed** (`sde_crtc.c` +
  `drm_vblank.c`): the event is sent from the commit kthread once the
  latch vsync IRQ fires (`sde_kms_wait_for_commit_done` ->
  `sde_crtc_complete_flip`), with the timestamp copied from the DRM
  vblank cache. The driver never holds a vblank reference and
  `drm_crtc_vblank_on()` resets the cached time to zero, so without help
  every event carries tv_sec/tv_usec = 0. The `drmCrtcGetSequence` call
  in the flip handler takes a momentary vblank reference, which
  refreshes the cache from the hardware timestamp
  (`vblank_disable_immediate` drops it right after - the per-frame
  vblank enable/disable pairs in the SDE evtlog). Shows up as: zero or
  noisy flip timestamps (the handler falls back to sequence queries and
  delivery-time stamps). Fallback if it breaks for good: the connector
  retire fence is signalled with the same hardware vsync timestamp
  (`sde_fence_signal` from the frame-event worker), once per frame, with
  no event machinery involved.

- **Hardware vsync timestamps** (`sde_encoder.c`, `sde_hw_intf.c`): this
  SDE (v9.3.0, `has_precise_vsync_ts`) latches a 56-bit 19.2 MHz QTimer
  snapshot at every MDP vsync (`INTF_MDP_VSYNC_TIMESTAMP0/1`), converted
  to CLOCK_MONOTONIC in `sde_encoder_calc_last_vsync_timestamp()`.
  Measured lattice jitter: 0.3 us RMS. Event *delivery* lags the vsync
  by 2 ms to nearly a period - the timestamps are trusted, the arrival
  times are not, and nothing in the loop may wait on an event arriving.

- **MDP vsync != panel vsync** (`sde_encoder_phys_vid.c`): programmable
  fetch is enabled because the needed prefill (60 lines scaled by
  vrefresh/60) exceeds VSW+VBP = 19 lines; it consumes the entire VFP
  and starts fetching at line vtotal-VFP+1, so the MDP vsync - the IRQ,
  the timestamp and the latch point - leads the panel vsync by
  VFP-1 = 28 lines (this board trips the kernel's `low vbp+vfp`
  warn_once). First active line = latch + (VFP-1) + (VSW+VBP) =
  47 lines. The pacer derives its scanout-begin offset from exactly
  these mode fields at runtime, so DT porch changes adapt automatically;
  a change to the prefetch *policy* (e.g. a
  `qcom,sde-intf-max-prefetch-lines` DT entry appearing, or
  `delay_prg_fetch_start` behaviour) silently shifts photon timing by up
  to the VFP and needs the formula revisited.

- **Flush window** (`sde_encoder.c`): CTL_FLUSH is forbidden in the last
  1 ms (`VSYNC_THRESHOLD_WINDOW_NS`) before the next MDP vsync,
  recomputed at every vsync IRQ from the hardware timestamp and enforced
  with sleep-and-retry in the kickoff path; it is active here because
  ctl_op_sync and HW fences are enabled on this dual-DPU config. This
  sets `XRT_COMPOSITOR_GALAXYXR_LATCH_MARGIN_MS` (1 ms window plus
  ~0.3-0.5 ms plane programming plus slack) and the 1.5 ms pickup
  deadline in the drain detector.

- **dfps is HFP stretch** (`dsi_panel.c`,
  `dfps_immediate_porch_mode_hfp`): vertical timing is identical at
  90/72/60 - only the horizontal front porch, and with it the line time,
  changes - so the 47-line offset holds at every rate scaled by that
  rate's line time, and the per-INTF pixel clock stays constant.

If pacing ever looks wrong after an update, re-run the method that
produced these numbers: `XRT_COMPOSITOR_LOG=debug XRT_LOG=debug`,
reconstruct the per-frame predict/present/flip order from the traces,
and compare the desired-present lattice against the flip-timestamp
lattice; `sudo cat /sys/kernel/debug/dri/0/debug/dump` is the
kernel-side kickoff/vsync event log (it returns only entries since the
last read). Healthy steady state reads: commit wait ~0.5 ms, latch err
~0.001 ms, fence->latch ~4 ms, zero late, zero dropped.

### Pacing

A dedicated pacer (`comp_window_galaxyxr_pacing.c`) owns the model,
since this platform provides none of the standard feedback sources:

- **vblank lattice**: the newest hardware flip timestamp anchors the
  lattice, extrapolated with the exact mode period (htotal*vtotal/clock,
  not the integer-rounded vrefresh). The desired present time is the
  vsync the commit latches at - verified per frame by the latch matcher,
  steady state error ~1 us.
- **gpu_end**: Turnip on kgsl has no VK_EXT_calibrated_timestamps, so the
  render fence's sync_file signal time (SYNC_IOC_FILE_INFO) is reported
  on a later present. The adaptive wake lead targets the fence signaling
  `XRT_COMPOSITOR_GALAXYXR_LATCH_MARGIN_MS` (default 2) plus ~1.5 ms of
  slack before the latch vsync.
- **wake lead past one period**: under GPU overload (heavy clients) the
  honest pipeline is longer than a frame; the wake lead may grow to
  2.5 periods, which lands predictions on a correspondingly later,
  reachable vsync. Measured with xrgears: 45 fps with latch error ~0
  instead of promising vsyncs a full period too early.
- **present-to-display offset**: derived from the mode's vertical timing
  as the start of panel scanout (latch vsync + 47 lines = 0.134 ms at
  90 Hz; the renderer treats the display time as the *begin* of the
  timewarp window, end +10.97 ms). On top sits the panel's
  write-to-emission lag, unmeasured so far (needs a photodiode or a
  high-fps camera through the lens), assumed zero and tunable via
  `XRT_COMPOSITOR_GALAXYXR_EMISSION_LAG_MS` or the debug GUI.

### Renderers and rates

- The graphics/mesh renderer is the backend default: ~4-6 ms of GPU for
  the 2x 3552x3840 distortion (per-vertex calibration LUT, nothing shaded
  outside the mesh circle, ROP UBWC writes). The compute renderer costs
  ~12 ms (per-pixel LUT, 3 colors sampled even in mono) - over the 90 Hz
  budget by itself; still selectable via `XRT_COMPOSITOR_COMPUTE=1`, and
  could get a CAC on/off fast path some day.
- `XRT_COMPOSITOR_GALAXYXR_HZ` = 90 (default) / 72 / 60 picks the panel
  mode; the driver's advertised nominal follows the same env. 90 and 72
  run locked and panel-synced. 60 is broken at the display-stack level
  (51-57 fps, the panels drift 4.7-7.5 ms apart, jittery flip periods) -
  video-mode dfps here is HFP stretch (`dfps_immediate_porch_mode_hfp`,
  only the horizontal front porch differs between the rates: 28/190/352
  pclk per INTF at 90/72/60), suspected to interact with the ctl_op_sync
  ganging; investigate with the evtlog if it ever matters.

### Diagnostics

- Stats every 256 presents: ms/frame, fence wait, commit-block time
  ("commit wait", healthy ~0.5 ms of plain ioctl overhead - more means
  commits queued behind a pending one), pacer lead, latch err (flip
  timestamp minus predicted vsync, matched per frame in commit order -
  the kernel sends one event per commit, in order, and the commit gate
  makes genuinely early latches impossible, so older-than-expected
  events are discarded as spurious; healthy ~0.001 ms), fence->latch
  margin, R-L flip delta, flip period avg/min/max. A WARN reports frames
  that latched a period late and commits dropped to drain - that is the
  authoritative missed-frame signal.
- `XRT_COMPOSITOR_LOG=debug`: per-frame predict/present traces; the flip
  traces additionally need `XRT_LOG=debug`;
  `XRT_COMPOSITOR_GALAXYXR_PACING_LOG=debug` logs wake-lead adaptation.
- Kernel-side ground truth: `sudo cat /sys/kernel/debug/dri/0/debug/dump`
  is the SDE event log (global, covers both devices); one
  `sde_encoder_phys_in_skewed_flush_window` entry per frame per display
  shows the kickoff-vs-flush-window timing directly.


## Passthrough

Video see-through from the two VST cameras, exposed as
`XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND` (advertised after OPAQUE when the
camera provider initializes). The Galaxy XR driver owns the Titan connection
and exposes raw frames plus calibration through the generic
`xrt_passthrough_stream` interface. The stream is demand-driven: the
compositor enables it only while passthrough will actually be rendered
(alpha-blend frames on the gfx N-layer path), and disabling closes the Titan
stream and releases every camera buffer until it is needed again.
`comp_passthrough.c` imports those frames and supplies them to a passthrough
specialization of the gfx N-layer renderer. The camera is the opaque bottom-most accumulator value; submitted
application layers composite over it in the same final draw.

There is no rectified intermediate image, compositor-owned camera swapchain,
or separate undistortion dispatch. This path is compile-gated by
`XRT_FEATURE_TITAN_PASSTHROUGH` (default ON with wayland-direct) and currently
requires the distortion-mesh gfx N-layer fast path (`XRT_GFX_NLAYER=true`,
`XRT_COMPOSITOR_COMPUTE=false`). Unsupported layer combinations or another
renderer do not currently get a camera fallback.

Display and camera distortion are independent debug-GUI toggles:
`Distortion OFF` disables the display lens correction, `Camera distortion OFF`
disables the camera model (capture reprojection, curved window, forward KB4)
for a raw head-locked camera view. Both keep the camera visible. With only
display distortion off the pipeline still rasterizes the distortion mesh with
identity source UVs so the per-vertex camera projection keeps running; with
camera distortion off the source UV passes straight through to the raw NV12
frame and none of the camera-model code exists in that shader variant; with
both off the mesh is dropped for the full-screen triangle. The same mesh path
serves a driver with no display distortion (`GALAXYXR_DISTORTION=none`): with
the passthrough feature compiled in, the identity none-mesh is generated at
full `XRT_MESH_SIZE` density instead of a single cell so per-vertex camera
projection keeps its tessellation. `ATW OFF` only
stops application-layer reprojection, the camera reprojection is independent
of it. `Disable fast path` (and the debug-GUI mirror or window peek, which
clear the fast path) still drop the camera background.

### Camera stream

titan-server owns the ISP; the Galaxy XR provider is a client on its
SOCK_SEQPACKET socket (`/run/titan-server.sock`, with the session socket as a
fallback, protocol v12; see `src/external/titan/titan_proto.h`). It opens one
virtual-stereo stream (`cam_slot -2`): the imx564 pair at 3000x3000@93,
choosing a mode named `main`/`single` and excluding `sub`. OPEN requests
`TITAN_WIRE_FMT_NV12` explicitly, since the zero default selects native RAW,
and STREAM_INFO must confirm it. Output is linear
NV12, one dma-buf per ring slot and view (Y at 0, CbCr at `chroma_offset`,
common stride), with ring descriptors delivered once by SCM_RIGHTS.

Frames arrive paired and all-or-nothing. The provider keeps only the newest
pending pair, releases superseded pairs immediately, and transfers a held
frame to the compositor with duplicated buffer handles. Protocol v12 RELEASE
messages identify every view by `{slot, frame_id}`; delivered frames remain
held until release or disconnect. Frames carry mandatory per-view sensor and
exposure request identities, SOF, first-row exposure start, exposure duration,
and ready metadata. A held compositor frame is released only after its GPU
fence completes; disconnected-generation holds are invalidated without
sending RELEASE. The client thread reconnects in the background, and a new
STREAM_INFO generation invalidates all old Vulkan imports.

Immediately before recording a draw that samples the selected pair, the
compositor reports one use tick, including ticks that repeat the same camera
pair. The Galaxy driver samples Titan's `CLOCK_BOOTTIME` domain at that
boundary and queues a v12 FRAME_USED message on its Titan client thread. The
message echoes the delivery's opaque `delivery_id`; it is independent of
RELEASE and changes no buffer ownership. Only a controller sends feedback.
The queue keeps the newest tick if the socket thread falls behind while still
advancing `use_sequence`, so local drops do not distort the receiver cadence
reported to titan-server. Titan uses this clock to adapt camera rate and phase
so a fresh pair arrives shortly before the compositor selects it.

### GPU path

Each ring dma-buf is imported once: 2-plane
`VK_FORMAT_G8_B8R8_2PLANE_420_UNORM`, explicit `DRM_FORMAT_MOD_LINEAR`
plane layouts, dedicated allocation, `VK_QUEUE_FAMILY_FOREIGN_EXT`
acquire/release barriers around the final draw. One persistent descriptor set
selects the current pair after the preceding compositor fence has completed.
The fragment shader samples the appropriate view through distinct left/right
bindings with immutable BT.601 full-range YCbCr samplers, midpoint siting and
linear chroma. Separate bindings avoid dynamically indexing YCbCr samplers,
which Vulkan does not permit. The shader explicitly decodes sRGB, then
composites application layers in linear space.

Specialization constant id 11 selects dedicated passthrough vertex/fragment
shader modules and a pipeline layout with the camera descriptor set; id 12
folds the camera model in or out of the passthrough vertex stage. Normal
gfx N-layer pipelines retain their original descriptor interface and compile
without any camera code or resources.

### Geometry

The driver reads the complete per-unit
`/mnt/vendor/efs/device_profile.textproto`
`rgb-left_curved`/`rgb-right_curved` family. It parses the direct
`IMU_0 -> camera` extrinsics, display-module eye transforms, KB4 intrinsics,
distortion offsets, valid radii, curved-window shell, and timing as one atomic
model. The curved-window alpha LUT is regenerated from the physical shell by
Snell ray tracing. Passthrough initialization fails rather than mixing in
unrelated XML geometry when that family is incomplete.

The passthrough vertex shader starts with each display distortion-mesh
vertex's green predistort coordinate (the identity screen coordinate under
the `Distortion OFF` debug toggle), constructs the eye ray, applies the
ordinary projection-layer capture-to-display timewarp rotation and the full
live eye-to-camera translation on a 2 m fallback plane, rotates through the
direct IMU-to-camera matrix, applies the curved-window LUT and forward KB4
projection, and emits a signed-validity camera UV. The fragment shader
therefore needs one raw NV12 sample and no iterative inverse or intermediate
resample. The EFS quaternions' near-180 degree rotation about x is the expected
OpenXR/head (y up, forward -z) to camera-CV (y down, forward +z) mapping.

#### RGB calibration source policy

For RGB passthrough, the efs `rgb-left_curved`/`rgb-right_curved` entries
are authoritative. Treat each calibration family as an atomic camera
model; do not combine efs extrinsics with XML intrinsics, XML extrinsics
with efs intrinsics, or otherwise mix their distortion and timing fields.
The implementation accepts only the complete EFS curved-camera set. The XML
RGB set does not include the production curved-window model and is not a
passthrough fallback.

The two sources do not merely express the same baseline in different
axes. Their left-to-right translation vectors and lengths are:

| source | baseline vector in that source's frame (mm) | length (mm) |
|---|---|---|
| efs `rgb-*_curved` | (+61.538, +1.303, +0.192) | 61.553 |
| XML `rgb-*` | (-63.289, +1.964, -2.026) | 63.352 |

The component signs cannot be compared directly because the coordinate
frames differ, but rotation cannot change vector length: the stereo
baselines genuinely disagree by 1.799 mm. This likely reflects separate
calibration models/runs and possibly different effective optical centres
through the cover. Do not average the values. A stereo rectification or
depth model built from one family must use that same family's projection
and extrinsics throughout.

This policy applies to RGB VST geometry. It does not make the XML globally
obsolete: its tracking/depth-camera data, IMU parameters, and other QVR
resources may still be useful for their respective pipelines.

Composing the profile's IMU -> display-module and module -> eye transforms
gives these positions for this unit (millimetres, IMU axes):

| eye | camera position | eye position | camera minus eye |
|---|---|---|---|
| left | (+29.773, -26.780, -1.450) | (+28.954, -28.776, +51.533) | (+0.820, +1.996, -52.983) |
| right | (+91.312, -25.477, -1.258) | (+90.441, -25.548, +51.694) | (+0.871, +0.071, -52.952) |

Thus both entrance pupils are about 53 mm in front of their eyes, not the
temporary 40 mm default, and the small x/y offsets must not be discarded.
Use the direct per-eye q and full three-component translation. Do not
hard-code the numbers above: the efs profile is per-unit. The live IPD
actuator moves the display eyes while the cameras remain fixed, so the
camera-minus-eye vector is recomputed from each current eye pose around the
calibrated EFS eye midpoint.

The efs `rgb-*_curved` intrinsics also differ materially from the generic
XML entries: focal lengths are around 1456 px, and each eye has its own
principal point, KB4 coefficients, nonzero `distortion_offset`, and valid
radii. The layered model describes the cover as a spherical window
(134.61066 mm radius, 1.03522 mm thickness, refractive index 1.5167) with
a per-eye sphere center. The shipped Android compositor has
`KB4_CURVED_WINDOW`/`CURVED_WINDOW` shader paths. Its mapping first
corrects a normalized ray with a one-dimensional alpha LUT indexed by the
dot product with the camera-to-sphere axis, then subtracts
`distortion_offset` and applies KB4. The host calibration plugins contain
`curved_window_model` and `curved_window_distortion_lut` implementations;
the LUT can be regenerated from the physical shell with Snell ray tracing.
Plain KB4 leaves cover-refraction error which can look like a translation
error, especially near the image edge.

#### Translation is depth-dependent

For an output eye ray `r_E`, predicted display time `t_d`, and the capture
time `t_c` of the source row, the general backward mapping is:

```
P_W = T_W_E(t_d) * (lambda * r_E)
P_C = inverse(T_W_C(t_c)) * P_W
source_uv = project_curved_KB4(P_C)
```

`lambda` comes from the visible surface depth or a scene mesh. With a
fronto-parallel plane at axial distance D and the shader's unnormalised
`r_E.z = -1`, `lambda = D`; the current transform of
`eye_position + focus * r_E` into the capture frame, followed by subtracting
`camera_position`, is this special case. It uses the calibrated rotation and
full live camera-eye offset, but it can only align content on that one plane.
At infinity translation contributes nothing and the correct warp is
rotation-only. No global image shift or single homography can align near and
far geometry simultaneously.

The production compositor confirms this division: its late-stage
reprojection code contains identity, rotational, planar, and backward
depth modes. The normal no-depth fallback is a fronto-parallel 2.0 m
plane. Keep an explicit focus-plane control as an honest fallback; unlike
pitch and camera position, its value cannot be recovered from factory
calibration.

| variable | default | meaning |
|---|---|---|
| `XRT_PASSTHROUGH` | true | master switch |
| `XRT_PASSTHROUGH_FOCUS_M` | 2.0 | fallback plane where translation aligns; 0 = rotation-only |
| `XRT_PASSTHROUGH_CALIBRATION` | unset | override the EFS device-profile path |
| `XRT_PASSTHROUGH_LOG` | info | Galaxy provider and compositor bridge log level |
| `XRT_GALAXY_PASSTHROUGH_MAGICAL_TIMESTAMP_OFFSET` | true | experimentally apply the profile's +4.3 ms camera timestamp alignment |
| `TITAN_SERVER_SOCKET` | auto | override the titan-server socket path |

### Capture time and late reprojection

Protocol v12 carries each view's frame, sensor-request, and exposure-request
identity; hardware SOF and first-row exposure start in both QTimer and
`CLOCK_BOOTTIME`; applied exposure duration; and buffer-ready time. Every
field is mandatory and the client rejects internally inconsistent frames.
The Galaxy provider treats QTimer as the authoritative exposure clock shared
by camera CSID and the SSC IMU. Immediately after a complete stereo frame is
received and validated, the provider resolves its first- and last-row capture
poses directly from the QTimer-indexed IMU relation history. It freezes those
poses before publishing the frame to the compositor. Only the resulting
device poses cross the generic `xrt_passthrough_stream` interface; QTimer
remains private to the Galaxy driver and Titan protocol. The timestamps
reconstructed by Titan in `CLOCK_BOOTTIME` remain informational and are not
used for reprojection because that reconstruction has no clock-skew
compensation.

A windowed minimum-skew clock tracker correlates local `CLOCK_MONOTONIC`
observations with the remote SSC hardware QTimer while filtering SSC delivery
delay and jitter. The HMD retains one fused relation history directly in
QTimer. The clock tracker remains necessary for ordinary Monado head-pose
requests, including predicted display poses, whose timestamp contract is
system monotonic. Camera capture poses do not use that conversion because
their timestamps and the IMU history already share QTimer. If the camera
timestamps cannot be resolved in the IMU history, the provider logs a warning
and supplies the latest available pose for both capture endpoints, keeping the
image visible while collapsing rolling-camera compensation for that frame.

The `rgb-*_curved` profile entries specify the missing camera timing
semantics:

- exposure timestamp means beginning of exposure;
- camera timestamp alignment is 4.3 ms (apply with the profile/service
  convention; do not guess its sign);
- rolling-shutter readout is 9.676306 ms in positive image-y;
- the left obtains rolling-shutter data from image metadata, while the
  right uses the device-profile value.

Titan reports first-row exposure start in QTimer. The provider optionally
applies the experimental profile timestamp alignment, adds half of the
applied exposure duration to use the row's exposure midpoint, then uses the
calibrated rolling-shutter readout time for the last-row midpoint. Both
endpoints are looked up directly in the QTimer relation history before the
frame becomes available to the compositor:

```
t_first = t_first_row_exposure_start + optional_alignment + exposure_duration / 2
t_row = t_first + y * 9.676306 ms
```

The generic renderer reuses the rotational part of normal projection-layer
timewarp, `R_capture^-1 * R_display`. It evaluates the four combinations of
camera readout begin/end and panel scanout begin/end. The vertex shader first
projects with the mid-readout transform, uses the resulting camera `v`
coordinate as the row time, and projects a second time with the bilinearly
interpolated transform. Providers always supply capture poses; an exceptional
provider-side fallback uses the same latest pose for both capture endpoints.

The 4.3 ms profile alignment is enabled by default because `+4.3 ms` looked
preferable in a visual A/B test. It remains deliberately labelled "magical":
the profile/service sign convention has not been objectively verified for
Titan's reconstructed exposure timestamp. Per-frame rolling-readout metadata
would still be preferable for the left camera when Titan exposes it.

The current tracker is 3DoF. It can correct capture-to-display rotation
and the rigid camera/eye offset, but cannot know real head translation
between those times. Full temporal translation also requires 6DoF head
poses.

### Depth and final warp

The Android stack's `/vendor/etc/depth_config.json` selects
`input_camera_mode: kStereoRGB` and enables the curved-window depth model.
`/vendor/etc/opx/depth/stereo_ml_depth_qnn_model_board_config.json`
identifies its model as `XR_VST_DEPTH`. The production compositor also
contains depth-mesh and backward-reprojection paths. This is strong
evidence that stereo RGB depth, not a scalar camera offset, is the intended
solution. The current Titan dToF work does not yet provide usable readout,
so it is not a near-term dependency.

The complete compositor path should sample raw NV12 plus a corresponding
depth map/scene mesh in the final pass and map predicted-display rays
backward into the capture cameras. This combines per-unit camera/eye
extrinsics, predicted and capture poses, per-row time, depth, curved-window
refraction and KB4 in one mapping, avoids an extra rectilinear resample,
and leaves only genuinely disoccluded pixels for confidence-aware filling.

#### Fused graphics path without a rectified intermediate

The implemented gfx N-layer specialization issues one indexed display-mesh
draw per view. The existing application layer descriptor set remains set 0;
set 1 contains two immutable-sampler NV12 images and the two curved-window
LUTs. There is one physical output mesh and one `gl_Position`, but the vertex
shader emits independent application and camera coordinates. Starting from
the green display predistort coordinate:

```
panel_position = display_mesh.position
eye_ray = uv_to_tanangle(display_mesh.predistort_green)
camera_uv = project_curved_KB4(imu_to_camera *
    (eye_ray * focus - full_camera_eye_offset))
```

The camera UV and signed validity margin are `noperspective` varyings. The
fragment shader samples raw NV12 once, converts its gamma-encoded RGB to
linear, initializes the accumulator with that opaque background, and then
runs the unchanged N-layer application loop:

```
accum = srgb_to_linear(texture(camera, camera_uv))
for each submitted layer:
    accum = accum * (1 - layer.a) + layer.rgb
```

Thus camera “undistortion” is backward texture mapping: the shader asks where
the desired display ray occurs in the distorted source and applies the
*forward* curved-window plus KB4 camera projection. It neither constructs a
rectified image nor iteratively inverts KB4, and the camera does not consume
a normal N-layer slot. Dedicated shader modules declare the extra set only
for specialization id 11; the ordinary modules retain the old pipeline
layout.

Curved-window and KB4 mapping are nonlinear, so evaluating them at display
mesh vertices approximates their interior values by affine interpolation.
The remaining error is controlled by display-mesh tessellation and needs an
eyes-on/full-field comparison against TitanXR’s denser reference mesh.
Per-fragment projection or a camera-UV lookup field remain fallbacks if the
display mesh cannot meet the error bound.

The EFS model, full live translation, fused sampling, frame identity,
timestamped IMU history, rotational late reprojection, and row-dependent
rolling-shutter correction are implemented. Remaining work is to:

1. expose per-frame readout metadata and confirm the profile
   timestamp-alignment sign;
2. add stereo depth/mesh backward reprojection and disocclusion handling;
3. validate whether display-mesh interpolation is dense enough for the
   time-varying camera projection.

### Passthrough performance

The retired intermediate path dropped into a half-rate orbit (45/36/30 fps
at 90/72/60): although its compute conversion was cheap, injecting the
result as an additional generic N-layer slot pushed the final pass over the
target-rate orbit. The fused path removes both that dispatch/resample and the
extra generic slot. Curved-window and KB4 work runs per mesh vertex; the
per-fragment increment is one YCbCr sample, sRGB decode, and background
blend. With one quad at 90 Hz, the fused path settled at 90.0 fps over
repeated 256-frame windows (11.11 ms period, about 0.03 ms compositor fence
wait, no steady-state slipped-vblank warnings). Re-measure target-rate
behavior and full-field quality after shader or mesh changes.

Test: run titan-server, then the service with `XRT_GFX_NLAYER=true`, then
any alpha-blend client, e.g. `openxr_quads_and_circles --quads` (its
default blend mode preference is alpha).


## Performance

- Prerequisite: `gxr_perf.sh on` pins the GPU at 788 MHz (the kgsl
  msm-adreno-tz governor parks the bursty VR load at 421 MHz). A
  persistent perf vote is still TODO.
- Prerequisite: realtime priority. Monado raises its compositor thread to
  SCHED_FIFO by itself, the user running monado-service just needs the
  rlimit for that to succeed:
  `echo 'neko - rtprio 99' | sudo tee /etc/security/limits.d/25-monado-rlimits.conf`
  and re-login (pam_limits); startup then logs "Raised priority of thread
  'Multi Client Module'". Without it the present loop is a plain CFS task:
  walt sees ~5% load and keeps it on low clocks, and a busy client (kwin)
  preempts it for up to a whole frame right around the commits - measured
  as ~13 ms/s of runqueue delay, the eyes latching several ms out of phase
  (R-L, normally 0.00) and constant slipped vblanks at ~70 fps. Note that
  `setcap cap_sys_nice` on the binary cannot work here: /data is mounted
  nosuid, so file capabilities are ignored.
- Single-projection-layer client (opengl_xr_triangle): locked 90.0 fps,
  11.11 ms/frame flat, zero slipped frames in steady state, stable across
  restarts; 72 Hz likewise at 13.89 ms.
- xrgears (multi-layer): ~36 fps, purely GPU-bound (~27 ms chain on the
  mesh path: layer squash + distortion). Needs the squasher cost work
  and/or the compute nlayer mono fast path.


## Known issues / TODO

- Performance, see above.
- Distortion geometry is eyes-on validated across several OpenXR apps; CAC
  (`GALAXYXR_DISTORTION=profile`, exact per-color rays in v2) still awaits
  an eyes-on verdict and is off by default.
- The display LUT was calibrated at 62 mm (`device_ipd_in_mm`); at very
  different motor positions the distortion may drift slightly (the driver
  logs when the live `ipd` value strays >1 mm from the calibration point).
- UBWC image content correctness is unverified by eye: if the output ever
  looks scrambled/tiled, Turnip and the SDE disagree on UBWC parameters,
  set `XRT_COMPOSITOR_GALAXYXR_UBWC=0` and report.
- The client library embeds the git version and the service refuses
  mismatches: rebuild everything (not just monado-service) after
  committing, or set `IPC_IGNORE_VERSION=1`.
- 3DoF only; no mag fusion. The gyro bias is corrected (factory intrinsics
  + the SSC `gyro_cal` online estimate) and the yaw drift went from
  visible-over-minutes to near-invisible, eyes-on validated. All the
  camera calibration for real 6DoF exists (see above).
- Passthrough currently exists only in the distortion-mesh gfx N-layer fast
  path and accepts two-view linear NV12. The EFS curved model and live
  three-axis camera-eye offsets, projection-layer rotational timewarp, and
  row-dependent rolling shutter are wired in, but exposure timing, the planar
  focus assumption, vertex-interpolation error, depth/disocclusion, and
  vignetting still need validation or compensation.
- Panel emission timing is unmeasured: the display time assumes photons
  start with the signal (scanout begin, zero lag). Rolling emission
  matches the eyes-on scanout-compensation validation, but the constant
  write-to-emission lag and the persistence window need a photodiode or
  high-fps camera measurement; the result then becomes the
  `XRT_COMPOSITOR_GALAXYXR_EMISSION_LAG_MS` default (and mid-persistence
  should be the per-row reference).
- Panel backlight is not touched (`/sys/class/backlight/panel{0,1}-backlight`,
  root-only); kwin's last brightness setting persists.
- Lease revocation returns `VK_ERROR_SURFACE_LOST_KHR` but there is no
  reacquire path.
- One unexplained: hello_xr (Vulkan2 plugin) once failed to reach the
  session loop while xrgears worked; not reproduced enough to root-cause.
