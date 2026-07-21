# doppler-smoothing
Smoothing GPS trajectories using a Kalman filter that fuses raw position fixes with GPS Doppler-derived speed/course measurements.

Consumer GPS position fixes are noisy from sample to sample, but most GPS receivers also report an instantaneous speed and course derived from the Doppler shift of the satellite signals, which is typically far more accurate than position-differenced speed. This project uses that speed/course as an additional input to a Kalman filter, alongside the raw lat/lon fixes, to produce a smoother, more physically consistent trajectory.

![Raw vs. smoothed trajectory][def]

## How it works

The filter runs in a local North-East-Down (NED) frame anchored at the first GPS fix.

**State:** `x = [N, E, vN, vE]`.

**Process model:** constant-velocity kinematics, `x_k = F * x_(k-1)` with the standard `[1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]` transition matrix.

**Measurement model:** the raw fix is converted to NED and used as a position measurement. When speed ≥ 0.5 m/s, speed/course are converted to `(s*cos(psi), s*sin(psi))` and folded in as a velocity measurement (`H = I(4)`); otherwise it's a position-only update. `psi` comes from the current sample's course, gated against the `-1` unavailable sentinel and a course-accuracy threshold (`maxCourseAccuracy`, default 45°); if gated out, `psi` holds its last value.

Course (not device heading) is used deliberately — course is derived from consecutive fixes and *is* the direction of travel, while heading is compass-derived device orientation, which can differ from travel direction by tens of degrees.

**Noise:** `R` comes from the GPS-reported `hAccuracy`/`sAccuracy` (1-sigma). `Q` uses a discretized white-noise-acceleration model scaled by the actual elapsed `dt` between samples, rather than a fixed matrix:

```
q = Q_sacc^2
Q = [dt^3/3*q      0        dt^2/2*q     0
     0             dt^3/3*q 0            dt^2/2*q
     dt^2/2*q      0        dt*q         0
     0             dt^2/2*q 0            dt*q]
```

**Corner handling:** a constant-velocity model overshoots at sharp turns. The filter detects large course changes between fixes (`maneuverThresholdDeg`, default 20°) and temporarily inflates `q` (`maneuverBoost`, default 8×) for that step so it lets go of the straight-line assumption right at the turn.

The filtered NED position is converted back to lat/lon (`ned2llh`) each step for plotting.

## Files

### Coordinate transforms
| File | Purpose |
|---|---|
| `llh2xyz.m` | Geodetic → ECEF |
| `xyz2llh.m` | ECEF → geodetic |
| `llh2ned.m` | Geodetic → local NED |
| `ned2llh.m` | Local NED → geodetic |
| `xyz2ned.m` | ECEF → local NED |
| `ned2xyz.m` | Local NED → ECEF |

### Filtering and analysis
| File | Purpose |
|---|---|
| `plotLocations_kf_unfiltered.m` | Plots the raw GPS track |
| `plotLocations_kf_speed.m` | Runs the Kalman filter and plots the smoothed track |
| `plotLocations_filter_v_unfiltered.m` | Both tracks overlaid for comparison |
| `distances.m` | Point-to-point NED distances and std. dev. of reported speed. Still uses the older CSV layout below, not the raw export layout the other scripts use. |

### Example output
| File | Purpose |
|---|---|
| `Raw_vs_Smoothed.svg` | Example comparison plot.m|

## Input data

`plotLocations_kf_speed.m` / `plotLocations_kf_unfiltered.m` expect a raw iOS location-logging CSV (`readmatrix`), using only these columns (others, e.g. heading/IMU fields, are ignored):

| Column | Field |
|---|---|
| 3 | `locationTimestamp_since1970` (s) |
| 4 | `locationLatitude` (WGS84) |
| 5 | `locationLongitude` (WGS84) |
| 7 | `locationSpeed` (m/s) |
| 8 | `locationSpeedAccuracy` (m/s) |
| 9 | `locationCourse` (deg), `-1` if unavailable |
| 10 | `locationCourseAccuracy` (deg), `-1` if unavailable |
| 12 | `locationHorizontalAccuracy` (m) |

If your export logs at a higher rate than the GPS actually updates (location fields repeated across rows between fixes), filter it down to rows where the timestamp actually changes before running these scripts — not currently handled automatically.

CSVs are not included in this repo; place them in the MATLAB working directory and match the filename in the `readmatrix(...)` call (or edit it).

`distances.m` still expects the older, reformatted layout: col 1 = timestamp, 2 = lat, 3 = lon, 4 = course (`-1` if unavailable), 5 = speed (negative if unavailable), 7 = horizontal accuracy, 9 = speed accuracy.

## Usage

1. Place a matching GPS log CSV in the MATLAB path.
2. Run `plotLocations_filter_v_unfiltered.m` for the overlaid comparison, or run either plotting script individually.
3. Run `distances.m` (older column layout) for point-to-point distances and speed std. dev.

## Known limitations / TODOs

- The corner boost and `Q` are heuristics, not a true innovation-adaptive filter.
- Course accuracy gates trust in a sample rather than being propagated into `R` as a weighted variance (would need an EKF-style Jacobian, since course enters nonlinearly).
- Heading isn't fused with course at all; a real sensor-fusion approach would likely help further.
- Long stretches of poor `hAccuracy` cause the filter to mostly dead-reckon from speed/course — expected given the reported accuracy, but not currently flagged in the output.
- `Q_hacc` and the pre-loop `Q` in `plotLocations_kf_speed.m` are dead code left over from before `Q` became time-scaled.
- `distances.m` wasn't updated to the current column layout used elsewhere.


[def]: Raw_vs_Smoothed.svg