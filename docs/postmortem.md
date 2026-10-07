# Post-mortem

[← README](../README.md) · **English** | [한국어](postmortem_ko.md)

What held the car back, how each problem was found and fixed, and what we would do differently.

## Summary

| Root cause | Effect | Found | Status on `main` |
|---|---|---|---|
| Heavy per-frame pipeline at a forced 1280×720 on a Raspberry Pi 4 | Car inched forward and stopped until the last night | During the contest | Replaced by the lightweight node (07.17–18) |
| Image coordinates read as Cartesian | Lane-slope steering and headings reversed | During the contest | Avoided in the final node (07.18) |
| `control_node` lost its auto-mode subscriptions | Auto mode ignored the lane node | After the contest (git history) | Fixed during the contest (07.16) |
| Camera topic mismatch in the launch file | `/camera/image` published, `/camera/image_raw` subscribed | After the contest | Fixed, untested on the car |
| Crosswalk and finish stops not integrated | +2 min of penalties | — | Open |

## 1. The Pipeline Was Too Heavy for the Pi

- **Symptom**: the car moved a few centimetres, stopped, and repeated (07.14–17)
- **Per-frame work**: HSV mask → selective erosion → Canny → Hough → left/right regression (`fitLine`, later quadratic `polyfit`) → vanishing point → heading → PID → `imshow` windows
- **Resolution**: the driver resized every frame to 1280×720 (`usb_camera_driver.cpp`), so all stages ran on ~0.9 MP per frame
- **Stop-and-go logic**: several versions sent throttle 0 whenever one lane vanished for a single frame
- **Attempts**: C++ port (lane5c), removing debug windows, simpler variants (lane7) — none ready in time
- **Fix (07.17 night)**: a new 640×480 camera and the Instructables "Lane Keeping Car" pipeline (Hough → one averaged line per side → atan heading → proportional steering); a single visible line now steers instead of stopping the car
- **Why it transferred**: the original targets 320×240; our camera is 640×480 with the same 4:3 aspect ratio, and the ROI and steering math are ratio-based
- **Missed clues**: on 07.03 the organizers advised "keep the algorithm simple and fast because of the Raspberry Pi" and "tune colour values on site"

## 2. Image Coordinates Read as Cartesian

- **Approach**: steer from the slope of the representative Hough line on each side
- **Symptoms**: the Pure Pursuit version steered in a completely different direction and did not move (07.14); a hybrid node printed "turn left" for a lane centre on the right while its formula steered right (07.15)
- **Cause**: the image origin is top-left and y grows downward, but slopes were read as in a regular x-y plot
  - a vanishing point straight ahead gave about -90° instead of 0° (07.14)
  - a positive image slope was labelled "negative" and mapped to a left turn (07.17 22:22)
  - with one lane visible, a steeper line got a stronger turn, although in the image a steep line means the car is aligned (07.17)
- **What was right**: left/right lane classification by slope sign and x position was correct from the start (07.10 example, 07.12 lane3)
- **Fix**: flip y when converting to vehicle coordinates (`img_h - vp_y`, 07.14); the final node no longer steers from the slope but from the lane-centre offset (`atan(x_offset / y_offset)`) and mirrors the heading (`180 - angle`)
- **Second cause in Pure Pursuit** *(code analysis)*: the angle was measured from the image x-axis, so it steered hard to one side either way
- **Hidden typo**: `teering_angle = 180 - steering_angle` in the 07.18 08:21 version silently disabled the mirror on the white-line path; fixed in the 13:02 race version

## 3. Auto Mode Stopped Following the Lane Node

- **Symptom (07.15 16:30)**: mode switching, buttons, manual and auto driving all "did nothing, with no error"; the organizers' original code still worked; the team suspected other teams' gamepads
- **One cause (git history)**: commit "Add joystick auto/manual mode switching" (07.14) removed `control_node`'s `/steering` and `/throttle` subscriptions, so auto mode ignored the lane node; this does not explain the manual-mode failure, which was never diagnosed
- **Fix**: subscriptions restored on 07.16 ("Restore auto-mode subscriptions in control_node"); the timer also changed 0.2 s → 0.02 s → 0.2 s that day
- **Lesson**: `ros2 topic info /steering` or `rqt_graph` would have shown the missing edge at once

## 4. The Final Controller Is Proportional, Not PD

- `compute_pd_control` comes from the Instructables code, where PD sets the **motor speed** and steering is on/off
- In our node the steering is `deviation / 180` with a ±8° dead band (output -0.23), clipped to [-0.55, 0.28], with an extra -0.39 when the lane centre is to the right of the image centre; the real output range is about -0.88 to 0.28, so the publish clip [-0.9, 0.7] never applies
- The PD value only feeds the throttle, and `max(min(|pd|, 0.25), 0.26)` always returns 0.26 — neither `kp = 0.45` nor the D term affects the output; in the red zone the throttle is a fixed 0.23 and the steering limit becomes 0.3
- Defined but never called: the yellow-lane branch and `detective_stop_line` (the white-pixel count for the start line and crosswalk, dropped because it was unreliable)
- The constant throttle was too fast for curves: the car kept drifting out on bends, which caused the ~2 lane departures in race 2 and the decision to skip the roundabout
- Slowing down on curves had been tried — slower when the vanishing point was far (lane3, 07.12), throttle lowered when the centre error passed 20 px (lane5, 07.16–17) — but none of it worked reliably in time, so the team judged a fast fixed throttle with ~2 departures to be the best trade-off
- Next step: a real PD on steering and a throttle that drops with the steering angle or lane curvature

## 5. If the Missions Had Been Integrated

- Recorded: 4:52.5 = raw lap + about 2 lane departures (+30 s each) + crosswalk and finish failures (+1 min each), the penalties applied at the contest
- Raw lap ≈ 1:52.5 *(lane-departure count from the team's recollection)*
- The roundabout (yellow lanes) had no time penalty; leaving it sent the car back to the roundabout entry — we skipped it and took the outer loop, since the car already left the lane on curves and the yellow mask did not work
- With the crosswalk and finish stops working *(estimate)*: ≈ 2:52.5; the drafts existed (white-pixel stop-line count, vertical-line crosswalk check, 7 s ROS timer, chessboard detection) but none ran as pasted (07.18 10:48–13:18)
- Rank effect unknown — other teams' times were not recorded

## Lane-Node Evolution

Each version was written because the previous one failed in a specific way.

| Version (date) | Idea | On the car | Why we moved on |
|---|---|---|---|
| lane1–3 (07.12) | Brightness threshold → lower-half ROI → Hough → midpoint between the lanes | Image node worked, ~7 s display lag | Cause of the lag not recorded |
| lane4 (07.12–16) | HSV + line regression + vanishing point, fixed steering table (07.13), later dynamic x centre | Followed lanes but oscillated | Table too coarse |
| **lane6 v1** (07.14) | Vanishing point, fixed table (-0.5 / -0.23 / 0.2) | First short runs ("seems to drive now") | Stopped when only one lane was visible |
| lane6 Pure Pursuit (07.14) | Pure Pursuit to the vanishing point | Wrong direction, did not move | Image axes (section 2) |
| lane6 PD (07.14 17:58) | PD (kp 0.4, kd 0.1) on the vanishing point | Not recorded, never committed | Replaced by a heading-based lane6 (21:33) |
| box-ROI node (07.15, run as lane5) | 6 + 5 fixed lane boxes and 2 stop-line boxes, white-pixel centroids | "Right lane not detected → stop" ~190 times | Lanes did not fall inside the boxes *(likely sized for another resolution)* |
| hybrid (07.15) | Quadratic fit (Sihyeon Park's idea) + centroids + straight-line fallback (~850 lines) | Not usable in time | Too heavy; left/right labels inverted |
| lane5 PID (07.16–17) | DonkeyCar PID on Sihyeon Park's pipeline; straight-line fit on 07.17 daytime, quadratic fit with target at 0.6·H from 19:28 | "It drove" on 07.16, then inching | Still heavy; throttle 0 on flicker |
| lane5c (07.16–17) | C++ port | Did not build | Out of time |
| lane7 (07.17 00:28) | Histogram two-peak centre + PID | No lanes detected | Needed tuning |
| **lane6 v2** (07.17–18) | Lightweight Hough + heading pipeline | 21:57 file did not run as written; 23:20 crashed (averaging loop); from 01:25 **drove continuously** | — |
| **final** (07.18 13:02) | lane6 v2 + child-zone slowdown (Sihyeon Park, Yoonju Jeong) | **Completed race 2, child zone passed** | — |

## Troubleshooting Log

| Problem | Diagnosis | Fix | Date |
|---|---|---|---|
| No camera image from the Pi | Driver missing → pydantic errors (a ChatGPT guess) → `xeyes` also failed, so X11 was the blocker | VcXsrv with access control off + `DISPLAY=<laptop IP>:0` (driver install and `pydantic<2` also applied); the camera failed to open again on 07.11 | 07.10 |
| Subscriber printed nothing | Missing dependencies, QoS mismatch, wrong topic | Recreated package, `qos_profile_sensor_data`, `/camera/image` | 07.12 |
| ~7 s display lag | Not recorded; later versions also had blocking `cv2.waitKey(0)` calls | Debug windows removed over 07.12–17 | 07.12 |
| lane6 would not start | `setup.py` entry pointed at `lane5_node`; a local fix was lost to a pull | Re-applied the fix | 07.14 |
| Pi offline at the lab | Cause not recorded | Chaeyeon Kim's phone hotspot via a lab monitor | 07.15 |
| Steering not centred at 0 | Hardware offset (other teams too) | -0.23 to -0.25 trim in most lane nodes (back in the final node from 01:25) | 07.15 |
| Yellow not detected | Track yellow samples H 0–12 / 146–176, S 15–46, V 75–102 vs mask H 18–48, S ≥ 94, V ≥ 140 | Yellow branch no longer called | 07.18 |
| Red zone | Red measured at H 174–179 | Mask H 170–180, throttle 0.23 above 20,000 px | 07.18 |
| lane6 v2 crash (`UnboundLocalError: x_offset`) | Lane averaging inside the loop → more than two lanes | Average once after the loop | 07.18 01:25 |
| Pinned code cut off | The chat notice truncated the box-ROI node, so the first run errored | Re-pasted and fixed by hand | 07.15 |

## Lessons

- **Measure on the target first**: fix the camera resolution and a frame-time budget on day 0, then add sophistication only when a measurement shows it is needed
- **Simplest end-to-end driver first**: Hough + average + P control would have driven on 07.14
- **Missions early**: each mission is worth a minute; crude state-machine versions beat polished lane code
- **Coordinate frames on paper**: one synthetic test image for left/right and slope sign
- **Version control, not chat pastes**: ~50 versions moved through the chat, a pinned copy was cut off, a local fix was lost to a pull; `main` was abandoned on 07.13 and the race code was never committed during the contest
