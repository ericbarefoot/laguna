# Gocator 2690 concepts (encoderless surface scanning)

> **Archived 2026-08-05.** Kept for historical context; no longer maintained.

Research notes on the LMI Gocator 2690 laser line-profile sensor, focused on
running it **without an encoder** — an external gantry axis moves the sensor
at constant velocity, and we issue software start/stop triggers around the
move. Target IP `192.168.1.10`, firmware/SDK family 6.5.x, G2 hardware line.

Manual family: `https://am.lmi3d.com/manuals/gocator/gocator-6.5/G2/` (the
live TOC-driven site gated most content behind a login-console shell; a
mirrored copy of the adjacent 6.1 release — same G2 product line, same page
structure — is reachable at
`https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/` and is
cited below where the live 6.5 tree didn't render). Companion doc:
`GOCATOR_SDK_NOTES.md` (SDK/GoSdk-specific details, if present).

---

## 1. Measurement modes: Profile vs Surface

| Mode | What it produces | Use for |
|------|-------------------|---------|
| **Profile** | A single 2D range slice (X, Z) per exposure — one line per trigger. On point-profile sensors it can also assemble a fixed-length "part profile" from many range samples, but that's a different concept from 3D surface generation. | Line-scan measurement of a single cross-section, or high-speed rangefinder-style use. |
| **Surface** | Combines a *sequence* of profiles, gathered as the target moves under the sensor, into a single 3D dataset (resampled to a grid, or a raw 3D point cloud). | Building a 3D scan of a part/area as it passes under the sensor — this is what we want. |

> "Profile sensors create a single profile with each exposure. GoPxL can
> combine the series of profiles gathered as a target moves under the sensor
> to generate Surface data of the entire target."
> — [Surface Generation (Theory of Operation)](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G5/Content/TheoryOfOperation/DataGeneration/SurfaceGeneration.htm)

**For the gantry-moves-sensor-over-a-target setup: use Surface mode.** Profile
mode alone only gives you individual cross-sections; Surface mode is the
sensor-side feature that stitches profiles into the 3D result over the
travel axis (called Y in Gocator's convention — X is across the laser line,
Z is height, Y is the direction of relative motion).

---

## 2. Encoderless surface generation (the core question)

Surface Generation is configured in the web UI at **Inspect/Acquire > Scan
page > Scan Mode / Surface Generation panel**. Two orthogonal settings matter:

### a. How a surface is started/stopped ("Start Trigger")

| Start Trigger | Behavior |
|---|---|
| **Sequential** | Continuously generates back-to-back fixed-length surfaces with no external event. |
| **External Input** | A pulse on the digital input triggers generation of one fixed-length surface. |
| **Software Trigger** | "Allows starting fixed length surfaces on command from PLC or PC" — this is the one we want for software start/stop around a gantry move. |

Variable-length surfaces are also supported: profiles collected while the
external digital input is held high are combined into one surface (an
input-gated alternative to fixed length + software trigger).

Source: [Surface Generation panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G5/Content/WebInterface/Acquire/ProfileAndSurfaceGenerationPanel/SurfaceGeneration.htm)

### b. How individual profiles are spaced along the travel axis ("Trigger source")

This is set on the **Trigger panel**, independent of the surface start
trigger above, and is where the encoderless case is handled:

| Trigger source | Spacing driven by | Notes |
|---|---|---|
| **Encoder** | Physical quadrature encoder pulses, converted via encoder resolution + a user-set **Spacing** distance | Not applicable — no encoder in our setup. |
| **Time** | Sensor's internal clock, "fixed-frequency triggers" (a **Frame Rate**, supporting fractional Hz or "maximum rate") | **This is the encoderless path.** Profiles are taken at a fixed time interval; Y-spacing between profiles is then *assumed*, not measured. |
| **External Input** | Digital input edge (e.g. photocell) | Could gate Time triggering on/off but doesn't itself provide spacing. |
| **Software** | A single network command per profile | Not for continuous scanning — too slow/jittery for a full surface; software trigger is for the surface start/stop event, not per-profile spacing. |

Source: [Trigger panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/WebInterface/Acquire/TriggerPanel/TriggerPanel.htm)

### c. Telling the sensor the assumed travel speed — CORRECTED

**Confirmed on real 2690 hardware (2026-07-30) and against the manual page
directly** — this is simpler than originally written here. Travel speed is
a standalone setting, not buried inside the Bar/Disk Alignment workflow:

> "The Travel Speed setting enables correct scan scaling in systems lacking
> an encoder but using a conveyor moving at constant velocity... used to
> correctly scale scans in the direction of travel." Units: **mm/sec**.
> Configured via **Manage > Motion and Alignment > Speed**, either by manual
> entry, or automatically via an Alignment pass with Type set to `Moving`.
> — [Travel Speed](https://am.lmi3d.com/manuals/gocator/gocator-6.5/G2/Default.htm#WebInterface/Manage/MotionAndAlignment/TravelSpeed.htm)
> (gocator-6.5/G2, confirmed reachable — unlike most `am.lmi3d.com` pages,
> this specific deep link rendered content instead of a login wall)

SDK-side, this is `GoTransform_SetSpeed(transform, k64f value)` /
`GoTransform_Speed(transform)` in `GoTransform.h`, obtained via
`GoSensor_Transform(sensor)` — see `GOCATOR_SDK_NOTES.md` §2 for the
corresponding C API. The Bar/Disk **Moving**-type Alignment pass described
below is one way to *populate* this Speed setting automatically (by
measuring a known-width target), but manual entry of a known gantry feed
rate works directly and is simpler for our case, since we command the
gantry's velocity ourselves and already know it precisely.

### d. Profile spacing along X (across the laser line) — a related but separate knob

Not to be confused with Y (travel) spacing: **Uniform Spacing** governs
resampling *within* a profile, along X (across the laser line), independent
of encoder/time triggering:

> "When Enable uniform spacing is enabled, the ranges that make up a profile
> are resampled so that the spacing is uniform along the X axis... You can
> set the size of the spacing using the **Uniform spacing interval**
> parameter." Presets: **Speed** (lowest X resolution), **Balanced**
> (mid-range X resolution), **Resolution** (highest X resolution), or
> **Custom** (explicit µm value).
> — [What is Uniform Spacing](https://support.lmi3d.com/hc/en-us/articles/360033661471-What-is-Uniform-Spacing), [Uniform Spacing panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/WebInterface/Acquire/Uniform_spacing.htm)

Disabling uniform spacing (native/non-uniform X spacing) is one of the
documented ways to push scan rate toward the high end of the 2690's 900–10000
Hz range (see §6) — the 2600-series datasheet explicitly lists "uniform
spacing disabled" as part of its high-speed configuration recipe.

**Summary for our config (confirmed working on hardware 2026-07-30)**:
Trigger source = Time (set frame rate ≤ max scan rate for the field-of-view/
exposure chosen); Surface Generation type = Fixed Length, sized to the
expected travel distance, with Start Trigger = Software; Travel Speed set
manually under Manage > Motion and Alignment > Speed to match the gantry's
commanded feed rate (no Alignment/Bar/Disk pass needed for our case, since
we command the exact velocity ourselves). Procedure: start gantry motion,
wait for constant velocity, then fire the software start-scan trigger; the
sensor emits one correctly-Y-scaled surface per pass.

---

## 3. Trigger sources (recap, all sources)

| Source | Fires on | Typical use |
|---|---|---|
| Time | Internal clock, fixed frequency | Encoderless constant-velocity scanning (our case) |
| Encoder | Quadrature encoder pulses (Track Backward / Ignore Backward / Bi-directional modes, with a Reversal Distance setting for jitter) | Conveyor/variable-speed lines |
| External Input | Digital input rising edge | Photocell-gated capture |
| Software | Network command (SDK call, REST call, or protocol command) | One-off / PLC-scripted single triggers, or (as covered above) the *surface* start/stop event layered on top of Time-triggered profiles |

How software triggering is issued, in order of increasing abstraction:
- **Gocator Protocol** (raw TCP/UDP command channel; discovery broadcasts on
  UDP port 2016) — "Send commands to run sensors, provide software triggers,
  read/write files, etc." This is the lowest-level, protocol-only interface.
- **ASCII protocol** — a subset/variant reachable by sending plain text
  commands (e.g. a `Trigger` command) over a socket; documented for PLC/robot
  integrations (see the FANUC ASCII integration guide referenced from
  support.lmi3d.com).
- **GoSdk** (C library) — "open-source software libraries... used to
  programmatically access and control Gocator sensors," implementing the
  same network commands/data formats as the raw protocol. This is the
  SDK generation matching firmware 6.5.x / G2 hardware (as opposed to the
  newer "GoPxL SDK/REST API," which targets LMI's newer sensor families —
  see Open Questions).
- Also available at the PLC/industrial layer: Modbus, EtherNet/IP, PROFINET,
  each of which can also carry a software-trigger command.

Sources: [Gocator Protocol](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/Protocols/GocatorProtocol/GocatorProtocol.htm), [GoSDK](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/SoftwareDevelopmentKit/GoSDK.htm), [Trigger panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/WebInterface/Acquire/TriggerPanel/TriggerPanel.htm)

---

## 4. Data output & point cloud

- In **Surface mode**, the sensor's internal representation is "a random 3D
  point cloud where each individual point is an (X,Y,Z) coordinate triplet";
  this is then **resampled to an even grid in the X-Y plane** ("the resampling
  divides the X-Y plane into fixed size square bins... points that fall into
  the same bin are combined into a single Z value"). So the native Surface
  Data output is effectively a **Z-height grid** (heightmap), not an
  unordered XYZ list, once surface generation has run — X/Y positions are
  implied by array index and bin size rather than transmitted per-point.
- Over the **Gocator Protocol / Ethernet output**, when uniform spacing is
  enabled "only the range values (Z) are reported. The X positions can be
  reconstructed through the array index at the receiving end (the client)" —
  i.e., the wire format favors compact Z arrays over full XYZ triples.
- **Export formats**: the web UI's Recorded-data download path supports
  **CSV** ("Recorded data can be downloaded and saved on your computer in
  the CSV format, and the downloaded file can then be opened in Excel").
  GitHub community tooling (GoSdk-based, e.g. `ZiqiChai/gocator_3x00`,
  `beta-robots/gocator_3100`) commonly converts the SDK's native point-cloud
  structures to **PLY** for visualization, but that's a client-side
  convention built on top of GoSdk output, not a sensor-native export format
  we found documented for the G2/2690 line. No `.zdf`/OPC reference was
  found for the G2 (classic Gocator) product line — `.zdf` appears to be a
  format associated with other LMI product lines; treat as unconfirmed for
  2690 (see Open Questions).
- Other output data types selectable per-sensor over the Gocator protocol
  include Image, Range/Profile, Surface, Intensity, and measurement-tool
  results (Section, etc.), depending on sensor model and configured job.

Sources: [Uniform Data and Point Cloud Data](https://am.lmi3d.com/manuals/gopxl/gopxl-1.1/LMILaserLineProfiler/Content/TheoryOfOperation/Profile_RangeOutput/ResampledAndUniformSpacingProfile.htm), [Viewing Gocator data in a spreadsheet](https://support.lmi3d.com/hc/en-us/articles/360033298212-Viewing-Gocator-data-in-a-spreadsheet), [Gocator Communication Protocol](https://am.lmi3d.com/manuals/gopxl/gopxl-1.1/LMIFringeSnapshot/Content/WebInterface/Control/Gocator.htm)

---

## 5. Network/control interfaces (scriptable from Python)

| Interface | Transport | Notes for a Python-only stack |
|---|---|---|
| **Gocator Protocol** | Raw TCP (+ UDP discovery on port 2016) | Plain socket protocol; no C library required in principle, but the format isn't fully published in the pages we could reach without a login — would need the full Gocator Protocol Reference Manual (likely a PDF, may need an LMI account to download) to hand-roll a client. |
| **ASCII protocol** | TCP, plain text commands (e.g. `Trigger`) | Lightest-weight option for scripting from Python with nothing but `socket` — designed for PLC/robot integration (see FANUC integration guide). Good candidate for our software start/stop trigger. |
| **GoSdk** | C library (`libGoSdk.so` on Linux), with community Python/ROS wrappers | Requires either linking the C lib via ctypes/cffi or an existing wrapper (e.g. `robotsorcerer/gocator` builds `libGoSdk.so` on Linux; several GitHub repos wrap GoSDK4/5 for older Gocator 3x00 sensors, not 2690-verified). |
| **Modbus / EtherNet/IP / PROFINET** | Industrial fieldbus | Listed on the 2600-series datasheet as supported "Factory Communication" protocols alongside ASCII and Gocator; usable from Python via `pymodbus`/`cpppo` etc. without any LMI-specific library. |
| **GoPxL REST API** | HTTP/JSON | Exists for LMI's GoPxL-branded SDK/sensor family — unclear whether it applies to classic G2/6.5.x firmware or only newer hardware (see Open Questions). If it does apply, this would be the easiest pure-Python path (plain `requests` calls, JSON payloads for settings/trigger). |

Given `laguna`'s existing pattern of talking to sensors over plain
socket/HTTP protocols without vendor C libraries (see
`docs/subsystems/rangefinder.md`), **ASCII protocol or the raw Gocator
Protocol are the most laguna-idiomatic choices**, with GoSdk-via-ctypes and
Modbus as fallbacks.

Sources: [Gocator Protocol](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/Protocols/GocatorProtocol/GocatorProtocol.htm), [FANUC Socket/ASCII Communication Integration Guide](https://support.lmi3d.com/hc/en-us/articles/34376747695771-FANUC-Socket-ASCII-Communication-Integration-Guide), [GoPxL SDK and REST API](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/SoftwareDevelopmentKit/GoPxL_SDK.htm)

---

## 6. Gocator 2690 specifications

From the official 2600-series datasheet (`DATASHEET_Gocator_2600_US-1.4`,
LMI Technologies, 2023):

| Spec | 2690 value |
|---|---|
| Data points / profile | 3700 |
| Scan rate | 900 – 10000 Hz (low end = default/full FOV config; high end = reduced FOV + measurement range + uniform spacing disabled + optimized data spacing/output) |
| Resolution X (Profile Data Interval) | 124 – 550 µm |
| Linearity Z | ± 0.08% of Measurement Range |
| Repeatability Z | 12.00 µm |
| Clearance Distance (CD) | 325 mm |
| Measurement Range (MR, i.e. Z depth) | 1550 mm |
| Field of View (FOV, i.e. X width) | 385 – 2000 mm |
| Laser class | 2, 3R (660 nm red) |
| Dimensions | 55 × 105 × 280 mm |
| Weight | 2.12 kg |
| Interface | Gigabit Ethernet |
| Inputs | Differential Encoder, Laser Safety Enable, Trigger |
| Outputs | 2× Digital output, RS-485 Serial (115 kBaud) |
| Factory Communication | PROFINET, Modbus, EtherNet/IP, ASCII, Gocator |
| Power | +24 to +48 VDC (15 W); ripple ±10% |
| Housing | IP67 gasketed metal enclosure |
| Operating temp | 0–50 °C |

**Gantry velocity implication**: at Time-trigger frame rates of up to
~10 kHz (at the reduced-FOV/high-speed end) or ~900 Hz (full FOV, default),
the achievable along-travel (Y) sample spacing = gantry velocity / frame
rate. E.g. at 900 Hz and a gantry speed of 10 mm/s, Y spacing ≈ 11 µm;
at 5 mm/s, ≈ 5.6 µm. The X resolution (124–550 µm across the 385–2000 mm FOV)
will likely be the coarser axis in most of our scans, not the frame rate.

Source (spec table): [DATASHEET_Gocator_2600_US-1.4](https://ftp.stemmer-imaging.com/webdavs/docmanager/166909-DATASHEET_Gocator_2600.pdf)

---

## 7. Health/status & calibration/alignment concepts

**Alignment** establishes the sensor's coordinate frame relative to the
physical setup (and, for moving setups, the travel-speed/encoder scaling
used to convert triggers into real-world Y distances). It is described as
"a one-time setup step" performed via the System/Scan page's Alignment
panel before production scanning; once run, the calculated transform is
displayed and shown in the Sensor panel, and subsequent data is reported in
the aligned "system" reference frame rather than raw sensor coordinates.

| Alignment type | Target | Degrees of freedom compensated |
|---|---|---|
| Stationary | Flat Surface (e.g. conveyor bed) | Y angle, X/Z offset (3 DoF) |
| Stationary | Bar / Polygon (ring layout) | Same, plus reference-hole-based refinement |
| Moving | Disk (40 mm / 100 mm standard, or custom) | Adds Y offset, Z angle — up to 5 DoF |
| Moving | Bar (on a moving transport) | Same as Disk; also back-calculates travel speed / encoder resolution from the bar's known width |

For our encoderless setup: we should run a **Moving** alignment (Bar or Disk
target) once, at commissioning, moving the target under the sensor at the
same gantry feed rate we intend to scan at — this both aligns the coordinate
frame *and* lets the sensor infer the correct travel-speed value for Time
triggering, per §2c above. If the gantry's feed rate changes between scans,
either re-run alignment or (if the UI allows a direct manual entry) update
the travel-speed value to match, since Time-triggered spacing is entirely
dependent on that assumed velocity being correct.

Source: [Aligning Sensors with up to 5 Degrees of Freedom](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/WebInterface/Scan/AlignmentPanel/AligningSensors_5DoF.htm)

---

## References

- [Surface Generation (Theory of Operation)](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G5/Content/TheoryOfOperation/DataGeneration/SurfaceGeneration.htm)
- [Surface Generation panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G5/Content/WebInterface/Acquire/ProfileAndSurfaceGenerationPanel/SurfaceGeneration.htm)
- [Trigger panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/WebInterface/Acquire/TriggerPanel/TriggerPanel.htm)
- [Aligning Sensors with up to 5 Degrees of Freedom (gocator-6.1/G2)](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/WebInterface/Scan/AlignmentPanel/AligningSensors_5DoF.htm)
- [Gocator Protocol (gocator-6.1/G2)](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/Protocols/GocatorProtocol/GocatorProtocol.htm)
- [GoSDK (gocator-6.1/G2)](https://d3ejaiy6gq5z4s.cloudfront.net/manuals/gocator/gocator-6.1/G2/Content/SoftwareDevelopmentKit/GoSDK.htm)
- [GoPxL SDK and REST API](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/SoftwareDevelopmentKit/GoPxL_SDK.htm)
- [Gocator Communication Protocol (GoPxL manual)](https://am.lmi3d.com/manuals/gopxl/gopxl-1.1/LMIFringeSnapshot/Content/WebInterface/Control/Gocator.htm)
- [What is Uniform Spacing](https://support.lmi3d.com/hc/en-us/articles/360033661471-What-is-Uniform-Spacing)
- [Uniform Spacing panel](https://am.lmi3d.com/manuals/gopxl/gopxl-1.0/G2/Content/WebInterface/Acquire/Uniform_spacing.htm)
- [Uniform Data and Point Cloud Data](https://am.lmi3d.com/manuals/gopxl/gopxl-1.1/LMILaserLineProfiler/Content/TheoryOfOperation/Profile_RangeOutput/ResampledAndUniformSpacingProfile.htm)
- [Viewing Gocator data in a spreadsheet](https://support.lmi3d.com/hc/en-us/articles/360033298212-Viewing-Gocator-data-in-a-spreadsheet)
- [Encoder Spacing](https://support.lmi3d.com/hc/en-us/articles/360033298252-Encoder-Spacing)
- [Triggering Gocator via Ethernet](https://support.lmi3d.com/hc/en-us/articles/360033298752-Triggering-Gocator-via-Ethernet)
- [FANUC Socket/ASCII Communication Integration Guide](https://support.lmi3d.com/hc/en-us/articles/34376747695771-FANUC-Socket-ASCII-Communication-Integration-Guide)
- [DATASHEET_Gocator_2600_US-1.4](https://ftp.stemmer-imaging.com/webdavs/docmanager/166909-DATASHEET_Gocator_2600.pdf) (spec table, §6)
- Community GoSdk usage examples: [ZiqiChai/gocator_3x00](https://github.com/ZiqiChai/gocator_3x00), [beta-robots/gocator_3100](https://github.com/beta-robots/gocator_3100), [robotsorcerer/gocator](https://github.com/robotsorcerer/gocator) (older 3x00-series sensors, not 2690-verified, but demonstrate GoSdk-on-Linux patterns)

---

## Open questions (needs hardware verification or LMI account access)

1. **Which SDK generation actually ships for firmware 6.5.x / G2 / 2690?**
   Pages under `/manuals/gopxl/...` describe a newer "GoPxL SDK + REST API"
   (JSON/HTTP), while pages under `/manuals/gocator/gocator-6.x/G2/...`
   describe the older C-only **GoSdk**. It's unclear from public pages
   whether the 2690 on 6.5.x firmware exposes the REST API at all, or only
   the classic GoSdk/Gocator Protocol/ASCII stack. This materially changes
   whether Python control can be pure-HTTP (`requests`) or needs
   ctypes-wrapping `libGoSdk.so`. **Verify against the actual sensor's web
   UI / firmware release notes once we have hardware access.**
2. **Exact Gocator Protocol / ASCII command syntax** (e.g. the literal
   command to start/stop a Software-Trigger-sourced surface, response
   format, port number for the main protocol channel) — the full Protocol
   Reference Manual PDF wasn't reachable without an LMI account login; the
   live am.lmi3d.com TOC pages we could reach mostly rendered a login
   console instead of content. Recommend downloading the manual directly
   from `support.lmi3d.com` with proper LMI account credentials, or via
   the sensor's own embedded "Help" download once connected to 192.168.1.10.
3. **Raw surface message wire format specifics**: we found strong evidence
   the transmitted Surface Data is a Z-only grid (X/Y implied by array
   index/bin size) when uniform spacing is enabled, but didn't find the
   exact byte layout / GoSdk struct name (e.g. whether it's exposed as
   `GoSurfaceMsg` with a documented row/column stride) — needed before
   writing a parser.
4. **Whether `.ply`/`.zdf`/OPC export exists natively on the sensor** (as
   opposed to being a GoSdk-consuming client tool's convention) for G2/2690
   specifically — not confirmed; the CSV export via the web UI's Recorded
   Data feature is the only sensor-native export path we could confirm from
   docs.
5. **Time-trigger frame rate vs. exposure/data-quality tradeoffs** at the
   gantry speeds we care about — the datasheet's scan-rate range (900–10000
   Hz) is under specific FOV/spacing tradeoffs; we should measure achievable
   rate empirically once we've picked a working FOV/exposure for our target
   surface, rather than assume the datasheet max is available at our chosen
   settings.
6. **How to enter "travel speed" manually for Time triggering** without
   running a physical bar/disk alignment pass every time the gantry feed
   rate changes — is there a direct numeric field, or does every speed
   change require a fresh alignment run? Needs verification in the actual
   web UI.
