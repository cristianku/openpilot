# Speed Limit Vision: international-unit integration

Research source integration on `speed-limits`, 2 November 2026. The implementation
is present in this checkout; border data, GPS policy, real mph recognition and
device behavior remain unvalidated. Without them the unit comes from the country
of the settings (see Country lookup).

## Number, unit and physical speed

The classifier's numeric classes represent the digits on the sign. `class_speed`
and `read_signs` return that original number. `SpeedLimitMemory.update` converts
each new observation before confirmation: km/h divides by 3.6, mph multiplies by
0.44704. Candidates, held limits and comparisons use m/s. `speedvisiond` publishes
the held value directly as `speedVisionSP.speedLimit`; zero means unavailable.
The resolver does not convert it again. Display units do not reinterpret a held
vision reading.

The unit table is independent of `IMPLICIT_LIMITS`. GB/US numeric signs use
mph, but this implementation supplies no implicit legal defaults for them. Legacy
implicit, zone, garage and parking settings retain their physical km/h meaning.
Existing legal tables are preserved, not re-certified by this integration. An
explicit number equal to a town default remains an explicit sign.

`rawSignSpeed` retains physical km/h for old log consumers, zero when its unit is
unknown. The appended fields are `rawSignValue` (original digits), `rawSignUnit`
and `rawSignCountry` at that reading. Existing schema ordinals 0..11 are unchanged.

## Country lookup: open validation work

`country.py` uses only offline country polygons and the selected GPS service.
It does not read map limits, road names, map road types or junctions. Map/vision
fusion stays in the speed-limit resolver.

There are no approved policy defaults. All five values must be supplied explicitly
through the release's `settings.country_policy`: `gps_max_age_s`,
`gps_max_accuracy_m`, `border_margin_m`, `confirm_s`, `max_sample_gap_s`.
The lookup reads `country-borders.json.gz` beside `country.py`, with `countries`
containing ISO codes, bounding boxes and polygon rings. It is the coarse Natural Earth
asset the server labels the brain's country with, so the car gives the brain the country
of its training. Tested offline it missed observed crossings: near a border the country
can stay the old one for a while. A release carries a policy only with a brain, and its
values are a first try, not validated on the road; there the country is an input of the
network and picks the unit of the raw reading in the logs, nothing else. Without a brain
no policy is published, so the borders are not used and a unit never comes from them.

Until GPS confirms a country, `speedvisiond` uses the country chosen in
Settings › OSM (`OsmLocationName`, an ISO code), or CH when none is chosen, as
before the GPS lookup existed. This is the case without a policy or usable
borders, which is every release without a brain. Once GPS has confirmed a country, invalid
or stale GPS and ambiguous borders keep that last confirmed country.
`vision.sign_unit` gives mph in the UK, the US and the places that follow them
(`MPH_COUNTRIES`), km/h in every other country; only without a country is the
unit unknown and its numeric readings blocked. Which signs the networks read
is a matter of their training (Mapillary for other countries), not of this
table. The memory retains its last physical limit during unknown intervals,
while handling cancellation. A confirmed new jurisdiction clears pending candidates and
re-evaluates memory/implicit rules without converting an old number again. No new retention
timeout has been validated or introduced. These state choices are code-tested,
not device-validated.

For Vision Only and Map Data + Vision, a missing current allowed solution clears
the resolver's previous limit as well: the planner also checks the previous-limit
validity flag. Mixed mode can use a current map fallback; Vision Only cannot.

## The brain

When the release has a `brain` (`speed_limits.json`, `brain.onnx`), it is the vision limit: "Map Data + Vision" stays the one
choice in sunnylink and the resolver does not change. `brain.BrainReader` steps once a second with the frame, what `brain_inputs.py`
makes of the signs read, the car ahead (`radarState`), the local hour and the country from GPS, gas, brake and the car's speed
(`carState`). The speed it says is
`speedLimit`, with `speedLimitReason` "brain". The rules of `SpeedLimitMemory`, the road-type network and `LimitAhead` do not run
then; SignNet, the classifier and the distance to a sign do.

The contract holds the sha256 of the `brain_inputs.py` it was trained with: the car refuses another one, and speedvisiond then
sends nothing, so the resolver falls to the map. Without a `brain` the old limit runs as before.

The brain always says a number, so with "Map Data + Vision" the map is not used while it runs.

## A number with hours

A limit valid at some hours (a German 30 with a "22-6" plate below it) is a class of its own, `maximum_speed_hours:30`: one
box around the disc and the plate, read by the classifier like any other sign. `class_speed` gives its number, so
`SpeedLimitMemory`, `SignDistance` and `LimitAhead` treat it as a 30 at every hour. No code turns the hours into a limit: only
the brain can tell them apart, from the plate and the local hour.

`vision.class_plate` gives the plate, "time" for this class. It reaches the brain only when the contract's `step` says
`"plates": true`: a brain without that key was trained with the plate unknown on every frame, so it gets the signs as before.

The class says that hours exist, not which hours. The classifier sees only SignNet's box, so a SignNet that did not learn the
tall box cuts the disc alone and reads a plain 30; a plate far away is unreadable, so the sign reads as a plain number until
near. `SignDistance` measures this class by the box's width, the disc's side, since its height holds the plate too; that a
plate is no wider than its disc is assumed, not measured. A plain 30 on the same tall box is still measured by its height, so
it looks nearer than it is; when the class changes the sightings start again, as for another sign.

A number valid only on a wet road (a German 80 with "bei Nässe" below it) is the class `maximum_speed_wet:80`, the same way:
one box, its number for the rules at any weather, measured by its width. `class_plate` gives "wet", and it reaches the brain
only when the contract's `step` says `"wet": true`: `plate_wet` is a group of its own, so the `plate` group of a published brain
stays as it is and such a brain reads a plain 80. Whether the road is wet the brain sees in the picture, which it already gets:
it learns that from passing the plate on a wet road and on a dry one, with the speed of each in Speed review. No code turns a
plate and a wet road into a limit.

## Full resolution switches

Two Developer switches, both off by default and applied from the next drive. With them off nothing changes.

- **Record Road at Full Resolution** (`RecordRoadFullResolution`): `encoderd` writes `fcamera.hevc` from the 2688x1520
  stream, `qcamera` stays on the 1344x760 one. Same file, same index, 10 Mbit/s and a keyframe every 30 frames, so about
  twice the disk.
- **Read Signs at Full Resolution** (`ReadSignsFullResolution`): SignNet stays on the 1344x760 frame; the classifier's
  crop is the same window multiplied by 2, cut in memory from the frame with the same `frame_id`
  (`vision.classifier_input`). A frame the full stream does not have is read from the low one.

Both need `VISION_STREAM_NARROW_ROAD_FULL` from `camerad`. Without that stream `encoderd` records as usual and
`speedvisiond` reads the low frames.

`camerad` gives that stream with either switch on and a sensor that scales (OS04C10; the OX03C10 has one size and stays
as it is). The road camera then goes through the BPS, as the cabin does, and one pass writes both sizes of a frame: the
full one into the new stream, sent first, the low one where it always was. So the driving model reads a BPS picture too:
another debayer, no vignetting correction, about 7 ms later. It costs about 250 MB of memory. With both switches off
the road camera stays on the IFE.

## Tests in this checkout

From the repository root, with the project's Python 3.12 test environment:

```sh
python -m pytest openpilot/sunnypilot/speed_limits/tests -q -o addopts=''
```

The existing vision/schema tests and `tests/international/` use the sources in
this checkout, without applying a patch to copied sources. New tests cover the
required numeric examples, unknown units, country changes, cancellation, implicit
and parking physical values, log round-trips, GPS faults and border oscillation.
Daemon I/O is simulated; no model is loaded or inferred. The resolver/SLA chain
tests execute the actual checkout's class bodies with platform imports isolated.
They do not constitute a complete native messaging/planner stack test.

On this server Python 3.12, NumPy and Cap'n Proto were reused from the existing
distill environment; pure Python pytest dependencies were made available from
the existing speedlimits environment, with plugin autoload disabled. Nothing was
installed. The separate native resolver/SLA suites cannot collect here because
`msgq` and `openpilot/common/libparams_c.so` are unavailable.

The last baseline had 86 passing tests; the integrated suite has 123 passing
tests. No inference, training, import, publication, restart or device test was
performed in this integration phase. Evidence is in
`/root/work/speed-vision-units-2026-11-02/runtime-*.txt` on the server. The previous
preparation/replay report remains in the speedlimits repository at
`docs/2026-11-02-speed-vision-units-report.md`.
