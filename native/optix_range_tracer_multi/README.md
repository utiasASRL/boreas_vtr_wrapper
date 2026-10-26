# optix_range_tracer_multi (concept test)

A copy of `../optix_range_tracer` with optional multi-channel depth, kept separate so the single-ray
tracer used by the localization pipeline is untouched. Delete this folder and
`build/optix_range_tracer_multi` to revert.

`tracer.set_channels(mode, count, max_range, layer_skip)`, then `trace()` returns `[B, C, H, W]`:
- `single` (default): closest hit, `[B, H, W]`, identical to `optix_range_tracer`.
- `bands`: closest hit inside each of `count` equal range bands of `[0, max_range]` (min range still applies).
- `layers`: first hit, then the closest hit at least `layer_skip` m past the previous hit, up to `max_range`.

Build inside the container: `bash native/optix_range_tracer_multi/build.sh`

Don't import `optix_range_tracer` and `optix_range_tracer_multi` in the same process: both link the
OptiX SDK's sutil library under the same name, so the second one would load the first one's device code.
Check against the original tracer: `python3 native/optix_range_tracer_multi/test_multi_channel.py`
