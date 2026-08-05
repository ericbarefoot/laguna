## Summary

<!-- What changed and why. Lead with the problem, not the diff — a short
     paragraph beats a bullet list of files touched. -->

## Testing

<!-- What you ran and what it confirmed: `pytest` output, hardware verified
     by hand, docs built and eyeballed, etc. "Not tested" is a valid answer
     if you say why (e.g. no hardware available) rather than omitting it. -->

## Safety-relevant changes

<!-- Delete this section entirely if none of this applies. -->

- [ ] Touches motion, `safety.py`, `robot/macron/fences.py`, or exclusion-zone logic
- [ ] Changes default motion limits, fence geometry, or config defaults affecting them
- [ ] Touches recorded experiment data, checkpoints, or run manifests
- [ ] `safe_mode` still defaults to `True` for any motion path this touches

## Notes for reviewers

<!-- Judgment calls worth a second look, things intentionally left out,
     follow-up issues this opens. Delete if there's nothing non-obvious. -->
