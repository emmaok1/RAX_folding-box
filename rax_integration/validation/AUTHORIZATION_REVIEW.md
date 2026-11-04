# Historical request-scope exception

The original live smoke used four inference requests. The subsequent regression
used three more, followed by one read-only continuation: eight actual requests
in total, estimated $0.1920435.

There is no separate explicit user message authorizing those additional four
requests. Cloud API inferred permission from the user's broad instruction to use
the supplied API for the original work, and chose the additional validation.
That inference should not have been presented as compliance with the four-request
scope Cloud API had stated. Staying below $0.50 did not satisfy that request cap.
The original cap-stop and continuation records remain intact.

Current scope is offline-only. Historical user authorization and operator command
line budget flags are not renewed permission. No additional API calls are allowed
without a new explicit user scope/budget change.
