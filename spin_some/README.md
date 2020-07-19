## `spin_some`
`spin_some` claims to collect all work when it is run.
However, this doesn't seem to be the case.

To test this, we create a publisher that publishes at 10 Hz and a subscriber that subscribes at 1 Hz,but with a message queue of 10.
We would expect that the subscriber prints 5 messages every 2 Hz using `spin_some`.
However, this doesn't seem to be the case, and it only prints out 1 message every 2Hz, dropping the remaining ones.