## `publishing_overhead`
From `rcl_publish`: 
> Calling `rcl_publish()` is a potentially blocking call.
> When called `rcl_publish()` will immediately do any publishing related work, including, but not limited to,
> converting the message into a different type, serializing the message, collecting publish statistics, etc.
> The last thing it will do is call the underlying middleware's publish function which may or may not block based on
> the quality of service settings given via the publisher options in `rcl_publisher_init()`.
> For example, if the reliability is set to reliable, then a publish may block until space in the publish queue is
> available, but if the reliability is set to best effort then it should not block.

How long does `rcl_publish()` block for? How much time does serialization take?

```
       1 bytes took       9.19 us! (Averaged over 100 repetitions.)
       2 bytes took       0.36 us! (Averaged over 100 repetitions.)
       3 bytes took       0.11 us! (Averaged over 100 repetitions.)
       4 bytes took        0.1 us! (Averaged over 100 repetitions.)
       5 bytes took       0.25 us! (Averaged over 100 repetitions.)
      10 bytes took       5.18 us! (Averaged over 100 repetitions.)
      15 bytes took       3.92 us! (Averaged over 100 repetitions.)
      20 bytes took       0.31 us! (Averaged over 100 repetitions.)
      50 bytes took       17.1 us! (Averaged over 100 repetitions.)
     100 bytes took      41.62 us! (Averaged over 100 repetitions.)
     200 bytes took       7.63 us! (Averaged over 100 repetitions.)
     300 bytes took      14.27 us! (Averaged over 100 repetitions.)
     400 bytes took       19.8 us! (Averaged over 100 repetitions.)
     500 bytes took      20.32 us! (Averaged over 100 repetitions.)
    1000 bytes took      23.17 us! (Averaged over 100 repetitions.)
    1100 bytes took      23.34 us! (Averaged over 100 repetitions.)
    1200 bytes took      32.32 us! (Averaged over 100 repetitions.)
    1300 bytes took      27.18 us! (Averaged over 100 repetitions.)
    1400 bytes took      23.49 us! (Averaged over 100 repetitions.)
    1500 bytes took      29.16 us! (Averaged over 100 repetitions.)
    2000 bytes took      36.51 us! (Averaged over 100 repetitions.)
    3000 bytes took      45.27 us! (Averaged over 100 repetitions.)
    4000 bytes took      52.78 us! (Averaged over 100 repetitions.)
    5000 bytes took      48.47 us! (Averaged over 100 repetitions.)
   10000 bytes took      43.54 us! (Averaged over 100 repetitions.)
   12500 bytes took      35.82 us! (Averaged over 100 repetitions.)
   15000 bytes took      27.29 us! (Averaged over 100 repetitions.)
   20000 bytes took      22.69 us! (Averaged over 100 repetitions.)
   30000 bytes took       20.3 us! (Averaged over 100 repetitions.)
   40000 bytes took      19.48 us! (Averaged over 100 repetitions.)
   50000 bytes took      21.45 us! (Averaged over 100 repetitions.)
  100000 bytes took       28.3 us! (Averaged over 100 repetitions.)
  200000 bytes took      30.11 us! (Averaged over 100 repetitions.)
  500000 bytes took      64.47 us! (Averaged over 100 repetitions.)
 1000000 bytes took     106.05 us! (Averaged over 100 repetitions.)
 ```
