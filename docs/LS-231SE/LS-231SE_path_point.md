# Coordinated motion control

Each LS-231SE drive contains a 256-element buffer for path points. Each path point is a goal position for the motor. When in path mode, the drive transitions to the next point at a predictable user selectable rate with 51.2 μSec resolution.

The drive transitions between goal points at a constant velocity, guaranteeing the next path point will be reached exactly at the pre-calculated time. By sending sets of path points to multiple drives and starting the paths simultaneously, all drives will execute their paths in synchrony.

As an example, with a path point clock of 5.12 mSec, the buffered points can run for about 1.3 seconds of motion. The host computer loads the buffer, then starts the path mode. While the drives are executing the stored moves, the host sends additional path points. This continues until the path is complete.

Drives are often organized into multi-axis machines. The host computer calculates coordinated path sets, then sends each drive's individual set. 

The host computer calculates the coordinated paths, allowing any concceivable path to be generated and executed, including coordinated straight-line motions, multi-axis circular motions, and S-curve profiling motions. The host must consider the physical limits and remain  within the acceleration and velocity limits of the system being controlled.

The host must monitor, compute, and send new path points before the buffer is empty. The path transition rate Even non-real time hosts like PCs running Windows, are sufficiently responsive. 

Note that motions created with the path mode are independent of any acceleration or velocity values loaded using the Load Trajectory command.

Operation details

**Path Point Mechanics**:
1. Each 2-byte value is added to the desired position every servo tick
2. The value is applied **Path Point Buffer Timer** times (set via I/O Control command (TODO: Add heading link))
3. This creates a linear segment from current position to next path point
4. Multiple points create a continuous trajectory

**Buffer Capacity**: 256 path points maximum

**Timing**:
- Time per point = Path Point Buffer Counter × 51.2 µs
- Example: Counter = 100 → 5.12 ms per point
- Must be set via I/O Control (0x8) command before path execution

**Status Monitoring**:
- Use Status Packet item path_count (bit 7) to monitor path points remaining in buffer
- Use Auxiliary Status bit 6 (path_mode) to check if path is executing
- Buffer refill when path_count drops below threshold

**Notes**:
- Path buffer holds 256 points total
- Points are consumed at the rate set by Path Point Buffer Counter
- Servo must be enabled before starting path execution
- Path mode stops when buffer empties or Stop Motor/Load Trajectory command sent
- Each point defines an incremental velocity, not absolute position
- Fractional component (1/256 count) allows smooth motion at slow speeds
- For multi-axis coordination, use group commands to start paths simultaneously


## Related Commands

TODO: Make this a bulleted list.

TODO: Add super short summary of relevant I/O control command as it relates to path points and a link to I/O Control command in the commands file with a header link.

TODO: Also add a summary and link.

## Related Status

TODO 