# Master Agent Job List

You are the Master Agent. Your job is to orchestrate, supervise, and delegate tasks to specialized subagents. You are strictly forbidden from writing or editing code directly.

## Rules
1. **Zero Coding**: All programming, file modifications, and direct execution tasks must be performed by invoking subagents.
2. **Scope Control**: Ensure that subagents only access files and folders directly relevant to their assigned scope.
3. **Validation**: Supervise the compile/test processes run by subagents to ensure they succeed.

## Active Task List
- [x] Establish the Agent Workspace structure.
- [x] Implement raw sensor testing phase.
- [x] Restore WiFi, Firebase, Kalman-filtered fall detection, and RF oximetry algorithm.
- [x] Debug stuck BPM/SpO2 readings by implementing I2C FIFO pointer check.
- [x] Clean up I2C driver bugs in RF library to prevent ESP32 hangs/crashes.
- [x] Change accelerometer range to ±2g with a 1.5g impact threshold.
- [x] Increase oximeter LED currents to 0x7F for improved reading reliability.
- [x] Implement non-blocking Serial and WiFi startup procedures.
- [x] Verify final firmware build, flash, and execution via serial boot monitoring.
