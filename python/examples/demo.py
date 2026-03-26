#!/usr/bin/env python3
"""SafeMRC CAN driver demo."""
import time
import safemrc_can as mrc

def main():
    sm = mrc.SafeMRC("can0", use_fd=False)
    sm.add_device(1)
    sm.start()
    print(f"Started SafeMRC with {sm.device_count()} device(s)")

    sm.set_zero(1)
    time.sleep(0.01)

    print("FIX_LIMIT mode at 2.0 A")
    for i in range(100):
        sm.set_fix_limit(1, 2.0)
        time.sleep(0.001)
        state = sm.device_state(1)
        if not state.is_stale():
            print(f"[{i:3d}] mode={state.mode} collision={state.collision} "
                  f"pos={state.position_rad:.4f} rad cur={state.current_amp:.3f} A")

    if sm.device_state(1).collision:
        print("Collision! Resetting...")
        sm.reset_collision(1)
        time.sleep(0.01)

    sm.set_free_all()
    sm.stop()
    print("Done")

if __name__ == "__main__":
    main()
