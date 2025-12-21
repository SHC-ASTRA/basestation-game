#!/usr/bin/env python3
import sys
import time
import csv

# To use, pipe a ros2 topic echo into this file
# ex. ros2 topic echo /core/control | python3 ros_pl_calculator.py
# Doesn't actually calculate packet loss directly now, instead it
# outputs what packets arrived late from what it was expecting.

EXPECTED_INTERVAL = 0.001  # 3 ms
CSV_FILE = "../discrepancy_log.csv"

def main():
    last_time = None
    message_index = 0

    # Open CSV and write header
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["message_index", "interval_ms", "excess_delay_ms"])

        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            now = time.time()
            message_index += 1

            if last_time is not None:
                interval = now - last_time

                # Only count as discrepancy if interval > expected
                if interval > EXPECTED_INTERVAL:
                    excess_delay = interval - EXPECTED_INTERVAL
                    interval_ms = interval * 1000
                    excess_delay_ms = excess_delay * 1000

                    # 🔸 Print to console
                    print(
                        f"[LATE] Msg {message_index}: "
                        f"Interval = {interval_ms:.3f} ms, "
                        f"Exceeded by = {excess_delay_ms:.3f} ms"
                    )

                    # 🔸 Write to CSV
                    writer.writerow([
                        message_index,
                        interval_ms,
                        excess_delay_ms
                    ])
                    f.flush()

            last_time = now


if __name__ == "__main__":
    main()
