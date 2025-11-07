#!/usr/bin/env python3
import sys
import csv
import re

# Goated ChadGPT script that tells you the PL of the ROS network that you subscribe to.
# To launch, run ros2 topic echo {TOPIC} | python3 ros_pl_calculator.py
# Where {TOPIC} is the topic you wish to target.
# Even more useful with a shell integration like starship to tell you the elapsed time automagically

def main():
    print("Listening for ROS2 message headers on stdin... (Ctrl+C to stop)")
    print("Expecting messages in format:\n---\ndata: x\n---\n")

    packet_ids = []
    output_csv = "../ros_packets.csv"

    # Regex to extract "data: <number>"
    data_pattern = re.compile(r"data:\s*(\d+)")

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Packet_ID"])  # CSV header

        buffer = []
        try:
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue

                buffer.append(line)

                # End of ROS2 message header
                if line == "---":
                    for bline in buffer:
                        match = data_pattern.match(bline)
                        if match:
                            try:
                                pid = int(match.group(1))
                                packet_ids.append(pid)
                                writer.writerow([pid])
                                csvfile.flush()
                                break
                            except ValueError:
                                print(f"⚠️  Invalid data value: {bline}")
                    buffer = []  # Reset for next message

        except KeyboardInterrupt:
            print("\n🛑 Stopped by user.\n")

    # Analyze received packets
    if not packet_ids:
        print("No valid packet IDs were received.")
        return

    packet_ids = sorted(set(packet_ids))
    start, end = packet_ids[0], packet_ids[-1]
    total_expected = end - start + 1
    total_received = len(packet_ids)
    lost_count = total_expected - total_received
    packet_loss_percent = (lost_count / total_expected) * 100

    print(f"✅ Received {total_received}/{total_expected} packets.")
    print(f"📄 Logged to: {output_csv}")
    print(f"⚠️ Packet loss: {packet_loss_percent:.2f}% ({lost_count} packets lost)")

if __name__ == "__main__":
    main()
