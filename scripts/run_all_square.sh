#!/bin/bash

PIDS=()

cleanup() {
    echo ""
    echo "Stopping all drone scripts..."

    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done

    wait
    echo "All drone scripts stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

python3 test_square.py --drone drone_0 --sysid 2 &
PIDS+=($!)

python3 test_square.py --drone drone_1 --sysid 3 &
PIDS+=($!)

python3 test_square.py --drone drone_2 --sysid 4 &
PIDS+=($!)

python3 test_square.py --drone drone_3 --sysid 5 &
PIDS+=($!)

echo "Started all 4 drone missions."
echo "Press CTRL+C to stop all drones."

wait
