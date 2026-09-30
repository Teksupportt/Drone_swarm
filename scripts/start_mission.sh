#!/bin/bash
PX4_DIR=~/PX4-Autopilot
BUILD=$PX4_DIR/build/px4_sitl_default

# ─────────────────────────────────────────────
# Kill existing sessions
# ─────────────────────────────────────────────
echo "[mission] Killing existing sessions..."
pkill -x px4 || true
pkill gzserver || true
pkill gzclient || true
screen -ls | grep -E "px4_|xrce_|mav_|rover_" | awk '{print $1}' | xargs -I{} screen -S {} -X quit 2>/dev/null || true
sleep 2

# ─────────────────────────────────────────────
# Clear saved params
# ─────────────────────────────────────────────
rm -f $BUILD/rootfs/*/parameters.bson
rm -f $BUILD/rootfs/*/parameters_backup.bson
echo "[mission] Cleared saved parameters"

# ─────────────────────────────────────────────
# Gazebo environment
# ─────────────────────────────────────────────
source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  $PX4_DIR $BUILD 2>/dev/null
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_common
export PX4_SIM_MODEL=gazebo-classic_iris

# ─────────────────────────────────────────────
# Gazebo server
# ─────────────────────────────────────────────
echo "[mission] Starting Gazebo server..."
gzserver $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  --verbose &
sleep 5

# ─────────────────────────────────────────────
# Spawn 4 PX4 drone instances
# ─────────────────────────────────────────────
POSITIONS=("0 0" "10 0" "0 10" "10 10")

for i in 1 2 3 4; do
  N=$((i-1))
  X=$(echo ${POSITIONS[$N]} | cut -d' ' -f1)
  Y=$(echo ${POSITIONS[$N]} | cut -d' ' -f2)
  XRCE_PORT=$((8888+i))
  mkdir -p $BUILD/rootfs/$N

  echo "[mission] Spawning drone $N (PX4 instance $i) at ($X,$Y) XRCE port $XRCE_PORT"

  screen -dmS px4_$i bash -c "
    cd $BUILD/rootfs/$N
    PX4_UXRCE_DDS_PORT=$XRCE_PORT \
    PX4_UXRCE_DDS_NS=drone_$N \
    $BUILD/bin/px4 -i $i -d $BUILD/etc 2>&1 | tee out.log
  "

  rm -f /tmp/iris_$i.sdf
  python3 $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
    --mavlink_tcp_port $((4560+i)) \
    --mavlink_udp_port $((14560+i)) \
    --mavlink_id $((1+i)) \
    --gst_udp_port $((5600+i)) \
    --video_uri $((5600+i)) \
    --mavlink_cam_udp_port $((14530+i)) \
    --output-file /tmp/iris_$i.sdf

  sed -i "s|<ros><namespace>camera</namespace></ros>|<ros><namespace>drone_$N</namespace></ros>|g" /tmp/iris_$i.sdf
  sed -i "s|name=\"camera_plugin\"|name=\"camera_plugin_$N\"|g" /tmp/iris_$i.sdf

  gz model --spawn-file=/tmp/iris_$i.sdf \
    --model-name=iris_$i -x $X -y $Y -z 0.83

  sleep 2
done

# ─────────────────────────────────────────────
# Spawn TurtleBot3 Waffle rover at (1, 1)
# ─────────────────────────────────────────────
echo "[mission] Spawning TurtleBot3 Waffle rover at (1, 1)..."
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
source /opt/ros/humble/setup.bash
source ~/ros/sar_ws/install/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -file /home/teksupportt/.gazebo/models/turtlebot3_waffle_sar/model.sdf \
  -entity turtlebot3_waffle \
  -x 1.0 -y 1.0 -z 0.0 \
  -R 0.0 -P 0.0 -Y 0.0
sleep 3

gzclient &
sleep 3

# ─────────────────────────────────────────────
# Start XRCE-DDS agents
# ─────────────────────────────────────────────
echo "[mission] Starting XRCE-DDS agents..."
for i in 1 2 3 4; do
  N=$((i-1))
  XRCE_PORT=$((8888+i))
  screen -dmS xrce_$N bash -c "
    source /opt/ros/humble/setup.bash
    echo '[xrce_$N] Agent starting on port $XRCE_PORT'
    MicroXRCEAgent udp4 -p $XRCE_PORT
  "
  echo "[mission] XRCE agent for drone $N started (screen: xrce_$N, port: $XRCE_PORT)"
done

# ─────────────────────────────────────────────
# Wait for PX4 instances to boot
# ─────────────────────────────────────────────
echo "[mission] Waiting 15s for PX4 instances to boot..."
sleep 15

# ─────────────────────────────────────────────
# Set arming params on all drones
# ─────────────────────────────────────────────
echo "[mission] Setting arming params on all drones..."

python3 << 'PYEOF'
import time
from pymavlink import mavutil

GCS_PORTS = [18571, 18572, 18573, 18574]
PARAMS = {
    "COM_RC_IN_MODE": 4,
    "NAV_RCL_ACT":    0,
    "COM_RCL_EXCEPT":  7,
}

for idx, port in enumerate(GCS_PORTS):
    print(f"  [drone_{idx}] Connecting on port {port}...")
    try:
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=255)
        conn.wait_heartbeat(timeout=10)
        print(f"  [drone_{idx}] Heartbeat received (sysid={conn.target_system})")
        for name, value in PARAMS.items():
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                name.encode("utf-8"),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.3)
            print(f"  [drone_{idx}] SET {name}={value}")
        time.sleep(1)
        conn.close()
        print(f"  [drone_{idx}] Done")
    except Exception as e:
        print(f"  [drone_{idx}] FAILED: {e}")

print("[mission] Param set complete")
PYEOF

# ─────────────────────────────────────────────
# Start MAVProxy debug sessions
# ─────────────────────────────────────────────
echo "[mission] Starting MAVProxy debug sessions..."
for i in 1 2 3 4; do
  N=$((i-1))
  GCS_PORT=$((18570+i))
  screen -dmS mav_$N bash -c "
    echo '[mav_$N] MAVProxy debug session for drone $N (port $GCS_PORT)'
    mavproxy.py --master=udp:0.0.0.0:$GCS_PORT
  "
  echo "[mission] MAVProxy for drone $N started (screen: mav_$N)"
done

# ─────────────────────────────────────────────
# Launch rover stack (SLAM + Nav2 + rover node)
# ─────────────────────────────────────────────
echo "[mission] Launching rover stack (SLAM + Nav2 + rover node)..."
screen -dmS rover_stack bash -c "
  source /opt/ros/humble/setup.bash
  source ~/ros/sar_ws/install/setup.bash
  export TURTLEBOT3_MODEL=waffle
  ros2 launch sar_rover sar_phase2.launch.py
"
echo "[mission] Rover stack started (screen: rover_stack)"

# ─────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo " Mission ready"
echo "════════════════════════════════════════"
echo " PX4 instances:   screen -r px4_1  .. px4_4"
echo " XRCE-DDS agents: screen -r xrce_0 .. xrce_3"
echo " MAVProxy debug:  screen -r mav_0  .. mav_3"
echo " Rover stack:     screen -r rover_stack"
echo " ROS 2 topics:    source /opt/ros/humble/setup.bash"
echo "                  ros2 topic list | grep drone"
echo "════════════════════════════════════════"
echo ""
wait
