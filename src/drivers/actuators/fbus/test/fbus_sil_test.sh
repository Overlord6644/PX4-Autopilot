#!/bin/bash
# FBUS driver software loopback bench: PX4 SITL + fake Xact servo on a pty.
cd "$HOME/PX4-Autopilot/build/px4_sitl_nolockstep" || exit 1
[ -e bin/px4-fbus ] || ln -s px4 bin/px4-fbus

rm -f /tmp/fbus_params /tmp/fake_xact.log /tmp/px4_fbus.log

python3 /tmp/fake_xact.py 40 > /tmp/fake_xact.log 2>&1 &
FAKE_PID=$!
sleep 1

cat > /tmp/fbus_rcS <<'EOF'
. px4-alias.sh
uorb start
param select /tmp/fbus_params
fbus start -d /tmp/fbus_pty
EOF

export PATH="$PWD/bin:$PATH"
./bin/px4 -d -s /tmp/fbus_rcS > /tmp/px4_fbus.log 2>&1 &
PX4_PID=$!
sleep 3

echo '=== status: bus must be silent ==='
./bin/px4-fbus status 2>&1

echo '=== cfg read physid (wakes the bus, expects 0x0C = 12) ==='
./bin/px4-fbus cfg read physid 2>&1

sleep 7  # discovery scan window (5 s) + telemetry settling

echo '=== status after discovery ==='
./bin/px4-fbus status 2>&1

echo '=== cfg write center -10 then read back ==='
./bin/px4-fbus cfg write center -10 2>&1
./bin/px4-fbus cfg read center 2>&1

echo '=== servo_status topic ==='
./bin/px4-listener servo_status 2>&1

./bin/px4-shutdown 2>/dev/null
sleep 1
kill $PX4_PID 2>/dev/null
wait $FAKE_PID 2>/dev/null

echo '=== fake xact summary ==='
cat /tmp/fake_xact.log
echo '=== px4 log tail ==='
tail -8 /tmp/px4_fbus.log
