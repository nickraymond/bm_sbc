#!/usr/bin/env bash
# scripts/udp_multinode_test.sh — UDP transport multiprocess validation
#
# Same shape as multinode_test.sh, but every node runs --transport udp on
# 127.0.0.1 with distinct ports.  Validates:
#   u1  2-node mesh over UDP: neighbors, BCMP ping reply, pub/sub both ways
#   u2  3-node chain A -- B -- C over UDP: correct neighbors AND the chain
#       invariant — the end nodes must NOT see each other as neighbors
#       (a star here would stop testing L2 forwarding)
#   u3  oversize datagram is dropped with a logged length + ingress port
#
# Usage: ./scripts/udp_multinode_test.sh [path/to/bm_sbc_multinode]

set -euo pipefail

BINARY="${1:-./build/all/bm_sbc_multinode}"

if [[ ! -x "$BINARY" ]]; then
  echo "Binary not found: $BINARY"
  echo "Build with: cmake --preset all && cmake --build --preset all"
  exit 1
fi

WORK=$(mktemp -d /tmp/bm_sbc_udp_test_XXXXXX)
PASS=0; FAIL=0

# check <description> <log-file> <grep-string>
check() {
  local desc="$1" file="$2" pattern="$3"
  if grep -qF "$pattern" "$file" 2>/dev/null; then
    echo "  PASS: $desc"
    PASS=$((PASS + 1))
  else
    echo "  FAIL: $desc"
    echo "        (pattern not found: '$pattern' in $file)"
    FAIL=$((FAIL + 1))
  fi
}

# check_absent <description> <log-file> <grep-string>
check_absent() {
  local desc="$1" file="$2" pattern="$3"
  if grep -qF "$pattern" "$file" 2>/dev/null; then
    echo "  FAIL: $desc"
    echo "        (pattern unexpectedly present: '$pattern' in $file)"
    FAIL=$((FAIL + 1))
  else
    echo "  PASS: $desc"
    PASS=$((PASS + 1))
  fi
}

# start_node <log-file> [binary-args...]  — launches in background, prints PID
start_node() {
  local log="$1"; shift
  BM_SBC_LOG_STDOUT=1 BM_SBC_LOG_LEVEL=debug "$BINARY" --log-dir "$WORK/logs" \
    --transport udp "$@" >"$log" 2>&1 &
  echo $!
}

kill_nodes() { kill "$@" 2>/dev/null; wait "$@" 2>/dev/null || true; }

# Rate shaping is disabled (--udp-rate-mbps 0) in these tests: they verify
# correctness, not throughput, and CI machines shouldn't wait on a shaper.

# ---------------------------------------------------------------------------
# Test u1: 2-node mesh over UDP (A <-> B)
# ---------------------------------------------------------------------------
echo "=== Test u1: 2-node mesh over UDP (A <-> B) ==="
LA="$WORK/A.log"; LB="$WORK/B.log"

PA=$(start_node "$LA" --node-id 0x0000000000000001 \
                       --udp-listen 127.0.0.1:23101 \
                       --udp-peer 127.0.0.1:23102 \
                       --udp-rate-mbps 0)
PB=$(start_node "$LB" --node-id 0x0000000000000002 \
                       --udp-listen 127.0.0.1:23102 \
                       --udp-peer 127.0.0.1:23101 \
                       --udp-rate-mbps 0)

echo "  Nodes started (PIDs: $PA $PB). Waiting 15 s for convergence..."
sleep 15

check "A produced output"              "$LA" "multinode app: setup"
check "B produced output"              "$LB" "multinode app: setup"
check "A received ping reply from B"   "$LA" "bcmp_seq="
check "B received ping reply from A"   "$LB" "bcmp_seq="
check "B received PUBSUB from A"       "$LB" "PUBSUB_RX from=0000000000000001"
check "A received PUBSUB from B"       "$LA" "PUBSUB_RX from=0000000000000002"
check "A discovered B (NEIGHBOR_UP)"   "$LA" "NEIGHBOR_UP node=0000000000000002"
check "B discovered A (NEIGHBOR_UP)"   "$LB" "NEIGHBOR_UP node=0000000000000001"

kill_nodes "$PA" "$PB"
echo ""

# ---------------------------------------------------------------------------
# Test u2: 3-node chain over UDP  A -- B -- C  (+ chain invariant)
# ---------------------------------------------------------------------------
echo "=== Test u2: 3-node chain over UDP (A -- B -- C) ==="
LA3="$WORK/A3.log"; LB3="$WORK/B3.log"; LC3="$WORK/C3.log"

PA3=$(start_node "$LA3" --node-id 0x0000000000000001 \
                         --udp-listen 127.0.0.1:23201 \
                         --udp-peer 127.0.0.1:23202 \
                         --udp-rate-mbps 0)
PB3=$(start_node "$LB3" --node-id 0x0000000000000002 \
                         --udp-listen 127.0.0.1:23202 \
                         --udp-peer 127.0.0.1:23201 \
                         --udp-peer 127.0.0.1:23203 \
                         --udp-rate-mbps 0)
PC3=$(start_node "$LC3" --node-id 0x0000000000000003 \
                         --udp-listen 127.0.0.1:23203 \
                         --udp-peer 127.0.0.1:23202 \
                         --udp-rate-mbps 0)

echo "  Nodes started (PIDs: $PA3 $PB3 $PC3). Waiting 15 s..."
sleep 15

check "A sees B as neighbor"   "$LA3" "NEIGHBOR_UP node=0000000000000002"
check "B sees A as neighbor"   "$LB3" "NEIGHBOR_UP node=0000000000000001"
check "B sees C as neighbor"   "$LB3" "NEIGHBOR_UP node=0000000000000003"
check "C sees B as neighbor"   "$LC3" "NEIGHBOR_UP node=0000000000000002"
# Chain invariant: the end nodes never neighbor each other directly.
check_absent "A does NOT see C (chain, not star)" \
  "$LA3" "NEIGHBOR_UP node=0000000000000003"
check_absent "C does NOT see A (chain, not star)" \
  "$LC3" "NEIGHBOR_UP node=0000000000000001"

kill_nodes "$PA3" "$PB3" "$PC3"
echo ""

# ---------------------------------------------------------------------------
# Test u3: oversize datagram dropped with a log line (REV-14 backstop)
# ---------------------------------------------------------------------------
echo "=== Test u3: oversize datagram drop is logged ==="
LD="$WORK/D.log"

PD=$(start_node "$LD" --node-id 0x0000000000000004 \
                       --udp-listen 127.0.0.1:23301 \
                       --udp-peer 127.0.0.1:23399 \
                       --udp-rate-mbps 0)
sleep 2

# 1600 B datagram (port byte 0x01 + 1599 frame bytes) > 1515 max.
python3 - <<'EOF'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b"\x01" + b"\xa5" * 1599, ("127.0.0.1", 23301))
EOF
sleep 2

check "oversize drop logged with length + port" \
  "$LD" "oversize datagram dropped (len=1600, ingress port=1"

kill_nodes "$PD"
echo ""

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo "=== Results: $PASS passed, $FAIL failed ==="
rm -rf "$WORK"
[[ $FAIL -eq 0 ]]
