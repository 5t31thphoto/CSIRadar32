# ═══════════════════════════════════════════════════════════════
#  retry <cmd...>  — for every step in CI that touches the network
# ═══════════════════════════════════════════════════════════════
#  Every external fetch in this pipeline (board index, core, libraries,
#  pip, espup, the GitHub API, crates.io) has failed transiently at some
#  point on shared runners.  A transient failure must cost a retry, not a
#  red build.  A REAL failure still fails: after four tries, with the
#  command named in the log.
#  Usage:  source tools/ci_retry.sh; retry arduino-cli core update-index
retry() {
  local n=0 max=${RETRY_MAX:-4}
  until "$@"; do
    n=$((n + 1))
    if [ "$n" -ge "$max" ]; then
      echo "::error::failed after $max attempts: $*"
      return 1
    fi
    echo "::warning::attempt $n failed, retrying in $((n * 15))s: $*"
    sleep $((n * 15))
  done
}
