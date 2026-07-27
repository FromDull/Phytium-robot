#!/usr/bin/env bash
# Run a development command and mirror its command line and output to a log
# that can be followed from another SSH terminal.
set -u

LOG_FILE="${PHYTIUM_DEV_LOG:-/tmp/phytium-codex.log}"

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 command [argument ...]" >&2
  exit 64
fi

{
  printf '\n[%s] codex@phytiumpi$ ' "$(date '+%F %T')"
  printf '%q ' "$@"
  printf '\n'
  "$@"
  status=$?
  printf '[exit %s]\n' "$status"
} >>"$LOG_FILE" 2>&1

exit "$status"
