#!/usr/bin/env bash
# picam-remote.sh — run on your own (client) machine to view / snapshot a Pi
# camera that is only reachable through the laguna PC.
#
#   client --ssh--> laguna --ssh--> pi      (commands)
#   client <------- laguna <------- pi      (image bytes, over the ssh pipes)
#
# Usage:
#   picam-remote.sh list
#   picam-remote.sh view <camera> [laguna-picam stream options]
#   picam-remote.sh snap <camera> [-o local_file] [laguna-picam snap options]
#
# <camera> is a hostname or a 1-based index into pi_cameras.hosts.
#
# Environment:
#   LAGUNA_SSH_HOST  ssh host/alias of the laguna PC            (default: laguna)
#   LAGUNA_PICAM     command to run laguna-picam on laguna       (default: laguna-picam)
#                    A non-interactive ssh does not activate conda, so you will
#                    probably need the full path, e.g.
#                    ~/miniconda3/envs/flumelab/bin/laguna-picam
#   LAGUNA_CONFIG    config path on laguna, forwarded for you     (optional)
#
# Make it a short command with:  alias picam='/path/to/laguna/scripts/picam-remote.sh'
# Needs ffplay (ffmpeg) or mpv on the client for `view`.
#
# The Pi camera is exclusive: do not leave `view` open when a scheduled
# capture is due, or that capture fails and cannot be re-taken.

set -euo pipefail

LAGUNA_SSH_HOST="${LAGUNA_SSH_HOST:-laguna}"
LAGUNA_PICAM="${LAGUNA_PICAM:-laguna-picam}"

usage() { sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-2}"; }
[ $# -ge 1 ] || usage

action="$1"; shift
case "$action" in
  -h|--help|help) usage 0 ;;
  list|snap|view) ;;
  *) echo "unknown action: $action" >&2; usage ;;
esac

# `view` is `stream` on laguna, piped into a local player.
remote_action="$action"; [ "$action" = view ] && remote_action=stream

# Quote each arg so spaces/globs survive the shell on laguna.
quoted=""
for a in "$@"; do quoted+=" $(printf '%q' "$a")"; done
cfg=""
[ -n "${LAGUNA_CONFIG:-}" ] && cfg="--config $(printf '%q' "$LAGUNA_CONFIG")"

# `snap`: image comes back on stdout and is saved HERE on the client, never on
# laguna. -o/--output picks the local file; default is <camera>_<utc>.jpg.
if [ "$action" = snap ]; then
  cam="${1:?snap needs a camera}"; shift
  out=""; rest=""
  while [ $# -gt 0 ]; do
    case "$1" in
      -o|--output) out="${2:?-o needs a file}"; shift 2 ;;
      *) rest+=" $(printf '%q' "$1")"; shift ;;
    esac
  done
  [ -n "$out" ] || out="${cam}_$(date -u +%Y%m%dT%H%M%SZ).jpg"
  # shellcheck disable=SC2086
  ssh "$LAGUNA_SSH_HOST" "$LAGUNA_PICAM $cfg snap $(printf '%q' "$cam") -o -$rest" > "$out.part" || { rm -f "$out.part"; exit 1; }
  mv "$out.part" "$out"
  echo "saved $out" >&2
  exit 0
fi

# shellcheck disable=SC2086
if [ "$action" = view ]; then
  if command -v ffplay >/dev/null; then
    player=(ffplay -loglevel error -fflags nobuffer -flags low_delay -f mjpeg -i -)
  elif command -v mpv >/dev/null; then
    player=(mpv --profile=low-latency --demuxer-lavf-format=mjpeg -)
  else
    echo "need ffplay or mpv on this machine" >&2; exit 1
  fi
  ssh "$LAGUNA_SSH_HOST" "$LAGUNA_PICAM $cfg $remote_action$quoted" | "${player[@]}"
else
  ssh "$LAGUNA_SSH_HOST" "$LAGUNA_PICAM $cfg $remote_action$quoted"
fi
