#!/usr/bin/env bash
# follow — tail -F the most recently modified file matching a path or prefix.
#
#   follow logs/          newest file directly in logs/
#   follow logs/run_      newest file in logs/ whose name starts with run_
#   follow logs/app.log   exactly that file
#   follow run_           same as ./run_

set -euo pipefail

usage() {
  echo "Usage: follow [-n LINES] [-p] [PATH_OR_PREFIX]    (default: current directory)"
  echo "  -n LINES  lines to show initially (default: 200)"
  echo "  -p        print the chosen file and exit instead of tailing it"
}

lines=200
print_only=0
while getopts ':n:ph' opt; do
  case $opt in
    n) lines=$OPTARG ;;
    p) print_only=1 ;;
    h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
(( $# <= 1 )) || { usage >&2; exit 2; }
target=${1:-.}

shopt -s nullglob dotglob
if [[ -f $target ]]; then
  candidates=("$target")
elif [[ -d $target ]]; then
  candidates=("${target%/}"/*)
else
  candidates=("$target"*)   # quoted part is literal; only the trailing * is a glob
fi

newest=
for f in "${candidates[@]}"; do
  if [[ -f $f && ( -z $newest || $f -nt $newest ) ]]; then
    newest=$f
  fi
done
[[ -n $newest ]] || { echo "follow: no files match '$target'" >&2; exit 1; }

if (( print_only )); then
  printf '%s\n' "$newest"
  exit 0
fi

echo "==> $newest" >&2
exec tail -n "$lines" -F -- "$newest"