#!/usr/bin/env bash
# follow — 'tail -F' the most recently modified file matching a path or path prefix.
#
#   follow logs/          newest file anywhere under logs/ (recursive)
#   follow logs/run_      newest file whose path starts with logs/run_
#   follow logs/app.log   exactly that file
#   follow run_           same as ./run_
#
# An existing file or directory is used as-is; anything else is treated as a
# prefix, and matching directories are searched recursively.

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
    n) lines="$OPTARG" ;;
    p) print_only=1 ;;
    h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
(( $# <= 1 )) || { usage >&2; exit 2; }
target=${1:-.}

shopt -s nullglob
if [[ -e $target ]]; then
  roots=("$target")
else
  roots=("$target"*)   # quoted part is literal; only the trailing * is a glob
fi

# Newest regular file under the roots. NUL-separated so any filename is safe;
# LC_ALL=C so sub-second mtimes sort correctly in comma-decimal locales (fi_FI).
newest=
if (( ${#roots[@]} )); then
  IFS= read -r -d '' newest < <(
    find -H "${roots[@]}" -type f -printf '%T@\t%p\0' 2>/dev/null |
      LC_ALL=C sort -z -rn | head -z -n 1
  ) || true
fi
[[ -n $newest ]] || { echo "follow: no files match '$target'" >&2; exit 1; }
newest=${newest#*$'\t'}

if (( print_only )); then
  printf '%s\n' "$newest"
  exit 0
fi

echo "==> $newest" >&2
exec tail -n "${lines}" -F -- "$newest"
