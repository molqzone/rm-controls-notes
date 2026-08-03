#!/usr/bin/env bash
#
# push_articles.sh — stage, review, commit, and push the rm-controls-notes
# articles to the remote (origin/main).
#
# Usage:
#   ./push_articles.sh                       # default commit message, asks to confirm
#   ./push_articles.sh "docs: rewrite gimbal" # custom commit message
#   ./push_articles.sh -y                     # skip the confirmation prompt
#   ./push_articles.sh "msg" -y               # both
#
# Only the published article files below are staged. Working notes
# (draft.md, ideas.md) and this script itself are intentionally left out —
# add them to the ARTICLES list if you want them pushed too.

set -euo pipefail

# Always operate on the repo this script lives in.
cd "$(dirname "$0")"

ARTICLES=(
  README.md
  overview.md
  communication.md
  hardware.md
  transform.md
  chassis.md
  gimbal.md
  shooter.md
  manual.md
)

# ---- parse args: any non-flag word is the commit message; -y/--yes skips prompt
MSG=""
ASSUME_YES="no"
for arg in "$@"; do
  case "$arg" in
    -y|--yes) ASSUME_YES="yes" ;;
    *)        MSG="$arg" ;;
  esac
done
MSG="${MSG:-docs: update rm-controls-notes articles}"

BRANCH="$(git rev-parse --abbrev-ref HEAD)"

# ---- stage only the article files that actually exist
to_add=()
for f in "${ARTICLES[@]}"; do
  [[ -f "$f" ]] && to_add+=("$f")
done
if [[ ${#to_add[@]} -eq 0 ]]; then
  echo "No article files found in $(pwd)." >&2
  exit 1
fi
git add -- "${to_add[@]}"

# ---- show what will be pushed (the git diff, staged vs HEAD)
echo "== Articles staged for commit (diff vs HEAD) =="
git --no-pager diff --staged --stat
echo
# Uncomment the next line to review the full line-by-line diff before pushing:
# git --no-pager diff --staged

if git diff --staged --quiet; then
  echo "Nothing to commit — articles already match HEAD."
  exit 0
fi

# ---- confirm
if [[ "$ASSUME_YES" != "yes" ]]; then
  read -r -p "Commit the above and push to origin/${BRANCH}? [y/N] " ans
  case "$ans" in
    y|Y) : ;;
    *)   echo "Aborted. (Changes remain staged; run 'git restore --staged .' to unstage.)"; exit 1 ;;
  esac
fi

# ---- commit and push
git commit -m "$MSG"
git push origin "$BRANCH"
echo "Done — pushed to origin/${BRANCH}."
