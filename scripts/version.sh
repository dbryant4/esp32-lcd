#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_FILE="$PROJECT_ROOT/arduino/JC4827W543_LVGLv9/JC4827W543_LVGLv9.ino"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/version.sh show
  ./scripts/version.sh set <major.minor.patch>
  ./scripts/version.sh bump <major|minor|patch>
  ./scripts/version.sh new-chat

Notes:
  - Version source is FW_VERSION_MAJOR/MINOR/PATCH in the main .ino file.
  - new-chat increments minor and resets patch to 0.
EOF
}

read_define() {
  local name="$1"
  local value
  value="$(sed -nE "s/^#define[[:space:]]+${name}[[:space:]]+([0-9]+).*$/\1/p" "$TARGET_FILE" | head -n1)"
  if [[ -z "$value" ]]; then
    echo "Could not read ${name} from ${TARGET_FILE}" >&2
    exit 1
  fi
  printf '%s' "$value"
}

current_major() { read_define "FW_VERSION_MAJOR"; }
current_minor() { read_define "FW_VERSION_MINOR"; }
current_patch() { read_define "FW_VERSION_PATCH"; }

print_current() {
  echo "$(current_major).$(current_minor).$(current_patch)"
}

write_version() {
  local major="$1"
  local minor="$2"
  local patch="$3"
  local tmp_file
  tmp_file="$(mktemp)"

  awk -v major="$major" -v minor="$minor" -v patch="$patch" '
    /^#define[[:space:]]+FW_VERSION_MAJOR[[:space:]]+[0-9]+/ { print "#define FW_VERSION_MAJOR " major; next }
    /^#define[[:space:]]+FW_VERSION_MINOR[[:space:]]+[0-9]+/ { print "#define FW_VERSION_MINOR " minor; next }
    /^#define[[:space:]]+FW_VERSION_PATCH[[:space:]]+[0-9]+/ { print "#define FW_VERSION_PATCH " patch; next }
    { print }
  ' "$TARGET_FILE" > "$tmp_file"

  mv "$tmp_file" "$TARGET_FILE"
  echo "Updated version: ${major}.${minor}.${patch}"
}

set_version() {
  local version="$1"
  if [[ ! "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    echo "Invalid version '$version'. Expected format: major.minor.patch" >&2
    exit 1
  fi
  write_version "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}" "${BASH_REMATCH[3]}"
}

bump_version() {
  local part="$1"
  local major minor patch
  major="$(current_major)"
  minor="$(current_minor)"
  patch="$(current_patch)"

  case "$part" in
    major)
      major=$((major + 1))
      minor=0
      patch=0
      ;;
    minor)
      minor=$((minor + 1))
      patch=0
      ;;
    patch)
      patch=$((patch + 1))
      ;;
    *)
      echo "Unknown bump part '$part'. Use major, minor, or patch." >&2
      exit 1
      ;;
  esac

  write_version "$major" "$minor" "$patch"
}

main() {
  local command="${1:-show}"
  case "$command" in
    show)
      print_current
      ;;
    set)
      [[ $# -eq 2 ]] || { usage; exit 1; }
      set_version "$2"
      ;;
    bump)
      [[ $# -eq 2 ]] || { usage; exit 1; }
      bump_version "$2"
      ;;
    new-chat)
      bump_version "minor"
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
