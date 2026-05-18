#!/usr/bin/env bash

# Source this script from an activated virtual environment to set up the Force Dimension SDK.
#
# Usage:
#   source /path/to/omega_control/omega_control/scripts/omega_env.sh [SDK_ROOT]
#
# If SDK_ROOT is omitted, the script looks in common install locations and uses the first
# matching SDK root it finds.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed."
  return 1 2>/dev/null || exit 1
fi

_omega_env_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_omega_detect_sdk_root() {
  local provided_root="$1"

  if [[ -n "$provided_root" ]]; then
    printf '%s\n' "$provided_root"
    return 0
  fi

  if [[ -n "${FDSDK:-}" ]]; then
    printf '%s\n' "$FDSDK"
    return 0
  fi

  if [[ -n "${FORCEDIM_SDK:-}" ]]; then
    printf '%s\n' "$FORCEDIM_SDK"
    return 0
  fi

  local search_roots=(
    "$HOME/SDKs/ForceDimension"
    "$HOME/ForceDimension"
    "/opt/ForceDimension"
    "/usr/local/ForceDimension"
  )
  local candidates=()
  local root
  for root in "${search_roots[@]}"; do
    if [[ -d "$root" ]]; then
      for candidate in "$root"/sdk-*; do
        [[ -d "$candidate" ]] && candidates+=("$candidate")
      done
    fi
  done

  if [[ ${#candidates[@]} -eq 0 ]]; then
    return 1
  fi

  printf '%s\n' "${candidates[@]}" | sort -V | tail -n 1
}

_omega_library_subdir() {
  local system
  system="$(uname -s 2>/dev/null || printf '')"

  case "$system" in
    Linux)
      case "$(uname -m 2>/dev/null || printf '')" in
        x86_64|amd64)
          printf '%s\n' "lib/release/linux-x86_64-gcc"
          ;;
        aarch64|arm64)
          printf '%s\n' "lib/release/linux-aarch64-gcc"
          ;;
        armv7l|armv7*)
          printf '%s\n' "lib/release/linux-armv7-gcc"
          ;;
      esac
      ;;
    Darwin)
      case "$(uname -m 2>/dev/null || printf '')" in
        arm64|aarch64)
          printf '%s\n' "lib/release/mac-arm64-clang"
          ;;
        *)
          printf '%s\n' "lib/release/mac-x86_64-clang"
          ;;
      esac
      ;;
  esac
}

_omega_prepend_path() {
  local new_path="$1"
  local current_path="$2"

  if [[ -z "$new_path" ]]; then
    printf '%s\n' "$current_path"
    return 0
  fi

  case ":$current_path:" in
    *":$new_path:"*)
      printf '%s\n' "$current_path"
      ;;
    *)
      if [[ -n "$current_path" ]]; then
        printf '%s\n' "$new_path:$current_path"
      else
        printf '%s\n' "$new_path"
      fi
      ;;
  esac
}

omega_env_deactivate() {
  if [[ -n "${_OMEGA_ENV_ACTIVE:-}" ]]; then
    if [[ -n "${_OMEGA_ENV_OLD_FDSDK+x}" ]]; then
      export FDSDK="$_OMEGA_ENV_OLD_FDSDK"
    else
      unset FDSDK
    fi

    if [[ -n "${_OMEGA_ENV_OLD_FORCEDIM_SDK+x}" ]]; then
      export FORCEDIM_SDK="$_OMEGA_ENV_OLD_FORCEDIM_SDK"
    else
      unset FORCEDIM_SDK
    fi

    if [[ -n "${_OMEGA_ENV_OLD_LD_LIBRARY_PATH+x}" ]]; then
      export LD_LIBRARY_PATH="$_OMEGA_ENV_OLD_LD_LIBRARY_PATH"
    else
      unset LD_LIBRARY_PATH
    fi

    if [[ -n "${_OMEGA_ENV_OLD_DYLD_LIBRARY_PATH+x}" ]]; then
      export DYLD_LIBRARY_PATH="$_OMEGA_ENV_OLD_DYLD_LIBRARY_PATH"
    else
      unset DYLD_LIBRARY_PATH
    fi

    unset _OMEGA_ENV_ACTIVE
  fi
}

_omega_sdk_root="$(_omega_detect_sdk_root "${1:-}")"
if [[ -z "$_omega_sdk_root" ]]; then
  echo "omega_env.sh: unable to locate the Force Dimension SDK root." >&2
  echo "Set FDSDK or pass the SDK root as the first argument." >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -z "${_OMEGA_ENV_ACTIVE:-}" ]]; then
  export _OMEGA_ENV_ACTIVE=1
  export _OMEGA_ENV_OLD_FDSDK="${FDSDK-}"
  export _OMEGA_ENV_OLD_FORCEDIM_SDK="${FORCEDIM_SDK-}"
  export _OMEGA_ENV_OLD_LD_LIBRARY_PATH="${LD_LIBRARY_PATH-}"
  export _OMEGA_ENV_OLD_DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH-}"
fi

export FDSDK="$_omega_sdk_root"
export FORCEDIM_SDK="$_omega_sdk_root"

_omega_lib_dir="$_omega_sdk_root/$(_omega_library_subdir)"
if [[ -n "$_omega_lib_dir" && -d "$_omega_lib_dir" ]]; then
  case "$(uname -s 2>/dev/null || printf '')" in
    Linux)
      export LD_LIBRARY_PATH="$(_omega_prepend_path "$_omega_lib_dir" "${LD_LIBRARY_PATH-}")"
      ;;
    Darwin)
      export DYLD_LIBRARY_PATH="$(_omega_prepend_path "$_omega_lib_dir" "${DYLD_LIBRARY_PATH-}")"
      ;;
  esac
fi

_omega_bin_dir="$_omega_sdk_root/bin"
if [[ -d "$_omega_bin_dir" ]]; then
  export PATH="$(_omega_prepend_path "$_omega_bin_dir" "$PATH")"
fi

echo "omega_env.sh: configured Force Dimension SDK at $_omega_sdk_root"