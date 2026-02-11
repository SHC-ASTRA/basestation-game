#!/bin/sh
echo -ne '\033c\033]0;basestation-game\a'
base_path="$(dirname "$(realpath "$0")")"
"$base_path/basestationgame.x86_64" "$@"
