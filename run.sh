#!/bin/bash
# Wrapper script for running the LLM robot agent with correct Go env
# Sets proxy for Chinese mainland users and uses local Go 1.23

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export GOPROXY=https://goproxy.cn,direct
export GOTOOLCHAIN=local
# Use local Go 1.23 to match go.mod requirement
GO_BIN="$HOME/go1.23/bin/go"
if [ -x "$GO_BIN" ]; then
    export PATH="$HOME/go1.23/bin:$PATH"
fi

cd "$SCRIPT_DIR"

# Load .env if present
if [ -f "$SCRIPT_DIR/.env" ]; then
    set -a
    source "$SCRIPT_DIR/.env"
    set +a
fi

if [ "$1" = "test" ]; then
    shift
    exec go run ./cmd/test_scheduler/ "$@"
fi

exec go run main.go "$@"
