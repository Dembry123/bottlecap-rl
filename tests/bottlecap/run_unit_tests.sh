#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CC="${CC:-cc}"
"$CC" -std=c11 -O2 -Wall -Wextra -I "$ROOT/ocean/bottlecap"   -o /tmp/test_g0_fk "$ROOT/tests/bottlecap/test_gradient0_fk.c" -lm
/tmp/test_g0_fk
"$CC" -std=c11 -O2 -Wall -Wextra -I "$ROOT/ocean/bottlecap"   -o /tmp/test_helix "$ROOT/tests/bottlecap/test_helix.c" -lm
/tmp/test_helix
echo "bottlecap unit tests OK"
