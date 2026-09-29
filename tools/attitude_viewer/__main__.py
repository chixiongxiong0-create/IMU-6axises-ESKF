"""Run with ``py -m tools.attitude_viewer [first.csv [second.csv]]``."""

import sys

try:
    from .app import main
except ImportError as error:
    print(f"attitude viewer: {error}\nInstall dependencies: py -m pip install PyVista PyVistaQt PySide6", file=sys.stderr)
    raise SystemExit(1) from None


if __name__ == "__main__":
    raise SystemExit(main())
