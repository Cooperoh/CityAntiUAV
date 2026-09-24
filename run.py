"""Serve the built frontend locally. All simulation runs in the browser."""
from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import webbrowser


class FrontendHandler(SimpleHTTPRequestHandler):
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".css": "text/css",
        ".json": "application/json",
        ".svg": "image/svg+xml",
    }

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()


def main() -> None:
    parser = argparse.ArgumentParser(description="Start City Pursuit on localhost.")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent / "dist"
    if not (root / "index.html").is_file():
        raise SystemExit("Frontend build missing. Run: pnpm install && pnpm run build")
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(FrontendHandler, directory=str(root)))
    except OSError as error:
        raise SystemExit(f"Cannot start local server on port {args.port}: {error}\nTry: python run.py --port 8766") from error
    address = f"http://127.0.0.1:{server.server_port}/"
    print(f"City Pursuit: {address}\nPress Ctrl+C to stop.", flush=True)
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(address)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
