from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlsplit

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from backend.data.session_store import (
    add_point,
    clear_blocks,
    create_session,
    delete_point,
    list_sessions,
    load_session,
    reconfigure_session,
    session_path,
    update_point,
    update_session,
)

_SESSION_RE = re.compile(r"^/api/sessions/([^/]+)$")
_POINT_RE = re.compile(r"^/api/sessions/([^/]+)/points/([^/]+)$")
_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}
_MAX_BODY = 1_048_576
_SERVER_HOST = "127.0.0.1"
_SERVER_PORT = 8765
_SSE_CLIENTS: dict[str, set[threading.Event]] = {}
_SSE_PENDING: dict[str, dict[str, Any]] = {}
_SSE_LOCK = threading.Lock()


class _HttpError(Exception):
    status = 500
    allow = ""


class _BadRequest(_HttpError):
    status = 400


class _NotFound(_HttpError):
    status = 404


class _MethodNotAllowed(_HttpError):
    status = 405


def _frontend_dirs() -> list[Path]:
    candidates = (_REPO_ROOT / "frontend", Path(__file__).resolve().parents[1] / "frontend")
    return [candidate.resolve() for candidate in candidates if candidate.is_dir()] or [candidates[0].resolve()]


def _allowed_origins() -> set[str]:
    origins = {f"http://{_SERVER_HOST}:{_SERVER_PORT}"}
    if _SERVER_HOST in {"0.0.0.0", "::", "localhost"}:
        origins.add(f"http://127.0.0.1:{_SERVER_PORT}")
        origins.add(f"http://localhost:{_SERVER_PORT}")
    return origins


def _register_client(session_id: str) -> threading.Event:
    event = threading.Event()
    with _SSE_LOCK:
        _SSE_CLIENTS.setdefault(session_id, set()).add(event)
    return event


def _unregister_client(session_id: str, event: threading.Event) -> None:
    with _SSE_LOCK:
        clients = _SSE_CLIENTS.get(session_id)
        if clients is not None:
            clients.discard(event)
            if not clients:
                _SSE_CLIENTS.pop(session_id, None)


def _notify(session_id: str, session: dict[str, Any]) -> None:
    with _SSE_LOCK:
        clients = _SSE_CLIENTS.get(session_id)
        if clients:
            _SSE_PENDING[session_id] = session
            for event in clients:
                event.set()


def _file_signature(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except OSError:
        return None
    return stat.st_mtime_ns, stat.st_size


class ServerHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _security_headers(self) -> None:
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")

    def _validate_browser_request(self, method: str) -> None:
        if method not in {"POST", "PATCH", "DELETE"}:
            return
        content_type = self.headers.get("Content-Type", "")
        if not content_type.lower().startswith("application/json"):
            raise _BadRequest("Content-Type must be application/json")
        allowed = _allowed_origins()
        origin = self.headers.get("Origin")
        if origin and origin not in allowed:
            raise _BadRequest("invalid Origin")
        referer = self.headers.get("Referer")
        if not origin and referer:
            parsed = urlsplit(referer)
            if f"{parsed.scheme}://{parsed.netloc}" not in allowed:
                raise _BadRequest("invalid Referer")

    def _send_json(self, status: int, payload: Any, extra: dict[str, str] | None = None) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self._security_headers()
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, status: int, content_type: str, data: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self._security_headers()
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict[str, Any]:
        raw_length = self.headers.get("Content-Length")
        if not raw_length:
            return {}
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise _BadRequest("invalid Content-Length") from exc
        if length < 0 or length > _MAX_BODY:
            raise _BadRequest("request body too large")
        raw = self.rfile.read(length)
        if not raw:
            return {}
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise _BadRequest("malformed JSON body") from exc
        if not isinstance(value, dict):
            raise _BadRequest("JSON body must be an object")
        return value

    def _serve_static(self, filename: str) -> None:
        for base in _frontend_dirs():
            candidate = (base / filename).resolve()
            if candidate.is_file() and candidate.parent == base:
                data = candidate.read_bytes()
                self._send_bytes(200, _MIME.get(candidate.suffix, "application/octet-stream"), data)
                return
        raise _NotFound("not found")

    def _commit(self, session_id: str, session: dict[str, Any]) -> None:
        _notify(session_id, session)
        self._send_json(200, session)

    def _route(self, method: str) -> None:
        parsed = urlsplit(self.path)
        path = unquote(parsed.path)
        if path == "/":
            path = "/index.html"

        if path in {"/index.html", "/app.js", "/styles.css"}:
            if method != "GET":
                raise _MethodNotAllowed("GET")
            self._serve_static(path.lstrip("/"))
            return

        if path == "/api/health":
            if method != "GET":
                raise _MethodNotAllowed("GET")
            self._send_json(200, {"ok": True})
            return

        if path == "/api/sessions":
            if method == "GET":
                self._send_json(200, {"sessions": list_sessions()})
                return
            if method == "POST":
                body = self._read_json()
                if "config" not in body:
                    raise _BadRequest("config is required")
                session = create_session(body["config"], body.get("name", "Field scan"))
                _notify(session["id"], session)
                self._send_json(201, session)
                return
            raise _MethodNotAllowed("GET, POST")

        if re.fullmatch(r"^/api/sessions/[^/]+/points$", path):
            session_id = path.split("/")[3]
            if method != "POST":
                raise _MethodNotAllowed("POST")
            self._commit(session_id, add_point(session_id, self._read_json()))
            return

        match = _POINT_RE.fullmatch(path)
        if match:
            session_id = match.group(1)
            point_id = match.group(2)
            if not _ID_RE.fullmatch(point_id):
                raise _BadRequest("invalid point id")
            if method == "PATCH":
                self._commit(session_id, update_point(session_id, point_id, self._read_json()))
                return
            if method == "DELETE":
                self._commit(session_id, delete_point(session_id, point_id))
                return
            raise _MethodNotAllowed("PATCH, DELETE")

        match = _SESSION_RE.fullmatch(path)
        if match:
            session_id = match.group(1)
            if method == "GET":
                self._send_json(200, load_session(session_id))
                return
            if method == "PATCH":
                body = self._read_json()
                allowed = {"name", "status"}
                unknown = set(body) - allowed
                if unknown:
                    raise _BadRequest(f"unsupported fields: {', '.join(sorted(unknown))}")
                def mutate(session: dict[str, Any]) -> dict[str, Any]:
                    if "name" in body:
                        if not isinstance(body["name"], str) or not body["name"]:
                            raise _BadRequest("name must be a non-empty string")
                        session["name"] = body["name"]
                    if "status" in body:
                        if body["status"] not in {"draft", "active", "complete"}:
                            raise _BadRequest("invalid status")
                        session["_force_status"] = body["status"]
                    return session

                self._commit(session_id, update_session(session_id, mutate))
                return
            raise _MethodNotAllowed("GET, PATCH")

        suffix_match = re.fullmatch(r"^/api/sessions/([^/]+)/([a-z_]+)$", path)
        if suffix_match:
            session_id, action = suffix_match.groups()
            if action == "reconfigure":
                if method != "POST":
                    raise _MethodNotAllowed("POST")
                body = self._read_json()
                config_patch = body.get("config", body)
                if not isinstance(config_patch, dict):
                    raise _BadRequest("config must be an object")
                self._commit(session_id, reconfigure_session(session_id, config_patch))
                return
            if action == "demo":
                if method != "POST":
                    raise _MethodNotAllowed("POST")
                self._commit(session_id, self._demo(session_id, self._read_json()))
                return
            if action == "events":
                if method != "GET":
                    raise _MethodNotAllowed("GET")
                self._stream_events(session_id)
                return
            raise _NotFound("not found")

        block_match = re.fullmatch(r"^/api/sessions/([^/]+)/blocks/(rescan|clear)$", path)
        if block_match:
            session_id, action = block_match.groups()
            if method != "POST":
                raise _MethodNotAllowed("POST")
            body = self._read_json()
            blocks = body.get("blocks")
            if not isinstance(blocks, list):
                raise _BadRequest("blocks must be a list")
            self._commit(session_id, clear_blocks(session_id, blocks, action == "rescan"))
            return

        if path.startswith("/api/"):
            raise _NotFound("not found")
        raise _NotFound("not found")

    def _demo(self, session_id: str, body: dict[str, Any]) -> dict[str, Any]:
        def mutate(session: dict[str, Any]) -> dict[str, Any]:
            config = session["config"]
            size = float(config["block_size_m"])
            rows = int(round(float(config["y_length_m"]) / size))
            cols = int(round(float(config["x_length_m"]) / size))
            row = body.get("block_row")
            col = body.get("block_col")
            if not isinstance(row, int) or isinstance(row, bool) or not isinstance(col, int) or isinstance(col, bool):
                raise _BadRequest("block_row and block_col must be integers")
            if not 0 <= row < rows or not 0 <= col < cols:
                raise _BadRequest("block row/col out of range")
            count = body.get("count", 1)
            if not isinstance(count, int) or isinstance(count, bool) or not 1 <= count <= 1000:
                raise _BadRequest("count must be between 1 and 1000")

            points = session.setdefault("points", [])
            if not isinstance(points, list):
                raise _BadRequest("invalid session points")
            existing = sum(1 for point in points if isinstance(point, dict) and point.get("block_row") == row and point.get("block_col") == col)
            for offset in range(count):
                index = existing + offset + 1
                fraction = (offset + 1) / (count + 1)
                points.append(
                    {
                        "id": f"demo-{index:08d}-{row:03d}-{col:03d}",
                        "block_row": row,
                        "block_col": col,
                        "scan_index": index,
                        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()),
                        "x_m": col * size + fraction * size,
                        "y_m": row * size + (1.0 - fraction) * size,
                        "depth_mm": 50.0 + 12.5 * index,
                        "lidar_distance_m": 1.5,
                        "theta_v_deg": 45.0,
                        "theta_h_deg": 0.0,
                        "imu_pitch_deg": 0.0,
                        "yaw_deg": 0.0,
                        "platform_x_m": col * size + fraction * size,
                        "platform_y_m": row * size + (1.0 - fraction) * size,
                        "reflector_height_m": float(config["reflector_height_m"]),
                        "accepted": True,
                        "source": "demo",
                        "raw": {"demo": True, "scan": index},
                    }
                )
            return session

        return update_session(session_id, mutate)

    def _write_event(self, session: dict[str, Any]) -> None:
        payload = json.dumps(session, ensure_ascii=True, separators=(",", ":"))
        self.wfile.write(f"event: session\ndata: {payload}\n\n".encode("utf-8"))
        self.wfile.flush()

    def _stream_events(self, session_id: str) -> None:
        session = load_session(session_id)
        path = session_path(session_id)
        event = _register_client(session_id)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self._security_headers()
        self.end_headers()
        self.close_connection = False
        signature = _file_signature(path)
        last_heartbeat = time.monotonic()
        try:
            self._write_event(session)
            while True:
                notified = event.wait(0.5)
                if notified:
                    event.clear()
                    with _SSE_LOCK:
                        pending = _SSE_PENDING.pop(session_id, None)
                    if pending is not None:
                        self._write_event(pending)
                        signature = _file_signature(path)
                current = _file_signature(path)
                if current != signature:
                    signature = current
                    if current is not None:
                        try:
                            self._write_event(load_session(session_id))
                        except (ValueError, OSError):
                            pass
                if time.monotonic() - last_heartbeat >= 15:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                    last_heartbeat = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            _unregister_client(session_id, event)
            self.close_connection = True

    def do_GET(self) -> None:
        self._handle("GET")

    def do_POST(self) -> None:
        self._handle("POST")

    def do_PATCH(self) -> None:
        self._handle("PATCH")

    def do_DELETE(self) -> None:
        self._handle("DELETE")

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self._security_headers()
        self.end_headers()

    def do_PUT(self) -> None:
        self._send_json(405, {"error": "method not allowed"}, {"Allow": "GET, POST, PATCH, DELETE, OPTIONS"})

    def do_HEAD(self) -> None:
        self._send_json(405, {"error": "method not allowed"}, {"Allow": "GET, POST, PATCH, DELETE, OPTIONS"})

    def _handle(self, method: str) -> None:
        try:
            self._validate_browser_request(method)
            self._route(method)
        except _HttpError as exc:
            extra = {"Allow": exc.allow} if exc.allow else None
            self._send_json(exc.status, {"error": str(exc)}, extra)
        except (FileNotFoundError, KeyError) as exc:
            self._send_json(404, {"error": str(exc)})
        except (ValueError, TypeError) as exc:
            self._send_json(400, {"error": str(exc)})
        except Exception as exc:
            self._send_json(500, {"error": str(exc)})


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Surface Level Mapping local API server.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    global _SERVER_HOST, _SERVER_PORT
    _SERVER_HOST = args.host
    _SERVER_PORT = args.port
    server = Server((args.host, args.port), ServerHandler)
    print(f"serving on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
