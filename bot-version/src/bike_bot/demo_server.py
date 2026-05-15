from __future__ import annotations

import argparse
from typing import Any

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

app = FastAPI(title="Bike Bot Demo Server")
EVENTS: list[dict[str, Any]] = []


@app.post("/api/telemetry/ingest/")
async def ingest(payload: dict[str, Any], request: Request) -> JSONResponse:
    EVENTS.append(
        {
            "headers": dict(request.headers),
            "payload": payload,
        }
    )
    return JSONResponse(
        {
            "ok": True,
            "received_sequence_id": payload.get("sequence_id"),
            "detections": len(payload.get("detections") or []),
        }
    )


@app.get("/api/telemetry/events/")
async def events() -> dict[str, Any]:
    return {"count": len(EVENTS), "items": EVENTS[-50:]}


@app.get("/healthz")
async def healthz() -> dict[str, str]:
    return {"status": "ok"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Demo telemetry receiver.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    return parser.parse_args()


def main() -> None:
    app.mount("/static", StaticFiles(directory="."), name="static")
    args = parse_args()
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
