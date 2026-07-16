"""FastAPI application for the read-only Go2 telemetry dashboard."""

from __future__ import annotations

import math
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, Response
from pydantic import BaseModel, Field

from . import __version__
from .ros_bridge import RosBridge, RosConfig
from .state import DashboardState


class SemanticPose(BaseModel):
    frame_id: str = Field(default="map", min_length=1, max_length=80)
    x: float
    y: float
    yaw: float


class SemanticTaskUpdate(BaseModel):
    """Status metadata only; this schema has no command or goal endpoint."""

    task_id: str = Field(default="", max_length=128)
    target_name: str = Field(default="", max_length=128)
    status: Literal[
        "idle",
        "received",
        "resolving",
        "resolved",
        "navigating",
        "succeeded",
        "canceled",
        "failed",
    ]
    message: str = Field(default="", max_length=500)
    pose: SemanticPose | None = None


def _model_dict(model: BaseModel) -> dict[str, Any]:
    if hasattr(model, "model_dump"):
        return model.model_dump()  # type: ignore[attr-defined,no-any-return]
    return model.dict()


def create_app(
    state: DashboardState | None = None,
    config: RosConfig | None = None,
) -> FastAPI:
    ros_config = config or RosConfig.from_environment()
    dashboard_state = state or DashboardState(ros_config.topic_specs())
    bridge = RosBridge(dashboard_state, ros_config)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        bridge.start()
        try:
            yield
        finally:
            bridge.stop()

    app = FastAPI(
        title="Robonix Go2 Read-only Dashboard",
        version=__version__,
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )
    app.state.dashboard_state = dashboard_state
    app.state.ros_bridge = bridge

    @app.get("/", response_class=HTMLResponse)
    async def dashboard() -> HTMLResponse:
        page = (
            Path(__file__).resolve().parent / "static" / "index.html"
        ).read_text(encoding="utf-8")
        return HTMLResponse(page, headers={"Cache-Control": "no-store"})

    @app.get("/healthz")
    async def health() -> JSONResponse:
        snapshot = dashboard_state.snapshot()
        return JSONResponse(
            {
                "ok": True,
                "read_only": True,
                "ros_connected": bool(snapshot["bridge"]["connected"]),
                "version": __version__,
            },
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/status")
    async def status() -> JSONResponse:
        return JSONResponse(
            dashboard_state.snapshot(), headers={"Cache-Control": "no-store"}
        )

    @app.get("/api/camera.jpg")
    async def camera() -> Response:
        image, sequence = dashboard_state.camera_image()
        if image is None:
            raise HTTPException(status_code=503, detail="camera frame unavailable")
        return Response(
            content=image,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store",
                "X-Telemetry-Sequence": str(sequence),
            },
        )

    @app.get("/api/map.png")
    async def map_preview() -> Response:
        image, sequence = dashboard_state.map_image()
        if image is None:
            raise HTTPException(status_code=503, detail="map unavailable")
        return Response(
            content=image,
            media_type="image/png",
            headers={
                "Cache-Control": "no-store",
                "X-Telemetry-Sequence": str(sequence),
            },
        )

    @app.get("/api/semantic-task")
    async def semantic_task() -> JSONResponse:
        return JSONResponse(
            dashboard_state.semantic_task(),
            headers={"Cache-Control": "no-store"},
        )

    @app.post("/api/semantic-task")
    async def update_semantic_task(update: SemanticTaskUpdate) -> JSONResponse:
        payload = _model_dict(update)
        pose = payload.get("pose")
        if pose is not None:
            numeric_values = (pose["x"], pose["y"], pose["yaw"])
            if not all(math.isfinite(float(value)) for value in numeric_values):
                raise HTTPException(
                    status_code=422, detail="pose values must be finite"
                )
        try:
            result = dashboard_state.update_semantic_task(payload)
        except (KeyError, TypeError, ValueError) as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        return JSONResponse(result, headers={"Cache-Control": "no-store"})

    return app
