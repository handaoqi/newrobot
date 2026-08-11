from __future__ import annotations

import json
import time

from django.db.models import Max
from django.http import HttpResponseForbidden, StreamingHttpResponse
from django.shortcuts import get_object_or_404
from django.utils import timezone
from rest_framework import status
from rest_framework.authtoken.models import Token
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import DevelopmentAgentState, DevelopmentTask, Robot


ALLOWED_WORKSPACES = {"robot-main", "cloud-platform"}


def serialize_event(event) -> dict:
    return {
        "id": event.id,
        "sequence": event.sequence,
        "type": event.event_type,
        "stream": event.stream,
        "text": event.text,
        "payload": event.payload,
        "occurred_at": event.occurred_at,
    }


def serialize_task(task: DevelopmentTask, *, include_events: bool = False) -> dict:
    data = {
        "id": str(task.id),
        "robot": task.robot_id,
        "robot_code": task.robot.code,
        "robot_name": task.robot.name,
        "workspace": task.workspace,
        "prompt": task.prompt,
        "status": task.status,
        "status_label": task.get_status_display(),
        "created_at": task.created_at,
        "published_at": task.published_at,
        "started_at": task.started_at,
        "finished_at": task.finished_at,
        "exit_code": task.exit_code,
        "error_message": task.error_message,
        "last_message": task.last_message,
        "codex_thread_id": task.codex_thread_id,
        "operator": task.operator_id,
        "can_cancel": task.status in DevelopmentTask.ACTIVE_STATES,
    }
    if include_events:
        data["events"] = [serialize_event(event) for event in task.events.order_by("sequence")[:2000]]
    return data


class DevelopmentTaskListCreateView(APIView):
    def get(self, request):
        queryset = DevelopmentTask.objects.select_related("robot", "operator")
        robot_id = request.query_params.get("robot")
        if robot_id:
            queryset = queryset.filter(robot_id=robot_id)
        return Response([serialize_task(task) for task in queryset[:100]])

    def post(self, request):
        robot = get_object_or_404(Robot, pk=request.data.get("robot"))
        workspace = str(request.data.get("workspace") or "robot-main").strip()
        prompt = str(request.data.get("prompt") or "").strip()
        if workspace not in ALLOWED_WORKSPACES:
            return Response({"detail": "不允许的工作目录"}, status=status.HTTP_400_BAD_REQUEST)
        if not prompt:
            return Response({"detail": "请输入开发指令"}, status=status.HTTP_400_BAD_REQUEST)
        if len(prompt) > 50000:
            return Response({"detail": "指令长度不能超过 50000 字符"}, status=status.HTTP_400_BAD_REQUEST)
        active = DevelopmentTask.objects.filter(
            robot=robot,
            status__in=DevelopmentTask.ACTIVE_STATES,
        ).first()
        if active:
            return Response(
                {"detail": "该机器狗已有远程开发任务正在执行", "active_task_id": str(active.id)},
                status=status.HTTP_409_CONFLICT,
            )
        task = DevelopmentTask.objects.create(
            robot=robot,
            workspace=workspace,
            prompt=prompt,
            operator=request.user,
        )
        return Response(serialize_task(task), status=status.HTTP_201_CREATED)


class DevelopmentTaskDetailView(APIView):
    def get(self, request, task_id):
        task = get_object_or_404(
            DevelopmentTask.objects.select_related("robot", "operator"),
            pk=task_id,
        )
        return Response(serialize_task(task, include_events=True))


class DevelopmentTaskCancelView(APIView):
    def post(self, request, task_id):
        task = get_object_or_404(DevelopmentTask.objects.select_related("robot"), pk=task_id)
        if task.status in DevelopmentTask.TERMINAL_STATES:
            return Response(serialize_task(task))
        if task.status == "created":
            task.status = "cancelled"
            task.cancel_requested_at = timezone.now()
            task.finished_at = timezone.now()
            task.save(update_fields=["status", "cancel_requested_at", "finished_at", "updated_at"])
        else:
            task.status = "cancelling"
            task.cancel_requested_at = timezone.now()
            task.cancel_published_at = None
            task.save(update_fields=["status", "cancel_requested_at", "cancel_published_at", "updated_at"])
        return Response(serialize_task(task))


class DevelopmentAgentListView(APIView):
    def get(self, request):
        states = DevelopmentAgentState.objects.select_related("robot").order_by("robot__code")
        now = timezone.now()
        result = []
        for item in states:
            effective_status = item.status
            if now - item.last_seen_at > timezone.timedelta(seconds=35):
                effective_status = "offline"
            result.append({
                "robot": item.robot_id,
                "robot_code": item.robot.code,
                "status": effective_status,
                "agent_version": item.agent_version,
                "workspaces": item.workspaces,
                "last_seen_at": item.last_seen_at,
            })
        return Response(result)


def development_task_stream(request, task_id):
    token = request.GET.get("token", "")
    if not token or not Token.objects.filter(key=token).exists():
        return HttpResponseForbidden("invalid token")
    task = get_object_or_404(DevelopmentTask, pk=task_id)
    try:
        after = max(0, int(request.GET.get("after", 0)))
    except ValueError:
        after = 0

    def stream():
        last_sequence = after
        last_status = ""
        idle_ticks = 0
        while True:
            events = list(task.events.filter(sequence__gt=last_sequence).order_by("sequence")[:200])
            for event in events:
                last_sequence = event.sequence
                payload = json.dumps(serialize_event(event), ensure_ascii=False, default=str)
                yield f"id: {event.sequence}\nevent: dev_output\ndata: {payload}\n\n"
            task.refresh_from_db()
            if task.status != last_status:
                last_status = task.status
                payload = json.dumps(serialize_task(task), ensure_ascii=False, default=str)
                yield f"event: dev_status\ndata: {payload}\n\n"
            if task.status in DevelopmentTask.TERMINAL_STATES and not events:
                return
            idle_ticks += 1
            if idle_ticks % 30 == 0:
                yield ": keepalive\n\n"
            time.sleep(0.5)

    response = StreamingHttpResponse(stream(), content_type="text/event-stream")
    response["Cache-Control"] = "no-cache"
    response["X-Accel-Buffering"] = "no"
    return response
