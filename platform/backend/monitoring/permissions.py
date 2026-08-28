import hmac

from django.contrib.auth.hashers import check_password
from django.conf import settings
from rest_framework.permissions import BasePermission

from .models import Robot, RobotCredential


class IsAuthenticatedOrDeviceCredential(BasePermission):
    message = "需要用户 Token 或有效设备凭证"

    def has_permission(self, request, view):
        if request.user and request.user.is_authenticated:
            return True
        credential_id = request.headers.get("X-Device-Id", "").strip()
        secret = request.headers.get("X-Device-Key", "")
        if not credential_id or not secret:
            return False
        credential = (
            RobotCredential.objects.select_related("robot")
            .filter(credential_id=credential_id, status="active")
            .first()
        )
        if not credential or not credential.secret_hash or not check_password(secret, credential.secret_hash):
            return False
        request.device_robot = credential.robot
        return True


class IsAudioDeviceCredential(IsAuthenticatedOrDeviceCredential):
    """Require device credentials in production, with an unprovisioned DEBUG fallback."""

    def has_permission(self, request, view):
        if super().has_permission(request, view):
            return True
        if not settings.DEBUG:
            return False
        robot_code = (
            request.headers.get("X-Device-Code", "").strip()
            or request.query_params.get("robot_code", "").strip()
        )
        if not robot_code:
            return False
        robot = Robot.objects.filter(code=robot_code).first()
        if robot is None or robot.credentials.filter(status="active").exists():
            return False
        request.device_robot = robot
        return True


class IsValidationRunner(BasePermission):
    message = "需要有效的 Validation Runner 凭证"

    def has_permission(self, request, view):
        expected = str(getattr(settings, "VALIDATION_RUNNER_TOKEN", ""))
        received = request.headers.get("X-Validation-Runner-Token", "")
        return bool(expected and received and hmac.compare_digest(expected, received))


class IsValidationGateway(BasePermission):
    message = "需要有效的 Validation Gateway 凭证"

    def has_permission(self, request, view):
        expected = str(getattr(settings, "VALIDATION_GATEWAY_TOKEN", ""))
        received = request.headers.get("X-Validation-Gateway-Token", "")
        return bool(expected and received and hmac.compare_digest(expected, received))
