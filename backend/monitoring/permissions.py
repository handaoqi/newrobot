from django.contrib.auth.hashers import check_password
from rest_framework.permissions import BasePermission

from .models import RobotCredential


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
