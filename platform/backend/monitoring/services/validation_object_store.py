from __future__ import annotations

import hashlib
import shutil
import uuid
from pathlib import Path
from typing import BinaryIO

from django.conf import settings


class ObjectStoreError(RuntimeError):
    pass


class ValidationObjectStore:
    """Small S3/local abstraction used only for immutable validation blobs."""

    def __init__(self) -> None:
        self.backend = settings.VALIDATION_OBJECT_STORE_BACKEND
        self.bucket = settings.VALIDATION_OBJECT_STORE_BUCKET
        self.root = Path(settings.VALIDATION_OBJECT_STORE_ROOT).resolve()
        self._client = None
        self._public_client = None

    def _s3_client(self, *, public: bool = False):
        if self.backend != "s3":
            raise ObjectStoreError("S3 object store is not enabled")
        attribute = "_public_client" if public else "_client"
        cached = getattr(self, attribute)
        if cached is not None:
            return cached
        try:
            import boto3
            from botocore.config import Config
        except ImportError as exc:  # pragma: no cover - deployment dependency
            raise ObjectStoreError("boto3 is required for the S3 validation object store") from exc
        endpoint = (
            settings.VALIDATION_OBJECT_STORE_PUBLIC_ENDPOINT
            if public and settings.VALIDATION_OBJECT_STORE_PUBLIC_ENDPOINT
            else settings.VALIDATION_OBJECT_STORE_ENDPOINT
        )
        client = boto3.client(
            "s3",
            endpoint_url=endpoint or None,
            aws_access_key_id=settings.VALIDATION_OBJECT_STORE_ACCESS_KEY or None,
            aws_secret_access_key=settings.VALIDATION_OBJECT_STORE_SECRET_KEY or None,
            region_name=settings.VALIDATION_OBJECT_STORE_REGION,
            config=Config(signature_version="s3v4", s3={"addressing_style": "path"}),
        )
        setattr(self, attribute, client)
        return client

    def ensure_bucket(self) -> None:
        if self.backend == "local":
            self.root.mkdir(parents=True, exist_ok=True)
            return
        client = self._s3_client()
        try:
            client.head_bucket(Bucket=self.bucket)
        except Exception:
            try:
                client.create_bucket(Bucket=self.bucket)
            except Exception as exc:  # pragma: no cover - depends on object store
                raise ObjectStoreError(f"cannot create validation bucket {self.bucket}: {exc}") from exc

    def _local_path(self, object_key: str) -> Path:
        candidate = (self.root / object_key).resolve()
        if candidate != self.root and self.root not in candidate.parents:
            raise ObjectStoreError("object key escapes validation storage root")
        return candidate

    def save_stream(self, object_key: str, stream: BinaryIO) -> tuple[int, str]:
        """Stream an upload to local storage; production clients use S3 multipart."""
        if self.backend != "local":
            raise ObjectStoreError("direct uploads are disabled for the S3 backend")
        self.ensure_bucket()
        target = self._local_path(object_key)
        target.parent.mkdir(parents=True, exist_ok=True)
        staging = target.with_name(f".{target.name}.partial-{uuid.uuid4().hex}")
        digest = hashlib.sha256()
        size = 0
        with staging.open("wb") as output:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                output.write(chunk)
                digest.update(chunk)
                size += len(chunk)
        staging.replace(target)
        return size, digest.hexdigest()

    def open_local(self, object_key: str):
        if self.backend != "local":
            raise ObjectStoreError("object is not stored on the local backend")
        return self._local_path(object_key).open("rb")

    def local_path(self, object_key: str) -> Path:
        if self.backend != "local":
            raise ObjectStoreError("object is not stored on the local backend")
        return self._local_path(object_key)

    def delete(self, object_key: str) -> None:
        if not object_key:
            return
        if self.backend == "local":
            path = self._local_path(object_key)
            if path.is_file():
                path.unlink()
            return
        self._s3_client().delete_object(Bucket=self.bucket, Key=object_key)

    def head(self, object_key: str) -> dict:
        if self.backend == "local":
            path = self._local_path(object_key)
            if not path.is_file():
                raise ObjectStoreError(f"object not found: {object_key}")
            return {"size_bytes": path.stat().st_size, "content_type": "application/octet-stream"}
        try:
            result = self._s3_client().head_object(Bucket=self.bucket, Key=object_key)
        except Exception as exc:  # pragma: no cover - depends on object store
            raise ObjectStoreError(f"cannot inspect object {object_key}: {exc}") from exc
        return {
            "size_bytes": int(result.get("ContentLength") or 0),
            "content_type": result.get("ContentType") or "application/octet-stream",
            "etag": str(result.get("ETag") or "").strip('"'),
        }

    def download_url(self, object_key: str) -> str | None:
        if self.backend == "local":
            return None
        try:
            return self._s3_client(public=True).generate_presigned_url(
                "get_object",
                Params={"Bucket": self.bucket, "Key": object_key},
                ExpiresIn=settings.VALIDATION_SIGNED_URL_TTL_SECONDS,
            )
        except Exception as exc:  # pragma: no cover - depends on object store
            raise ObjectStoreError(f"cannot sign download for {object_key}: {exc}") from exc

    def upload_url(self, object_key: str, content_type: str) -> str:
        if self.backend != "s3":
            raise ObjectStoreError("pre-signed upload requires the S3 backend")
        self.ensure_bucket()
        try:
            return self._s3_client(public=True).generate_presigned_url(
                "put_object",
                Params={"Bucket": self.bucket, "Key": object_key, "ContentType": content_type},
                ExpiresIn=settings.VALIDATION_SIGNED_URL_TTL_SECONDS,
            )
        except Exception as exc:  # pragma: no cover - depends on object store
            raise ObjectStoreError(f"cannot sign upload for {object_key}: {exc}") from exc

    def initiate_multipart(self, object_key: str, content_type: str) -> str:
        if self.backend != "s3":
            raise ObjectStoreError("multipart upload requires the S3 backend")
        self.ensure_bucket()
        result = self._s3_client().create_multipart_upload(
            Bucket=self.bucket, Key=object_key, ContentType=content_type
        )
        return str(result["UploadId"])

    def presign_parts(self, object_key: str, upload_id: str, part_numbers: list[int]) -> list[dict]:
        if self.backend != "s3":
            raise ObjectStoreError("multipart upload requires the S3 backend")
        client = self._s3_client(public=True)
        result = []
        for part_number in part_numbers:
            if part_number < 1 or part_number > 10000:
                raise ObjectStoreError("part_number must be between 1 and 10000")
            result.append(
                {
                    "part_number": part_number,
                    "url": client.generate_presigned_url(
                        "upload_part",
                        Params={
                            "Bucket": self.bucket,
                            "Key": object_key,
                            "UploadId": upload_id,
                            "PartNumber": part_number,
                        },
                        ExpiresIn=settings.VALIDATION_SIGNED_URL_TTL_SECONDS,
                    ),
                }
            )
        return result

    def complete_multipart(self, object_key: str, upload_id: str, parts: list[dict]) -> dict:
        if self.backend != "s3":
            raise ObjectStoreError("multipart upload requires the S3 backend")
        normalized = [
            {"PartNumber": int(part["part_number"]), "ETag": str(part["etag"]).strip()}
            for part in sorted(parts, key=lambda item: int(item["part_number"]))
        ]
        if not normalized:
            raise ObjectStoreError("at least one uploaded part is required")
        try:
            self._s3_client().complete_multipart_upload(
                Bucket=self.bucket,
                Key=object_key,
                UploadId=upload_id,
                MultipartUpload={"Parts": normalized},
            )
        except Exception as exc:  # pragma: no cover - depends on object store
            raise ObjectStoreError(f"cannot complete multipart upload: {exc}") from exc
        return self.head(object_key)

    def copy_local(self, source: Path, object_key: str) -> tuple[int, str]:
        """Test/developer helper that keeps the immutable rename semantics."""
        with source.open("rb") as stream:
            return self.save_stream(object_key, stream)


object_store = ValidationObjectStore()
