import io

import pytest
import requests

from roamerx_edge.config import MediaConfig
from roamerx_edge.media_client import MediaClient
from roamerx_edge.protocol import ProtocolError


def test_upload_map_package_uses_configured_timeout(tmp_path, monkeypatch):
    package = tmp_path / "map_package.zip"
    package.write_bytes(b"zip")
    seen = {}

    def fake_post(*_args, **kwargs):
        seen["timeout"] = kwargs["timeout"]
        response = requests.Response()
        response.status_code = 200
        response._content = b'{"id": 1}'
        response.raw = io.BytesIO(response._content)
        return response

    monkeypatch.setattr(requests, "post", fake_post)
    client = MediaClient(
        MediaConfig(map_upload_url="http://center.example/upload", map_upload_timeout_seconds=1800),
        "ZSL-1A-07",
    )

    assert client.upload_map_package(str(package), {"map_name": "indoor"}) == {"id": 1}
    assert seen["timeout"] == (30, 1800)


def test_upload_map_package_timeout_becomes_protocol_error(tmp_path, monkeypatch):
    package = tmp_path / "map_package.zip"
    package.write_bytes(b"zip")
    monkeypatch.setattr(
        requests,
        "post",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            requests.exceptions.ConnectionError("Connection aborted.", TimeoutError("timed out"))
        ),
    )
    client = MediaClient(MediaConfig(map_upload_url="http://center.example/upload"), "ZSL-1A-07")

    with pytest.raises(ProtocolError) as error:
        client.upload_map_package(str(package), {"map_name": "indoor"})

    assert error.value.code == "MAP_UPLOAD_TIMEOUT"
    assert "1800s" in error.value.message
