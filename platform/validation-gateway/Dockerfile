FROM python:3.12-slim

WORKDIR /opt/roamerx-validation-gateway
COPY requirements.txt ./
RUN python -m pip install --no-cache-dir -r requirements.txt
COPY gateway.py ./
USER 65532:65532
ENTRYPOINT ["python", "/opt/roamerx-validation-gateway/gateway.py"]
