@echo off
setlocal

if "%DOG_RK_HOST%"=="" set DOG_RK_HOST=192.168.234.1
if "%DOG_RK_USER%"=="" set DOG_RK_USER=firefly
if "%DOG_ORIN_HOST%"=="" set DOG_ORIN_HOST=192.168.234.234
if "%DOG_ORIN_USER%"=="" set DOG_ORIN_USER=jszr

if "%DOG_REMOTE_SERVER_HOST%"=="" (
  echo Please set DOG_REMOTE_SERVER_HOST before running this script.
  exit /b 1
)

scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=NUL "%~dp0dog_orin_reverse_tunnel_setup.sh" %DOG_RK_USER%@%DOG_RK_HOST%:/tmp/dog_orin_reverse_tunnel_setup.sh
ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=NUL %DOG_RK_USER%@%DOG_RK_HOST% "scp /tmp/dog_orin_reverse_tunnel_setup.sh %DOG_ORIN_USER%@%DOG_ORIN_HOST%:/tmp/dog_orin_reverse_tunnel_setup.sh && ssh %DOG_ORIN_USER%@%DOG_ORIN_HOST% 'DOG_REMOTE_SERVER_HOST=%DOG_REMOTE_SERVER_HOST% DOG_REMOTE_SERVER_PORT=%DOG_REMOTE_SERVER_PORT% DOG_REMOTE_SERVER_USER=%DOG_REMOTE_SERVER_USER% bash /tmp/dog_orin_reverse_tunnel_setup.sh'"

echo.
echo If the script printed an Orin public key, make sure that key exists in /home/mluser/.ssh/authorized_keys on the cloud server.
pause
