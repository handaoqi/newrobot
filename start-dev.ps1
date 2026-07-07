$ErrorActionPreference = "Stop"

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BackendDir = Join-Path $RootDir "backend"
$FrontendDir = Join-Path $RootDir "frontend"
$PythonExe = Join-Path $RootDir ".venv\Scripts\python.exe"
$ZlmComposeFile = Join-Path $RootDir "docker-compose.zlmediakit.yml"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Python virtual environment was not found at: $PythonExe"
}

if (-not (Test-Path -LiteralPath (Join-Path $BackendDir "manage.py"))) {
    throw "Django manage.py was not found in: $BackendDir"
}

if (-not (Test-Path -LiteralPath (Join-Path $FrontendDir "package.json"))) {
    throw "Frontend package.json was not found in: $FrontendDir"
}

if (-not (Test-Path -LiteralPath $ZlmComposeFile)) {
    throw "ZLMediaKit compose file was not found at: $ZlmComposeFile"
}

$ZlmCommand = @"
`$Host.UI.RawUI.WindowTitle = 'ZLMediaKit - Docker'
Set-Location -LiteralPath '$RootDir'

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error 'Docker was not found. Please install Docker Desktop and make sure docker is in PATH.'
    return
}

docker compose version *> `$null
if (`$LASTEXITCODE -eq 0) {
    docker compose -f '$ZlmComposeFile' up -d
    docker compose -f '$ZlmComposeFile' ps
} elseif (Get-Command docker-compose -ErrorAction SilentlyContinue) {
    docker-compose -f '$ZlmComposeFile' up -d
    docker-compose -f '$ZlmComposeFile' ps
} else {
    Write-Error 'Neither docker compose nor docker-compose is available.'
}
"@

$BackendCommand = @"
`$Host.UI.RawUI.WindowTitle = 'Backend - Django'
Set-Location -LiteralPath '$BackendDir'
& '$PythonExe' manage.py migrate
if (`$LASTEXITCODE -eq 0) {
    & '$PythonExe' manage.py runserver
}
"@

$FrontendCommand = @"
`$Host.UI.RawUI.WindowTitle = 'Frontend - Vite'
Set-Location -LiteralPath '$FrontendDir'
npm run dev
"@

function ConvertTo-EncodedCommand {
    param (
        [Parameter(Mandatory = $true)]
        [string] $Command
    )

    return [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($Command))
}

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-EncodedCommand", (ConvertTo-EncodedCommand $ZlmCommand)
)

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-EncodedCommand", (ConvertTo-EncodedCommand $BackendCommand)
)

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-EncodedCommand", (ConvertTo-EncodedCommand $FrontendCommand)
)

Write-Host "Started ZLMediaKit, backend, and frontend dev servers."
Write-Host "ZLMediaKit HTTP: http://127.0.0.1:8080/"
Write-Host "Backend:        http://127.0.0.1:8000/"
Write-Host "Frontend:       http://127.0.0.1:5173/"
