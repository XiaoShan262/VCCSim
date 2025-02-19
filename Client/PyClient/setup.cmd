@echo off
setlocal

@REM Install required packages
pip install -r requirements.txt

@REM Generate VCCSim_pb2.py and VCCSim_pb2_grpc.py and copy them to the GRPC folder
set CURRENT_DIR=%~dp0
set PROTO_PATH=%CURRENT_DIR%..\..\Source\pb\VCCSim.proto
set OUTPUT_PATH=%CURRENT_DIR%VCCSim
python -m grpc_tools.protoc -I "%~dp0..\..\Source\pb" --python_out="%OUTPUT_PATH%" --grpc_python_out="%OUTPUT_PATH%" "%PROTO_PATH%"

@REM Fix the import statement in VCCSim_pb2_grpc.py
set GRPC_FILE=%OUTPUT_PATH%\VCCSim_pb2_grpc.py
set TEMP_FILE=%OUTPUT_PATH%\temp.py

@REM Use PowerShell to modify the file because it handles text encoding better
powershell -Command ^
    "(Get-Content '%GRPC_FILE%') -replace 'import VCCSim_pb2 as VCCSim__pb2', 'from . import VCCSim_pb2 as VCCSim__pb2' | Set-Content '%TEMP_FILE%'" ^
    && move /y "%TEMP_FILE%" "%GRPC_FILE%"

echo Proto generation completed with import fixes.