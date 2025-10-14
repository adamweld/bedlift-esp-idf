@echo off
pushd .
cd /d C:\Espressif\frameworks\esp-idf-v5.5.1\
powershell -ExecutionPolicy Bypass -File "C:\Espressif\Initialize-Idf.ps1" -IdfId esp-idf-29323a3f5a0574597d6dbaa0af20c775
popd