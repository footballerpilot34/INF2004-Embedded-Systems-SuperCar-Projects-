@echo off
rem Connect this laptop to the car over MQTT: see tools\mqtt_car.ps1.
rem From PowerShell in this folder, type:  .\mqtt
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\mqtt_car.ps1" %*
