@echo off
REM Batch file to run Node.js commands with DEP0190 deprecation warning suppressed
REM Usage: node-safe.bat <command>

set NODE_OPTIONS=--no-deprecation
call %*