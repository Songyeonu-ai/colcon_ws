"""
Constants for Package Manager
"""

# Package execution types
LAUNCH = 0
RUN = 1

# Process states
STATE_STOPPED = "stopped"
STATE_RUNNING = "running"
STATE_ERROR = "error"
STATE_UNKNOWN = "unknown"

# Timeouts (milliseconds)
PROCESS_STOP_TIMEOUT = 1000
PROCESS_KILL_TIMEOUT = 3000
RESTART_DELAY = 5000

# UI Colors
COLOR_RUNNING = "green"
COLOR_STOPPED = "red"
COLOR_WARNING = "orange"

# Status symbols
SYMBOL_RUNNING = "✓"
SYMBOL_STOPPED = "●"
SYMBOL_ERROR = "✗"
