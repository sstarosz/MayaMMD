# Run this in Maya Script Editor to setup debugging
import debugpy

# Start debug server
debugpy.listen(("localhost", 5678))
print("Debugger listening on port 5678")

# Optional: Wait for VS Code to attach
# debugpy.wait_for_client()
