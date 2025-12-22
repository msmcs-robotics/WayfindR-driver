"""
Pre-build script for PlatformIO.
Prepares SPIFFS filesystem if data directory exists.
"""

Import("env")

def before_build(source, target, env):
    """Run before build."""
    pass

# Register callback
env.AddPreAction("buildprog", before_build)
