import subprocess
import sys
import threading
from queue import Queue, Empty


def readOutput(pipe, callback):
    """Safely read output to avoid encoding errors."""
    try:
        for line in iter(pipe.readline, ''):
            if line:
                callback(line.rstrip())
    except UnicodeDecodeError:
        # If encoding error occurs, try different encodings
        try:
            pipe.seek(0)
            content = pipe.read()
            # Try multiple common encodings
            for encoding in ['utf-8', 'gbk', 'latin-1', 'cp1252']:
                try:
                    decoded = content.decode(encoding, errors='replace')
                    for line in decoded.splitlines():
                        callback(line)
                    break
                except:
                    continue
        except:
            callback("Unable to decode output content.")
    finally:
        pipe.close()


def runCuraSimple(settings=None, verbose=True):
    """Simplified CuraEngine execution function."""

    # Basic command
    cmd = [
        "wsl",
        "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
        "slice",
        "-v",
        "-j",
        "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/Cura/resources/definitions/fdmprinter.def.json",
        "-j",
        "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/Cura/resources/definitions/fdmextruder.def.json"
    ]

    # Default settings
    defaultSettings = {
        "layer_height": 0.2,
        "wall_thickness": 0.8,
        "roofing_layer_count": 0,
        "flooring_layer_count": 0,
        "top_layers": 4,
        "bottom_layers": 4
    }

    # Update settings
    if settings:
        defaultSettings.update(settings)

    # Add setting parameters
    for key, value in defaultSettings.items():
        cmd.extend(["-s", f"{key}={value}"])

    # Add output and input files
    cmd.extend([
        "-o", "/mnt/c/users/xuanouye/desktop/output.gcode",
        "-l", "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/tests/model/monk.stl"
    ])

    print(f"Executing command: {' '.join(cmd)}")
    print("-" * 50)

    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            encoding='utf-8',
            errors='replace'  # Replace undecodable characters with question marks
        )

        # Read output
        outputLines = []

        def collectOutput(line):
            outputLines.append(line)
            if verbose:
                print(line)

        def enqueueOutput(pipe, queue):
            for line in iter(pipe.readline, ''):
                queue.put(line)
            pipe.close()

        stdoutQueue = Queue()
        stderrQueue = Queue()

        stdoutThread = threading.Thread(target=enqueueOutput, args=(process.stdout, stdoutQueue))
        stderrThread = threading.Thread(target=enqueueOutput, args=(process.stderr, stderrQueue))

        stdoutThread.daemon = True
        stderrThread.daemon = True

        stdoutThread.start()
        stderrThread.start()

        # Wait for process to finish
        process.wait()

        # Collect all output
        stdoutOutput = []
        stderrOutput = []

        while True:
            try:
                stdoutOutput.append(stdoutQueue.get_nowait())
            except Empty:
                break

        while True:
            try:
                stderrOutput.append(stderrQueue.get_nowait())
            except Empty:
                break

        # Print output
        if verbose and stdoutOutput:
            print("\nStandard output:")
            for line in stdoutOutput:
                print(line, end='')

        if verbose and stderrOutput:
            print("\nStandard error:")
            for line in stderrOutput:
                print(line, end='')

        print(f"\n{'=' * 50}")
        print(f"Process exit code: {process.returncode}")

        if process.returncode == 0:
            print("Slicing completed successfully!")
            return True
        else:
            print("Slicing failed!")
            return False

    except Exception as e:
        print(f"Error during execution: {e}")
        return False

# Usage example
if __name__ == "__main__":
    customSettings = {
        "layer_height": 0.1,  # Finer layer height
        "wall_thickness": 1.2,  # Thicker wall thickness
        "top_layers": 6,  # More top layers
    }

    print("Using safe output reading method")
    success = runCuraSimple(customSettings, verbose=True)

    if success:
        print("\nG-code file generated successfully!")
        print("Output path: /mnt/c/users/xuanouye/desktop/output.gcode")
    else:
        print("\nG-code file generation failed!")