import subprocess
import sqlite3
import os
import sys
import argparse
import json
import io
from pathlib import Path

# Reconfigure stdout/stderr to use UTF-8, resolving cp949 encoding errors on Windows
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

def find_workspace_dir(start_path):
    current = Path(start_path).resolve()
    for _ in range(10):
        if (current / ".agent").exists() or (current / ".git").exists() or (current / "platformio.ini").exists():
            return current
        if current.parent == current:
            break
        current = current.parent
    return current.parent.parent

# --- 1. Load Configuration ---
def load_config():
    parser = argparse.ArgumentParser(description="Graphify CodeGraph Clean Tool")
    parser.add_argument("--config", type=str, default=None, help="Path to graphify_config.json")
    args, unknown = parser.parse_known_args()

    # Determine workspace_dir from script location dynamically
    script_dir = Path(__file__).resolve().parent
    workspace_dir = find_workspace_dir(script_dir)

    # Try to find config file
    config_path = None
    if args.config:
        config_path = Path(args.config)
    elif (workspace_dir / "graphify_config.json").exists():
        config_path = workspace_dir / "graphify_config.json"
    elif (workspace_dir / ".agent" / "skills" / "graphify" / "graphify_config.json").exists():
        config_path = workspace_dir / ".agent" / "skills" / "graphify" / "graphify_config.json"
    elif (script_dir / "graphify_config.json").exists():
        config_path = script_dir / "graphify_config.json"
    elif (script_dir.parent / "graphify_config.json").exists():
        config_path = script_dir.parent / "graphify_config.json"
    
    if not config_path or not config_path.exists():
        print(f"Error: Config file not found. Checked: CLI, {workspace_dir}/graphify_config.json, and skill folder.", file=sys.stderr)
        sys.exit(1)

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
    except Exception as e:
        print(f"Error loading config file: {e}", file=sys.stderr)
        sys.exit(1)

    return config, workspace_dir

config, workspace_dir = load_config()

# --- 2. Virtual Environment Setup (Bootstrap) ---
venv_cfg = config.get("venv", {})
if venv_cfg:
    venv_rel_path = venv_cfg.get("dir", ".agent/skills/graphify/.venv")
    venv_path = (workspace_dir / venv_rel_path).resolve()
    
    # Platform-specific paths
    if sys.platform == "win32":
        venv_python = venv_path / "Scripts" / "python.exe"
        venv_pip = venv_path / "Scripts" / "pip.exe"
    else:
        venv_python = venv_path / "bin" / "python"
        venv_pip = venv_path / "bin" / "pip"

    # Check if currently running under the correct virtualenv
    current_python = Path(sys.executable).resolve()
    is_in_venv = False
    try:
        is_in_venv = current_python.samefile(venv_python)
    except Exception:
        pass

    if not is_in_venv:
        # Create virtualenv if it does not exist
        if not venv_path.exists():
            print(f"Virtual environment not found. Creating venv at: {venv_path}")
            subprocess.run([sys.executable, "-m", "venv", str(venv_path)], check=True)
            print("Venv created successfully.")

        # Install / Update requirements
        requirements = venv_cfg.get("requirements", ["graphifyy", "networkx"])
        if requirements:
            print(f"Installing/Updating requirements in venv: {requirements}")
            subprocess.run([str(venv_python), "-m", "pip", "install", "--upgrade", "pip"], check=True)
            subprocess.run([str(venv_python), "-m", "pip", "install"] + requirements, check=True)
            print("Dependencies installed successfully.")

        # Re-execute the script using the venv interpreter
        print(f"Re-executing script via virtualenv python: {venv_python}")
        cmd = [str(venv_python)] + sys.argv
        sys.exit(subprocess.run(cmd).returncode)

# --- 3. CodeGraph Synchronization and Clean DB (Executed inside the Venv) ---
# Setup database path from config
codegraph_cfg = config.get("codegraph", {})
db_relative_path = codegraph_cfg.get("db_path", ".codegraph/codegraph.db")
db_path = workspace_dir / db_relative_path

# Stop running node/codegraph processes to unlock the database
print("Stopping codegraph background processes to unlock DB...")
try:
    if sys.platform == "win32":
        # Windows PowerShell/Command
        subprocess.run(
            'powershell -Command "Get-Process node -ErrorAction SilentlyContinue | Where-Object { $_.Path -like \'*codegraph*\' } | Stop-Process -Force"',
            shell=True
        )
    else:
        # Unix
        subprocess.run("pkill -f codegraph", shell=True)
except Exception as e:
    print(f"Warning: Failed to stop background processes: {e}")

# Run codegraph sync or index to update database
# Check if database exists
is_new = not db_path.exists()
cmd = "codegraph index" if is_new else "codegraph sync"
print(f"Running '{cmd}'...")
try:
    subprocess.run(cmd, shell=True, check=True, cwd=str(workspace_dir))
    print(f"Codegraph database updated successfully via '{cmd}'.")
except subprocess.CalledProcessError as e:
    print(f"Error: Failed to run '{cmd}': {e}")
    sys.exit(1)

# Clean up unwanted records from the database
if not db_path.exists():
    print(f"Error: Database file not found at {db_path}")
    sys.exit(1)

print(f"Connecting to database at {db_path} to remove unwanted records...")
conn = sqlite3.connect(str(db_path))
cursor = conn.cursor()

# Get all file paths
cursor.execute("SELECT path FROM files")
all_files = [row[0] for row in cursor.fetchall()]

# Determine files to remove based on dynamic config patterns
to_remove = []
remove_patterns = codegraph_cfg.get("remove_patterns", [])

for f in all_files:
    for pattern in remove_patterns:
        match = False
        
        # 1. Prefix and extension exclude checks
        prefix = pattern.get("prefix")
        if prefix:
            norm_f = f.replace("\\", "/")
            norm_prefix = prefix.replace("\\", "/")
            if norm_f.startswith(norm_prefix):
                exclude_exts = pattern.get("exclude_extensions", [])
                if exclude_exts:
                    suffix = Path(f).suffix.lower()
                    if suffix not in [ext.lower() for ext in exclude_exts]:
                        match = True
                else:
                    match = True
                    
        # 2. Substring matching check
        contains = pattern.get("contains")
        if contains:
            if contains.lower() in f.lower():
                match = True
                
        if match:
            to_remove.append(f)
            break

print(f"Found {len(to_remove)} unwanted files in codegraph.db.")
if not to_remove:
    print("No unwanted files to remove. Codegraph database is clean.")
    conn.close()
    sys.exit(0)

try:
    conn.execute("PRAGMA foreign_keys = OFF")
    
    # Get nodes belonging to these files
    placeholders = ",".join(["?"] * len(to_remove))
    cursor.execute(f"SELECT id FROM nodes WHERE file_path IN ({placeholders})", to_remove)
    node_ids = [row[0] for row in cursor.fetchall()]
    print(f"Found {len(node_ids)} nodes associated with the files to remove.")
    
    # Delete from edges
    if node_ids:
        node_placeholders = ",".join(["?"] * len(node_ids))
        cursor.execute(f"DELETE FROM edges WHERE source IN ({node_placeholders}) OR target IN ({node_placeholders})", node_ids + node_ids)
        edges_deleted = cursor.rowcount
        print(f"Deleted {edges_deleted} edges.")
    else:
        edges_deleted = 0
        
    # Delete from unresolved_refs
    cursor.execute(f"DELETE FROM unresolved_refs WHERE file_path IN ({placeholders})", to_remove)
    unresolved_deleted = cursor.rowcount
    if node_ids:
        cursor.execute(f"DELETE FROM unresolved_refs WHERE from_node_id IN ({node_placeholders})", node_ids)
        unresolved_deleted += cursor.rowcount
    print(f"Deleted {unresolved_deleted} unresolved references.")

    # Delete from nodes
    cursor.execute(f"DELETE FROM nodes WHERE file_path IN ({placeholders})", to_remove)
    nodes_deleted = cursor.rowcount
    print(f"Deleted {nodes_deleted} nodes.")

    # Delete from files
    cursor.execute(f"DELETE FROM files WHERE path IN ({placeholders})", to_remove)
    files_deleted = cursor.rowcount
    print(f"Deleted {files_deleted} files.")

    conn.commit()
    print("Database changes committed successfully.")

    # Vacuum database to reclaim space
    print("Vacuuming database...")
    conn.execute("VACUUM")
    print("Vacuum complete.")

except Exception as e:
    conn.rollback()
    print(f"An error occurred during database cleanup: {e}")
    sys.exit(1)
finally:
    conn.close()

print("Codegraph cleanup successfully completed.")
