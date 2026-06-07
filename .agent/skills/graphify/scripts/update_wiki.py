import json
import os
import sys
import shutil
import re
import hashlib
import argparse
import io
import subprocess
from pathlib import Path
from collections import Counter

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
    parser = argparse.ArgumentParser(description="Graphify Wiki & Obsidian Update Tool")
    parser.add_argument("--config", type=str, default=None, help="Path to graphify_config.json")
    parser.add_argument("--python-path", type=str, default=None, help="Additional path to append to sys.path (deprecated)")
    args, unknown = parser.parse_known_args()

    # Add python path if provided, else default to platformio python package path using environment variable
    if args.python_path:
        sys.path.append(args.python_path)
    else:
        user_profile = os.environ.get("USERPROFILE", os.path.expanduser("~"))
        sys.path.append(os.path.join(user_profile, r".platformio\penv\Lib\site-packages"))

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

    return config, args, workspace_dir

config, args, workspace_dir = load_config()

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

# --- 3. Imports and Setup (Executed inside the Venv) ---
import networkx as nx
from graphify.build import build_from_json
from graphify.cluster import cluster, score_all
from graphify.analyze import god_nodes, surprising_connections, suggest_questions
from graphify.report import generate
from graphify.export import to_json, to_obsidian, to_canvas, to_html
from graphify.llm import generate_community_labels
from graphify.wiki import to_wiki

# Monkeypatch graphify export/wiki modules to handle Windows MAX_PATH and KeyError issues
import graphify.export
import graphify.wiki

original_cap = graphify.export._cap_filename
def patched_cap(s, limit=200):
    return original_cap(s, limit=100)
graphify.export._cap_filename = patched_cap

def _clean_communities(G, communities):
    clean_communities = {}
    for cid, members in communities.items():
        clean_members = [m for m in members if G.has_node(m)]
        clean_communities[cid] = clean_members
    return clean_communities

original_to_obsidian = graphify.export.to_obsidian
def patched_to_obsidian(*args, **kwargs):
    args = list(args)
    args[1] = _clean_communities(args[0], args[1])
    return original_to_obsidian(*args, **kwargs)
graphify.export.to_obsidian = patched_to_obsidian

original_to_canvas = graphify.export.to_canvas
def patched_to_canvas(*args, **kwargs):
    args = list(args)
    args[1] = _clean_communities(args[0], args[1])
    return original_to_canvas(*args, **kwargs)
graphify.export.to_canvas = patched_to_canvas

original_to_wiki = graphify.wiki.to_wiki
def patched_to_wiki(*args, **kwargs):
    args = list(args)
    args[1] = _clean_communities(args[0], args[1])
    return original_to_wiki(*args, **kwargs)
graphify.wiki.to_wiki = patched_to_wiki


# --- 4. Setup Paths dynamically based on Config ---
wiki_out_setting = config.get("wiki_out_dir", "src/T2_MSFM_250_wiki")
wiki_out_dir = workspace_dir / wiki_out_setting
wiki_name = Path(wiki_out_setting).name

# Derive target directories and files
target_graphify_out = wiki_out_dir / f"{wiki_name}_graphify_out" / "graphify-out"
target_obsidian_dir = wiki_out_dir / f"{wiki_name}_Obsidian"
pre_existing_extract = target_graphify_out / ".graphify_extract.json"

# Check compatibility fallbacks for existing structure if the default path doesn't exist
if not pre_existing_extract.exists():
    fallback_extract = wiki_out_dir / "T2_MSFM_250_wiki_graphify_out" / "graphify-out" / ".graphify_extract.json"
    if fallback_extract.exists():
        pre_existing_extract = fallback_extract
        target_graphify_out = wiki_out_dir / "T2_MSFM_250_wiki_graphify_out" / "graphify-out"
        target_obsidian_dir = wiki_out_dir / "T2_MSFM_250_wiki_Obsidian"

target_wiki_dir = target_graphify_out / "wiki"

# Ensure target directories exist
target_graphify_out.mkdir(parents=True, exist_ok=True)
target_obsidian_dir.mkdir(parents=True, exist_ok=True)
target_wiki_dir.mkdir(parents=True, exist_ok=True)

# Helper functions for Obsidian filename sanitization (copied from graphify.export)
def _cap_filename(s: str, limit: int = 100) -> str:
    b = s.encode("utf-8")
    if len(b) <= limit:
        return s
    digest = hashlib.sha1(s.encode("utf-8")).hexdigest()[:8]
    keep = limit - 9
    truncated = b[:keep].decode("utf-8", "ignore")
    return f"{truncated}_{digest}"

def safe_name(label: str) -> str:
    cleaned = re.sub(r'[\\/*?:"<>|#^[\]]', "", label.replace("\r\n", " ").replace("\r", " ").replace("\n", " ")).strip()
    cleaned = re.sub(r"\.(md|mdx|qmd|markdown)$", "", cleaned, flags=re.IGNORECASE)
    return _cap_filename(cleaned) if cleaned else "unnamed"

# Check extraction files
if not pre_existing_extract.exists():
    print(f"Error: extraction data not found at {pre_existing_extract}", file=sys.stderr)
    sys.exit(1)

# Load extraction and existing label/analysis data
data = json.loads(pre_existing_extract.read_text(encoding="utf-8"))

labels_path = target_graphify_out / ".graphify_labels.json"
old_labels = json.loads(labels_path.read_text(encoding="utf-8")) if labels_path.exists() else {}

analysis_path = target_graphify_out / ".graphify_analysis.json"
old_analysis = json.loads(analysis_path.read_text(encoding="utf-8")) if analysis_path.exists() else {}

# Backup old outputs before modifying
for file_name in ["graph.json", "GRAPH_REPORT.md", "graph.html", ".graphify_labels.json"]:
    old_file = target_graphify_out / file_name
    if old_file.exists():
        backup_file = target_graphify_out / f"{file_name}.bak"
        if backup_file.exists():
            backup_file.unlink()
        old_file.rename(backup_file)

# Build node to old community ID mapping
node_to_old_cid = {}
if old_analysis and "communities" in old_analysis:
    for cid, nodes in old_analysis["communities"].items():
        for nid in nodes:
            node_to_old_cid[nid] = int(cid)

# Setup component paths based on config
components = {name: workspace_dir / path for name, path in config.get("components", {}).items()}
filters = config.get("filters", {})

def resolve_component(sf):
    if not sf:
        return None
    sf_lower = sf.lower()
    
    # Check explicit patterns based on defined components
    comp = None
    for c in components:
        # Match normalized names (underlines replaced with spaces, lowercase)
        c_pattern = c.lower().replace("_", " ")
        if c_pattern in sf_lower:
            comp = c
            break
            
    # Check physical file existence in workspace if name matching failed
    if not comp:
        sf_path = Path(sf)
        for c, c_dir in components.items():
            if (c_dir / sf_path).exists() or (c_dir / sf_path.name).exists() or list(c_dir.rglob(sf_path.name)):
                comp = c
                break
            
    # Fallback exception for test_dsp.c (if not matched yet and esp-dsp component exists)
    if not comp and "test_dsp.c" in sf_lower and "esp-dsp" in components:
        comp = 'esp-dsp'
        
    # Check filter extensions if specified for this component
    if comp in filters:
        allowed_exts = filters[comp].get("extensions", [])
        if allowed_exts:
            suffix = Path(sf).suffix.lower()
            if suffix not in [ext.lower() for ext in allowed_exts]:
                return None
            
    return comp

# --- 5. Filter nodes and edges ---
retained_nodes = []
retained_node_ids = set()

for n in data['nodes']:
    comp = resolve_component(n.get('source_file'))
    if comp is not None:
        retained_nodes.append(n)
        retained_node_ids.add(n['id'])

retained_edges = []
for e in data['edges']:
    src = e.get('source')
    tgt = e.get('target')
    if src in retained_node_ids and tgt in retained_node_ids:
        retained_edges.append(e)

# Handle hyperedges if present
retained_hyperedges = []
for h in data.get('hyperedges', []):
    h_nodes = h.get('nodes', [])
    if all(node_id in retained_node_ids for node_id in h_nodes):
        retained_hyperedges.append(h)

filtered_extraction = {
    'nodes': retained_nodes,
    'edges': retained_edges,
    'hyperedges': retained_hyperedges,
    'input_tokens': data.get('input_tokens', 0),
    'output_tokens': data.get('output_tokens', 0)
}

print(f"Filtered nodes: {len(retained_nodes)} / {len(data['nodes'])}")
print(f"Filtered edges: {len(retained_edges)} / {len(data['edges'])}")

# Save filtered extraction to the target folder
(target_graphify_out / ".graphify_extract.json").write_text(json.dumps(filtered_extraction, indent=2, ensure_ascii=False), encoding="utf-8")

# --- 6. Build Graph ---
G = build_from_json(filtered_extraction)

# --- 7. Cluster & Analyze ---
communities = cluster(G)
cohesion = score_all(G, communities)
tokens = {'input': filtered_extraction.get('input_tokens', 0), 'output': filtered_extraction.get('output_tokens', 0)}
gods = god_nodes(G)
surprises = surprising_connections(G, communities)

# --- 8. Map new communities to existing Korean labels using node majority voting ---
print("Mapping new communities to existing Korean labels...")
labels = {}
fallback_communities = {}

for cid, nids in communities.items():
    old_cids = [node_to_old_cid[nid] for nid in nids if nid in node_to_old_cid]
    if old_cids:
        most_common_old_cid = Counter(old_cids).most_common(1)[0][0]
        label = old_labels.get(str(most_common_old_cid))
        if label:
            labels[cid] = label
            continue
            
    # If no mapping was found, mark for LLM naming fallback
    fallback_communities[cid] = nids

# If there are any communities that couldn't be mapped, name them using LLM fallback
if fallback_communities:
    print(f"Need to fallback to Gemini for {len(fallback_communities)} communities.")
    fallback_labels, _ = generate_community_labels(G, fallback_communities, backend='gemini', gods=gods)
    labels.update(fallback_labels)

# Save labels and analysis
(target_graphify_out / ".graphify_labels.json").write_text(json.dumps({str(k): v for k, v in labels.items()}, ensure_ascii=False), encoding="utf-8")

analysis = {
    'communities': {str(k): v for k, v in communities.items()},
    'cohesion': {str(k): v for k, v in cohesion.items()},
    'gods': gods,
    'surprises': surprises,
}
(target_graphify_out / ".graphify_analysis.json").write_text(json.dumps(analysis, indent=2, ensure_ascii=False), encoding="utf-8")

# --- 9. Generate report and write outputs ---
questions = suggest_questions(G, communities, labels)
detection = {
    'total_files': len(retained_nodes),
    'total_words': 0, 
    'files': {'code': [n.get('source_file') for n in retained_nodes], 'document': [], 'paper': [], 'image': [], 'video': []}
}

report = generate(G, communities, cohesion, labels, gods, surprises, detection, tokens, str(wiki_out_dir), suggested_questions=questions)
(target_graphify_out / "GRAPH_REPORT.md").write_text(report, encoding="utf-8")
to_json(G, communities, str(target_graphify_out / "graph.json"))

# --- 10. Incremental clean of Obsidian notes ---
print("Performing incremental cleanup of pruned nodes/communities in Obsidian vault...")
old_graph_path = target_graphify_out / "graph.json.bak"
old_filenames = set()
if old_graph_path.exists():
    try:
        old_graph_data = json.loads(old_graph_path.read_text(encoding="utf-8"))
        seen_names = {}
        for node in old_graph_data.get("nodes", []):
            base = safe_name(node.get("label", node["id"]))
            if base in seen_names:
                seen_names[base] += 1
                old_filenames.add(f"{base}_{seen_names[base]}.md")
            else:
                seen_names[base] = 0
                old_filenames.add(f"{base}.md")
    except Exception as e:
        print("Failed to calculate old node filenames:", e)

new_filenames = set()
seen_names = {}
for node_id, ndata in G.nodes(data=True):
    base = safe_name(ndata.get("label", node_id))
    if base in seen_names:
        seen_names[base] += 1
        new_filenames.add(f"{base}_{seen_names[base]}.md")
    else:
        seen_names[base] = 0
        new_filenames.add(f"{base}.md")

to_delete = old_filenames - new_filenames
print(f"Deleting {len(to_delete)} pruned nodes from Obsidian vault...")
for fname in to_delete:
    fpath = target_obsidian_dir / fname
    if fpath.exists():
        print(f"Deleting pruned node note: {fname}")
        fpath.unlink()

# Calculate and prune community notes
old_comm_filenames = set()
if old_analysis and "communities" in old_analysis:
    for old_cid in old_analysis["communities"].keys():
        old_name = old_labels.get(str(old_cid), f"Community {old_cid}")
        old_comm_filenames.add(f"_COMMUNITY_{safe_name(old_name)}.md")

new_comm_filenames = set()
for new_cid in communities.keys():
    new_name = labels.get(new_cid, f"Community {new_cid}")
    new_comm_filenames.add(f"_COMMUNITY_{safe_name(new_name)}.md")

comms_to_delete = old_comm_filenames - new_comm_filenames
print(f"Deleting {len(comms_to_delete)} pruned communities from Obsidian vault...")
for fname in comms_to_delete:
    fpath = target_obsidian_dir / fname
    if fpath.exists():
        print(f"Deleting pruned community note: {fname}")
        fpath.unlink()

# --- 11. Export viz/outputs ---
to_html(G, communities, str(target_graphify_out / "graph.html"), community_labels=labels)
to_obsidian(G, communities, str(target_obsidian_dir), community_labels=labels, cohesion=cohesion)
to_canvas(G, communities, str(target_obsidian_dir / "graph.canvas"), community_labels=labels)
to_wiki(G, communities, str(target_wiki_dir), community_labels=labels, cohesion=cohesion, god_nodes_data=gods)

# --- 12. Automatically cleanup individual graphify-out folders as requested ---
print("Cleaning up individual graphify-out folders...")
folders_to_clean = [workspace_dir / folder_path for folder_path in config.get("folders_to_clean", [])]
for folder in folders_to_clean:
    if folder.exists():
        print(f"Removing: {folder}")
        shutil.rmtree(folder, ignore_errors=True)

print("Rebuild and automated cleanup completed successfully in target directory!")
