---
name: graphify
description: Turn any folder of files into a navigable knowledge graph and refine project wiki/codegraph
---

# Workflow: graphify

Follow the graphify skill instructions (installed at `.agent/skills/graphify/SKILL.md`) to run the full pipeline, or run project-specific customization pipelines using the general-purpose configurations.

If no path argument is given, use `.` (current directory).

## Commands & Workflows

### 1. General Pipeline `/graphify`
Builds a fresh knowledge graph from target files.
```powershell
/graphify [path]
```

### 2. Custom Project Update Pipeline
Updates wiki pages, Obsidian canvas, and refines the database using predefined components & filters configuration.

#### Step 1. Define Settings
Configure paths and exclude filters in the configuration file:
* File Path: [graphify_config.json](../skills/graphify/graphify_config.json) (or override at project root).

#### Step 2. Sync and Clean CodeGraph Database
Stop active node processes, run codegraph sync/index, and wipe out unwanted records (e.g. non-header files or raw test libraries) as per config:
```powershell
python .agent/skills/graphify/scripts/update_codegraph.py
```

#### Step 3. Process Extraction and Update Wiki
Filters nodes/edges, resolves component boundaries, matches labels with Korean dictionary, fallbacks to LLM for new terms, regenerates reports, and exports to Obsidian, canvas, and wiki:
```powershell
python .agent/skills/graphify/scripts/update_wiki.py
```
