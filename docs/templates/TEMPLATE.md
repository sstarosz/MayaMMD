# Title — One-line summary

<!--
  ============================================================================
  TEMPLATE — Docs style guide for the MayaMMD project.

  Two doc types exist; pick the one that fits:

    REFERENCE  — design docs, architecture notes, feature specs.
                 Sections: Overview, Architecture, Implementation Details,
                 Key Source Files, (optional: UI Components, Quick Reference),
                 References.

    PROCEDURAL — how-to guides, setup walkthroughs, release checklists.
                 Sections: Overview / Prerequisites, Steps, (optional:
                 Troubleshooting), References.

  Formatting rules (apply to both):
    • Separate every H2 section with `---`.
    • Code references use backticks: `` `mmd/ui/foo.py` ``.
    • Cross-doc links use Markdown: `[Other Doc](./OtherDoc.md)`.
    • Max heading depth: `###` (H3).  Use lists for deeper detail.
    • Tables for structured data (enums, status, file maps).
    • Inline notes as `> **Note:** …` blockquotes.
  ============================================================================
-->

## Overview

<!-- REQUIRED.  2–4 sentences: what this doc covers, who it's for. -->

---

## Quick Reference

<!-- OPTIONAL.  Tables, enums, key facts — the "at a glance" lookup. -->

---

## Architecture

<!-- REQUIRED for reference docs.
     High-level design: components, data flow, how things connect.
     Mermaid diagrams encouraged for complex flows. -->

---

## Implementation Details

<!-- REQUIRED for reference docs.
     Per-component deep dives.  Use `###` sub-headings for each component.
     Include code snippets, data structures, and diagrams as needed. -->

---

## UI Components

<!-- Only include if the feature has a UI.  Document widget architecture,
     key classes, and user-facing behaviour. -->

---

## Key Source Files

<!-- REQUIRED.  Table: file path → purpose.  Gives devs a code map. -->

| File              | Purpose      |
| ----------------- | ------------ |
| `path/to/file.py` | What it does |

---

## References

<!-- REQUIRED.  Links to related docs, source files, meta issues, memory files.
     Use Markdown links. -->

- [Related Doc](./OtherDoc.md)
- Meta issue: [#N Title](https://github.com/sstarosz/MayaMMD/issues/N)
