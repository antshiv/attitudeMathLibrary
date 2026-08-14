#!/usr/bin/env python3
"""Compile authored documentation pages through a shared HTML shell."""

from __future__ import annotations

import argparse
import html
import json
import re
import shutil
import subprocess
from datetime import date
from pathlib import Path


DOCS = Path(__file__).resolve().parent
ROOT = DOCS.parent
PAGES = DOCS / "_pages"
PARTIALS = DOCS / "_partials"
BUILD = DOCS / "build"


def metadata(source: str, name: str, fallback: str) -> str:
    match = re.search(rf"^<!--[ ]*{name}:[ ]*(.*?)[ ]*-->$", source, re.MULTILINE)
    return match.group(1) if match else fallback


def render_navigation(config: dict[str, object], active: str) -> str:
    links = []
    for label, target, key in config["navigation"]:
        css_class = ' class="active" aria-current="page"' if key == active else ""
        links.append(
            f'<a href="{html.escape(target)}"{css_class}>{html.escape(label)}</a>'
        )
    links.append(
        '<a class="repository-link" href="'
        + html.escape(str(config["repository_url"]))
        + '" target="_blank" rel="noopener">GitHub</a>'
    )
    return "\n".join(links)


def build(require_doxygen: bool) -> None:
    config = json.loads((DOCS / "site.json").read_text(encoding="utf-8"))
    header = (PARTIALS / "header.html").read_text(encoding="utf-8")
    footer = (PARTIALS / "footer.html").read_text(encoding="utf-8")

    if BUILD.exists():
        shutil.rmtree(BUILD)
    BUILD.mkdir(parents=True)

    doxygen = shutil.which("doxygen")
    if require_doxygen and not doxygen:
        raise RuntimeError("Doxygen is required but was not found")
    if doxygen:
        subprocess.run([doxygen, "Doxyfile"], cwd=ROOT, check=True)

    for page in sorted(PAGES.glob("*.html")):
        source = page.read_text(encoding="utf-8")
        title = metadata(source, "TITLE", "Documentation")
        description = metadata(source, "DESCRIPTION", str(config["tagline"]))
        active = metadata(source, "NAV", "")
        content = re.sub(r"^<!--.*?-->\s*", "", source, flags=re.MULTILINE)
        canonical = str(config["site_url"]).rstrip("/") + "/"
        if page.name != "index.html":
            canonical += page.name

        values = {
            "{{PAGE_TITLE}}": html.escape(title),
            "{{META_DESCRIPTION}}": html.escape(description, quote=True),
            "{{CANONICAL_URL}}": html.escape(canonical, quote=True),
            "{{PROJECT_NAME}}": html.escape(str(config["project_name"])),
            "{{PROJECT_MARK}}": html.escape(str(config["project_mark"])),
            "{{TAGLINE}}": html.escape(str(config["tagline"])),
            "{{NAVIGATION}}": render_navigation(config, active),
            "{{YEAR}}": str(date.today().year),
            "{{REPOSITORY_URL}}": html.escape(
                str(config["repository_url"]), quote=True
            ),
        }
        rendered = header + content + footer
        for placeholder, value in values.items():
            rendered = rendered.replace(placeholder, value)
        unresolved = re.findall(r"{{[A-Z_]+}}", rendered)
        if unresolved:
            raise RuntimeError(f"{page}: unresolved placeholders {unresolved}")
        (BUILD / page.name).write_text(rendered, encoding="utf-8")

    (BUILD / ".nojekyll").touch()
    print(f"Built {len(list(PAGES.glob('*.html')))} pages in {BUILD}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-doxygen", action="store_true")
    args = parser.parse_args()
    build(args.require_doxygen)
