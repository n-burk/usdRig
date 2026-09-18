#!/usr/bin/env python
"""Generate a Houdini-style multi-page HTML node reference.

Usage: python docs/build_html.py

Reads the operator prose from docs/operator_notes.py and the attribute
reference from libs/rigExecSchema/schema.usda (via build_pages), the same
sources as the Markdown pages, and writes docs/site/: an index gallery,
one page per operator under nodes/, a shared stylesheet, and copied
icons/, gifs/, examples/ and specs/ trees so the site is
self-contained.  Open docs/site/index.html in a browser to read it.

The theme follows the SideFX Houdini help: a black 32px top bar whose
search field filters the node list, a fixed left table of contents with
node icons, breadcrumbs, the pale metatable under the title, ruled
section headings, and the side-by-side parameter grid whose right-aligned
label cell carries the monospace parameter name over a small type/default
signature.  It follows the system colour scheme: that look in light, and
a near-black usdview-ish palette when the OS prefers dark.

Every tree is copied with dot-directories ignored, so editor and tool
metadata such as icons/.omc never reaches the site.
"""
import html as _html
import os
import re
import shutil
import sys

RIG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SITE = os.path.join(RIG, "docs", "site")

sys.path.insert(0, os.path.join(RIG, "docs"))
from operator_notes import OPERATORS, CATEGORIES  # noqa: E402
from build_pages import parse_schema, example_stage, check_examples  # noqa: E402

try:
    import markdown
except ImportError:
    raise SystemExit("python-markdown is required: pip install markdown")

SCHEMA = os.path.join(RIG, "libs", "rigExecSchema", "schema.usda")

FONTS = ("https://fonts.googleapis.com/css2?family=Source+Sans+3:"
         "wght@300;400;600;700&amp;family=Source+Code+Pro&amp;display=swap")

SECTIONS = [
    ("overview", "Overview"),
    ("how-it-works", "How it works"),
    ("wiring", "Wiring"),
    ("parameters", "Parameters"),
    ("example", "Example"),
    ("tips", "Tips"),
    ("see-also", "See also"),
]

CSS = """\
/* RigExec node reference: a port of the SideFX Houdini help look.
   Black 32px top bar, fixed left TOC, ruled sections, side-by-side
   parameter grid. Every colour is a custom property, so the dark
   scheme further down is a token swap and nothing else. */
:root {
    color-scheme: light dark;
    /* chrome */
    --nav-bg: #000;
    --nav-fg: #fff;
    --nav-dim: #888;
    --search-bg: rgba(255, 255, 255, 0.8);
    --search-border: #999;
    --search-fg: #222;
    --search-icon: url("data:image/svg+xml;charset=utf-8,%3Csvg \
xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16' fill='none' \
stroke='%23555' stroke-width='1.6'%3E%3Ccircle cx='6.6' cy='6.6' r='4.4'\
/%3E%3Cpath d='M10 10l4 4'/%3E%3C/svg%3E");
    /* page */
    --page-bg: #fff;
    --ink: #333;
    --ink-strong: #000;
    --muted: #666;
    --faint: #999;
    --rule: rgba(0, 0, 0, 0.12);
    --rule-soft: rgba(0, 0, 0, 0.07);
    --wash: #f6f6f6;
    --meta-bg: #f6f6ef;
    --pre-bg: #f6f6f6;
    --pre-fg: #000;
    /* table of contents */
    --toc-bg: #fff;
    --toc-fg: #000;
    --toc-line: rgba(0, 0, 0, 0.07);
    /* links */
    --link: #1782ba;
    --link-visited: #2c7ba5;
    --link-hover: #21a1e3;
    /* accents */
    --teal: #42a0a4;
    --accent: #f90;
    --accent-active: #f60;
    --button: #58a4ff;
    --button-active: #069;
    --tag-bg: #e0e0e0;
    --tag-fg: #666;
    --tab-selected-bg: #e0e0e0;
    --tab-body-line: rgba(0, 0, 0, 0.1);
    --chip-line: #c8c8c8;
    --here-bg: #ffe8b1;
    --here-fg: #000;
    /* figures: the GIFs already carry a viewport of their own */
    --fig-line: rgba(0, 0, 0, 0.12);
    --fig-bg: #f6f6f6;
    /* parameter grid */
    --parm-label-bg: rgba(0, 0, 0, 0.05);
    --parm-content-bg: rgba(0, 0, 0, 0.075);
    --parm-gap: #fff;
    /* notices */
    --tip: #69cc00;
    --tip-bg: rgba(0, 255, 0, 0.05);
    --note: #9d4dff;
    --note-bg: rgba(255, 0, 255, 0.05);
    --warning: #ff3300;
    --warning-bg: rgba(255, 0, 0, 0.05);
    /* type */
    --sans: "Source Sans 3", "Source Sans Pro", "Segoe UI", Arial,
        sans-serif;
    --mono: "Source Code Pro", Consolas, "Courier New", monospace;
}

/* ---- dark scheme -------------------------------------------------
   The SideFX help at night over a usdview viewport: a near-black page,
   warm amber for the node you are on, and a page-coloured gap between
   the parameter cells so the striped grid still reads.  Follows the
   system preference; [data-theme] is only a hook for forcing one.  The
   --dark-* names are just the palette, held once and switched on by the
   two blocks below, so the media query and the attribute cannot drift. */
:root {
    --dark-nav-dim: #8d8d8d;
    --dark-search-bg: rgba(255, 255, 255, 0.08);
    --dark-search-border: #555;
    --dark-search-fg: #e6e6e6;
    --dark-search-icon: url("data:image/svg+xml;charset=utf-8,%3Csvg \
xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16' fill='none' \
stroke='%23aaa' stroke-width='1.6'%3E%3Ccircle cx='6.6' cy='6.6' r='4.4'\
/%3E%3Cpath d='M10 10l4 4'/%3E%3C/svg%3E");
    --dark-page-bg: #1b1d20;
    --dark-ink: #d8d8d8;
    --dark-ink-strong: #f2f2f2;
    --dark-muted: #9a9a9a;
    --dark-faint: #6e6e6e;
    --dark-rule: rgba(255, 255, 255, 0.12);
    --dark-rule-soft: rgba(255, 255, 255, 0.07);
    --dark-wash: #24272b;
    --dark-meta-bg: #25282c;
    --dark-pre-bg: #121416;
    --dark-pre-fg: #e6e6e6;
    --dark-toc-bg: #1f2124;
    --dark-toc-fg: #e0e0e0;
    --dark-toc-line: #2a2d31;
    --dark-link: #58a4ff;
    --dark-link-visited: #8ab4f8;
    --dark-link-hover: #9cc8ff;
    --dark-teal: #5fb6ba;
    --dark-button-active: #9cc8ff;
    --dark-tag-bg: #333;
    --dark-tag-fg: #bbb;
    --dark-tab-selected-bg: #34383d;
    --dark-tab-body-line: rgba(255, 255, 255, 0.12);
    --dark-chip-line: #555;
    --dark-here-bg: #4a3a1a;
    --dark-here-fg: #f90;
    --dark-fig-line: rgba(255, 255, 255, 0.15);
    --dark-fig-bg: #111;
    --dark-parm-label-bg: rgba(255, 255, 255, 0.05);
    --dark-parm-content-bg: rgba(255, 255, 255, 0.075);
    --dark-tip-bg: rgba(0, 255, 0, 0.08);
    --dark-note-bg: rgba(255, 0, 255, 0.08);
    --dark-warning-bg: rgba(255, 0, 0, 0.08);
}

@media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
        --nav-dim: var(--dark-nav-dim);
        --search-bg: var(--dark-search-bg);
        --search-border: var(--dark-search-border);
        --search-fg: var(--dark-search-fg);
        --search-icon: var(--dark-search-icon);
        --page-bg: var(--dark-page-bg);
        --ink: var(--dark-ink);
        --ink-strong: var(--dark-ink-strong);
        --muted: var(--dark-muted);
        --faint: var(--dark-faint);
        --rule: var(--dark-rule);
        --rule-soft: var(--dark-rule-soft);
        --wash: var(--dark-wash);
        --meta-bg: var(--dark-meta-bg);
        --pre-bg: var(--dark-pre-bg);
        --pre-fg: var(--dark-pre-fg);
        --toc-bg: var(--dark-toc-bg);
        --toc-fg: var(--dark-toc-fg);
        --toc-line: var(--dark-toc-line);
        --link: var(--dark-link);
        --link-visited: var(--dark-link-visited);
        --link-hover: var(--dark-link-hover);
        --teal: var(--dark-teal);
        --button-active: var(--dark-button-active);
        --tag-bg: var(--dark-tag-bg);
        --tag-fg: var(--dark-tag-fg);
        --tab-selected-bg: var(--dark-tab-selected-bg);
        --tab-body-line: var(--dark-tab-body-line);
        --chip-line: var(--dark-chip-line);
        --here-bg: var(--dark-here-bg);
        --here-fg: var(--dark-here-fg);
        --fig-line: var(--dark-fig-line);
        --fig-bg: var(--dark-fig-bg);
        --parm-label-bg: var(--dark-parm-label-bg);
        --parm-content-bg: var(--dark-parm-content-bg);
        --parm-gap: var(--dark-page-bg);
        --tip-bg: var(--dark-tip-bg);
        --note-bg: var(--dark-note-bg);
        --warning-bg: var(--dark-warning-bg);
    }
}

:root[data-theme="dark"] {
    --nav-dim: var(--dark-nav-dim);
    --search-bg: var(--dark-search-bg);
    --search-border: var(--dark-search-border);
    --search-fg: var(--dark-search-fg);
    --search-icon: var(--dark-search-icon);
    --page-bg: var(--dark-page-bg);
    --ink: var(--dark-ink);
    --ink-strong: var(--dark-ink-strong);
    --muted: var(--dark-muted);
    --faint: var(--dark-faint);
    --rule: var(--dark-rule);
    --rule-soft: var(--dark-rule-soft);
    --wash: var(--dark-wash);
    --meta-bg: var(--dark-meta-bg);
    --pre-bg: var(--dark-pre-bg);
    --pre-fg: var(--dark-pre-fg);
    --toc-bg: var(--dark-toc-bg);
    --toc-fg: var(--dark-toc-fg);
    --toc-line: var(--dark-toc-line);
    --link: var(--dark-link);
    --link-visited: var(--dark-link-visited);
    --link-hover: var(--dark-link-hover);
    --teal: var(--dark-teal);
    --button-active: var(--dark-button-active);
    --tag-bg: var(--dark-tag-bg);
    --tag-fg: var(--dark-tag-fg);
    --tab-selected-bg: var(--dark-tab-selected-bg);
    --tab-body-line: var(--dark-tab-body-line);
    --chip-line: var(--dark-chip-line);
    --here-bg: var(--dark-here-bg);
    --here-fg: var(--dark-here-fg);
    --fig-line: var(--dark-fig-line);
    --fig-bg: var(--dark-fig-bg);
    --parm-label-bg: var(--dark-parm-label-bg);
    --parm-content-bg: var(--dark-parm-content-bg);
    --parm-gap: var(--dark-page-bg);
    --tip-bg: var(--dark-tip-bg);
    --note-bg: var(--dark-note-bg);
    --warning-bg: var(--dark-warning-bg);
}

*, *::before, *::after { box-sizing: border-box; text-indent: 0; }

html { font-size: 15px; padding: 0; }

body, input, button {
    font-family: var(--sans);
}

body {
    margin: 0;
    padding: 32px 0 0;
    line-height: 1.5em;
    color: var(--ink);
    background-color: var(--page-bg);
    overflow-wrap: break-word;
}

p { margin: 1em 0; }

a:link { color: var(--link); text-decoration: none; }
a:visited { color: var(--link-visited); text-decoration: none; }
a:hover { color: var(--link-hover); text-decoration: underline; }

ul, ol { margin: 0 0 0 2em; padding: 0; }
ul > li { list-style: disc; }
li { margin: 0.5em 0; line-height: 1.5em; }
li > :first-child { margin-top: 0; }
li > :last-child { margin-bottom: 0; }

img { max-width: 100%; }

/* ---- code ------------------------------------------------------- */
pre, code, tt, kbd {
    font-family: var(--mono);
}
code { font-size: 0.92em; }
pre {
    padding: 0.5em 0.75em;
    overflow: auto;
    line-height: 1.35rem;
    color: var(--pre-fg);
    background-color: var(--pre-bg);
    font-size: 0.875rem;
}
pre code { font-size: inherit; }

/* ---- tables ----------------------------------------------------- */
table, td, th { font-size: 1rem; }
table {
    border-collapse: collapse;
    border-spacing: 0;
    margin: 1em 0;
}
td, th { text-align: left; vertical-align: top; padding: 0.7em 0.8em; }
th {
    font-weight: 800;
    border-bottom: 1px solid var(--rule);
    background-color: transparent;
    padding-bottom: 0.35em;
}
td { border-top: 1px solid var(--rule-soft); }
table tr:first-child > td { border-top: none; }
table.table { width: 100%; }
.table-scroll { overflow-x: auto; margin: 1em 0; }
.table-scroll > table { margin: 0; min-width: 26rem; }

/* ---- top nav ---------------------------------------------------- */
nav#topnav {
    position: fixed;
    z-index: 10;
    top: 0;
    left: 0;
    width: 100%;
    height: 32px;
    min-height: 32px;
    overflow: hidden;
    display: flex;
    align-items: center;
    gap: 0.75em;
    padding: 0 0.75em;
    background-color: var(--nav-bg);
}
.brandname {
    font-size: 18px;
    line-height: 32px;
    color: var(--nav-dim);
    white-space: nowrap;
}
.brandname a:link, .brandname a:visited, .brandname a:hover {
    color: var(--nav-fg);
    text-decoration: none;
    font-weight: 600;
}
#navsearch { margin-left: auto; display: flex; align-items: center; }
.barlinks {
    display: flex;
    align-items: center;
    gap: 0.9em;
    font-size: 0.875rem;
    white-space: nowrap;
}
.barlinks a:link, .barlinks a:visited { color: var(--nav-dim); }
.barlinks a:hover { color: var(--nav-fg); text-decoration: none; }
#q {
    width: 20em;
    max-width: 44vw;
    padding: 0.2rem 0.5rem 0.2rem 1.75rem;
    border: 1px solid var(--search-border);
    border-radius: 1rem;
    background-color: var(--search-bg);
    background-image: var(--search-icon);
    background-repeat: no-repeat;
    background-position: 0.4rem 50%;
    background-size: 0.95rem 0.95rem;
    color: var(--search-fg);
    font-size: 0.875rem;
    line-height: 1.2;
}
#q:focus { outline: 2px solid var(--link-hover); outline-offset: -1px; }

/* ---- table of contents ------------------------------------------ */
.toc-switch {
    position: absolute;
    opacity: 0;
    width: 1px;
    height: 1px;
}
.toc-toggle {
    display: none;
    padding: 0.5em 1em;
    background-color: var(--wash);
    border-bottom: 1px solid var(--rule);
    font-weight: 600;
    cursor: pointer;
}
.toc-toggle::before { content: "\\25B8\\00a0"; color: var(--muted); }
.toc-switch:checked ~ .toc-toggle::before { content: "\\25BE\\00a0"; }

#toc {
    position: fixed;
    top: 32px;
    left: 0;
    width: 20%;
    height: calc(100vh - 32px);
    overflow: auto;
    font-size: 0.857em;
    padding: 0.75em 0 3em;
    border-right: 1px solid var(--toc-line);
    background-color: var(--toc-bg);
}
#toc-body { padding: 0 1em; opacity: 0.6; transition: opacity 0.25s; }
#toc:hover > #toc-body, #toc:focus-within > #toc-body { opacity: 1; }
#toc .tochome {
    display: block;
    font-weight: 600;
    color: var(--toc-fg);
    margin-bottom: 0.5em;
}
#toc h3 {
    margin: 1.25em 0 0.25em;
    font-size: 0.95em;
    font-weight: 600;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--muted);
}
#toc ul { margin: 0; padding: 0; }
#toc li {
    list-style: none;
    margin: 0 0 0 -0.125em;
    padding: 0.125em;
    border: 1px solid transparent;
}
#toc li:hover { border-color: var(--rule-soft); }
#toc a:link, #toc a:visited {
    color: var(--toc-fg);
    display: flex;
    align-items: center;
    gap: 0.5em;
}
#toc a:hover { color: var(--link-hover); }
#toc img { width: 16px; height: 16px; flex: none; }
#toc li.here {
    background-color: var(--here-bg);
    position: relative;
    margin-right: 12px;
}
#toc li.here::after {
    content: "";
    position: absolute;
    width: 0;
    height: 0;
    top: -0.05em;
    right: -12px;
    border: 0 solid var(--here-bg);
    border-left-width: 12px;
    border-left-style: solid;
    border-right: 0 solid transparent;
    border-top: 12px solid transparent;
    border-bottom: 12px solid transparent;
}
#toc li.here > a:link, #toc li.here > a:visited {
    font-weight: 600;
    color: var(--here-fg);
}

/* ---- main column ------------------------------------------------ */
main { margin-left: 20%; margin-bottom: 32px; }
#title, #content { max-width: 60rem; margin: 0 auto; padding: 0 1em 1em; }
#content { padding-bottom: 6rem; }
#content > :first-child { margin-top: 0; }
header { padding-top: 1.5rem; margin-bottom: 1rem; }
.title-content { position: relative; }
.title-content::after { content: ""; display: block; clear: both; }

p.ancestors { margin: 0 0 0.25em; font-size: 0.857rem; color: var(--muted); }
.pathsep { margin: 0 0.4em; font-style: normal; color: var(--faint); }

.pageicon { float: left; margin-right: 1rem; }
.pageicon img { width: 3rem; height: 3rem; }
h1.title {
    margin: 0;
    font-size: 2rem;
    font-weight: 300;
    line-height: 1.15;
}
h1.title .nodename { font-weight: 600; }
h1.title .subtitle { font-weight: 300; color: var(--muted); }
h1.title .tag {
    font-size: 0.857rem;
    padding: 0.1rem 0.5rem;
    background-color: var(--tag-bg);
    color: var(--tag-fg);
    border-radius: 1rem;
    vertical-align: super;
}
p.summary { font-style: italic; font-weight: 300; margin: 0.35em 0 0; }
/* keep the title text in its own column beside the floated page icon */
.title-content > h1.title, .title-content > p.summary { display: flow-root; }

/* ---- metatable -------------------------------------------------- */
table.metatable {
    background-color: var(--meta-bg);
    border-radius: 8px;
    border-style: none;
    width: 100%;
    margin: 1em 0;
}
table.metatable td { border: none; }
table.metatable td.label { font-weight: bold; width: 8rem; }
table.metatable ul.minitoc {
    margin: 0;
    padding: 0;
    display: flex;
    flex-wrap: wrap;
    gap: 0 1.25em;
}
table.metatable ul.minitoc li { list-style: none; margin: 0; }

/* ---- sections --------------------------------------------------- */
section.heading { margin: 1em 0; }
h1, h2, h3, h4 { font-family: var(--sans); margin: 1.5em 0 0.75em; }
h2.heading {
    font-size: 2rem;
    font-weight: 500;
    line-height: 1.15;
    margin: 1em 0 0.5em;
    padding-bottom: 0.15em;
    border-bottom: 1px solid var(--rule);
}
h3 { font-size: 1.65rem; font-weight: 600; line-height: 1.2; }
h4 { font-size: 1.4rem; font-weight: 700; line-height: 1.2; }
.headerlink { font-size: 1rem; font-weight: 400; }
.headerlink a:link, .headerlink a:visited {
    color: var(--faint);
    visibility: hidden;
}
h2:hover .headerlink a:link, h2:hover .headerlink a:visited,
.headerlink a:focus { visibility: visible; }

/* ---- figures ---------------------------------------------------- */
figure.fig { margin: 1.5em 0; }
figure.fig img {
    display: block;
    max-width: 100%;
    height: auto;
    border: 1px solid var(--fig-line);
    background-color: var(--fig-bg);
}
figure.fig figcaption {
    margin-top: 0.4em;
    font-size: 0.857rem;
    color: var(--muted);
}

/* ---- notices ---------------------------------------------------- */
.notice {
    padding: 0.25em 1em 0.25em 1.5em;
    margin: 1em 0;
    border-left: 4px solid var(--ink-strong);
    background-color: var(--rule-soft);
}
.notice > .label { font-weight: bold; margin-top: 0.75em; }
.notice ul { margin-left: 1.25em; }
.notice.tip { border-left-color: var(--tip); background-color: var(--tip-bg); }
.notice.tip > .label { color: var(--tip); }
.notice.note {
    border-left-color: var(--note);
    background-color: var(--note-bg);
}
.notice.note > .label { color: var(--note); }
.notice.warning {
    border-left-color: var(--warning);
    background-color: var(--warning-bg);
}
.notice.warning > .label { color: var(--warning); }

/* ---- parameters: tabs + side-by-side grid ----------------------- */
.tab-group { position: relative; margin: 1.5em auto; clear: both; }
.tab-group .tab-heading {
    display: flex;
    flex-wrap: wrap;
    gap: 2px;
    margin-bottom: 0.4rem;
}
.tab-group .tab-heading > .label {
    position: relative;
    display: inline-block;
    font-size: 0.857em;
    color: var(--link);
    background-color: transparent;
    padding: 0.1rem 0.75rem;
    cursor: pointer;
}
.tab-group .tab-heading > .label:hover {
    color: var(--link-hover);
    text-decoration: none;
    background-color: var(--wash);
}
.tab-group .tab-heading > .label.selected {
    background-color: var(--tab-selected-bg);
    color: var(--ink-strong);
}
.tab-group .tab-heading > .label.selected::after {
    content: "";
    position: absolute;
    width: 0;
    height: 0;
    left: 50%;
    margin-left: -6px;
    top: 100%;
    border: 6px solid transparent;
    border-top-color: var(--tab-selected-bg);
    border-bottom-width: 0;
}
.tab-group .tab-bodies {
    border: 1px solid var(--tab-body-line);
    padding: 1em 0.5em 0.5em;
}
.tab-group .tab-bodies > .content + .content { margin-top: 1.5em; }
p.tab-body-label {
    margin: 0 0 0.5em;
    padding: 0.3em 0.8em;
    font-weight: 600;
    font-size: 1rem;
    color: var(--ink-strong);
    background-color: var(--wash);
    border: 1px solid var(--rule-soft);
}
.tab-bodies > .content:target > p.tab-body-label { color: var(--accent); }

.parameters.sbs-group {
    display: table;
    width: 100%;
    border-collapse: collapse;
}
.parameter.sbs-item { display: table-row; }
.parameter.sbs-item > .label {
    display: table-cell;
    width: 33%;
    margin: 0;
    font-weight: 600;
    text-align: right;
    padding: 0.5em 1em;
    border: 1px solid var(--parm-gap);
    background-color: var(--parm-label-bg);
}
.parameter.sbs-item > .content {
    display: table-cell;
    width: 67%;
    padding: 0.5em 1em;
    border: 1px solid var(--parm-gap);
    background-color: var(--parm-content-bg);
}
.parameter > .content > :first-child { margin-top: 0; }
.parameter > .content > :last-child { margin-bottom: 0; }
.parameter > .label .pname {
    font-family: var(--mono);
    font-weight: 600;
    font-size: 0.95em;
    overflow-wrap: anywhere;
}
.parameter > .label .sig {
    display: block;
    margin-top: 0.15em;
    font-family: var(--mono);
    font-size: 0.75rem;
    line-height: 1.4;
    font-weight: 400;
    color: var(--muted);
    overflow-wrap: anywhere;
}
.parameter code { overflow-wrap: break-word; }

dl.menu { margin: 0 0 0.6em; font-size: 0.93em; }
dl.menu dt {
    float: left;
    clear: left;
    width: 7rem;
    font-weight: 600;
    color: var(--teal);
}
dl.menu dd { margin: 0 0 0.2em 7rem; }
dl.menu::after { content: ""; display: block; clear: both; }
.chips { display: flex; flex-wrap: wrap; gap: 0.25rem; }
.chip {
    font-family: var(--mono);
    font-size: 0.78rem;
    line-height: 1.6;
    padding: 0 0.4em;
    border: 1px solid var(--chip-line);
    border-radius: 2px;
    background-color: var(--page-bg);
    overflow-wrap: anywhere;
}

/* ---- example ---------------------------------------------------- */
.example { margin-top: 2em; clear: left; }
.example-buttons { float: left; width: 12em; }
.example > .label { font-weight: bold; margin-left: 12em; margin-bottom: 0; }
.example > .content { margin-left: 12em; }
.example > .content > .summary { margin-top: 0; }
.button {
    display: block;
    border: 1px solid var(--button);
    background-color: transparent;
    color: var(--button);
    border-radius: 0;
    margin: 0 0.75em 0.5em 0;
    padding: 0.5em 1em;
    font-weight: normal;
    text-align: center;
    cursor: pointer;
}
a.button:link, a.button:visited { color: var(--button); }
.button:hover {
    background-color: var(--button);
    color: var(--page-bg);
    text-decoration: none;
}
.button:active { background-color: var(--button-active); }
.button.primary {
    border-color: var(--accent);
    color: var(--accent);
    font-weight: 600;
}
a.button.primary:link, a.button.primary:visited { color: var(--accent); }
.button.primary:hover {
    background-color: var(--accent);
    color: var(--page-bg);
}
.button.primary:active { background-color: var(--accent-active); }

/* ---- see also + category lists ---------------------------------- */
ul.seealso { list-style: none; margin: 0; padding: 0; }
ul.seealso li { margin: 0.35em 0; list-style: none; }
ul.seealso a { display: inline-flex; align-items: center; gap: 0.5em; }
ul.seealso img { width: 20px; height: 20px; flex: none; }

ul.catlist { list-style: none; margin: 1em 0; padding: 0; }
ul.catlist li {
    display: flex;
    align-items: baseline;
    flex-wrap: wrap;
    gap: 0 0.6em;
    margin: 0;
    padding: 0.35em 0;
    border-top: 1px solid var(--rule-soft);
    list-style: none;
}
ul.catlist li:first-child { border-top: none; }
ul.catlist img {
    width: 20px;
    height: 20px;
    flex: none;
    align-self: center;
}
ul.catlist .t { font-weight: 600; white-space: nowrap; }
ul.catlist li.current .t { color: var(--ink-strong); }
ul.catlist .d { margin: 0; color: var(--muted); font-size: 0.93em; }

/* ---- footer ----------------------------------------------------- */
.footer {
    margin-top: 3em;
    padding-top: 0.75em;
    border-top: 1px solid var(--rule);
    color: var(--muted);
    font-size: 0.857rem;
}

/* ---- responsive ------------------------------------------------- */
@media (max-width: 72rem) {
    main { margin-left: 25%; }
    #toc { width: 25%; }
}
@media (max-width: 50rem) {
    html { font-size: 14px; }
    main { margin-left: 0; }
    #title, #content { padding-left: 16px; padding-right: 16px; }
    .toc-toggle { display: block; }
    #toc {
        display: none;
        position: static;
        width: auto;
        height: auto;
        max-height: 70vh;
        border-right: none;
        border-bottom: 1px solid var(--rule);
    }
    .toc-switch:checked ~ #toc { display: block; }
    #toc-body { opacity: 1; }
    #q { width: 12em; }
    .brandname .brandsub { display: none; }
    h1.title { font-size: 1.6rem; }
    h2.heading { font-size: 1.45rem; }
    h3 { font-size: 1.25rem; }
    h4 { font-size: 1.1rem; }
    .pageicon img { width: 2.25rem; height: 2.25rem; }
    .parameters.sbs-group, .parameter.sbs-item,
    .parameter.sbs-item > .label, .parameter.sbs-item > .content {
        display: block;
        width: auto;
    }
    .parameter.sbs-item > .label { text-align: left; }
    .parameter.sbs-item { margin-bottom: 0.5em; }
    .parameter.sbs-item > .label .sig { display: inline; margin-left: 0.6em; }
    .parameter.sbs-item > .content.empty { display: none; }
    .parameter.sbs-item > .label, .parameter.sbs-item > .content {
        padding: 0.5em 0.7em;
    }
    .tab-group .tab-bodies { padding: 0.75em 0.4em 0.4em; }
    .example-buttons { float: none; width: auto; display: flex; gap: 0.5em; }
    .example-buttons .button { flex: 1; margin-right: 0; }
    .example > .label, .example > .content { margin-left: 0; }
    table.metatable td, table.metatable td.label { padding: 0.5em; }
    table.metatable td.label { width: 6.5rem; }
    .table-scroll > table { min-width: 0; }
    .table-scroll td, .table-scroll th { padding: 0.5em 0.4em; }
    .table-scroll code, table.metatable code { overflow-wrap: anywhere; }
    dl.menu dt { float: none; width: auto; }
    dl.menu dd { margin-left: 0; }
}
@media (prefers-reduced-motion: reduce) {
    * { transition: none !important; animation: none !important; }
}
"""

JS = """\
<script>
function rigFilter(q) {
    q = q.toLowerCase();
    var nodes = document.querySelectorAll('#toc li.node');
    for (var i = 0; i < nodes.length; i++) {
        var li = nodes[i];
        li.style.display = li.getAttribute('data-title').indexOf(q) >= 0
            ? '' : 'none';
    }
    var groups = document.querySelectorAll('#toc .navgroup');
    for (var g = 0; g < groups.length; g++) {
        var items = groups[g].querySelectorAll('li.node');
        var any = false;
        for (var k = 0; k < items.length; k++) {
            if (items[k].style.display !== 'none') { any = true; break; }
        }
        groups[g].style.display = any ? '' : 'none';
    }
    var sw = document.getElementById('toc-switch');
    if (sw && q) { sw.checked = true; }
}
function rigTab(id) {
    var labels = document.querySelectorAll('.tab-heading > .label');
    var hit = false;
    for (var i = 0; i < labels.length; i++) {
        var on = labels[i].getAttribute('href') === '#' + id;
        labels[i].className = on ? 'label selected' : 'label';
        hit = hit || on;
    }
    if (!hit && labels.length) { labels[0].className = 'label selected'; }
}
window.addEventListener('hashchange', function () {
    rigTab(location.hash.replace('#', ''));
});
</script>
"""


def _slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def _md_block(text):
    return markdown.markdown(text.strip(), extensions=["fenced_code"])


def _md_inline(text):
    out = markdown.markdown(text.strip(), extensions=["fenced_code"])
    if out.startswith("<p>") and out.endswith("</p>"):
        out = out[3:-4]
    return out


def _esc(text):
    return _html.escape(text, quote=True)


def _section(ident, title, inner):
    return ('<section class="heading">\n'
            '<h2 class="label heading" id="%s">%s '
            '<span class="headerlink"><a href="#%s" '
            'title="Link to this section">&para;</a></span></h2>\n'
            '<div class="content" id="%s-body">\n%s\n</div>\n</section>'
            % (ident, _esc(title), ident, ident, inner))


def _sidebar(active, prefix):
    chunks = ['<label class="toc-toggle" for="toc-switch">Nodes</label>',
              '<div id="toc"><div id="toc-body">',
              '<a class="tochome" href="%sindex.html">RigExec nodes</a>'
              % prefix]
    for category, keys in CATEGORIES:
        chunks.append('<div class="navgroup">')
        chunks.append("<h3>%s</h3>" % _esc(category))
        chunks.append("<ul>")
        for key in keys:
            note = OPERATORS[key]
            cls = "node here" if key == active else "node"
            chunks.append(
                '<li class="%s" data-title="%s">'
                '<a href="%snodes/%s.html">'
                '<img src="%sicons/%s.png" alt="">'
                "<span>%s</span></a></li>"
                % (cls, _esc(note["title"].lower()), prefix, key,
                   prefix, key, _esc(note["title"])))
        chunks.append("</ul></div>")
    chunks.append("</div></div>")
    return "\n".join(chunks)


def _shell(title, prefix, sidebar, header, body):
    return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="light dark">
<title>%s &mdash; RigExec nodes</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="%s">
<link rel="stylesheet" href="%sstyle.css">
</head>
<body>
<nav id="topnav" role="navigation">
<span class="brandname"><a href="%sindex.html">RigExec</a>\
<span class="brandsub">&nbsp;node reference</span></span>
<span id="navsearch"><input id="q" type="search" placeholder="Filter nodes"
 aria-label="Filter nodes" oninput="rigFilter(this.value)"></span>
<span class="barlinks">
<a href="%sexamples/">Examples</a>
<a href="%sspecs/">Specs</a>
</span>
</nav>
<input class="toc-switch" id="toc-switch" type="checkbox">
%s
<main>
<header>
%s
</header>
<div id="content">
%s
</div>
</main>
%s
</body>
</html>
""" % (_esc(title), FONTS, prefix, prefix, prefix, prefix, sidebar,
       header, body, JS)


def _titleblock(icon, prefix, ancestors, name, suffix, summary):
    chunks = ['<div id="title" class="title-content with-icon">',
              '<p class="ancestors">%s</p>' % ancestors]
    if icon:
        chunks.append('<div class="pageicon">'
                      '<img src="%sicons/%s.png" alt=""></div>'
                      % (prefix, icon))
    chunks.append('<h1 class="title"><span class="nodename">%s</span>'
                  '<span class="subtitle"> %s</span></h1>'
                  % (_esc(name), _esc(suffix)))
    chunks.append('<p class="summary">%s</p>' % summary)
    chunks.append("</div>")
    return "\n".join(chunks)


def _sig(attr):
    """The small monospace type/default line under a parameter name."""
    if attr["type"] == "rel":
        return "relationship"
    if attr["default"]:
        return "%s, default %s" % (attr["type"], attr["default"])
    return attr["type"]


def _parm_html(attr, seen=None):
    # A page can print the same attribute in two groups -- a weight page
    # carries `rigExec:representation` under both `RigExecWeightObject`
    # and its own node parameters -- and one id used twice is an invalid
    # document whose anchor links resolve to whichever came first. The
    # repeat is numbered instead.
    slug = "parm-" + _slug(attr["name"])
    if seen is not None:
        seen[slug] = seen.get(slug, 0) + 1
        if seen[slug] > 1:
            slug = "%s-%d" % (slug, seen[slug])
    chunks = ['<div class="parameter sbs-item" id="%s">' % slug,
              '<p class="label"><code class="pname">%s</code>'
              '<span class="sig">%s</span></p>'
              % (_esc(attr["name"]), _esc(_sig(attr))),
              '<div class="content">']
    filled = bool(attr["tokens"] or attr["doc"])
    if attr["tokens"]:
        chunks.append('<dl class="menu"><dt>Valid values</dt>'
                      '<dd><span class="chips">%s</span></dd></dl>'
                      % "".join('<span class="chip">%s</span>' % _esc(t)
                                for t in attr["tokens"]))
    if attr["doc"]:
        chunks.append(_md_block(attr["doc"]))
    if not filled:
        # A bare relationship says everything in its label; the empty cell
        # keeps the grid square on desktop and is dropped when it stacks.
        chunks[2] = '<div class="content empty">'
    chunks.append("</div></div>")
    return "\n".join(chunks)


def _category_of(key):
    for category, keys in CATEGORIES:
        if key in keys:
            return category
    raise KeyError(key)


def _catlist(keys, icon_prefix, link_prefix, current=None):
    out = ['<ul class="catlist">']
    for key in keys:
        note = OPERATORS[key]
        icon = '<img src="%sicons/%s.png" alt="">' % (icon_prefix, key)
        if key == current:
            out.append('<li class="current">%s<span class="t">%s</span>'
                       '<p class="d">%s</p></li>'
                       % (icon, _esc(note["title"]),
                          _md_inline(note["summary"])))
        else:
            out.append('<li>%s<a class="t" href="%s%s.html">%s</a>'
                       '<p class="d">%s</p></li>'
                       % (icon, link_prefix, key, _esc(note["title"]),
                          _md_inline(note["summary"])))
    out.append("</ul>")
    return "\n".join(out)


def render_node(key, category, classes):
    note = OPERATORS[key]
    schema = classes[note["schema"]]
    stage = example_stage(key, note)
    ancestors = ('<a href="../index.html">RigExec nodes</a>'
                 '<i class="pathsep">&rsaquo;</i>'
                 '<a href="../index.html#%s">%s</a>'
                 '<i class="pathsep">&rsaquo;</i>'
                 % (_slug(category), _esc(category)))
    header = _titleblock(key, "../", ancestors, note["title"], "rig node",
                         _md_inline(note["summary"]))

    body = []
    # -- metatable -------------------------------------------------
    onpage = " ".join('<li><a href="#%s">%s</a></li>' % (ident, title)
                      for ident, title in SECTIONS)
    body.append('<table id="premeta" class="metatable">')
    body.append('<tr><td class="label">On this page</td>'
                '<td class="content"><ul class="minitoc">%s</ul></td></tr>'
                % onpage)
    body.append('<tr><td class="label">Node type</td>'
                '<td class="content"><code>%s</code></td></tr>'
                % _esc(note["schema"]))
    if stage:
        body.append('<tr><td class="label">Example</td>'
                    '<td class="content"><a href="../examples/%s.usda">'
                    "%s.usda</a></td></tr>" % (stage, stage))
    body.append('<tr><td class="label">Category</td>'
                '<td class="content"><a href="../index.html#%s">%s</a>'
                "</td></tr>" % (_slug(category), _esc(category)))
    body.append("</table>")

    # -- overview --------------------------------------------------
    inner = []
    if not note.get("no_gif"):
        inner.append('<figure class="fig effect">'
                     '<img src="../gifs/%s.gif" width="640" height="360" '
                     'alt="%s in motion">'
                     "<figcaption>From "
                     '<a href="../examples/%s.usda">%s.usda</a>, rendered '
                     "live in an offscreen Storm viewport with the rig "
                     "guides on.</figcaption></figure>"
                     % (key, _esc(note["title"]), stage, stage))
    inner.append(_md_block(note["description"]))
    if schema["doc"]:
        inner.append(_md_block(schema["doc"]))
    body.append(_section("overview", "Overview", "\n".join(inner)))

    # -- how it works ----------------------------------------------
    body.append(_section("how-it-works", "How it works",
                         _md_block(note["how_it_works"])))

    # -- wiring ----------------------------------------------------
    inner = ['<div class="table-scroll"><table class="table">',
             "<tr><th>Relationship</th><th>Points to</th>"
             "<th>Required</th></tr>"]
    for rel, target, required in note["wiring"]:
        inner.append("<tr><td>%s</td><td>%s</td><td>%s</td></tr>"
                     % (_md_inline(rel), _md_inline(target),
                        _md_inline(required)))
    inner.append("</table></div>")
    body.append(_section("wiring", "Wiring", "\n".join(inner)))

    # -- parameters ------------------------------------------------
    groups = list(note.get("param_groups", []))
    groups.append(("Node parameters", note["schema"]))
    # The tab strip only earns its place when a page has more than one
    # group; the heading bar inside each body is always drawn, so a single
    # group is labelled once instead of twice.
    heads = ['<div class="tab-group">']
    if len(groups) > 1:
        heads.append('<div class="tab-heading">')
        for index, (group_title, _schema) in enumerate(groups):
            heads.append('<a class="label%s" href="#parms-%s">%s</a>'
                         % (" selected" if index == 0 else "",
                            _slug(group_title), _esc(group_title)))
        heads.append("</div>")
    bodies = ['<div class="tab-bodies">']
    seen_parms = {}
    for group_title, group_schema in groups:
        ident = "parms-" + _slug(group_title)
        bodies.append('<div class="content selected" id="%s">' % ident)
        bodies.append('<p class="tab-body-label">%s</p>' % _esc(group_title))
        bodies.append('<div class="parameters sbs-group">')
        for attr in classes[group_schema]["attrs"]:
            bodies.append(_parm_html(attr, seen_parms))
        bodies.append("</div></div>")
    bodies.append("</div></div>")
    body.append(_section("parameters", "Parameters",
                         "\n".join(heads + bodies)))

    # -- example ---------------------------------------------------
    inner = ['<div class="example">']
    if stage:
        inner.extend([
            '<div class="example-buttons">',
            '<span class="button" role="button" tabindex="0" '
            'title="bin\\launch_usdview.bat docs\\examples\\%s.usda">'
            "Open in usdview</span>" % stage,
            '<a class="button primary" href="../examples/%s.usda" '
            'download>Download .usda</a>' % stage,
            "</div>",
            '<p class="label">%s.usda</p>' % stage])
    inner.extend(['<div class="content">',
                  '<p class="summary">%s</p>' % _md_inline(note["example"])])
    if stage:
        inner.extend([
            "<p>Open it live with:</p>",
            "<pre><code>bin\\launch_usdview.bat docs\\examples\\%s.usda"
            "</code></pre>" % stage])
    if not note.get("no_gif"):
        inner.extend([
            "<p>Re-render the GIF above with:</p>",
            "<pre><code>python docs/render_media.py --page %s"
            "</code></pre>" % key])
    inner.append("</div></div>")
    body.append(_section("example", "Example", "\n".join(inner)))

    # -- tips ------------------------------------------------------
    inner = ['<div class="notice tip"><p class="label">Tips</p>',
             '<ul class="tips">']
    for tip in note["tips"]:
        inner.append("<li>%s</li>" % _md_inline(tip))
    inner.append("</ul></div>")
    body.append(_section("tips", "Tips", "\n".join(inner)))

    # -- see also --------------------------------------------------
    inner = ['<ul class="seealso">']
    for target, label in note["see_also"]:
        inner.append('<li><a href="%s.html">'
                     '<img src="../icons/%s.png" alt="">'
                     "<span>%s</span></a></li>"
                     % (target, target, _esc(label)))
    inner.append("</ul>")
    body.append(_section("see-also", "See also", "\n".join(inner)))

    # -- siblings --------------------------------------------------
    siblings = [k for _, keys in CATEGORIES for k in keys
                if _category_of(k) == category]
    body.append(_section("more", "More %s nodes" % category.lower(),
                         _catlist(siblings, "../", "", current=key)))
    body.append('<p class="footer"><a href="../index.html">RigExec nodes</a>'
                " reference</p>")
    return _shell(note["title"], "../", _sidebar(key, "../"), header,
                  "\n".join(body))


def render_index():
    ancestors = ('<a href="index.html">RigExec nodes</a>'
                 '<i class="pathsep">&rsaquo;</i>')
    header = _titleblock(None, "", ancestors, "RigExec", "node reference",
                         "Every rig operator, its wiring, its parameters, "
                         "and a minimal animated example.")
    body = []
    onpage = " ".join('<li><a href="#%s">%s</a></li>'
                      % (_slug(category), _esc(category))
                      for category, _ in CATEGORIES)
    body.append('<table id="premeta" class="metatable">')
    body.append('<tr><td class="label">On this page</td>'
                '<td class="content"><ul class="minitoc">%s</ul></td></tr>'
                % onpage)
    body.append('<tr><td class="label">Nodes</td>'
                '<td class="content">%d operators in %d categories</td></tr>'
                % (len(OPERATORS), len(CATEGORIES)))
    body.append('<tr><td class="label">Examples</td>'
                '<td class="content"><a href="examples/">examples/</a> '
                "(one stage per node)</td></tr>")
    body.append('<tr><td class="label">Specs</td>'
                '<td class="content"><a href="specs/">specs/</a> '
                "(design and implementation notes)</td></tr>")
    body.append("</table>")
    body.append("<p>One page per operator: what it does, how to wire it, "
                "every parameter, and a minimal animated example. Each "
                'example stage lives in <a href="examples/">examples/</a> '
                "and plays in usdview via "
                "<code>bin\\launch_usdview.bat</code>; the GIF on each page "
                "is rendered live from that stage by "
                "<code>docs/render_media.py</code>, an offscreen Storm "
                "viewport with the rig guides on.</p>")
    for category, keys in CATEGORIES:
        body.append(_section(_slug(category), category,
                             _catlist(keys, "", "nodes/")))
    body.append("<p>Implementation and design notes live in "
                '<a href="specs/">specs/</a>.</p>')
    body.append('<p class="footer">RigExec nodes reference</p>')
    return _shell("RigExec nodes", "", _sidebar(None, ""), header,
                  "\n".join(body))


def _copy_tree(src, dest, required=True):
    if not os.path.isdir(src):
        if required:
            raise SystemExit("media source not found: %s" % src)
        print("skipped missing %s" % src)
        return
    shutil.copytree(src, dest, ignore=shutil.ignore_patterns(".*"))


def main():
    classes = parse_schema(SCHEMA)
    # Every class a page reaches for, node schema and param groups alike:
    # an unchecked param group would surface as a bare KeyError deep in
    # render_page instead of naming the renamed class here.
    wanted = set()
    for note in OPERATORS.values():
        wanted.add(note["schema"])
        wanted.update(group_schema
                      for _title, group_schema in note.get("param_groups", []))
    missing = sorted(name for name in wanted if name not in classes)
    if missing:
        raise SystemExit("schema classes not found: %s" % ", ".join(missing))
    check_examples()
    if os.path.isdir(SITE):
        shutil.rmtree(SITE)
    nodes_dir = os.path.join(SITE, "nodes")
    os.makedirs(nodes_dir)
    with open(os.path.join(SITE, "style.css"), "w") as stream:
        stream.write(CSS)
    with open(os.path.join(SITE, "index.html"), "w") as stream:
        stream.write(render_index())
    print("wrote index.html")
    for key in OPERATORS:
        page = render_node(key, _category_of(key), classes)
        with open(os.path.join(nodes_dir, key + ".html"), "w") as stream:
            stream.write(page)
        print("wrote nodes/%s.html" % key)
    _copy_tree(os.path.join(RIG, "icons"), os.path.join(SITE, "icons"))
    _copy_tree(os.path.join(RIG, "docs", "gifs"), os.path.join(SITE, "gifs"))
    _copy_tree(os.path.join(RIG, "docs", "examples"),
               os.path.join(SITE, "examples"))
    _copy_tree(os.path.join(RIG, "docs", "specs"), os.path.join(SITE, "specs"))
    print("copied icons/, gifs/, examples/, specs/")


if __name__ == "__main__":
    main()
