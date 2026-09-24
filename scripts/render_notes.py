"""Render selected Markdown notes to offline HTML and verify local links."""
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit
import sys
import re

import markdown

ROOT = Path(__file__).resolve().parents[1]
STYLE = """
body{font:16px/1.8 'Microsoft YaHei',Arial,sans-serif;color:#20303b;background:#eef3f6;margin:0}
main{max-width:1100px;margin:28px auto;background:white;padding:32px 40px;border-top:5px solid #17698b}
h1{font-size:28px}h2{margin-top:30px}a{color:#096989}img{max-width:100%}
table{border-collapse:collapse;width:100%;font-size:14px}td,th{border:1px solid #cddbe5;padding:8px;text-align:left}
th{background:#e8f1f6}pre{padding:16px;background:#edf3f6;overflow:auto}code{font-family:Consolas,monospace}
@media(max-width:750px){main{margin:0;padding:16px}table{display:block;overflow:auto}}
"""


class LocalLinks(HTMLParser):
    def __init__(self, path):
        super().__init__()
        self.path = path
        self.count = 0

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key not in ("href", "src") or not value:
                continue
            url = urlsplit(value)
            if url.scheme or not url.path:
                continue
            target = self.path.parent / unquote(url.path)
            if not target.exists():
                raise FileNotFoundError(f"{self.path}: {value}")
            self.count += 1


def main():
    names = sys.argv[1:] or ["docs/noise_fix.md", "docs/TECHNICAL_OVERVIEW.md"]
    rendered = []
    for name in names:
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or path.suffix != ".md":
            raise ValueError("Only Markdown files inside this workspace are supported.")
        body = markdown.markdown(path.read_text(encoding="utf-8"), extensions=["tables", "fenced_code"])
        # Keep navigation inside the readable offline report when the linked
        # Markdown already has an HTML companion; leave source-only links.
        def offline_link(match):
            value = match.group(1)
            url = urlsplit(value)
            if not url.scheme and url.path.endswith(".md"):
                companion = path.parent / unquote(url.path[:-3] + ".html")
                if companion.is_file():
                    value = url._replace(path=url.path[:-3] + ".html").geturl()
            return 'href="' + value + '"'
        body = re.sub(r'href="([^"]+)"', offline_link, body)
        html = ('<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                '<link rel="icon" href="data:,">'
                '<title>杆接触力估计技术记录</title><style>' + STYLE + '</style></head><body><main>'
                + body + '</main></body></html>')
        path.with_suffix(".html").write_text(html, encoding="utf-8")
        rendered.append((path, html))
    for path, html in rendered:
        checker = LocalLinks(path)
        checker.feed(html)
        print(f"{path.relative_to(ROOT)} -> HTML; {checker.count} local links verified")


if __name__ == "__main__":
    main()
