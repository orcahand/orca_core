Install dependencies first with `uv`:

```sh
uv sync --group dev --group docs
```

To test localy:

```sh
uv run mkdocs serve --clean
```

To build: 

```sh
uv run mkdocs build
``` 

To test build:

```sh
uv run python -m http.server 8000 --directory site
``` 
