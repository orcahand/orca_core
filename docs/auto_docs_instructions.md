Install dependencies first (had issues with poetry, just pip install them)

```pip3 install -r requirements.txt```

To test localy:

```mkdocs serve --clean```

To build: 

```mkdocs build``` 

To test build:

```python -m http.server 8000 --directory site```