### This is not a proper solution but given our current setup it works for a basic api documentation. 

```
poetry run griffe dump orca_core > docs/orca_core_api.json
```

```
python docs/convert_griffe_to_md.py docs/orca_core_api.json docs/orca_core_core_api.mdx
```

Then mv the .mdx file in the docs website proper sidebar within repos directory. 
