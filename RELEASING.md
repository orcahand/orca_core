# Releasing `orca_core` to PyPI

This repository publishes to PyPI through GitHub Actions using PyPI Trusted Publishing.

## One-time setup

1. Create the `orca_core` project on PyPI if it does not exist yet.
2. In PyPI, open the project settings and add a Trusted Publisher.
3. Use these GitHub settings:
   - Owner: `orcahand`
   - Repository name: `orca_core`
   - Workflow name: `release.yml`
   - Environment name: `pypi`
4. In GitHub, keep the workflow environment named `pypi`.
5. If you want an approval gate before publishing, add protection rules to the `pypi` environment in GitHub.

## Release flow

1. Update `[project].version` in `pyproject.toml`.
2. Commit and push the version bump to GitHub.
3. Create an annotated tag that matches the package version:

```sh
git tag -a v1.0.1 -m "Release v1.0.1"
git push origin v1.0.1
```

4. Pushing the tag triggers `.github/workflows/release.yml`.
5. The workflow:
   - checks that the tag matches `pyproject.toml`
   - installs dependencies with `uv`
   - runs `uv run pytest tests/`
   - builds the wheel and source distribution with `uv build`
   - publishes the release artifacts to PyPI

## Notes

- The workflow only publishes on tags shaped like `v*`.
- A tag such as `v1.0.1` must match `version = "1.0.1"` in `pyproject.toml`.
- Users can then install the latest published version with `pip install -U orca_core`.
