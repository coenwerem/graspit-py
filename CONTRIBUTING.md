# Contributing

Bug reports, questions and pull requests are welcome through the GitHub
issue tracker.

## Reporting a problem

Include the wrapper commit, the GraspIt revision, how you built it (the
Docker image or a native build), the hand and object files, and a minimal
script that reproduces the problem.

## Development setup

The Docker image is the reference environment:

```bash
docker buildx build --load -f docker/Dockerfile -t graspit-py .
docker run --rm graspit-py      # runs the test suite under a virtual display
```

For a native build, follow [docs/INSTALL.md](docs/INSTALL.md) and run
`xvfb-run -a python -m pytest tests` from outside the source tree so the
tests import the installed package.

## Pull requests

- Keep each pull request to one change, with a test that fails without it.
- Pure-Python code (risk measures, kinematics, quality metrics, export)
  must stay importable without the native extension.
- New bindings need a docstring and a test in `tests/test_native.py`.
- By contributing you agree to license your contribution under the
  GPL-3.0-or-later terms of this project.
