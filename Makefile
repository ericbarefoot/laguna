.PHONY: docs-serve docs-build docs-install

docs-install:
	pip install -e ".[docs]"

docs-serve:
	mkdocs serve

docs-build:
	mkdocs build --strict
