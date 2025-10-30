# List available commands
[default]
_list:
    @just --list

# Format all source files
fmt:
    treefmt

# Run pre-commit hooks on all files
pre-commit:
    lefthook run --all-files --force pre-commit

# Release a new version on Github
release:
    #!/bin/sh -eux
    version=$(git cliff --bumped-version --with-commit 'chore(version): TODO')
    message="chore(version): $version"
    git cliff --bump -o CHANGELOG.md --with-commit "$message"
    git commit CHANGELOG.md -m "$message"

# Build & upload the container image
deliver:
    # TODO
    exit 1
    std //repo/containers/something:publish
