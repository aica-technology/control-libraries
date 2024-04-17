# Contributing

This repository is owned and maintained by [AICA SA](https://www.aica.tech/). We welcome user engagement to find bugs,
resolve issues and suggest useful features.

Before contributing to this repository, please first discuss the change you wish to make by opening an
[issue](https://github.com/aica-technology/control-libraries/issues).

When you are ready to contribute,
[fork the repository and open a Pull Request (PR) from your feature branch](https://docs.github.com/en/get-started/quickstart/contributing-to-projects). Refer to the [PR process](#pull-request-process) section for more information.

Before the PR can be accepted, contributors who are not employed by AICA must first read and agree to the
[Contributor License Agreement (CLA)](./licenses/CLA.md), which grants AICA the rights to use, modify and distribute
open source contributions.

## Development Environment

We provide a [VS Code Dev Containter configuration file](./.devcontainer/devcontainer.json) for development and testing
in a Docker container. Simply install VS Code and its Dev Container extension and then choose the
`Dev Containers: Open Folder in Container ...` option from the command palette.

## Style guide

This project follows the [AICA C++ Style Guide](https://github.com/aica-technology/.github/blob/main/guidelines/CPP_STYLE_GUIDE.md).

Doxygen headers are required for the public API.

## Unit Testing

Every new feature should have associated unit tests. This not only helps reduce the likelihood of bugs in the library,
but also helps the author and reviewers of a feature to understand what the expected behaviour really is.

Refer to the [Google Test Primer](https://github.com/google/googletest/blob/master/docs/primer.md) for a quickstart
guide to the testing framework.

If you are using a properly integrated development environment, you can run and debug the tests for each module locally
to check the behaviour.

You may also build the `python-test` stage of the main [Dockerfile](./Dockerfile) to build the project libraries and run
all tests within a Docker container. Because this process rebuilds the project from scratch, each evaluation may take
several minutes.

## Pull Request Process

1. Ensure your code follows the style guide and is portable; remove any references to local paths or files.
2. Check that the new changes are appropriately covered by unit tests.
3. Document the header files and public functions with doxygen comments, and update any relevant README.md
   or documentation files with details of changes to the interface.
4. Update the [changelog](CHANGELOG.md) with your feature / fix / improvement in the "Upcoming changes" section.
6. Open a pull request into the `main` branch. Write a meaningful title and description for the PR to make it
   clear what changes you have made, why you have made them, and how you have tested the changes.
7. You may merge the pull request into `main` once you have the sign-off of one other developer and all CI tests
   pass. Always use the "Squash and Merge" option to ensure your changes are contained within a single commit,
   maintaining a linear git history. If unsure, you may request another reviewer to merge it for you.

## Release strategy

All pull requests are merged into `main` using the "Squash and Merge" option. To trigger new releases, the version of
the project version is increased using traditional semantic versioning (major.minor.patch) at regular intervals.

The major release number should be incremented whenever there is a breaking change to the public API (for example, when
a previously existing function is deleted or renamed). Minor version numbers contain general new features and
improvements, while patch numbers represent simple and small fixes.

The `main` branch is always considered to be a "release candidate" that contains only release-ready code. If, at
release time, there are features on `main` that are considered unfinished or broken, they can be marked
as `EXPERIMENTAL` to exclude them from compilation.

At the time of release, usually when there is minor or major version update, a release branch should be made. The
release branch should be used to finalize the [changelog](CHANGELOG.md), which includes moving all content from
the "Upcoming changes (in development)" header under a new header with the corresponding release version.

The release on `main` should be tagged with the corresponding version as `vX.Y.Z`.
