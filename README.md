# eit-robotic-avatar
Move humans into robots. 

## Usage

`yarn install` To install dependencies.
`yarn start --https` To run the application.

`yarn start` Drop https if your are behind a https proxy like codespaces.

### Old Prof of concept.

`pipx install poetry` To install poetry.
`poetry install` To install dependencies.
`poetry run server` to start the main server application.

## Development
This project uses dev containers for development so all the tools/scrips needed to setup a working environment should be in the `.devcontainer` folder.

Its based on the [Python 3 template](https://github.com/microsoft/vscode-dev-containers/tree/main/containers/python-3). With node js support.

We use [pipx](https://pypa.github.io/pipx/) to install tools and [Poetry](https://python-poetry.org/) for dependency management for python.
