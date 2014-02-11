
# GitHub Pages Branch

This branch of the repo contains NONE of the code, it is here only to hold the compiled Doxygen documentation of the code so it can be hosted on GitHub pages.

## Instructions:

1) On the master branch, run `doxygen` to generate all the documentation, which will be placed in the `api_docs` folder.

2) Check out the `gh-pages` branch (the one you're on currently if you're reading this).

3) Move everything from `api_docs/html` into the main folder: `mv api_docs/html/* ./`.

4) Commit the changes in documentation: `git add --all && git commit -m 'updated docs'`

5) Push the changes so they become live at http://robojackets.github.io/robocup-software

