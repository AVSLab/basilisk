## Branches

-   Branches should be named `feature/bsk-XXX--short-desc` or `hotfix/short-desc`, where "bsk-XXX" is the associated
    Github project ticket.
    -   If a branch is not associated with a ticket, either it's a hotfix or it needs a ticket.
    -   Hotfixes should be used sparingly. For instance, a bug introduced in the same release cycle
    (or discovered very shortly after merging a PR) can be hotfixed. Bugs that a user may have been exposed to should
    be logged as tickets.

## Pull Requests

-   Pull requests should be made against `develop`, using the pull request template in `.github/pull_request_template.md`.

    -   GitHub will automatically populate a new PR with this template.
    -   Please fill out all information in the PR header, as well as any information in the subsections.
    -   Every PR should include a summary of changes that gives reviewers an idea of what they should pay attention to.
    -   Any unused or empty subsection may be removed.
-   PR branches should have as "clean" of a history as possible.
    -   Each commit should present one change or idea to a reviewer.
    -   Commits that merely "fix up" previous commits must be interactively rebased and squashed into their targets.
-   Prefer the use of `git rebase`.

    1.  `git rebase` `git rebase` actually _rebases_ your branch from the current development branch's endpoint. This
    localizes conflicts to the commits at which they actually appear, though it can become complicated when there are
    more than a few conflicts.
    2.  `git merge` `git merge` pulls in all the updates that your branch does not have, and combines them with the
    updates you have made in a single merge commit. This allows you to deal with any and all conflicts at once, but
    information such as when conflicts originated is lost.

    For more info on `git merge` vs `git rebase` see [here](https://www.atlassian.com/git/tutorials/merging-vs-rebasing).

-   Before merging a PR, the following requirements must be met. These requirements ensure that history is effectively
linear, which aids readability and makes `git bisect` more useful and easier to reason about.

    -   At least one (perferably two) reviewers have approved the PR.
    -   No outstanding review comments have been left unresolved.
    -   The branch passes continuous integration.
    -   The branch has been rebased onto the current `develop` branch.
-   The "Squash and merge" and "Rebase and merge" buttons on GitHub's PR interface should not be used (They are
disabled). Always use the "Merge" strategy.
    -   In combination with the restrictions above, this ensures that features are neatly bracketed by merge commits
    on either side, making a clear hierarchical separation between features added to `develop` and the work that went
    into each feature.

## Coding Conventions

A [coding conventions](https://hanspeterschaub.info/basilisk/Support/Developer/CodingGuidlines.html) document exists to
explain peculiarities and assist in onboarding.

-  All development should correspond to a Github ticket, and branch names and PRs should include the ticket name.

### pre-commit

Pre-commit is a tool used to automate code formatting for easy reading.
This allows the reviewer to focus on the architecture of a change and not simple nitpicks.

##### Installing pre-commit

Open a terminal window to perform the following tasks.
To install use the command:
```
pip install pre-commit
```
Verify pre-commit is installed with:
```
$ pre-commit --version
pre-commit 3.6.2
```

If you are using python virtual environments, you may need to activate your environment to use pre-commit.

Next, change your current directory to be the Basilisk repo folder.  Then run ```pre-commit install``` to set up the git hook scripts.
You must run this inside of the repo folder and the associated files will only be installed inside that repository.
```
$ pre-commit install
pre-commit installed at .git/hooks/pre-commit
```
Now ```pre-commit``` will run automatically whenever you run ```git commit```!

When ```pre-commit``` decides to edit some of your files,
you will need to add those changes to your commit and commit again.

#### ClangFormat

This repository uses a modified version of Mozilla's ```.clang-format``` file.
Please be sure to reformat any C/C++ file with clang-format before committing.
This can be done through the command line or with clion.

To prevent a section from getting reformatted, wrap it with ```// clang-format off``` and ```// clang-format on```.

For more information is available on the [clang-format](https://clang.llvm.org/docs/ClangFormat.html) website.

#### Clion

* To enable clang-format in clion, go to ```Settings - Editor - Code Style``` and enable ClangFormat.
* This will automatically detect the .clang-format file in the project root.
* To reformat code you have written, select that portion or the whole file
     and call <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>L</kbd>.

#### Command Line

* You must ```pip install clang-format``` to use ClangFormat through the command line.
* Use the command
  ```
  $ clang-format -i {file name(s)} -style=file
  ```
  where:
     * ```-i``` makes the suggested changes to the file, otherwise they will be outputted to the cli
     * ```-style=file``` tells ClangFormat to look for the .clang-format in your project directory,
       otherwise it will use the LLVM's style guide

## Write-Ups About Good Commit/PR/Code Review Practice

The following three articles describe in greater detail the ideals to which this repository adheres.

-   [How to write a good commit message](https://chris.beams.io/posts/git-commit/)
-   [Telling Stories Through Your Commits](https://blog.mocoso.co.uk/talks/2015/01/12/telling-stories-through-your-commits/)
-   [How to Make Your Code Reviewer Fall in Love with You](https://mtlynch.io/code-review-love/)
