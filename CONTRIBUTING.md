# Contributing to the Codebase
Welcome, programmers.

This file will go over what you need to know to effectively work on the codebase while following best practices.

## How To
This section goes over optional but useful information on what things are and how they work, especially for inexperienced developers.

### Markdown
It is useful to know how to write *markdown*, plain-text that automatically gets formatted. (`.md` file extension.)

It is used in codebase documentation text files (such as the README), in GitHub, in Slack, in Discord, etc.

If you don't know markdown, find a tutorial or crash course for it, such as this one: https://blog.webdevsimplified.com/2023-06/markdown-crash-course/.

### Git
Git is the version control system. It allows us to collaborate on one project without getting in each other's way.

Developers create changes, called *commits*, on independent versions of the codebases, called *branches*. Branches are *merged* into other branches to update that 'base' branch. \
The primary branch is the `main` branch.

A branch might be created and switched to, to work on a bug fix independently, without worrying about conflicts from development for an unrelated feature. Then, once the bug fix is completed on that branch, the branch is merged into `main` branch.

Another benefit of the version control system is the extremely useful *commit history*, which shows all commits ever made to the codebase, making it easy to track, revert, and review changes.

<details><summary>Example</summary>

For example, a branch is created from `main`, named `fix-vision-crash`, and switched to. Right when the branch is created, it's version of the codebase is exactly the same as `main`.

Then, commits are added to the `fix-vision-crash` branch to fix the vision code. But in the same timeframe, some commits are added to the `main` branch to update the README. Now, the `fix-vision-crash` branch and the `main` branch have *diverged* since they have different *commit histories*.

But it is no problem since the `fix-vision-crash` can be *merged* into `main`, which applies the fix commits from `fix-vision-crash` on `main`, while retaining the README commits on `main`.

The two branches should have no *merge conflicts* because they modified different parts of the codebase. But if they both changed the same or adjacent parts, there could be conflicts that have to be resolved manually.

</details>

#### Git Commands
In the *Terminal*, commands can be entered to perform Git operations. \
The most common operations are also available in the VS Code GUI, in the *Source Control* tab.
> *see more commands:* type `man git` in the Terminal. 

All Git commands start with `git`, followed by a specific command like `commit`, then optionally some options/flags like `--message` (or equivalently, `-m`), then possibly some arguments for the flags or the entire command. \
Example: `git switch -c fix-vision` .

<details><summary>List of useful commands</summary>

(All capitalized words below are placeholders for the actual information.)

#### - Staging changes
```
git add FILENAME
```
Prepares changes to be committed. \
(Use `git add .` to add all changes.) \
(VS Code: use `+` on *Changes* or specific files.)

#### - Committing staged changes
```
git commit -m "COMMIT MESSAGE"
```
Creates a new commit on the current branch, with all of the staged changes. \
(VS Code: enter message and press commit)

To override the author of the commit, add an `--author "AUTHOR NAME <email@address.com>` flag to the end of the command.

#### - Listing branches
```
git branch
```
Shows all branches on the local repository, including which is the currently selected one. \
(Use `git branch -a` to also see remote branches.)

#### - Switching current branch
```
git switch BRANCH
```
Switches the current branch to another one. \
(Use `git switch -` to switch to the previous selected branch.)

#### - Creating a branch
```
git switch -c BRANCH
```
Creates a new branch with the given name, locally.

#### - Deleting a branch
```
git branch -d BRANCH
```
Deletes the branch with the given name, locally.

#### - Merging a branch
```
git merge BRANCH
```
Merges the argument branch into the current branch. \
If the two branches have actually diverged (requiring a real merge commit rather than a simple *fast-forward*), then a message is required. Then, use `-m "MESSAGE"` flag.

#### - Rebasing a branch
```
git rebase BRANCH
```
Updates the *current* branch with the following commit history: the *argument* branch history but with all of the *current* branch's new commits added afterwards (hence, "rebase"). (Argument branch is not modified.) \
Similar to `git merge`, but creates a "cleaner" history. Use only for small changes.

(To rebase the current branch onto its own remote, in order to cleanly sync when local and remote have diverged, use just `git rebase`.)

#### - Fetching changes from remote
```
git fetch
```
Fetches information from the remote repository. Does not modify the local repository. \
This fetch allows one to be able to switch to newly created remote branches, and to merge or rebase the local branch with the remote branch's latest changes.

#### - Pulling changes
```
git pull
```
Pulls changes from the remote and merges it into the local for the current branch; syncing. \
(Equivalent to `git fetch` followed by `git merge`.)

#### - Pushing changes
```
git push
```
Pushes changes to the remote, syncing the remote of the current branch. \
This may only do simple 'fast-forward' merges; if the remote and local branches have diverged, then the command fails.

#### - Stashing Changes
```
git stash
```
Stores the current uncommitted changes away, which is useful to allow pulling from remote cleanly, or for switching to a different branch while uncommitted work is still in progress. \
Use `git stash pop` to then retrieve the stashed changes.

</details>

### Commit Messages
Git commit messages should always be descriptive and standard.

While they may seem like a chore, it is important to be able to clearly see the changes to a codebase just from the messages in the commit history.

Use imperative present tense for verbs, e.g. use "add" and "fix" instead of "added" or "fixed".

#### Conventional Commits
One convention that we will use is to also include a descriptive commit type or category at the start of every commit message. This specification is called *conventional commits*.

The format of a commit message is `TYPE: DESCRIPTION`, e.g. `feat: add flywheel motor` . \
Below are all of the different conventional commit types that you may use:
| Type | Meaning |
| - | - |
| `feat` | New feature of functionality; most common |
| `fix` | Bug fix |
| `refactor` | (Structural) code changes that don't fix a bug nor add a feature |
| `tune` | Changes to constants, numerical values, or configuration to improve behavior |
| `perf` | Code change to improve *performance* (efficiency, CPU) |
| `docs` | Only documentation changes |
| `style` | Formatting and whitespace code changes that don't affect its execution |
| `revert` | Reverting a previous change or commit |
| `build` | Changes to external dependencies or the build configuration |
| `ci` | Changes to *continuous integration* (GitHub workflows) |

You may optionally add a scope to the end of the type in the message, e.g. `feat(shooter): add flywheel motor`.

### GitHub
GitHub will be our primary means of managing and organizing work.

The GitHub repository (repo) bundles our codebase along with work management in a single place. Note: this is the *remote* repository, as opposed to your local repository on your computer.

#### Issues
Issues are a way to track bugs, feature requests, tasks, etc. \
They are used to assign and distribute work with all of the necessary information to complete it.

For example, one issue title may be `Add shooter subsystem` .

The issues are listed on the GitHub repo's *Issues* tab, but they are also visible in the *Project* (explained later).

Each issue has a bunch of fields that should be appropriately set when it is created:
- Assignees - which people are assigned to work on it, if any
- Labels - a set of tags that describe aspects of the issue, e.g. `documentation`, `help wanted`
- Type - a single issue type of either `Bug`, `Feature`, or `Task`. (While a `Feature` issue is a description of new functionality, a `Task` issue is a specific piece of work, that may or may not contribute to a feature.)
- Projects - usually, add the season/repository's one *Project*. The issue additionally has more fields on the Project (click dropdown) in the field -
    - Status - what stage of progress the work on the issue is in, e.g. `In progress`
    - Priority - how important or urgent the issue is, e.g. `P0`
    - Difficulty - how hard the work needed to address the issue is, e.g. `Easy`
    - Iteration - what loose 'stage' or period of the season the issue is in
    - Start date - target start date for working on issue
    - End date - target due date for completing the issue
- Relationships - if the issue has a parent issue, or is 'blocked' or 'blocking' other issues (must be completed before other can be done)
- Development - what branches or pull requests are associated with the issue

Sub-issues can be created of other issues and sub-issues. These can be used to break down a larger feature issue into many specific tasks like a checklist, ideal for distributing work.

Issues can be linked anywhere in other issues and pull requests with a hashtag followed by their number, e.g. `#5`.

#### Pull Requests
To merge one branch into another (especially into `main`), we typically use GitHub pull requests (PRs) rather than merging branches locally and then pushing. A pull request is simply a request to merge a branch, with features for reviewing and checking the changes.

Pull requests are available to view on the *Pull requests* tab of the GitHub repository.

The *Conversation* tab of a pull request is used to give comments and see the timeline of the PR. \
The *Commits* tab displays the commit history of the changes that will be merged.

The *Files changed* tab shows the difference between the two branches, the *diff*. Lines that will be added by the merge are highlighted in green. Lines that will be removed are in red. \
You can also submit a review using the *Submit review* button in the top right on this tab.

Pull requests also have fields, but we will generally **not** be adding fields such as labels to them. \
One can assign Reviewers to the pull request. If review is a significant task, it may also be added to the repo's Project.

PRs can be linked in other issues and pull requests with a hashtag followed by their number, e.g. `#6`.

You can link an issue that may be closed by the PR with a keyword before the issue tag, like `closes #8` or `Fixes #9`.

PRs can be marked as drafts to indicate that it is not ready for merging yet, such as if the branch is still in development.

#### Project
A GitHub project is an adaptable table or view of items (issues or PRs) that is very useful for managing the project or getting an overview of the work.

The project is accessible through the repository under Projects, or under the GitHub organization.

On the top, there are tabs for different views.

The *Prioritized tasks* view is generally the main view you will need. It shows each item along with its fields, grouped by priority. You can use this view to easily find suitable tasks to work on.

The *Status board* view shows items grouped by status, and filterable by assignee. This is useful for checking the progress of work for everything or for yourself or others.

The *Roadmap* shows a timeline, including iteration markers, for issues that have start or end dates.

You can add issues or even draft issues using the project.

## Policies
Read and follow these policies regarding development on this codebase.

### Documentation
All added robot controller bindings/controls must be documented in the [README](README.md).

Methods, and some fields, should generally have Javadoc comments (templated from autocomplete). (Know how to add them.)

Numeric values and constants must have their units documented somewhere. Best is to use the Units library, otherwise use comments at definition. \
Know the [Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html) (read up to `Human-readable Formatting` section, inclusive).

Commit messages must be descriptive and [*conventional*](#conventional-commits).

(For officers): On the repo, *after* a competition is finished, create a new *Release* on `main` with a new tag, so that the codebase state at competition end is documented.

### Issues
When creating a GitHub issue, add all of the appropriate fields including those for the project. (This does not apply to pull requests.)

If you are working on an issue or plan to, add yourself as an assignee. Unassign yourself if you are unable to continue working on it.

If you are assigned to an issue, complete it. If you can't, add a comment and unassign yourself.

To make a request, document a problem or task, or delegate work, create an issue so that it is visible and assignable to anyone. \
Issues are not limited to codebase things: e.g. mechanism testing.

Issues must have enough information for any programmer with the right skills to pick it up and do it.

Officers: one of your main responsibilities is to create and possibly assign issues for much of the main work that needs to be done.

### Branches and PRs
A new branch must be used for each new feature, bug fix, etc. \

Only very small, one-off changes should be directly committed(after testing) without its own branch.

A GitHub pull request must be opened, to merge a branch back into `main` or some other core development branch. \
(But updating an in-development branch itself such as merging `main` into `feature-1` is fine to do locally.)

Rookie programmer pull requests must not be merged until reviewed by a veteran programmer who merges it themselves. \
Veteran programmers may choose to merge their own PR without review for insignificant or difficult changes.

When a Veteran Programmer PRs, it is highly encouraged to be reviewed by a peer.  

Avoid committing directly to `main`, especially new features.

Branch names must describe the work, e.g. `fix-vision` or `climber`. Use hyphens between words, and use all lowercase.

When creating a branch, the base/parent branch should be `main` unless code is required from a specific branch, to ensure quicker merging into `main`.

If there are merge conflicts, then use VS Code to merge the base branch in, and resolve them (e.g. merge `main` into `feature-1` in VS Code to resolve conflicts, before merging `feature-1` into `main` via the PR.)

Use rebase for trivial or small merges (that have no conflicts), such as syncing the remote with local.

### Style
You must install the recommended Google Java Format extension on VS Code. It will format on save.

### Codebase Structure
The source code is separated into folders/packages by *feature* (e.g. `vision`, `drivetrain`), rather than by code type (e.g. `subsystems`).

Constants that are not expected to change over the codebase (e.g. mechanism locations, device IDs) should be placed in a `___Const` class in the relevant package. \
Configuration, which are tunable values and settings (e.g. motor configuration, PID values, tolerances), should be placed in a `___Config` class in the relevant package.

### AI
Do not use AI to generate large amounts of code, especially when the task is too difficult for your skills.

If using autocomplete or code generation, please review the code very carefully and make sure it works.

Rookies: do not generate code snippets using AI.

It is okay to use AI for research or guidance purposes.

## Workflow
Recommended step-by-step workflow or development process for programmers:

### 1. Find Work
- If you are currently assigned to issues, do those first
- Officers, veterans, reviewers: review and merge PRs
- Otherwise, find an issue to do with one of the below ways:
    - Choose an open and pressing issue from the Project view
    - Create a new issue for currently untracked work
    - Create a new sub-issue to further break down a large existing issue
    - Ask an officer or veteran to assign you work
- Assign yourself to the issue

### 2. Do Work
- Some issues like testing might not call for any code changes
    - Once done, close the issue with a comment describing the results, and return to Step 1
- For code changes, create a new branch and perform all of the work there
- If you find new problems or tasks while working, create issues for them

### 3. Test Work
- Test that the new code or fix works, if possible
- Consider creating an issue for testing for any of the below reasons:
    - Currently unable to test that the code works
    - Testing alone is a sizable task
    - Want someone else to test the code

### 4. Publish Work
- Push the branch to GitHub and create a pull request
- Provide a sufficient description for others to understand or review your work
- Link any related issues or PRs, using keywords like `closes #N` to automatically close issues when merged
- If you are a rookie programmer, leave the PR to be reviewed and merged by others
- Address any comments, tips, or criticisms from reviews, for the duration of the PR
- Return to Step 1 to continue working
