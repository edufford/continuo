# Working on continuo

continuo is a deterministic simulation orchestrator in Rust. It is built for
runs that can be reproduced and therefore trusted, for worlds where actors are
many and come and go mid-run, and for a design that survives moving from one
process to several hosts.

[README.md](README.md) says what the crates are and how to run the demo.
[PLAN.md](PLAN.md) describes the design as it stands, and
[DECISIONS.md](DECISIONS.md) says why it is that way, dated in the order the
questions were settled, including the roads not taken. A `M<n>-PLAN.md` at the
root is scaffolding for the milestone in progress and is deleted when it
closes, so nothing there is a permanent record.

## Determinism comes first

Simulations are tools for driving engineering decisions, so a run has to be
trustworthy, and repeatability is what earns that trust. A result nobody can
reproduce is a result nobody can check, so two runs of one world disagreeing
is something to explain rather than wave through. Where a component genuinely
cannot be made repeatable, the exception gets named and scoped rather than
left to be discovered.

- The demo's world hash is pinned as `DEMO_WORLD_HASH` in
  `crates/continuo-examples/tests/highway.rs`, and README quotes it in its
  sample output. A change that moves it moves both in the same commit, and the
  commit message says what joined the fingerprint.
- Most changes must not move it. When a refactor is meant to be inert, the pin
  still passing is the proof, and the commit message should say so.
- `HashMap` and `HashSet` are banned by `clippy.toml`. Use `BTreeMap` and
  `BTreeSet`, which iterate in key order.
- **Transcendentals go through `libm`**, a pure-Rust port of MUSL's, because
  `sin`, `cos`, `asin` and `atan2` are not required to be correctly rounded
  and each platform's C library differs in the last bits. `clippy.toml` bans
  the inherent methods, since nothing else would report reaching for one.
  Where there is still a choice, prefer what IEEE 754 pins outright: add,
  multiply, divide, sqrt.
- CI runs four agents (Ubuntu x86_64, Ubuntu arm64, Windows, macOS arm64),
  which is what measured that difference and what keeps it measured.
  `DEMO_WORLD_HASH` cannot see it: the demo's road is straight, so every yaw
  rate in it is exactly zero and every transcendental is evaluated where all
  implementations agree anyway. The ellipse in `continuo-actors`' determinism
  test is the one that steers, and it is pinned for that reason.
- The workspace contains no `unsafe`.

## Scale and churn are design constraints

A world is not a fixed handful of actors in one process. A design has to hold
up as the population grows, as actors join and leave mid-run, and as
components end up on separate hosts.

- **Many actors.** Prefer work that grows with what changed over work that
  grows with the population. `traffic_scale` is where that gets measured, and
  PLAN.md's deferred list already names what it expects to bite first: a
  subscriber that can ask for only the latest message per key, and a
  consolidated scene view for consumers that want the world rather than every
  message in it.
- **Actors come and go.** Join and leave are ordinary rather than an edge
  case. The conductor publishes each on the world's membership key as status,
  saying it already happened, and anything holding per-actor state must
  subscribe there and drop what it kept.
- **One process now, several hosts later.** Milestone 7 puts components on a
  network, so anything assuming shared memory, a shared clock, or that every
  component is reachable in this address space is written to be rewritten.
  Messages are data and a step is a request and a reply, which is what keeps
  that move a change of transport rather than of design.

## Verify before every commit

```sh
cargo xtask verify
```

It runs these in this order, cheapest first, so a formatting slip is reported
in seconds rather than after the workspace has compiled, and it stops at the
first failure:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-D warnings cargo doc --workspace --no-deps
cargo test --workspace
cargo run -p continuo-examples --example traffic
```

It adds the viewer's `ruff check .`, `ruff format --check .` and `pytest`
when the viewer is installed, and says they were skipped when it is not, so a
Rust-only change is not held up by a Python environment nobody set up.

It is a quick check rather than a thorough one, and CI stays the authority:
four platforms, both profiles, the packaged FMUs and the recorded-log smokes.
What this is for is catching the ordinary mistake before a push, so being
fast enough to sit in an editing loop is what matters most about it.

The doc build is not optional. The crates cross-reference each other heavily,
and a renamed item leaves a broken intra-doc link that still compiles.

Both tasks end by saying how many tests ran, and a step that ran none fails.
An elapsed time says a command ran, never that it found anything to do.

CI splits the test step in two, `--lib --bins` then `--test '*'`, so neither
reruns the other's tests. Between them they have to name every target that
runs, because anything they miss runs nowhere. One `cargo test --workspace`
covers the lot locally.

A change reaching an FMU crate or the laws it links needs more than that,
because a `.fmu` carries its own compiled copy of everything it links and the
checks above compare nothing against it:

```sh
cargo xtask verify-fmus
```

It packages every FMU, validates each with fmpy, and runs the tests that
check the packaged copy against the laws it was built from, asking for the
feature each FMU crate holds those tests behind. `verify` leaves all of it
out on purpose: packaging costs a release build of the FMU crate whenever a
law changed, and the tests say nothing without it, so the pair belongs with a
change that reaches a law rather than in an editing loop. Validation is
skipped, and says so, when fmpy is not installed.

`continuo-actors` is the one to watch, since editing a control law there
leaves the packaged FMU a build behind and `cargo test --workspace` stays
green. Packaging needs `cargo install cargo-fmi` once, and validating needs
`python -m pip install fmpy`.

## Turn a mistake into a check

A mistake that shipped, or nearly shipped, earns the cheapest automatic
barrier that would have caught it, added in the change that fixes it. Prefer
the check that fails loudly and close to the cause. Where a barrier is not
worth its cost, say so in the commit rather than leaving the reader to wonder
whether it was considered.

The rules that already exist here are mostly this. `clippy.toml` bans
`HashMap` and `HashSet` because an unspecified iteration order can reach a
fingerprint, and no compile error would say so. `.gitattributes` marks
`*.fmu binary` because converting line endings inside a zip breaks the
archive somewhere far from the cause. `DEMO_WORLD_HASH` is pinned because a
world hash that moves quietly is a change nobody sees. CI corrupts a recorded
log on purpose, because a verifier that accepts a tampered log fails only
when it matters. `string-literals.yml` reads for a run of four spaces inside
a string literal, which is what a line-continuation backslash lost from the
end of a source line leaves behind: the literal is still valid Rust, and
`rustfmt` does not reformat what is inside a string, so it accepts the
over-wide line as well. Ruff's `ISC` rules are the same barrier on the
viewer, where the mistake takes Python's own shape: a comma missing from a
collection joins two of its entries rather than separating them.

`string-literals.yml` is a heuristic where the rest are exact, so it is a
workflow of its own and stays out of the required checks: a rule that can be
wrong about correct code must not be able to block a merge. It reads only
the lines a pull request adds, for the same reason. A literal meaning to
carry a run of spaces is then answered once, in the description of the pull
request adding it, where reading the whole tree would fail every pull request
after that one, and a check which is always red is a check nobody reads.
`ISC` needs none of that, naming the cause rather than reading for a shape,
so it sits in `ruff check` with everything else and fails in the editing
loop rather than only on a pull request.

The mistakes worth this treatment are the ones nothing else catches: a
corrupt comment still compiles, and a reordered map gives the right answer
until the day it does not.

## Prose style

The same rules everywhere: code comments, doc comments, Markdown, commit
messages, PR descriptions, console strings.

- **American English.** "behavior", "initialize", "center".
- **No dash grammar.** Do not join clauses with an em dash, an en dash, or a
  spaced hyphen. Use a comma, a colon when what follows explains what came
  before, a semicolon between two independent clauses, or two sentences. A
  clause that needs a dash to attach usually wants to be its own sentence.
  Hyphens inside compound words, numeric ranges and CLI flags stay.
- **Plain ASCII characters.** Write three periods rather than an ellipsis
  character, `->` rather than an arrow, `x` rather than a multiplication
  sign, and spell out `pi` and `+/-` rather than reaching for the Greek
  letter or the plus-minus sign. Named here rather than shown, so this file
  stays ASCII itself and a search for the offenders never lands on the rule
  against them. README's architecture diagram is the one place a non-ASCII
  character earns its keep, since box drawing is what draws it.
- **Comments describe the code as it is now**, never what it used to do or
  what an earlier version got wrong. That history belongs in git and the PR.
  Saying why an alternative was rejected is welcome, in the conditional:
  "Keeping the previous one would go on steering from it without saying so."
- **Short and direct.** Plain terms over jargon, the shorter sentence over
  the cleverer one, and the subject doing the acting rather than a noun built
  out of a verb. A sentence that has to be read twice wants rewriting, not
  repunctuating, and a paragraph saying one thing two ways keeps the better
  half. Length is the last thing to add and the first thing to cut.
- **Revise a comment as a whole block.** When the code under it changes,
  reread every line of the comment and write what it should say now, rather
  than patching the one line that went stale. A block edited a line at a time
  ends up half describing code that is gone, and the halves read as a single
  claim, so the reader cannot tell which part to trust.

## Rust style

- Every multi-statement block whose final expression is the return gets a
  blank line and a `// Return ...` comment before it, so an implicit return is
  visible to a reader. Single-expression bodies are exempt.
- Test names are sentences stating what holds:
  `a_car_off_its_lane_turns_back_toward_it`, not `test_steering`.
- `TODO(M<n>)` marks work waiting on a numbered milestone and
  `TODO(PLAN "section")` work tracked in PLAN.md.
  `grep -rn "TODO(" crates/ python/continuo_viz/` lists them all.

## Commits and pull requests

- An imperative subject line, then a prose body saying what was decided and
  why, rather than a list of what changed, which the diff already says. End
  with a `Co-authored-by:` trailer naming the model that wrote it, so the log
  records which model produced which commit. The name is whichever model is in
  use, for example `Claude Opus 5 (1M context)`, and the address is
  `<noreply@anthropic.com>`.
- **A commit per review topic**, each building and passing on its own.
  Several comments can share one commit where they are really one decision,
  and that often reads better than splitting them. What to avoid is the
  single commit answering everything, which hides which change answers which
  comment and makes one decision impossible to revert without unpicking the
  rest. Unresolved threads are the live ones.
- Show a diff for review before committing it. It is read in the editor, so
  there is no need to paste it into chat.
- Never force push.
- **A pull request description is the squash commit message.** This
  repository squash-merges using the title and description, so what is
  written there is what lands on main, and the branch's own commits survive
  on the closed PR page and nowhere else. That setting is deliberate: a
  branch worked through with an AI coding agent accumulates dozens of small
  commits answering review comments, and main reads better as one account
  per PR than as a transcript of how it was reached. Write it as a commit
  body from the first push, in the same voice: what was decided and why, and
  what it cost to find out. Someone reading only the log should come away
  knowing what the PR settled.
  - **Merge from the web interface.** Only it honors the repository's
    setting. The iOS app writes the title alone and silently drops the
    description and the trailers with it, which is the worse failure for
    looking like an ordinary tidy subject line rather than like something
    that went wrong. `70b3a59` is a commit whose own reasoning survives
    nowhere but its closed PR page.
  - **Wrap it at 72 columns.** GitHub hard-wraps the description when it
    builds that commit, breaking each line on its own rather than reflowing
    the paragraph, so a 78-column line lands as 72 characters and an orphan
    fragment on the line below. `2540aed` is what that looks like.
  - **Keep every code span whole.** The same wrap, biting a second way. It
    does not know prose from code, and the backticks land in the commit as
    written, so a span it breaks arrives as a command or an identifier cut in
    half. `4c69fb7` carries a test name split mid-word, which is then a name
    nobody can grep for.
  - Nothing in it can be a branch SHA, a checkbox, or a link into the diff,
    since none of those mean anything against main once the branch is gone.
    The `Co-authored-by:` trailers are GitHub's to append, so the description
    carries none of its own.
- **A pull request opens as a draft, carrying its review notes below a
  marker.** The iterations happen there, and marking it ready is a separate
  step taken once they are done rather than part of opening it. The
  description is also where a to-do list and notes to reviewers want to
  live, and those must not reach main, so they go under a `---` and a
  heading reading `## Draft notes (deleted before merge)`. Everything above
  that marker is the commit message throughout, and the marker and all
  beneath it are deleted when the PR is marked ready.
- `pr-description.yml` holds a PR that is out of draft to all three of those:
  no `## Draft notes` heading, no line past 72 columns, and no code span the
  wrap has broken. The first is anchored to the start of a line, so a
  description explaining this convention can still name the marker in prose,
  as this one does. The last skips fenced blocks, so it stays exact and can
  sit in the required checks rather than needing a workflow of its own.
- Comments posted through `gh` authenticate as the repository owner. Every one
  Claude writes is italicized throughout and ends with a footer line reading
  "Posted by Claude Code" behind a robot-face emoji, italic like the rest, so
  it reads as distinct from the owner's own words.

## Editing files

- Never sweep a rename for an identifier that is also an ordinary English word
  (`requests`, `path`, `state`, `applied`). A regex reaches prose in comments
  across crates that were never part of the change, nothing fails to compile,
  and the damage ships. Rename site by site, or scope the pattern to code
  contexts such as `self.x` and `X::`, then check `git diff --name-only`.
- Do not rewrite source files through PowerShell. In Windows PowerShell 5.1,
  `Get-Content` reads a UTF-8 file without a BOM as Windows-1252 and
  `Set-Content -Encoding utf8` writes one back, so punctuation turns to
  mojibake and nothing fails, because corrupt comments still compile. Use
  `sed` from Bash, or an editing tool.
- A Python script rewriting a tracked file must open it with `newline="\n"`.
  `write_text` and plain `open(..., "w")` translate to CRLF, which
  `.gitattributes` then warns about on every commit touching the file, and
  `git checkout -- <file>` will not undo it.

## Worktrees

Create them inside the checkout, at `.claude/worktrees/<name>`. Nesting costs
nothing there: `.git/info/exclude` lists `.claude/`, so a worktree never shows
as untracked, and cargo and ripgrep both skip hidden directories, so neither
walks the second copy of the tree.

A session opened on a worktree still needs its memory pinned back to the
repository root's pool, since auto-memory is keyed to the working directory
and a worktree gets an empty one of its own. Set `autoMemoryDirectory` in the
worktree's own `.claude/settings.local.json` to the root's memory directory.

Never put one in a sibling directory such as `../continuo-<name>`. A session
rooted at the repository root cannot reach it with file tools at all, and that
is the difference that matters: a worktree inside the checkout is editable
from either.
