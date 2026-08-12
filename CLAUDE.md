# Working on continuo

continuo is a deterministic simulation orchestrator in Rust. The same world
run twice produces the same bytes, on every platform, and most of what follows
exists to keep that true.

[README.md](README.md) says what the crates are and how to run the demo.
[PLAN.md](PLAN.md) describes the design as it stands, and
[DECISIONS.md](DECISIONS.md) says why it is that way, dated in the order the
questions were settled, including the roads not taken. A `M<n>-PLAN.md` at the
root is scaffolding for the milestone in progress and is deleted when it
closes, so nothing there is a permanent record.

## Determinism comes first

- The demo's world hash is pinned as `DEMO_WORLD_HASH` in
  `crates/continuo-examples/tests/highway.rs`, and README quotes it in its
  sample output. A change that moves it moves both in the same commit, and the
  commit message says what joined the fingerprint.
- Most changes must not move it. When a refactor is meant to be inert, the pin
  still passing is the proof, and the commit message should say so.
- `HashMap` and `HashSet` are banned by `clippy.toml`. Use `BTreeMap` and
  `BTreeSet`, which iterate in key order.
- CI runs four agents (Ubuntu x86_64, Ubuntu arm64, Windows, macOS arm64)
  because `sin`, `cos`, `asin` and `atan2` are not required to be correctly
  rounded, so each platform's libm is free to differ in the last bits. Where
  there is a choice, prefer the operations IEEE 754 does pin: add, multiply,
  divide, sqrt.
- The workspace contains no `unsafe`.

## Verify before every commit

In CI's order, cheapest first, so a formatting slip is reported in seconds
rather than after the workspace has compiled:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
RUSTDOCFLAGS=-D warnings cargo doc --workspace --no-deps
cargo test --workspace
cargo run -p continuo-examples --example traffic
```

The doc build is not optional. The crates cross-reference each other heavily,
and a renamed item leaves a broken intra-doc link that still compiles.

Python changes also need `ruff check .`, `ruff format --check .` and `pytest`
from `python/`.

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
when it matters.

The mistakes worth this treatment are the ones nothing else catches: a
corrupt comment still compiles, and a reordered map gives the right answer
until the day it does not.

## Prose style

The same rules everywhere: code comments, doc comments, Markdown, commit
messages, PR descriptions, console strings.

- **American English.** "behavior", "initialize", "center", "semi truck".
- **No dash grammar.** Do not join clauses with an em dash, an en dash, or a
  spaced hyphen. Use a comma, a colon when what follows explains what came
  before, a semicolon between two independent clauses, or two sentences. A
  clause that needs a dash to attach usually wants to be its own sentence.
  Hyphens inside compound words, numeric ranges and CLI flags stay.
- **Plain ASCII characters.** Write `...` and not `…`, `->` and not `→`, `x`
  and not `×`, and spell out `pi` and `+/-` rather than reaching for `π` and
  `±`. README's architecture diagram is the one place a non-ASCII character
  earns its keep, since box drawing is what draws it.
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
- Comments posted through `gh` authenticate as the repository owner. Every one
  Claude writes is italicized throughout and ends with
  `*🤖 Posted by Claude Code*`, so it reads as distinct from the owner's own
  words.

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
rooted at the repository root cannot reach it with file tools, which forces
the session to move into the worktree, and transcripts and memory are keyed to
the working directory. Everything learned there is then invisible from every
other session and dies with the branch. Keep the editor and the session at the
repository root, and work in the worktree by path.
