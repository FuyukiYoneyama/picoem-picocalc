# Contributions

This is a personal project, published in case it's useful to others.
I'm happy to look at issues and pull requests, but I'm not making any
commitments about response times, triage, or whether things will get
merged or fixed. If you need certainty around a fix or feature, the
safest path is to fork — the dual MIT/Apache-2.0 license places no
obligations on you.

Contributions are dual-licensed as **MIT OR Apache-2.0** unless you
state otherwise. See `LICENSE-MIT` and `LICENSE-APACHE`.

## Shared checkout hygiene

The public `main` checkout is a shared, releasable working tree. Do not leave
one-off application investigations, temporary instrumentation, generated
artifacts, or an uncommitted working tree there after the investigation ends.

- Perform target-specific experiments in a separate worktree or branch, with
  generated output outside the repository.
- Before using a result as a final validation record, build from a clean,
  committed backend. A report with `backend_build.dirty: true` is exploratory
  evidence only; it is not a releasable conformance record.
- At the end of an experiment, choose one outcome: promote a reusable feature
  with documentation, tests, and an isolated commit; preserve only the
  resulting evidence outside this repository; or remove the experiment and
  restore the shared checkout to clean.
- Do not turn a local diagnostic into an always-on public report field merely
  to retain it. If a diagnostic has continuing value, make its activation,
  report schema, tests, and user-facing documentation explicit in a follow-up
  change.

Before handing the repository to another user or project, `git status --short`
must be empty. If work must continue, commit it to a clearly named non-`main`
branch rather than leaving anonymous local modifications behind.
