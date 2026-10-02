# Project instructions

## Existing environment and user changes

- Preserve unrelated user changes and generated state. Do not merge, push,
  flash firmware, clean worktrees or operate motor outputs unless requested.
- Before downloading/installing a runtime, SDK, toolchain, package manager or
  equivalent environment, ask explicit approval and state need, destination,
  lifetime and already-checked alternatives. Existing-tool detection/use is OK.
- For PlatformIO projects, first prefer
  `C:\Users\Cainiao\.platformio\penv\Scripts\pio.exe`; use `pio run -d <project>`.
  If it is not ready, initialize the platformio.ini directory with the existing
  VS Code extension before considering another environment. Do not commit
  generated `.pio/` or PlatformIO `.vscode/` unless explicitly required.

## Documentation must follow actual progress

- User explicitly requested continuously maintained progress, functionality
  documentation and protocol manuals. This is part of task completion, not an
  optional follow-up or scheduled automation.
- Before continuing, read `docs/README.md`, `docs/项目进度记录.md` and the relevant
  protocol/validation documents. Treat historical entries as dated evidence,
  not as current source truth.
- After substantive work, update the current ledger, applicable function and
  protocol manuals, and append execution history. Follow
  `docs/文档维护说明.md`; retain historical failures and corrections.
- Distinguish protocol definitions, backend implementation, software testing,
  diagnostic firmware hardware testing and normal-mode hardware acceptance.
  A successful build or communication Ready is not motor safety acceptance.
- No automatic enable/resume; software STOP ACK is not physical emergency-stop
  proof. Do not advertise capability bits for unimplemented target commands.
- Keep protocol layouts in protocol docs and link elsewhere. Validate document
  links, paths and evidence counts. Record build/profile identity when results
  depend on firmware configuration. Do not report old logs as newly run tests.
- Update while actually working on this task; do not infer changes in other
  chats or create periodic checks unless the user explicitly requests them.
