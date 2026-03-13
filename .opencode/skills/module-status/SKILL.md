---
name: module-status
description: Summarize implementation and milestone status for a module or the whole workspace. Pass a module name to focus, or no args for workspace-wide overview.
---

Read the relevant planning and source files to produce a status report.

## Steps

1. Read `repo-plans/todos.md` for active work items.
2. If a specific module was requested, read `repo-plans/modules/<module>.md`. Otherwise scan all files in `repo-plans/modules/`.
3. Check whether source files exist under `workspace/robotics/` for each module:
   - `include/` populated → headers defined
   - `src/` populated → implementation exists
   - `tests/` populated → tests written
4. For each module, check the **Observability Gate**: does any `.cpp` file `#include` from `common/logging/` and instantiate an `ILogger`?

## Output format

```
## Module Status: <module or "Workspace Overview">

### <domain>/<module-name>
- Milestone: M<N> — <phase description>
- Implementation: scaffolded / partial / complete
- Tests: none / skeleton / real coverage
- Observability Gate: ✅ passed / ❌ missing
- Active todos: <list from todos.md or "none">
- Next action: <one concrete next step>
```

For workspace overview, group by domain and list each module with emoji status:
- ✅ complete (impl + tests + observability)
- 🔨 in progress
- 📋 planned only
