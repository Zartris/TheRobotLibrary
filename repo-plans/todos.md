Description and intent of the file:
This file is listing down tasks and features that we want before making it into a concrete milestone. There is two  list. The quick todo list is for us to quickly jot down ideas and tasks, while the fleshed out todo list is for us to have a more detailed view of what we want to do and how we want to do it.

---
Quick todo list:
- [x] Make docker install claude code: "curl -fsSL https://claude.ai/install.sh | bash"
- [x] Make docker install copilot cli: "curl -fsSL https://gh.io/copilot-install | bash"
- [x] Make docker install opencode: "curl -fsSL https://opencode.ai/install | bash"
- [x] Make docker also take the opencode config folder (like we do with claude now).
- [x] Pretty sure that there is no global copilot config, but if there is, we should also take that one.
  - There IS a global copilot config at ~/.copilot/ — now mounted into the container.
- [x] We should move mirror the local agent folders to docker (.claude, .vscode . opencode)
  - All three home-level agent folders (~/.claude, ~/.vscode, ~/.opencode) are now mounted, plus ~/.copilot and ~/.config/opencode.

---
Fleshed out todo list:
