# Git Worktree 규칙 (Claude Code 공식 컨벤션)

병렬 작업이나 격리된 실험은 git worktree로 분리. **반드시 Claude Code 공식 컨벤션을 따른다.**

## 경로 / 브랜치명 컨벤션

- **경로**: `<repo>/.claude/worktrees/<name>` (메인 레포 안의 `.claude/worktrees/` 하위)
- **브랜치명**: `worktree-<name>`
- **base 브랜치**: `origin/HEAD` (현재 master)

`.gitignore`에 `.claude/worktrees/` 등록되어 있음 (메인 레포에 untracked로 안 잡히도록).

## 생성 방법 (둘 중 하나)

**A. Claude Code 시작과 동시에 (정석)**

```bash
claude --worktree iwdg-fault-recovery
# 또는 -w 약식
claude -w iwdg-fault-recovery
# 이름 생략 시 랜덤 이름 자동 생성 (예: bright-running-fox)
```

자동으로 `.claude/worktrees/iwdg-fault-recovery/`가 만들어지고 그 안에서 Claude 세션 시작.

**B. 이미 세션 안이라 수동으로 만들어야 할 때**

```bash
git worktree add -b worktree-<name> .claude/worktrees/<name> master
```

만든 다음에는 새 VSCode 창에서 `.claude/worktrees/<name>`을 열고 거기서 새 Claude Code 세션 시작.

## 정리

- **변경사항 없이 종료**: 워크트리+브랜치 자동 삭제됨
- **변경사항/커밋 있으면**: Claude가 keep/remove 묻는다
- **수동 정리**:
  ```bash
  git worktree remove .claude/worktrees/<name>
  git branch -d worktree-<name>   # 머지된 경우
  ```

## 안 하는 것 (금지)

- **레포 sibling 디렉토리에 만들지 말 것** (예: `../dev-upper-robot-foo`). 메인 레포 컨텍스트에서 분리되고 세션 resume도 어려워진다.
- **`feat/*` 같은 일반 브랜치명으로 워크트리 만들지 말 것**. `worktree-` 접두사가 있어야 자동 cleanup sweep과 세션 picker가 워크트리로 인식한다.

## 출처

[Claude Code Common Workflows — Run parallel sessions with Git worktrees](https://code.claude.com/docs/en/common-workflows#run-parallel-claude-code-sessions-with-git-worktrees)
