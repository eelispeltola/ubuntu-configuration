# Git aliases, modified from oh-my-bash and https://www.eficode.com/blog/10-levels-of-git-aliases-beginner-to-intermediate-concepts
# alias g='command git'

alias ga='command git add'
alias gaa='command git add --all'
alias gapa='command git add --patch'
alias gau='command git add --update'
alias gwip='command git add -A; command git rm $(git ls-files --deleted) 2> /dev/null; command git commit --no-verify --no-gpg-sign --message "--wip-- [skip ci]"'

alias gbl='command git blame -b -w'

alias gb='command git branch'
alias gba='command git branch -a'

alias gbs='command git bisect'

alias gcb='command git checkout -b'
alias gco='command git checkout'
alias gcor='command git checkout --recurse-submodules'

alias gcp='command git cherry-pick'

alias gclean='command git clean -fd'

alias gc='command git commit --verbose'
alias gcamend='command git commit --verbose --amend'
alias gcamendn='command git commit --verbose --no-edit --amend'
alias gca='command git commit --verbose --all'
alias gcma='command git commit --all --message'
alias gcm='command git commit --message'

alias gcf='command git config --list'

alias gdct='command git describe --tags `git rev-list --tags --max-count=1`'

alias gd='command git diff'
alias gdca='command git diff --cached'
alias gdw='git diff -w --word-diff=color --ignore-space-at-eol'
alias gdt='command git diff-tree --no-commit-id --name-only -r'
alias gdtool='command git difftool -d'

alias gf='command git fetch'
alias gfa='git fetch --all --prune --jobs=10'

alias gg='command git gui citool'

alias glg='command git log --stat'
alias glgg='command git log --graph'
alias glgga='command git log --graph --decorate --all'
alias glgm='command git log --graph --max-count=10'
alias glgp='command git log --stat -p'
alias glo='command git log --oneline --decorate'
alias glod='command git log --graph --pretty="%Cred%h%Creset -%C(auto)%d%Creset %s %Cgreen(%ad) %C(bold blue)<%an>%Creset"'
alias glods='command git log --graph --pretty="%Cred%h%Creset -%C(auto)%d%Creset %s %Cgreen(%ad) %C(bold blue)<%an>%Creset" --date=short'
alias glog='command git log --oneline --decorate --graph'
alias gloga='command git log --oneline --decorate --graph --all'
alias glol='command git log --graph --pretty="%Cred%h%Creset -%C(auto)%d%Creset %s %Cgreen(%ar) %C(bold blue)<%an>%Creset"'
alias glola='command git log --graph --pretty="%Cred%h%Creset -%C(auto)%d%Creset %s %Cgreen(%ar) %C(bold blue)<%an>%Creset" --all'
alias glols='command git log --graph --pretty="%Cred%h%Creset -%C(auto)%d%Creset %s %Cgreen(%ar) %C(bold blue)<%an>%Creset" --stat'
alias gslog='command git log --pretty=format:"%C(auto)%h %C(red)%as %C(blue)%aN%C(auto)%d%C(green) %s"' 
alias gl='glols'  # Preferred git log command
# Pretty log messages
function _git_log_prettily(){
  if [[ $1 ]]; then
    command git log --pretty="$1"
  fi
}
alias glp='_git_log_prettily'
#compdef _git glp=git-log


# what would be merged
alias gincoming='git log HEAD..@{upstream}'

# what would be pushed
alias goutgoing='git log @{upstream}..HEAD'

alias gpr='command git pull --rebase'

alias gpf='git push --force-with-lease --force-if-includes'

alias grb='command git rebase'
alias grba='command git rebase --abort'
alias grbc='command git rebase --continue'

alias gpristine='command git reset --hard && command git clean --force -dfx'

alias gunwip='command git rev-list --max-count=1 --format="%s" HEAD | grep -q "\--wip--" && command git reset HEAD~1'

alias gcount='command git shortlog --summary --numbered'

alias gsh='command git show'
alias gsps='command git show --pretty=short --show-signature'

alias gsb='command git status --short --branch'
alias gss='command git status --short'
alias gst='command git status'
alias gs='gst'

alias gsw='command git switch'
alias gswc='command git switch --create'
alias gswd='command git switch "$(git_develop_branch)"'
alias gswm='command git switch "$(git_main_branch)"'

alias gignore='command git update-index --assume-unchanged'
alias gunignore='command git update-index --no-assume-unchanged'

alias gwch='command git whatchanged -p --abbrev-commit --pretty=medium'

# Overview of labeled commits in repo
alias gstructure='git log --oneline --simplify-by-decoration --graph --all'

alias frequentgit='history | cut -c 8- | grep git | sort | uniq -c  | sort -n -r | head -n 10'

