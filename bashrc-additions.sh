## GENERAL

# From sensible-bash.sh and oh-my-bash

# Prevent file overwrite on stdout redirection
# Use `>|` to force redirection to an existing file
set -o noclobber
# Update window size after every command
shopt -s checkwinsize
# Automatically trim long paths in the prompt (requires Bash 4.x)
PROMPT_DIRTRIM=2
# Correct spelling errors during tab-completion
shopt -s dirspell 2> /dev/null

## HISTORY

# From sensible-bash.sh

# Append to the history file, don't overwrite it
shopt -s histappend
# Save multi-line commands as one command
shopt -s cmdhist
# Re-edit the command line for failing history expansions
shopt -s histreedit
# Re-edit the result of history expansions
shopt -s histverify
# save history with newlines instead of ; where possible
shopt -s lithist
# Record each line as it gets issued
PROMPT_COMMAND='history -a'
# Huge history. Doesn't appear to slow things down, so why not?
HISTSIZE=500000
HISTFILESIZE=100000
# Avoid duplicate entries
HISTCONTROL="ignoreboth"
# Don't record some commands
export HISTIGNORE="&:[ ]*:exit:ls:bg:fg:history:clear"
# Use standard ISO 8601 timestamp
# %F equivalent to %Y-%m-%d
# %T equivalent to %H:%M:%S (24-hours format)
HISTTIMEFORMAT='%F %T '

## CUSTOM

