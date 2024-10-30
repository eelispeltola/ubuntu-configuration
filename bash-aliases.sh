alias upall='sudo apt update -y && sudo apt upgrade -y && sudo apt dist-upgrade -y && sudo apt autoremove -y'
# If brew is installed, add it to upall
if [ "$(type -P -- "brew")" ]; then
  alias upall='sudo apt update -y && sudo apt upgrade -y && sudo apt dist-upgrade -y && sudo apt autoremove -y && brew update && brew upgrade'
fi
alias aptlist='apt list --installed'
alias newvenv='rm -r .venv; python3.11 -m venv .venv && source .venv/bin/activate && pip install --upgrade pip'
alias svenv='source .venv/bin/activate'
alias py='python'

# # Search in list of aliases (does not see functions)
# function _grep_aliases(){
#   if [[ $1 ]]; then
#     alias | grep "$1" --color=never
#   fi
# }
# alias aliases='_grep_aliases'

# Conda
alias coc='conda create --name ${PWD##*/}  # Creates conda env with the name of the current working directory'
alias coa='conda activate ${PWD##*/}'
alias cod='conda deactivate'
alias cor='conda remove --name ${PWD##*/} --all'


## Select aliases from oh-my-bash

# Ask before cp and mv, show result after
alias cp='cp -iv'
alias mv='mv -iv'

alias mkdir='mkdir -pv'                     # Make nested dirs with mkdir and show result
alias nano='nano -W'                        # Detect wordbounds in nano

alias less='less -FSRXc'                    # Preferred 'less' implementation
alias wget='wget -c'                        # Preferred 'wget' implementation (resume download)
alias c='clear'                             # c:            Clear terminal display
alias path='echo -e ${PATH//:/\\n}'         # path:         Echo all executable Paths
alias show_options='shopt'                  # Show_options: display bash options settings
alias h='history'

# Directory Listing aliases
alias ll='ls -lAFh'                         # Preferred 'ls' implementation
alias lc='ls -lAcr'                         # sort by change time
alias lh='ls -lASrh'                        # sort by size human readable
alias dud='du -d 1 -h'                      # Short and human-readable directory listing
alias duf='du -sh *'                        # Short and human-readable file listing
# lr:  Full Recursive Directory Listing
alias lr='ls -R | grep ":$" | sed -e '\''s/:$//'\'' -e '\''s/[^-][^\/]*\//--/g'\'' -e '\''s/^/   /'\'' -e '\''s/-/|/'\'' | less'

alias numFiles='printf "%s\n" $(ls -1 | wc -l)'       # numFiles:     Count of non-hidden files in current dir

#   show the n most used commands. defaults to 10
#   ------------------------------------------------------------
function hstats {
  if [[ $# -lt 1 ]]; then
    NUM=10
  else
    NUM=${1}
  fi
  history | awk '{print $4}' | sort | uniq -c | sort -rn | head -"$NUM"
}

#   lsgrep: search through directory contents with grep
#   ------------------------------------------------------------
# shellcheck disable=SC2010
function lsgrep { ls | grep "$*" ; }

#   mcd:   Makes new Dir and jumps inside
#   --------------------------------------------------------------------
function mcd { mkdir -p -- "$*" ; cd -- "$*" || exit ; }

#   mans:   Search manpage given in agument '1' for term given in argument '2' (case insensitive)
#           displays paginated result with colored search terms and two lines surrounding each hit.
#           Example: mans mplayer codec
#   --------------------------------------------------------------------
function mans { man "$1" | grep -iC2 --color=always "$2" | less ; }

#   aliases: to remind yourself of an alias (given some part of it), expects aliases to be in ~/.bash_aliases
#   ------------------------------------------------------------
function aliases { /usr/bin/grep --color=always -i -a1 "$@" ~/.bash_aliases | grep -v '^\s*$' | less -FSRXc ; }

#   quiet: mute output of a command
#   ------------------------------------------------------------
function quiet {
  "$@" &> /dev/null &
}

function zipf { zip -r "$1".zip "$1" ; }           # zipf:         To create a ZIP archive of a folder

#   extract:  Extract most know archives with one command
#   ---------------------------------------------------------
function extract {
  if [ -f "$1" ] ; then
    case "$1" in
    *.tar.bz2)   tar xjf "$1"     ;;
    *.tar.gz)    tar xzf "$1"     ;;
    *.bz2)       bunzip2 "$1"     ;;
    *.rar)       unrar e "$1"     ;;
    *.gz)        gunzip "$1"      ;;
    *.tar)       tar xf "$1"      ;;
    *.tbz2)      tar xjf "$1"     ;;
    *.tgz)       tar xzf "$1"     ;;
    *.zip)       unzip "$1"       ;;
    *.Z)         uncompress "$1"  ;;
    *.7z)        7z x "$1"        ;;
    *)     printf '%s\n' "'$1' cannot be extracted via extract()" ;;
    esac
  else
    printf '%s\n' "'$1' is not a valid file"
  fi
}

