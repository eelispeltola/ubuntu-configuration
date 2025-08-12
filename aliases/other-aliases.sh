# apt
# alias upall='sudo apt update -y && sudo apt upgrade -y && sudo apt dist-upgrade -y && sudo apt autoremove -y && flatpak update -y'
# Assumes brew, uv, and oh-my-posh are installed
alias upall='sudo apt update -y && sudo apt upgrade -y && sudo apt dist-upgrade -y && sudo apt autoremove -y && flatpak update -y && brew update && brew upgrade && oh-my-posh upgrade && uv self update'


# If brew is installed, add it to upall (this doesn't work for some reason)
# if [ "$(type -P -- "brew")" ]; then
#   alias upall='sudo apt update -y && sudo apt upgrade -y && sudo apt dist-upgrade -y && sudo apt autoremove -y && flatpak update -y && brew update && brew upgrade'
# fi
alias aptlist='apt list --installed'

# Python
alias newvenv='rm -r .venv; python3 -m venv .venv && source .venv/bin/activate && pip install --upgrade pip'
alias svenv='source .venv/bin/activate'
alias py='python'
alias coc='conda create --name ${PWD##*/}  # Creates conda env with the name of the current working directory'
alias coa='conda activate ${PWD##*/}'
alias cod='conda deactivate'
alias cor='conda remove --name ${PWD##*/} --all'

# ROS
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rteo='ros2 topic echo --once'
alias rti='ros2 topic info'
alias rth='ros2 topic hz'
alias rqtim='ros2 run rqt_image_view rqt_image_view'
alias frequentros='history | cut -c 8- | grep ros2 | sort | uniq -c  | sort -n -r | head -n 10'

