# First time install

A collection of configurations and applications for my Ubuntu configuration. Could be a script, but due to the infrequency of use it's maybe better to run these manually and inspect the output for any errors due to changes. 

## oh-my-posh

Nice bash terminal themes. Uses a custom theme blended from Catppuccin and Chips. Note the path export, which needs to be configured to your user.

```bash
sudo apt install unzip zip
curl -s https://ohmyposh.dev/install.sh | bash -s
cp tigerppuccin.omp.json ~/.cache/oh-my-posh/themes/
echo -e '\nexport PATH=$PATH:${HOME}/.local/bin' >> ~/.bashrc
echo 'eval "$(oh-my-posh init bash --config ~/.cache/oh-my-posh/themes/tigerppuccin.omp.json)"' >> ~/.bashrc
exec bash
```

Install Nerd Font (for example `UbuntuMono Nerd Font`, installable with the command below) either from [nerdfonts.com](https://www.nerdfonts.com/) or with [oh-my-posh](https://ohmyposh.dev/docs/installation/fonts), and configure terminal to use it. If using VSCode, set the integrated terminal font in VSCode settings.

```bash
oh-my-posh font install UbuntuMono
```

## Other tools

```bash
# Pip
sudo apt install python3-pip
# Use "j <(partial) path>", to jump to a previously visited directory in the terminal.
sudo apt install autojump
echo -e "\n. /usr/share/autojump/autojump.sh" >> ~/.bashrc
# Fuzzy find with a UI. For example "fzf ."
sudo apt install fzf
# git-extras adds more git commands. See "git extras"
brew install git-extras
# or 'sudo apt install git-extras' for an older version wihout homebrew
```

## Git configuration

Execute these lines one by one, `ssh-keygen` asks for additional information.

```bash
export EMAIL="your.email@here"
export MYNAME="Your Name"

ssh-keygen -t ed25519 -C ${EMAIL}
eval "$(ssh-agent -s)"
ssh-add -k ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub

git config --global user.name ${MYNAME}
git config --global user.email ${EMAIL}
```


## Nice aliases and bash configuration

Add the aliases from `bash-aliases.sh` and `git-aliases.sh` to your bash aliases list:

```bash
cat bash-aliases.sh git-aliases.sh >> $HOME/.bash_aliases
source $HOME/.bashrc
```

`bashrc-additions.sh` adds some options from sensible-bash.sh and oh-my-bash to make using bash a bit easier. Arguably most of these would be set if bash was made today. 

```bash
cat bashrc-additions.sh >> $HOME/.bashrc
source $HOME/.bashrc
```

## Conda (if needed)

For CLI use, miniconda is all that is needed.

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```

## Newer Python (if needed)

Conda installs its own Python version, but if using pip any other Python versions are best downloaded from `deadsnakes`.

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
PYTHON_VER="python3.11"; sudo apt install ${PYTHON_VER} ${PYTHON_VER}-dev ${PYTHON_VER}-venv
```

## Homebrew (if needed for other installations)

```bash
sudo apt install build-essential procps curl file git
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
test -d ~/.linuxbrew && eval "$(~/.linuxbrew/bin/brew shellenv)"
test -d /home/linuxbrew/.linuxbrew && eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"
echo "eval \"\$($(brew --prefix)/bin/brew shellenv)\"" >> ~/.bashrc
```
