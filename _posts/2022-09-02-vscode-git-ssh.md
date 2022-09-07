---
title: "Connect to GitHub via Git with VsCode"
categories:
  - Blog
tags:
  - Git
  - VsCode
---

This blog introduces the required settings if one wishes to connect to GitHub using VsCode via SSH protocal. For the sake of simplicity, we assume VsCode and Git are properly installed in the environment.

### Setup Global User Name and User Email in Git Bash

Open Git Bash, and enter the following:

```ruby
$ git config --global user.name "Your Name"
$ git config --global user.email "yourexample@example.com"
```

Note that `global` indicates all repositories in this enviornment will follow this setup.

### Setup Local SSH Key

Go to `C:\Users\$your_user_name` and check whether there is a `.ssh` folder with `id_rsa` and `id_rsa.pub`. Note that the folder might be hidden. If so, feel free to skip this part!

If not, open Git Bash and enter the following:

```ruby
$ ssh -keygen -t rsa -C "yourexample@example.com"
```

Keep pressing `Enter` until the key's randomart image is shown. You will find `id_rsa` and `id_rsa.pub` in file path `C:\Users\$your_user_name\.ssh`, where `id_rsa` is the private key and `id_rsa.pub` the public key.

### Associate SSH Key to GitHub

Go to GitHub settings. Find SSH and GPG keys in Access. Click on `New SSH key` on the top-right corner. The following page may show.

![ssh-key](/assets/images/ssh-key.png)

In ths page, copy the public key in `id_rsa.pub` mentioned previously and paste it in property Key. Click on `Add SSH key` when all properties are set.

Next, open Git Bash and enter the following:

```ruby
ssh -T git@github.com
```

If it returns `Hi your_user_name! You've successfully authenticated, but GitHub does not provide shell access`, you are all set for the next step!

### Git Clone via SSH

Copy the SSH address of the repository you hope to clone. Afterwards, Go to VsCode, click on `Ctrl+Shift+P` and enter `Git Clone`. Paste the SSH address here, then you may select the local storage location of this cloned repository.

### Commit Change to Remote Repository

![commit-change](/assets/images/commit-change.png)

The add icon indicates that you may stage the changes to local working tree. Commit the changes to local repository with the message you set. Make sure the message is not empty! 

The bottom-left sync icon indicates that you may push this repository to the branch that you set. Click on this and examine the changes on GitHub.
