---
title: "Connect to Github via Git using VsCode"
categories:
  - Blog
tags:
  - Git
  - VsCode
---

This blog introduces the required settings if one wishes to connect to Github using VsCode via SSH protocal. For the sake of simplicity, we assume VsCode and Git are properly installed in the environment.

### Setup Global User Name and User Email in Git Bash

Open Git Bash, and enter the following:

```ruby
$ git config --global user.name "Your Name"
$ git config --global user.email "example@example.com"
```

Note that `global` indicates all repositories in this enviornment will follow this setup.

### Setup Local SSH Key

`C:\User_Name\Test\.ssh`

![lena](/assets/images/lena.png)