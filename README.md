# Peripheral Nervous System Fiber radius
# NEURON+Python code
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/russelljjarvis/PeripherlNervousSystem/master)

[![Build Status](https://travis-ci.com/russelljjarvis/PeripheralNervousSystem.svg?branch=master)](https://travis-ci.com/russelljjarvis/PeripheralNervousSystem)

To install PyPNS run.
`git fetch origin master`

(to get the [dockerfile](https://github.com/russelljjarvis/PeripherlNervousSystem/blob/master/Dockerfile) in this repository).

inside the repository run:

```sh
sudo docker build -t "pypns" .
```

To mount a directory (i suggest the directory for this git repository) to the running docker container use the -v for volume argument:
```sh
cd pypns; docker run -v -e USERID=$UID $pwd:work/extra_code pypns notebook --ip=0.0.0.0 --NotebookApp.disable_check_xsrf=True
```

```
alias dipy1='cd pypns; docker run -v -e USERID=$UID $pwd:work/extra_code pypns notebook --ip=0.0.0.0 --NotebookApp.disable_check_xsrf=True'
```

Evidence it works.
click my-binder make new ipython3 notebook

Enter code like in this picture.
