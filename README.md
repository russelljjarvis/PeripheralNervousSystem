Peripheral Nervous System Fiber radius
# NEURON+Python code


To install PyPNS run.
`git fetch origin master`

(to get the [dockerfile](https://github.com/russelljjarvis/PeripherlNervousSystem/blob/master/Dockerfile) in this repository).

inside the repository run:

```sh
sudo docker build -t "pypns" .
```

To mount a directory (i suggest the directory for this git repository) to the running docker container use the -v for volume argument:
```sh
docker run -v $pwd:work/extra_code pypns
```
