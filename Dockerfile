# Distributed under the terms of the Modified BSD License.
ARG BASE_CONTAINER=jupyter/scipy-notebook
FROM $BASE_CONTAINER
LABEL maintainer="Russell <rjjarvis@asu.edu>"
USER root
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    fonts-dejavu \
    gfortran \
    gcc && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get upgrade    
RUN apt-get install -y ncurses-base ncurses-bin
RUN apt-get update && apt-get install -y bzip2 ca-certificates automake libtool  \
                       libncurses5-dev libreadline-dev libgsl0-dev cmake ssh
USER jovyan

WORKDIR $HOME
RUN \
  wget http://www.neuron.yale.edu/ftp/neuron/versions/v7.7/nrn-7.7.tar.gz && \
  tar -xzf nrn-7.7.tar.gz && \
  rm nrn-7.7.tar.gz
WORKDIR $HOME/nrn-7.7
ENV PATH /usr/bin/python3/python:/opt/conda/bin:/opt/conda/bin/conda:/opt/conda/bin/python:$PATH
RUN ./configure --prefix=`pwd` --without-iv --with-nrnpython=/opt/conda/bin/python3
USER root
RUN sudo make all && \
     make install
USER jovyan

WORKDIR src/nrnpython
RUN python3 setup.py install
RUN python3 -c "import neuron"
ENV NEURON_HOME $HOME/nrn-7.7/x86_64
ENV PATH $NEURON_HOME/bin:$PATH
USER jovyan
WORKDIR $HOME/work/extra_work
WORKDIR $HOME/work
RUN git clone https://github.com/chlubba/PyPNS
WORKDIR PyPNS
RUN pip install -e .
RUN pip install tk
RUN python3 -c "import sys;print(sys.path)"
WORKDIR mods
RUN nrnivmodl
RUN conda clean --all -f -y && \
    fix-permissions $CONDA_DIR && \
    fix-permissions /home/$NB_USERs
RUN python3 -c "import neuron"
RUN python3 -c "import PyPNS"
RUN python3 -c "import tk"
RUN python3 -c "import matplotlib as mpl"
RUN python3 -c "mpl.use('tk')"
WORKDIR $HOME 
ENTRYPOINT /bin/bash
